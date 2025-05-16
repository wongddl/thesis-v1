// Blynk configuration - MUST be defined before including Blynk libraries
#define BLYNK_TEMPLATE_ID "TMPL60oDGsEwd"
#define BLYNK_TEMPLATE_NAME "ESP32 Dev Board x Thesis"
#define BLYNK_AUTH_TOKEN "PF5xBm-ZDWKSqgHWEUOwQGgF41ieM5HD"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <DHT11.h>
#include <BlynkSimpleEsp32.h>

// Pin definitions for sensors
#define SOIL_MOISTURE_PIN 34 // Soil moisture sensor pin
DHT11 dht11(0);             // DHT11 on GPIO0

// Pin definitions for relays and LEDs
#define PUMP_RELAY_27 27    // Pump relay on GPIO27
#define LED_MODE 2          // LED to indicate mode (ON = Automated, OFF = Manual)
#define BUTTON_PUMP 32      // Button to control pump relay in manual mode
#define BUTTON_MODE 4       // Button to toggle between manual and automated modes

// Blynk virtual pins
#define MODE_TOGGLE_PIN V4
#define MANUAL_READ_ALL_PIN V5
#define MANUAL_READ_TEMP_HUM_PIN V6
#define MANUAL_READ_MOISTURE_PIN V8
#define PUMP_CONTROL_PIN V9

// Variables for mode control
bool isAutomatedMode = true;  // Start in automated mode by default

// Variables for button debouncing
bool lastModeButtonState = HIGH;
bool lastButtonPumpState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // Debounce time in milliseconds

// Variables for automated mode timing
unsigned long previousMillis = 0;
const long interval = 1000;  // Interval for LED blinking (1 second)

// State definitions for automated mode
#define AUTO_STATE_IDLE 0
#define AUTO_STATE_WATERING 1
#define AUTO_STATE_MONITORING 2

// Variables for non-blocking automated sequence
int automatedState = AUTO_STATE_IDLE;
unsigned long stateStartTime = 0;
const long wateringDuration = 5000; // 5 seconds for watering
const long idleDuration = 10000;    // 10 seconds between cycles

// Variables for manual mode timing
unsigned long pumpStartTime = 0;
bool pumpActive = false;
const long activeDuration = 1000;  // Duration to keep outputs active (1 second)

// Variables for soil moisture sensor
const int AIR_VALUE = 3000;  // Sensor reading in dry air
const int WATER_VALUE = 1500; // Sensor reading in water

// Variables for periodic sensor readings
unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 10000;  // 10 seconds

// Soil moisture thresholds
const int LOW_MOISTURE_THRESHOLD = 45;   // Below this percentage, start watering
const int HIGH_MOISTURE_THRESHOLD = 65;  // Above this percentage, stop watering

void setup() {
  Serial.begin(9600);
  
  // Set up output pins
  pinMode(PUMP_RELAY_27, OUTPUT);
  pinMode(LED_MODE, OUTPUT);
  
  // Set up button pins as inputs with pull-up resistors
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  pinMode(BUTTON_PUMP, INPUT_PULLUP);
  
  // Initialize outputs based on initial mode - HIGH means relays are OFF
  digitalWrite(LED_MODE, isAutomatedMode ? HIGH : LOW);
  digitalWrite(PUMP_RELAY_27, HIGH);  // Initialize to HIGH (relay inactive, pump OFF)
  
  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, "Salmentar2.4g", "Salmentar434!");
  
  // Initialize Blynk button states
  Blynk.virtualWrite(MODE_TOGGLE_PIN, isAutomatedMode ? 1 : 0);
  Blynk.virtualWrite(PUMP_CONTROL_PIN, 0);
  
  Serial.println("System started in Automated Mode");
}

void loop() {
  Blynk.run();
  
  // Check mode toggle button (physical)
  checkModeButton();
  
  // Update mode indicator LED
  digitalWrite(LED_MODE, isAutomatedMode ? HIGH : LOW);
  
  // Handle automated or manual mode
  if (isAutomatedMode) {
    // Run automated sequence
    runAutomatedSequence();
  } else {
    // Handle manual button inputs (physical)
    handleManualButtons();
    
    // Check if we need to turn off the pump after 1 second
    checkTimedOutputs();
  }
  
  // Periodically read and send sensor data regardless of mode
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorReadTime >= sensorReadInterval) {
    lastSensorReadTime = currentMillis;
    readAndSendAllSensorData();
  }
}

// Blynk handlers
BLYNK_WRITE(MODE_TOGGLE_PIN) {
  isAutomatedMode = param.asInt();
  
  // Turn off pump when switching modes
  digitalWrite(PUMP_RELAY_27, HIGH);
  pumpActive = false;
  
  Serial.print("Mode changed from Blynk to: ");
  Serial.println(isAutomatedMode ? "Automated" : "Manual");
}

BLYNK_WRITE(PUMP_CONTROL_PIN) {
  if (!isAutomatedMode && param.asInt() == 1) {
    // Turn on pump and start timer
    digitalWrite(PUMP_RELAY_27, LOW);
    pumpStartTime = millis();
    pumpActive = true;
    Serial.println("Manual (Blynk): Pump turned ON for 1 second");
    
    // Reset button state after 1 second
    Blynk.syncVirtual(PUMP_CONTROL_PIN);
  }
}

BLYNK_WRITE(MANUAL_READ_ALL_PIN) {
  if (param.asInt() == 1) {
    readAndSendAllSensorData();
  }
}

BLYNK_WRITE(MANUAL_READ_TEMP_HUM_PIN) {
  if (param.asInt() == 1) {
    readAndSendTempHumidity();
  }
}

BLYNK_WRITE(MANUAL_READ_MOISTURE_PIN) {
  if (param.asInt() == 1) {
    readAndSendMoisture();
  }
}

void checkModeButton() {
  // Read the state of the mode button
  int reading = digitalRead(BUTTON_MODE);
  
  // If the button is pressed (LOW due to pull-up resistor)
  if (reading == LOW && lastModeButtonState == HIGH) {
    // Wait for debounce
    delay(50);
    
    // Read the button again to make sure it's still pressed
    reading = digitalRead(BUTTON_MODE);
    
    if (reading == LOW) {
      // Toggle the mode
      isAutomatedMode = !isAutomatedMode;
      
      // Turn off pump when switching modes
      digitalWrite(PUMP_RELAY_27, HIGH);
      pumpActive = false;
      
      // Update Blynk button state
      Blynk.virtualWrite(MODE_TOGGLE_PIN, isAutomatedMode ? 1 : 0);
      
      Serial.print("Mode changed by physical button to: ");
      Serial.println(isAutomatedMode ? "Automated" : "Manual");
      
      // Wait for button release to prevent multiple toggles
      while (digitalRead(BUTTON_MODE) == LOW) {
        delay(10);
      }
    }
  }
  
  lastModeButtonState = reading;
}

void runAutomatedSequence() {
  unsigned long currentMillis = millis();
  int soilMoisturePercentage;
  
  // State machine for automated sequence
  switch (automatedState) {
    case AUTO_STATE_IDLE:
      // In idle state, check if it's time to start a new cycle
      if (currentMillis - stateStartTime >= idleDuration) {
        // Check soil moisture to see if watering is needed
        soilMoisturePercentage = getSoilMoisturePercentage();
        
        if (soilMoisturePercentage < LOW_MOISTURE_THRESHOLD) {
          Serial.print("Automated: Soil moisture is low (");
          Serial.print(soilMoisturePercentage);
          Serial.println("%), starting watering cycle");
          
          // Move to watering state
          automatedState = AUTO_STATE_WATERING;
          stateStartTime = currentMillis;
          
          // Start watering by activating pump
          digitalWrite(PUMP_RELAY_27, LOW);  // LOW activates the relay (pump ON)
          Serial.println("Automated: Pump ON for watering");
        } else {
          // Soil moisture is adequate, stay in idle but reset timer
          Serial.print("Automated: Soil moisture is adequate (");
          Serial.print(soilMoisturePercentage);
          Serial.println("%), no watering needed");
          stateStartTime = currentMillis;
        }
      }
      break;
      
    case AUTO_STATE_WATERING:
      // After initial watering period, move to monitoring state
      if (currentMillis - stateStartTime >= wateringDuration) {
        // Don't turn off the pump yet, just move to monitoring state
        Serial.println("Automated: Initial watering complete, monitoring soil moisture");
        automatedState = AUTO_STATE_MONITORING;
        stateStartTime = currentMillis;
      }
      break;
      
    case AUTO_STATE_MONITORING:
      // Check soil moisture every second in monitoring state
      if (currentMillis - stateStartTime >= 1000) {  // Check every second
        stateStartTime = currentMillis;  // Reset timer for next check
        
        // Read current soil moisture
        soilMoisturePercentage = getSoilMoisturePercentage();
        Serial.print("Automated: Monitoring - Current soil moisture: ");
        Serial.print(soilMoisturePercentage);
        Serial.println("%");
        
        // If moisture is above threshold, stop watering
        if (soilMoisturePercentage >= HIGH_MOISTURE_THRESHOLD) {
          digitalWrite(PUMP_RELAY_27, HIGH);  // HIGH deactivates the relay (pump OFF)
          Serial.print("Automated: Target moisture reached (");
          Serial.print(soilMoisturePercentage);
          Serial.println("%), pump OFF");
          
          Serial.println("------------------------------");
          
          // Return to idle state
          automatedState = AUTO_STATE_IDLE;
          stateStartTime = currentMillis;
        }
        // Otherwise, continue pumping and monitoring
      }
      break;
  }
}

void handleManualButtons() {
  // Handle pump button
  int buttonPumpReading = digitalRead(BUTTON_PUMP);
  
  if (buttonPumpReading != lastButtonPumpState) {
    if (buttonPumpReading == LOW) {  // Button pressed (LOW due to pull-up)
      // Turn on pump and start timer
      digitalWrite(PUMP_RELAY_27, LOW);  // LOW activates the relay (pump ON)
      pumpStartTime = millis();
      pumpActive = true;
      Serial.println("Manual: Pump turned ON for 1 second");
    }
    lastButtonPumpState = buttonPumpReading;
    delay(50);  // Simple debounce
  }
}

void checkTimedOutputs() {
  unsigned long currentMillis = millis();
  
  // Turn off pump after duration
  if (pumpActive && (currentMillis - pumpStartTime >= activeDuration)) {
    digitalWrite(PUMP_RELAY_27, HIGH);  // HIGH deactivates the relay (pump OFF)
    pumpActive = false;
    Serial.println("Manual: Pump turned OFF after timeout");
  }
}

// Sensor reading functions
int readSoilMoisture() {
  return analogRead(SOIL_MOISTURE_PIN);
}

int getSoilMoisturePercentage() {
  int rawValue = readSoilMoisture();

  // Ensure rawValue is within the expected range
  if (rawValue > AIR_VALUE) {
    rawValue = AIR_VALUE;
  } else if (rawValue < WATER_VALUE) {
    rawValue = WATER_VALUE;
  }
  
  // Map the raw value to a percentage (0-100%)
  return map(rawValue, AIR_VALUE, WATER_VALUE, 0, 100);
}

void readAndSendTempHumidity() {
  int temperature = 0;
  int humidity = 0;
  int dht_result = dht11.readTemperatureHumidity(temperature, humidity);
  if (dht_result == 0) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C\tHumidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    
    // Send to Blynk
    Blynk.virtualWrite(V0, temperature);
    Blynk.virtualWrite(V1, humidity);
  } else {
    Serial.println(DHT11::getErrorString(dht_result));
  }
}

void readAndSendMoisture() {
  int soilMoisturePercentage = getSoilMoisturePercentage();
  Serial.print("Soil Moisture: ");
  Serial.print(soilMoisturePercentage);
  Serial.println(" %");
  
  // Send to Blynk
  Blynk.virtualWrite(V3, soilMoisturePercentage);
}

void readAndSendAllSensorData() {
  // Read DHT11 (temperature and humidity)
  int temperature = 0;
  int humidity = 0;
  int dht_result = dht11.readTemperatureHumidity(temperature, humidity);
  if (dht_result == 0) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C\tHumidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    Serial.println(DHT11::getErrorString(dht_result));
    // Set to default values or previous values if reading fails
    temperature = 0;
    humidity = 0;
  }
  
  // Read soil moisture
  int soilMoisturePercentage = getSoilMoisturePercentage();
  Serial.print("Soil Moisture: ");
  Serial.print(soilMoisturePercentage);
  Serial.println(" %");
  
  // Send all data to Blynk at once
  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V3, soilMoisturePercentage);
  
  // Optional: Print a separator for better serial monitor readability
  Serial.println("------------------------------");
} 