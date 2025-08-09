// Blynk configuration - MUST be defined before including Blynk libraries
#define BLYNK_TEMPLATE_ID "TMPL60oDGsEwd"
#define BLYNK_TEMPLATE_NAME "ESP32 Dev Board x Thesis"
#define BLYNK_AUTH_TOKEN "PF5xBm-ZDWKSqgHWEUOwQGgF41ieM5HD"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <DHT11.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>

// WiFi credentials array - add your networks here
struct WiFiCredentials {
  const char* ssid;
  const char* password;
};

WiFiCredentials wifiNetworks[] = {
  {"Jayannsprgz", "mwaamwaa"},
  {"Hotspot", "waaypasswordnospace"},
  {"TP-Link_2AD8", "jenovah20"},
};

const int NUM_WIFI_NETWORKS = sizeof(wifiNetworks) / sizeof(wifiNetworks[0]);
int currentWiFiIndex = 0;
unsigned long wifiConnectStartTime = 0;
const unsigned long WIFI_TIMEOUT = 10000; // 10 seconds timeout per network

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin definitions for sensors
#define SOIL_MOISTURE_PIN 34 // Soil moisture sensor pin
#define WATER_SENSOR_PIN 35  // Water level sensor pin (added for water level monitoring)
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
#define MANUAL_READ_WATER_LEVEL_PIN V7        // Manual water level reading
#define MANUAL_READ_MOISTURE_PIN V8
#define PUMP_CONTROL_PIN V9
#define WATER_LEVEL_PIN V2                // Water level data pin

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

// Water level sensor calibration data (from watersensor_test.ino)
const int waterCalibrationPoints[6] = {0, 1497, 1735, 1852, 1902, 1946};  // ADC values
const int waterCalibrationLevels[6] = {0, 20, 40, 60, 80, 100};           // Corresponding percentages
const int WATER_THRESHOLD = 194;  // Water detection threshold

// In the variables section, add a state variable for the pump toggle
bool pumpToggleState = false;  // Track if pump is toggled on or off

// OLED display variables
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 1000;  // Update display every 1 second

// Multi-error tracking system
bool oledError = false;        // O - OLED display error
bool dhtError = false;         // HT - DHT11 sensor error (temperature + humidity)
bool moistureError = false;    // M - Moisture sensor error
bool waterLevelError = false;  // L - Water level sensor error
bool blynkError = false;       // B - Blynk connection error
bool wifiError = false;        // W - WiFi connection error

// Error tracking variables
bool blynkConnected = false;
unsigned long lastBlynkCheck = 0;
const unsigned long blynkCheckInterval = 5000;  // Check Blynk connection every 5 seconds
bool moistureSensorError = false;
unsigned long lastMoistureRead = 0;

// Function to print current network info
void printNetworkInfo() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("=== Network Information ===");
    Serial.print("Connected to: ");
    Serial.println(wifiNetworks[currentWiFiIndex].ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    Serial.print("Blynk Status: ");
    Serial.println(blynkConnected ? "Connected" : "Disconnected");
    Serial.println("===========================");
  } else {
    Serial.println("No WiFi connection");
  }
}

// Current sensor values for display
float currentTemperature = 0.0;
float currentHumidity = 0.0;
int currentSoilMoisture = 0;
int currentWaterLevel = 0;               // Current water level percentage

// Water level sensor functions
int getInterpolatedWaterLevel(int adcValue) {
  // If value is below or at dry level
  if (adcValue <= waterCalibrationPoints[0]) {
    return 0;
  }
  
  // If value is above or at wet level
  if (adcValue >= waterCalibrationPoints[5]) {
    return 100;
  }
  
  // Find which two calibration points the current reading falls between
  for (int i = 0; i < 5; i++) {
    if (adcValue >= waterCalibrationPoints[i] && adcValue <= waterCalibrationPoints[i + 1]) {
      // Linear interpolation between two points
      int x1 = waterCalibrationPoints[i];
      int x2 = waterCalibrationPoints[i + 1];
      int y1 = waterCalibrationLevels[i];
      int y2 = waterCalibrationLevels[i + 1];
      
      // y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
      int result = y1 + (adcValue - x1) * (y2 - y1) / (x2 - x1);
      return constrain(result, 0, 100);
    }
  }
  
  // Fallback (shouldn't reach here)
  return map(adcValue, waterCalibrationPoints[0], waterCalibrationPoints[5], 0, 100);
}

int getWaterLevelPercentage() {
  int rawValue = analogRead(WATER_SENSOR_PIN);
  
  // Check for sensor errors
  // If reading is 0 or 4095 (max ADC value), sensor might be disconnected
  if (rawValue == 0 || rawValue >= 4095) {
    waterLevelError = true;
    Serial.print("Water level sensor error - Raw value: ");
    Serial.println(rawValue);
    return currentWaterLevel; // Return last known good value
  }
  
  // Check if reading is extremely out of range (indicates sensor problem)
  if (rawValue > (waterCalibrationPoints[5] + 500) || rawValue < -200) {
    waterLevelError = true;
    Serial.print("Water level sensor out of range - Raw value: ");
    Serial.println(rawValue);
    return currentWaterLevel; // Return last known good value
  }
  
  // If we get here, sensor reading seems valid
  waterLevelError = false;
  
  // Convert to percentage using multi-point interpolation
  return getInterpolatedWaterLevel(rawValue);
}

// WiFi connection functions
bool connectToWiFi() {
  Serial.println("Scanning for available WiFi networks...");
  
  for (int attempt = 0; attempt < NUM_WIFI_NETWORKS; attempt++) {
    currentWiFiIndex = attempt;
    
    Serial.print("Trying WiFi ");
    Serial.print(attempt + 1);
    Serial.print("/");
    Serial.print(NUM_WIFI_NETWORKS);
    Serial.print(": ");
    Serial.println(wifiNetworks[currentWiFiIndex].ssid);
    
    WiFi.begin(wifiNetworks[currentWiFiIndex].ssid, wifiNetworks[currentWiFiIndex].password);
    wifiConnectStartTime = millis();
    
    // Wait for connection with timeout
    while (WiFi.status() != WL_CONNECTED && (millis() - wifiConnectStartTime < WIFI_TIMEOUT)) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("Connected to: ");
      Serial.println(wifiNetworks[currentWiFiIndex].ssid);
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      return true;
    } else {
      Serial.println(" Failed!");
      WiFi.disconnect();
      delay(1000); // Brief delay before trying next network
    }
  }
  
  Serial.println("Failed to connect to any WiFi network!");
  return false;
}

bool reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true; // Already connected
  }
  
  Serial.println("Attempting WiFi reconnection...");
  
  // Try current network first
  Serial.print("Retrying: ");
  Serial.println(wifiNetworks[currentWiFiIndex].ssid);
  
  WiFi.begin(wifiNetworks[currentWiFiIndex].ssid, wifiNetworks[currentWiFiIndex].password);
  wifiConnectStartTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiConnectStartTime < WIFI_TIMEOUT)) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" Reconnected!");
    return true;
  }
  
  Serial.println(" Failed, trying other networks...");
  
  // If current network fails, try all networks again
  return connectToWiFi();
}

void setup() {
  Serial.begin(9600);
  
  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    oledError = true;
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.display();
    Serial.println("OLED display initialized successfully");
  }
  
  // Set up output pins
  pinMode(PUMP_RELAY_27, OUTPUT);
  pinMode(LED_MODE, OUTPUT);
  
  // Set up button pins as inputs with pull-up resistors
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  pinMode(BUTTON_PUMP, INPUT_PULLUP);
  
  // Initialize outputs based on initial mode - LOW means pump is OFF
  digitalWrite(LED_MODE, isAutomatedMode ? HIGH : LOW);
  digitalWrite(PUMP_RELAY_27, LOW);  // Initialize to LOW (pump OFF)
  
  // Connect to WiFi first (cycle through available networks)
  Serial.println("Starting WiFi connection...");
  bool wifiConnected = connectToWiFi();
  
  if (wifiConnected) {
    // Print network information
    printNetworkInfo();
    
    // Connect to Blynk (non-blocking)
    Serial.println("Attempting Blynk connection...");
    Blynk.begin(BLYNK_AUTH_TOKEN, wifiNetworks[currentWiFiIndex].ssid, wifiNetworks[currentWiFiIndex].password);
    
    // Check initial connection status without blocking
    if (Blynk.connected()) {
      blynkConnected = true;
      Serial.println("Blynk connected immediately!");
      
      // Initialize Blynk button states
      Blynk.virtualWrite(MODE_TOGGLE_PIN, isAutomatedMode ? 1 : 0);
      Blynk.virtualWrite(PUMP_CONTROL_PIN, 0);
    } else {
      blynkConnected = false;
      Serial.println("Blynk not connected initially - will retry in background");
      // Don't set error code here, let the loop handle connection monitoring
    }
  } else {
    blynkConnected = false;
    Serial.println("No WiFi connection - Blynk unavailable");
  }
  
  Serial.println("System started in Automated Mode - proceeding regardless of Blynk status");
  
  // Initial display update
  updateDisplay();
}

void loop() {
  // Check WiFi and Blynk connection status periodically
  unsigned long currentMillis = millis();
  if (currentMillis - lastBlynkCheck >= blynkCheckInterval) {
    lastBlynkCheck = currentMillis;
    
    // First check WiFi status
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting reconnection...");
      blynkConnected = false; // Blynk can't work without WiFi
      
      // Try to reconnect WiFi
      if (reconnectWiFi()) {
        Serial.println("WiFi reconnected, attempting Blynk reconnection...");
        Blynk.begin(BLYNK_AUTH_TOKEN, wifiNetworks[currentWiFiIndex].ssid, wifiNetworks[currentWiFiIndex].password);
      }
    } else {
      // WiFi is connected, check Blynk
      if (Blynk.connected()) {
        if (!blynkConnected) {
          // Blynk reconnected
          blynkConnected = true;
          Serial.println("Blynk reconnected!");
          // Connection restored - let updateErrorCodes handle clearing the error
        }
      } else {
        if (blynkConnected) {
          // Lost Blynk connection
          blynkConnected = false;
          Serial.println("Blynk connection lost - system continues normally");
          // Don't immediately set error code - let updateErrorCodes handle it with delay
        }
      }
    }
  }
  
  // Always try to run Blynk (it handles disconnection gracefully)
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
    
    // No need to check timed outputs anymore
    // checkTimedOutputs();
  }
  
  // Periodically read and send sensor data regardless of mode
  if (currentMillis - lastSensorReadTime >= sensorReadInterval) {
    lastSensorReadTime = currentMillis;
    readAndSendAllSensorData();
  }
  
  // Update OLED display periodically
  if (currentMillis - lastDisplayUpdate >= displayUpdateInterval) {
    lastDisplayUpdate = currentMillis;
    updateDisplay();
  }
}

// Blynk handlers
BLYNK_WRITE(MODE_TOGGLE_PIN) {
  if (!blynkConnected) return;  // Don't process if not connected
  
  isAutomatedMode = param.asInt();
  
  // Turn off pump when switching modes
  digitalWrite(PUMP_RELAY_27, LOW);  // LOW = pump OFF
  pumpToggleState = false;
  
  // Update Blynk button state
  if (blynkConnected) {
    Blynk.virtualWrite(PUMP_CONTROL_PIN, 0);
  }
  
  Serial.print("Mode changed from Blynk to: ");
  Serial.println(isAutomatedMode ? "Automated" : "Manual");
}

BLYNK_WRITE(PUMP_CONTROL_PIN) {
  if (!blynkConnected) return;  // Don't process if not connected
  
  if (!isAutomatedMode) {
    pumpToggleState = param.asInt();
    
    if (pumpToggleState) {
      // Turn pump ON
      digitalWrite(PUMP_RELAY_27, HIGH);  // HIGH = pump ON
      Serial.println("Manual (Blynk): Pump toggled ON");
    } else {
      // Turn pump OFF
      digitalWrite(PUMP_RELAY_27, LOW);  // LOW = pump OFF
      Serial.println("Manual (Blynk): Pump toggled OFF");
    }
  }
}

BLYNK_WRITE(MANUAL_READ_ALL_PIN) {
  if (!blynkConnected) return;  // Don't process if not connected
  
  if (param.asInt() == 1) {
    readAndSendAllSensorData();
  }
}

BLYNK_WRITE(MANUAL_READ_TEMP_HUM_PIN) {
  if (!blynkConnected) return;  // Don't process if not connected
  
  if (param.asInt() == 1) {
    readAndSendTempHumidity();
  }
}

BLYNK_WRITE(MANUAL_READ_WATER_LEVEL_PIN) {
  if (!blynkConnected) return;  // Don't process if not connected
  
  if (param.asInt() == 1) {
    readAndSendWaterLevel();
  }
}

BLYNK_WRITE(MANUAL_READ_MOISTURE_PIN) {
  if (!blynkConnected) return;  // Don't process if not connected
  
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
      digitalWrite(PUMP_RELAY_27, LOW);  // LOW = pump OFF
      pumpToggleState = false;
      
      // Update Blynk button states
      Blynk.virtualWrite(MODE_TOGGLE_PIN, isAutomatedMode ? 1 : 0);
      Blynk.virtualWrite(PUMP_CONTROL_PIN, 0);
      
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
          digitalWrite(PUMP_RELAY_27, HIGH);  // HIGH = pump ON
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
          digitalWrite(PUMP_RELAY_27, LOW);  // LOW = pump OFF
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
  // Handle pump button as a toggle
  int buttonPumpReading = digitalRead(BUTTON_PUMP);
  
  if (buttonPumpReading != lastButtonPumpState) {
    // Wait for debounce
    delay(50);
    
    // Read the button again to make sure it's still pressed
    buttonPumpReading = digitalRead(BUTTON_PUMP);
    
    if (buttonPumpReading == LOW && lastButtonPumpState == HIGH) {  // Button pressed (LOW due to pull-up)
      // Toggle pump state
      pumpToggleState = !pumpToggleState;
      
      if (pumpToggleState) {
        // Turn pump ON
        digitalWrite(PUMP_RELAY_27, HIGH);  // HIGH = pump ON
        Serial.println("Manual: Pump toggled ON");
        
        // Update Blynk button state
        if (blynkConnected) {
          Blynk.virtualWrite(PUMP_CONTROL_PIN, 1);
        }
      } else {
        // Turn pump OFF
        digitalWrite(PUMP_RELAY_27, LOW);  // LOW = pump OFF
        Serial.println("Manual: Pump toggled OFF");
        
        // Update Blynk button state
        if (blynkConnected) {
          Blynk.virtualWrite(PUMP_CONTROL_PIN, 0);
        }
      }
    }
    
    lastButtonPumpState = buttonPumpReading;
  }
}

// Sensor reading functions
int readSoilMoisture() {
  return analogRead(SOIL_MOISTURE_PIN);
}

int getSoilMoisturePercentage() {
  int rawValue = readSoilMoisture();
  lastMoistureRead = millis();
  
  // Check for sensor errors
  // If reading is 0 or 4095 (max ADC value), sensor might be disconnected
  if (rawValue == 0 || rawValue >= 4095) {
    moistureSensorError = true;
    Serial.print("Moisture sensor error - Raw value: ");
    Serial.println(rawValue);
    return currentSoilMoisture; // Return last known good value
  }
  
  // Check if reading is extremely out of range (indicates sensor problem)
  if (rawValue > (AIR_VALUE + 500) || rawValue < (WATER_VALUE - 200)) {
    moistureSensorError = true;
    Serial.print("Moisture sensor out of range - Raw value: ");
    Serial.println(rawValue);
    return currentSoilMoisture; // Return last known good value
  }
  
  // If we get here, sensor reading seems valid
  moistureSensorError = false;
  
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
    
    // Update current values for display
    currentTemperature = temperature;
    currentHumidity = humidity;
    
    // Clear DHT error if it was previously set
    if (dhtError) {
      dhtError = false;
    }
    
    // Send to Blynk
    Blynk.virtualWrite(V0, temperature);
    Blynk.virtualWrite(V1, humidity);
  } else {
    Serial.println(DHT11::getErrorString(dht_result));
    dhtError = true;
  }
}

void readAndSendMoisture() {
  int soilMoisturePercentage = getSoilMoisturePercentage();
  
  if (moistureSensorError) {
    Serial.println("Moisture sensor error detected!");
    moistureError = true;
  } else {
    Serial.print("Soil Moisture: ");
    Serial.print(soilMoisturePercentage);
    Serial.println(" %");
    
    // Update current value for display
    currentSoilMoisture = soilMoisturePercentage;
    
    // Clear moisture error if it was previously set
    if (moistureError) {
      moistureError = false;
    }
    
    // Send to Blynk only if connected
    if (blynkConnected) {
      Blynk.virtualWrite(V3, soilMoisturePercentage);
    }
  }
}

void readAndSendWaterLevel() {
  int waterLevelPercentage = getWaterLevelPercentage();
  
  if (waterLevelError) {
    Serial.println("Water level sensor error detected!");
  } else {
    Serial.print("Water Level: ");
    Serial.print(waterLevelPercentage);
    Serial.println(" %");
    
    // Update current value for display
    currentWaterLevel = waterLevelPercentage;
    
    // Send to Blynk only if connected
    if (blynkConnected) {
      Blynk.virtualWrite(WATER_LEVEL_PIN, waterLevelPercentage);
    }
  }
}

void readAndSendAllSensorData() {
  // Read DHT11 (temperature and humidity)
  int temperature = 0;
  int humidity = 0;
  int dht_result = dht11.readTemperatureHumidity(temperature, humidity);
  bool currentDhtError = false;
  
  if (dht_result == 0) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C\tHumidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    
    // Update current values for display
    currentTemperature = temperature;
    currentHumidity = humidity;
    currentDhtError = false;
  } else {
    Serial.println(DHT11::getErrorString(dht_result));
    currentDhtError = true;
    // Set to default values or previous values if reading fails
    temperature = currentTemperature;
    humidity = currentHumidity;
  }
  
  // Read soil moisture
  int soilMoisturePercentage = getSoilMoisturePercentage();
  
  if (!moistureSensorError) {
    Serial.print("Soil Moisture: ");
    Serial.print(soilMoisturePercentage);
    Serial.println(" %");
    
    // Update current value for display
    currentSoilMoisture = soilMoisturePercentage;
  }
  
  // Read water level
  int waterLevelPercentage = getWaterLevelPercentage();
  
  if (!waterLevelError) {
    Serial.print("Water Level: ");
    Serial.print(waterLevelPercentage);
    Serial.println(" %");
    
    // Update current value for display
    currentWaterLevel = waterLevelPercentage;
  }
  
  // Update error codes based on current sensor states
  updateErrorCodes(currentDhtError, moistureSensorError, waterLevelError, !blynkConnected);
  
  // Send all data to Blynk at once (only if connected)
  if (blynkConnected) {
    Blynk.virtualWrite(V0, temperature);
    Blynk.virtualWrite(V1, humidity);
    Blynk.virtualWrite(V3, soilMoisturePercentage);
    Blynk.virtualWrite(WATER_LEVEL_PIN, waterLevelPercentage);
  }
  
  // Optional: Print a separator for better serial monitor readability
  Serial.println("------------------------------");
}

// Function to manage multiple error codes
void updateErrorCodes(bool dhtErr, bool moistErr, bool waterLevelErr, bool blynkErr) {
  // Update individual error flags
  dhtError = dhtErr;
  moistureError = moistErr;
  waterLevelError = waterLevelErr; // Add water level error tracking
  
  // Update WiFi error status
  wifiError = (WiFi.status() != WL_CONNECTED);
  
  // Handle Blynk error with startup delay
  if (blynkErr && millis() > 30000) {  // Wait 30 seconds after startup
    blynkError = true;
  } else if (!blynkErr) {
    blynkError = false;
  }
  // Note: oledError is set separately during setup and doesn't change
}

// Function to generate combined error code string
String getErrorCodeString() {
  String errorStr = "";
  
  if (oledError) errorStr += "O ";
  if (dhtError) errorStr += "HT ";
  if (moistureError) errorStr += "M ";
  if (waterLevelError) errorStr += "L "; // Add water level error to string
  if (wifiError) errorStr += "W ";
  if (blynkError) errorStr += "B ";
  
  return errorStr;
}

void updateDisplay() {
  if (oledError) {
    return; // Don't try to update if OLED failed to initialize
  }
  
  display.clearDisplay();
  
  drawModeIndicators();
  drawSensorReadings();
  drawPumpStatus();
  drawErrorMessage();
  
  display.display();
}

void drawModeIndicators() {
  // "MODE" label vertically on the left
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("m"));
  display.setCursor(0, 6);
  display.print(F("o"));
  display.setCursor(0, 13);
  display.print(F("d"));
  display.setCursor(0, 19);
  display.print(F("e"));
  
  // Mode selection box (moved right)
  display.drawRect(8, 0, 16, 35, SSD1306_WHITE);
  display.drawLine(8, 17, 22, 17, SSD1306_WHITE);
  
  // Automatic mode indicator (top half)
  display.setTextSize(2);
  if (isAutomatedMode) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Inverted when on
    display.fillRect(9, 1, 14, 15, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(11, 2);
  display.print(F("A"));
  
  // Manual mode indicator (bottom half)
  if (!isAutomatedMode) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Inverted when on
    display.fillRect(9, 18, 14, 16, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(11, 19);
  display.print(F("M"));
}

void drawSensorReadings() {
  display.setTextColor(SSD1306_WHITE);
  
  // Temperature section
  display.setTextSize(1);
  display.setCursor(35, 0);
  display.print(F("TEMP"));
  
  display.setTextSize(2);
  display.setCursor(45, 9);
  display.printf("%.0f", currentTemperature);
  
  display.setTextSize(1);
  display.setCursor(70, 9);
  display.print(F("c"));
  
  // Humidity section
  display.setTextSize(1);
  display.setCursor(82, 0);
  display.print(F("HUM"));
  
  display.setTextSize(2);
  display.setCursor(90, 9);
  display.print((int)currentHumidity);
  
  display.setTextSize(1);
  display.setCursor(114, 9);
  display.print(F("%"));
  
  // Soil moisture section
  display.setTextSize(1);
  display.setCursor(35, 27);
  display.print(F("MOIST"));
  
  display.setTextSize(2);
  display.setCursor(45, 39);
  display.print(currentSoilMoisture);
  
  display.setTextSize(1);
  display.setCursor(69, 39);
  display.print(F("%"));
  
  // Water level section
  display.setTextSize(1);
  display.setCursor(81, 27);
  display.print(F("WLVL"));
  
  display.setTextSize(2);
  display.setCursor(89, 39);
  display.print(currentWaterLevel);
  
  display.setTextSize(1);
  display.setCursor(114, 39);
  display.print(F("%"));
}

void drawPumpStatus() {
  // "PUMP" label vertically on the left
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 36);
  display.print(F("p"));
  display.setCursor(0, 43);
  display.print(F("u"));
  display.setCursor(0, 49);
  display.print(F("m"));
  display.setCursor(0, 55);
  display.print(F("p"));

  // Pump status indicator (circle) moved to the right
  int circleX = 15; // X position for the circle
  int circleY = 50; // Y position for the circle
  int radius   = 7;

  bool isPumpOn = (pumpToggleState || (isAutomatedMode && digitalRead(PUMP_RELAY_27) == HIGH));
  if (isPumpOn) {
    display.fillCircle(circleX, circleY, radius, SSD1306_WHITE); // Filled circle = ON
  } else {
    display.drawCircle(circleX, circleY, radius, SSD1306_WHITE); // Empty circle = OFF
  }
}

void drawErrorMessage() {
  String errorCodes = getErrorCodeString();
  if (errorCodes != "") {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(35, 56);
    display.print("E: ");
    display.print(errorCodes);
  }
} 