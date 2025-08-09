#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// System variables
bool isAutoMode = true;
bool isPumpOn = false;
float temperature = 25.6;
float humidity = 62.3;
int soilMoisture = 45;

void setup() {
  Serial.begin(115200);
  
  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  updateDisplay();
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);  // Set all text to size 1
  
  // Header with mode status
  display.setCursor(0, 0);
  display.print("Mode: ");
  display.print(isAutoMode ? "AUTO" : "MANUAL");
  
  // Pump status
  display.setCursor(75, 0);
  display.print("Pump:");
  display.setCursor(110, 0);
  if (isPumpOn) {
    display.print("ON");
  } else {
    display.print("OFF");
  }
  
  // Separator line
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // Soil moisture section (PRIORITY - TOP)
  display.setCursor(0, 15);
  display.print("Soil:");
  display.setCursor(35, 15);
  display.print(soilMoisture);
  display.setCursor(65, 15);
  display.print("%");
  
  // Soil moisture status text
  display.setCursor(85, 15);
  if (soilMoisture < 30) {
    display.print("DRY");
  } else if (soilMoisture < 60) {
    display.print("OK");
  } else {
    display.print("WET");
  }
  
  // Soil moisture progress bar
  int barWidth = 120;
  int barHeight = 8;
  int barX = 4;
  int barY = 25;
  
  // Draw progress bar outline
  display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
  
  // Fill progress bar based on soil moisture percentage
  int fillWidth = map(soilMoisture, 0, 100, 0, barWidth - 2);
  display.fillRect(barX + 1, barY + 1, fillWidth, barHeight - 2, SSD1306_WHITE);
  
  // Temperature and Humidity section (LOWER - Two columns in one row)
  // Temperature (Left column)
  display.setCursor(0, 40);
  display.print("T:");
  display.setCursor(20, 40);
  display.print(temperature, 1);
  display.setCursor(45, 40);
  display.print("C");
  
  // Humidity (Right column)
  display.setCursor(70, 40);
  display.print("H:");
  display.setCursor(90, 40);
  display.print(humidity, 1);
  display.setCursor(115, 40);
  display.print("%");
  
  display.display();
}

void loop() {
  // Simulate data changes for demo
  static unsigned long lastUpdate = 0;
  static int counter = 0;
  
  if (millis() - lastUpdate > 2000) {  // Update every 2 seconds
    lastUpdate = millis();
    counter++;
    
    // Simulate changing values
    temperature = 25.6 + sin(counter * 0.1) * 3.0;
    humidity = 62.3 + cos(counter * 0.15) * 10.0;
    soilMoisture = 45 + sin(counter * 0.05) * 25;
    
    // Ensure values stay in realistic ranges
    humidity = constrain(humidity, 0, 100);
    soilMoisture = constrain(soilMoisture, 0, 100);
    
    // Toggle mode every 10 seconds (5 cycles)
    if (counter % 5 == 0) {
      isAutoMode = !isAutoMode;
    }
    
    // Handle pump logic based on mode
    if (isAutoMode) {
      // In auto mode: pump based on soil moisture
      if (soilMoisture < 35 && !isPumpOn) {
        isPumpOn = true;
      } else if (soilMoisture > 65 && isPumpOn) {
        isPumpOn = false;
      }
    } else {
      // In manual mode: toggle pump every 4 seconds (2 cycles)
      if (counter % 2 == 0) {
        isPumpOn = !isPumpOn;
      }
    }
    
    updateDisplay();
  }
  
  // Add your sensor reading code here
  // temperature = readTemperature();
  // humidity = readHumidity();
  // soilMoisture = readSoilMoisture();
  
  delay(100);
}

// Function to toggle mode (call this when button is pressed)
void toggleMode() {
  isAutoMode = !isAutoMode;
  if (!isAutoMode) {
    // In manual mode, you might want to turn off pump by default
    // isPumpOn = false;
  }
  updateDisplay();
}

// Function to manually toggle pump (for manual mode)
void togglePump() {
  if (!isAutoMode) {
    isPumpOn = !isPumpOn;
    updateDisplay();
  }
}

// Functions to update sensor values (call these with your actual sensor readings)
void updateTemperature(float temp) {
  temperature = temp;
}

void updateHumidity(float hum) {
  humidity = hum;
}

void updateSoilMoisture(int moisture) {
  soilMoisture = constrain(moisture, 0, 100);
}