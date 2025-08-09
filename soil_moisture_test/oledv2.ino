#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===== CONFIGURATION VARIABLES =====
// Sensor Values (update these with actual readings)
float temperature = 50.0;    // Temperature in Celsius
int humidity = 60;           // Humidity percentage
int moisture = 70;           // Soil moisture percentage
int waterLevel = 80;         // Water level percentage

// System Status
bool automaticMode = true;   // true = Automatic, false = Manual
bool pumpOn = true;          // true = pump on (filled circle), false = pump off (empty circle)

// Error message
String errorMsg = "E: ht w l o m b";

void setup() {
  Serial.begin(115200);
  
  // Initialize display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  updateDisplay();
}

void loop() {
  // Update sensor values here
  // updateSensorValues();
  
  // Update display with new values
  // updateDisplay();
  
  delay(1000); // Update every second
}

// ===== DISPLAY FUNCTIONS =====
void updateDisplay() {
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
  if (automaticMode) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Inverted when on
    display.fillRect(9, 1, 14, 15, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(11, 2);
  display.print(F("A"));
  
  // Manual mode indicator (bottom half)
  if (!automaticMode) {
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
  display.printf("%.0f", temperature);
  
  display.setTextSize(1);
  display.setCursor(70, 9);
  display.print(F("c"));
  
  // Humidity section
  display.setTextSize(1);
  display.setCursor(82, 0);
  display.print(F("HUM"));
  
  display.setTextSize(2);
  display.setCursor(90, 9);
  display.print(humidity);
  
  display.setTextSize(1);
  display.setCursor(114, 9);
  display.print(F("%"));
  
  // Soil moisture section
  display.setTextSize(1);
  display.setCursor(35, 27);
  display.print(F("MOIST"));
  
  display.setTextSize(2);
  display.setCursor(45, 39);
  display.print(moisture);
  
  display.setTextSize(1);
  display.setCursor(69, 39);
  display.print(F("%"));
  
  // Water level section
  display.setTextSize(1);
  display.setCursor(81, 27);
  display.print(F("WLVL"));
  
  display.setTextSize(2);
  display.setCursor(89, 39);
  display.print(waterLevel);
  
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
  
    if (pumpOn) {
      display.fillCircle(circleX, circleY, radius, SSD1306_WHITE); // Filled circle = ON
    } else {
      display.drawCircle(circleX, circleY, radius, SSD1306_WHITE); // Empty circle = OFF
    }
  }

  void drawErrorMessage() {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(35, 56);
    display.print(errorMsg);
  }

// ===== UTILITY FUNCTIONS =====
void updateSensorValues() {
  // Example function to update sensor values
  // Replace with actual sensor reading code
  
  // temperature = readTemperatureSensor();
  // humidity = readHumiditySensor();
  // moisture = readMoistureSensor();
  // waterLevel = readWaterLevelSensor();
}

void setMode(bool automatic) {
  automaticMode = automatic;
  updateDisplay();
}

void setPumpStatus(bool status) {
  pumpOn = status;
  updateDisplay();
}

void updateTemperature(float temp) {
  temperature = temp;
}

void updateHumidity(int hum) {
  humidity = hum;
}

void updateMoisture(int moist) {
  moisture = moist;
}

void updateWaterLevel(int level) {
  waterLevel = level;
}

void setErrorMessage(String msg) {
  errorMsg = msg;
}