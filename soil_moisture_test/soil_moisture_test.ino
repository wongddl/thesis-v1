#include <Arduino.h>

// Pin definitions
#define SOIL_MOISTURE_PIN 34  // Soil moisture sensor pin
#define PUMP_RELAY_27 27      // Pump relay on GPIO27 (same as in main.ino)

// Soil moisture calibration values - using the same values from main.ino
const int AIR_VALUE = 3000;    // Sensor reading in dry air
const int WATER_VALUE = 1500;  // Sensor reading in water

// Soil moisture thresholds
const int LOW_MOISTURE_THRESHOLD = 20;   // Below this percentage, turn on pump
const int HIGH_MOISTURE_THRESHOLD = 70;  // Above this percentage, turn off pump

void setup() {
  Serial.begin(9600);
  
  // Set up output pin for relay
  pinMode(PUMP_RELAY_27, OUTPUT);
  
  // Initialize relay to OFF (HIGH for relay modules means OFF)
  digitalWrite(PUMP_RELAY_27, HIGH);
  
  Serial.println("ESP32 Soil Moisture Sensor Test");
  Serial.println("------------------------------");
  Serial.println("This test will read soil moisture values and");
  Serial.println("activate the pump relay when moisture is below threshold.");
  Serial.println();
}

void loop() {
  // Read soil moisture
  int rawValue = readSoilMoisture();
  int moisturePercentage = getSoilMoisturePercentage();
  
  // Print values to serial monitor
  Serial.print("Raw Moisture Value: ");
  Serial.print(rawValue);
  Serial.print("\tMoisture Percentage: ");
  Serial.print(moisturePercentage);
  Serial.println("%");
  
  // Control pump relay based on moisture level
  if (moisturePercentage < LOW_MOISTURE_THRESHOLD) {
    digitalWrite(PUMP_RELAY_27, LOW);  // Turn ON pump (LOW activates relay)
    Serial.println("Soil is DRY - Pump ON");
  } else if (moisturePercentage > HIGH_MOISTURE_THRESHOLD) {
    digitalWrite(PUMP_RELAY_27, HIGH); // Turn OFF pump (HIGH deactivates relay)
    Serial.println("Soil is WET - Pump OFF");
  }
  
  Serial.println("------------------------------");
  
  // Wait before next reading
  delay(1000);
}

// Read raw soil moisture value
int readSoilMoisture() {
  return analogRead(SOIL_MOISTURE_PIN);
}

// Convert raw value to percentage
int getSoilMoisturePercentage() {
  int rawValue = readSoilMoisture();
  
  // Ensure rawValue is within the expected range
  if (rawValue > AIR_VALUE) {
    rawValue = AIR_VALUE;
  } else if (rawValue < WATER_VALUE) {
    rawValue = WATER_VALUE;
  }
  
  // Map the raw value to a percentage (0-100%)
  // Note: map() function maps from AIR_VALUE (dry) to WATER_VALUE (wet)
  // Since higher raw values mean drier soil, we map from AIR to WATER
  return map(rawValue, AIR_VALUE, WATER_VALUE, 0, 100);
} 