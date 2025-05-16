#include <Arduino.h>

// Pin definitions
#define SOIL_MOISTURE_PIN 34  // Soil moisture sensor pin
#define PUMP_LED 27           // Green LED to indicate pump would be working

// Soil moisture calibration values
const int AIR_VALUE = 3000;    // Replace with the sensor reading in dry air
const int WATER_VALUE = 1500;  // Replace with the sensor reading in water

// Soil moisture thresholds
const int LOW_MOISTURE_THRESHOLD = 20;   // Below this percentage, turn on pump LED
const int HIGH_MOISTURE_THRESHOLD = 70;  // Above this percentage, turn off pump LED

void setup() {
  Serial.begin(9600);
  
  // Set up output pin for LED
  pinMode(PUMP_LED, OUTPUT);
  
  // Initialize LED to OFF (HIGH for relay-based systems)
  digitalWrite(PUMP_LED, HIGH);
  
  Serial.println("Soil Moisture Sensor Test");
  Serial.println("------------------------");
  Serial.println("This test will read soil moisture values and");
  Serial.println("turn on the pump LED when moisture is below threshold.");
  Serial.println();
}

void loop() {
  // Read soil moisture
  int rawValue = readSoilMoisture();
  int moisturePercentage = getSoilMoisturePercentage(rawValue);
  
  // Print values to serial monitor
  Serial.print("Raw Moisture Value: ");
  Serial.print(rawValue);
  Serial.print("\tMoisture Percentage: ");
  Serial.print(moisturePercentage);
  Serial.println("%");
  
  // Control pump LED based on moisture level
  if (moisturePercentage < LOW_MOISTURE_THRESHOLD) {
    digitalWrite(PUMP_LED, LOW);  // Turn ON pump LED (LOW activates)
    Serial.println("Soil is DRY - Pump LED ON");
  } else if (moisturePercentage > HIGH_MOISTURE_THRESHOLD) {
    digitalWrite(PUMP_LED, HIGH); // Turn OFF pump LED (HIGH deactivates)
    Serial.println("Soil is WET - Pump LED OFF");
  }
  
  Serial.println("------------------------");
  
  // Wait before next reading
  delay(1000);
}

// Read raw soil moisture value
int readSoilMoisture() {
  return analogRead(SOIL_MOISTURE_PIN);
}

// Convert raw value to percentage
int getSoilMoisturePercentage(int rawValue) {
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