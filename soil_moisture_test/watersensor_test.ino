/*
 * ESP32 Water Level Sensor - Ready to Use
 * Pre-calibrated with multi-point calibration data
 * 
 * Connections:
 * S (Signal) -> GPIO 34 (ADC1_CH6)
 * + (VCC)    -> 3.3V
 * - (GND)    -> GND
 */

// Pin definitions
#define WATER_SENSOR_PIN 34  // Analog pin for water level sensor
#define LED_PIN 2            // Built-in LED for visual indication

// Pre-calibrated sensor values (your calibration data)
const int calibrationPoints[6] = {0, 1497, 1735, 1852, 1902, 1946};  // ADC values
const int calibrationLevels[6] = {0, 20, 40, 60, 80, 100};           // Corresponding percentages
const int WATER_THRESHOLD = 194;  // Water detection threshold

// Variables
int sensorValue = 0;
int waterLevel = 0;
bool waterDetected = false;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("ESP32 Water Level Sensor - Ready to Use");
  Serial.println("========================================");
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(WATER_SENSOR_PIN, INPUT);
  
  // Turn off LED initially
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("Sensor Configuration:");
  Serial.println("- Sensor Pin: GPIO 34");
  Serial.println("- LED Pin: GPIO 2");
  Serial.println("- Pre-calibrated for accurate readings");
  Serial.println("");
  
  // Display calibration data
  Serial.println("=== CALIBRATION DATA ===");
  Serial.println("Multi-point calibration active:");
  for (int i = 0; i < 6; i++) {
    Serial.print("  ");
    Serial.print(calibrationLevels[i]);
    Serial.print("%: ADC = ");
    Serial.println(calibrationPoints[i]);
  }
  Serial.print("Water Detection Threshold: ");
  Serial.println(WATER_THRESHOLD);
  Serial.println("========================");
  Serial.println("");
  Serial.println("Starting water level monitoring...");
  Serial.println("----------------------------------");
}

void loop() {
  // Read analog value from water sensor
  sensorValue = analogRead(WATER_SENSOR_PIN);
  
  // Convert to percentage using multi-point interpolation
  waterLevel = getInterpolatedLevel(sensorValue);
  
  // Check if water is detected
  waterDetected = (sensorValue > WATER_THRESHOLD);
  
  // Control LED based on water detection
  digitalWrite(LED_PIN, waterDetected ? HIGH : LOW);
  
  // Print comprehensive readings to serial monitor
  Serial.print("Raw ADC: ");
  Serial.print(sensorValue);
  Serial.print(" | Water Level: ");
  Serial.print(waterLevel);
  Serial.print("% | Status: ");
  Serial.print(waterDetected ? "WATER DETECTED" : "NO WATER");
  Serial.print(" | LED: ");
  Serial.println(waterDetected ? "ON" : "OFF");
  
  // Detailed classification based on water level
  Serial.print("Classification: ");
  if (waterLevel <= 5) {
    Serial.println("DRY (0-5%)");
  } else if (waterLevel <= 25) {
    Serial.println("LOW (6-25%)");
  } else if (waterLevel <= 50) {
    Serial.println("MEDIUM-LOW (26-50%)");
  } else if (waterLevel <= 75) {
    Serial.println("MEDIUM-HIGH (51-75%)");
  } else if (waterLevel <= 95) {
    Serial.println("HIGH (76-95%)");
  } else {
    Serial.println("FULL (96-100%)");
  }
  
  Serial.println("----------------------------------");
  
  // Wait before next reading
  delay(1000);
}

// Function to get interpolated water level from multi-point calibration
int getInterpolatedLevel(int adcValue) {
  // If value is below or at dry level
  if (adcValue <= calibrationPoints[0]) {
    return 0;
  }
  
  // If value is above or at wet level
  if (adcValue >= calibrationPoints[5]) {
    return 100;
  }
  
  // Find which two calibration points the current reading falls between
  for (int i = 0; i < 5; i++) {
    if (adcValue >= calibrationPoints[i] && adcValue <= calibrationPoints[i + 1]) {
      // Linear interpolation between two points
      int x1 = calibrationPoints[i];
      int x2 = calibrationPoints[i + 1];
      int y1 = calibrationLevels[i];
      int y2 = calibrationLevels[i + 1];
      
      // y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
      int result = y1 + (adcValue - x1) * (y2 - y1) / (x2 - x1);
      return constrain(result, 0, 100);
    }
  }
  
  // Fallback (shouldn't reach here)
  return map(adcValue, calibrationPoints[0], calibrationPoints[5], 0, 100);
}