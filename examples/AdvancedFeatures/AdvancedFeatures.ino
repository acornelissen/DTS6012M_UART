#include <Arduino.h>
#include "DTS6012M_UART.h"

/*
 * Advanced DTS6012M UART Library Example
 * 
 * This example demonstrates the enhanced features of the DTS6012M library including:
 * - Custom configuration
 * - Error handling and recovery
 * - Data quality assessment
 * - Statistics tracking
 * - Calibration
 * - Performance optimization options
 * 
 * Hardware Connections:
 * - Arduino GND <-> Sensor Pin 6 (GND)
 * - Arduino 3.3V -> Sensor Pin 1 (3V3_LASER)  
 * - Arduino 3.3V -> Sensor Pin 2 (3V3)
 * - Arduino GND <-> Sensor Pin 5 (GPIO) ** IMPORTANT: Connect to GND for UART mode **
 * - Arduino RX1 (Pin 19 on Mega) <- Sensor Pin 3 (UART_TX)
 * - Arduino TX1 (Pin 18 on Mega) -> Sensor Pin 4 (UART_RX) ** Use logic level shifter for 5V boards **
 */

// Enhanced sensor configuration for high-performance applications
DTSConfig sensorConfig = {
  .baudRate = 921600,           // Standard baud rate
  .timeout_ms = 2000,           // 2 second timeout for robust operation
  .crcEnabled = true,           // Enable CRC for data integrity (can disable for speed)
  .maxValidDistance_mm = 18000, // Maximum expected measurement range
  .minValidDistance_mm = 50,    // Minimum valid measurement  
  .minIntensityThreshold = 150  // Minimum signal strength for good measurements
};

// Create sensor instance with custom configuration
HardwareSerial &SensorSerial = Serial1;
DTS6012M_UART dtsSensor(SensorSerial, sensorConfig);

// Timing and control variables
unsigned long lastPrintTime = 0;
unsigned long lastStatsTime = 0;
unsigned long lastCalibrationTime = 0;
const unsigned long PRINT_INTERVAL_MS = 100;
const unsigned long STATS_INTERVAL_MS = 5000;
const unsigned long CALIBRATION_INTERVAL_MS = 30000;

// Performance monitoring
unsigned long totalMeasurements = 0;
unsigned long startTime = 0;

// Error recovery tracking
int consecutiveErrors = 0;
const int MAX_CONSECUTIVE_ERRORS = 10;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("=== DTS6012M Advanced Features Example ===");
  Serial.println("Initializing sensor with enhanced configuration...");
  
  // Initialize sensor with error handling (v2.0 style)
  DTSResult initResult = dtsSensor.begin();
  if (initResult != DTSError::NONE) {
    Serial.print("ERROR: Sensor initialization failed with code: ");
    Serial.println(static_cast<int>(static_cast<DTSError>(initResult)));
    handleSensorError(static_cast<DTSError>(initResult));
  } else {
    Serial.println("Sensor initialized successfully!");
  }
  
  // Apply initial calibration (example: 5mm offset for mounting compensation)
  dtsSensor.setDistanceOffset(5);
  dtsSensor.setDistanceScale(1.02f); // 2% scaling correction
  Serial.println("Applied calibration: +5mm offset, 1.02x scale factor");
  
  // Optional: Disable CRC for maximum speed (use with caution)
  // dtsSensor.enableCRC(false);
  // Serial.println("CRC disabled for maximum performance");
  
  Serial.println("Starting measurements...");
  Serial.println("Format: Distance | Intensity | Quality | Error | Stats");
  Serial.println("-----------------------------------------------------");
  
  startTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Process sensor data with enhanced error handling
  DTSResult updateResult = dtsSensor.update();
  
  // Handle different update results
  switch (static_cast<DTSError>(updateResult)) {
    case DTSError::NONE:
      // New measurement received successfully
      totalMeasurements++;
      consecutiveErrors = 0;
      
      if (currentTime - lastPrintTime >= PRINT_INTERVAL_MS) {
        printMeasurementData();
        lastPrintTime = currentTime;
      }
      break;
      
    case DTSError::TIMEOUT:
      // Handle timeout - sensor may be disconnected or not responding
      consecutiveErrors++;
      if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
        handleSensorError(static_cast<DTSError>(updateResult));
      }
      break;
      
    case DTSError::CRC_CHECK_FAILED:
    case DTSError::FRAME_HEADER_INVALID:
    case DTSError::FRAME_LENGTH_INVALID:
      // Handle data corruption - may indicate electrical interference
      consecutiveErrors++;
      Serial.print("Data error detected: ");
      Serial.println(static_cast<int>(static_cast<DTSError>(updateResult)));
      
      if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
        handleSensorError(static_cast<DTSError>(updateResult));
      }
      break;
      
    default:
      // Other errors - continue processing
      break;
  }
  
  // Print statistics periodically
  if (currentTime - lastStatsTime >= STATS_INTERVAL_MS) {
    printStatistics();
    lastStatsTime = currentTime;
  }
  
  // Demonstrate dynamic calibration adjustment
  if (currentTime - lastCalibrationTime >= CALIBRATION_INTERVAL_MS) {
    demonstrateCalibrationAdjustment();
    lastCalibrationTime = currentTime;
  }
}

void printMeasurementData() {
  // Get complete measurement with quality assessment
  DTSMeasurement measurement = dtsSensor.getMeasurement();
  
  // Print primary target data
  Serial.print("Dist: ");
  if (measurement.primaryDistance_mm == DTS_INVALID_DISTANCE) {
    Serial.print("----");
  } else {
    Serial.print(measurement.primaryDistance_mm);
    Serial.print("mm");
  }
  
  Serial.print(" | Int: ");
  Serial.print(measurement.primaryIntensity);
  
  Serial.print(" | Quality: ");
  printDataQuality(measurement.primaryQuality);
  
  Serial.print(" | Error: ");
  if (measurement.lastError == DTSError::NONE) {
    Serial.print("OK");
  } else {
    Serial.print("E");
    Serial.print(static_cast<int>(measurement.lastError));
  }
  
  // Show data validation result
  Serial.print(" | Valid: ");
  Serial.print(dtsSensor.isDataValid() ? "Y" : "N");
  
  // Show secondary target if available
  if (measurement.secondaryDistance_mm != DTS_INVALID_DISTANCE) {
    Serial.print(" | Sec: ");
    Serial.print(measurement.secondaryDistance_mm);
    Serial.print("mm");
  }
  
  Serial.println();
}

void printDataQuality(DataQuality quality) {
  switch (quality) {
    case DataQuality::EXCELLENT: Serial.print("EXCL"); break;
    case DataQuality::GOOD:      Serial.print("GOOD"); break;
    case DataQuality::FAIR:      Serial.print("FAIR"); break;
    case DataQuality::POOR:      Serial.print("POOR"); break;
    case DataQuality::INVALID:   Serial.print("INVL"); break;
    default:                     Serial.print("UNKN"); break;
  }
}

void printStatistics() {
  DTSStatistics stats = dtsSensor.getStatistics();
  unsigned long elapsedTime = millis() - startTime;
  float measurementRate = (totalMeasurements * 1000.0f) / elapsedTime;
  
  Serial.println();
  Serial.println("=== STATISTICS ===");
  Serial.print("Total measurements: "); Serial.println(stats.measurementCount);
  Serial.print("Rate: "); Serial.print(measurementRate, 1); Serial.println(" Hz");
  Serial.print("Min distance: "); Serial.print(stats.minDistance); Serial.println(" mm");
  Serial.print("Max distance: "); Serial.print(stats.maxDistance); Serial.println(" mm");
  Serial.print("Avg distance: "); Serial.print(stats.avgDistance); Serial.println(" mm");
  Serial.print("Error count: "); Serial.println(stats.errorCount);
  Serial.print("Error rate: "); 
  if (stats.measurementCount > 0) {
    Serial.print((100.0f * stats.errorCount) / stats.measurementCount, 2);
    Serial.println("%");
  } else {
    Serial.println("N/A");
  }
  Serial.println("==================");
  Serial.println();
}

void handleSensorError(DTSError error) {
  Serial.println();
  Serial.println("!!! SENSOR ERROR DETECTED !!!");
  Serial.print("Error code: "); Serial.println(static_cast<int>(error));
  
  switch (error) {
    case DTSError::SERIAL_INIT_FAILED:
      Serial.println("Solution: Check serial port connection and baud rate");
      break;
      
    case DTSError::TIMEOUT:
      Serial.println("Solution: Check sensor power and UART connections");
      break;
      
    case DTSError::CRC_CHECK_FAILED:
      Serial.println("Solution: Check for electrical interference or try disabling CRC");
      break;
      
    case DTSError::FRAME_HEADER_INVALID:
    case DTSError::FRAME_LENGTH_INVALID:
      Serial.println("Solution: Check baud rate and electrical connections");
      break;
      
    default:
      Serial.println("Solution: Check sensor documentation and connections");
      break;
  }
  
  // Attempt recovery
  Serial.println("Attempting sensor recovery...");
  delay(1000);
  
  DTSResult recoveryResult = dtsSensor.begin();
  if (recoveryResult == DTSError::NONE) {
    Serial.println("Recovery successful!");
    consecutiveErrors = 0;
    dtsSensor.clearError();
  } else {
    Serial.println("Recovery failed - sensor may need manual reset");
    delay(5000); // Wait before next attempt
  }
  
  Serial.println();
}

void demonstrateCalibrationAdjustment() {
  static bool calibrationToggle = false;
  
  Serial.println("=== CALIBRATION DEMO ===");
  
  if (calibrationToggle) {
    // Apply temperature compensation example (hypothetical)
    dtsSensor.setDistanceScale(1.01f); // Slightly less scaling
    Serial.println("Applied temperature compensation: 1.01x scale");
  } else {
    // Reset to standard calibration
    dtsSensor.setDistanceScale(1.02f); // Back to original scaling
    Serial.println("Reset to standard calibration: 1.02x scale");
  }
  
  calibrationToggle = !calibrationToggle;
  Serial.println("========================");
  Serial.println();
}

// Performance test function (call from setup() if desired)
void runPerformanceTest() {
  Serial.println("=== PERFORMANCE TEST ===");
  
  // Test with CRC enabled
  dtsSensor.enableCRC(true);
  unsigned long crcStartTime = millis();
  int crcMeasurements = 0;
  
  while (millis() - crcStartTime < 5000) { // 5 second test
    if (dtsSensor.update() == DTSError::NONE) {
      crcMeasurements++;
    }
  }
  
  // Test with CRC disabled
  dtsSensor.enableCRC(false);
  unsigned long noCrcStartTime = millis();
  int noCrcMeasurements = 0;
  
  while (millis() - noCrcStartTime < 5000) { // 5 second test
    if (dtsSensor.update() == DTSError::NONE) {
      noCrcMeasurements++;
    }
  }
  
  // Re-enable CRC for normal operation
  dtsSensor.enableCRC(true);
  
  Serial.print("CRC enabled rate: "); 
  Serial.print(crcMeasurements / 5.0f, 1); 
  Serial.println(" Hz");
  
  Serial.print("CRC disabled rate: "); 
  Serial.print(noCrcMeasurements / 5.0f, 1); 
  Serial.println(" Hz");
  
  Serial.print("Performance gain: "); 
  Serial.print(((noCrcMeasurements - crcMeasurements) * 100.0f) / crcMeasurements, 1); 
  Serial.println("%");
  
  Serial.println("========================");
}