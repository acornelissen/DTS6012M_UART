#include <Arduino.h>
#include "DTS6012M_UART.h"

/*
 * DTS6012M Error Handling and Recovery Example
 * 
 * This example demonstrates robust error handling, automatic recovery,
 * and graceful degradation when sensor issues occur.
 * 
 * Features demonstrated:
 * - Comprehensive error detection and reporting
 * - Automatic recovery mechanisms
 * - Graceful degradation modes
 * - Error logging and analysis
 * - Sensor health monitoring
 */

HardwareSerial &SensorSerial = Serial1;

// Configure sensor for robust operation
DTSConfig robustConfig = {
  .baudRate = 921600,
  .timeout_ms = 1500,           // Shorter timeout for faster error detection
  .crcEnabled = true,           // Keep CRC enabled for reliability
  .maxValidDistance_mm = 20000,
  .minValidDistance_mm = 30,
  .minIntensityThreshold = 80   // Lower threshold for marginal conditions
};

DTS6012M_UART dtsSensor(SensorSerial, robustConfig);

// Error tracking and recovery state
struct ErrorState {
  DTSError lastError;
  unsigned long errorTime;
  int errorCount;
  int recoveryAttempts;
  bool inRecoveryMode;
  unsigned long recoveryStartTime;
};

ErrorState errorState = {DTSError::NONE, 0, 0, 0, false, 0};

// Operational modes
enum class OperationMode {
  NORMAL,           // Normal operation with full error checking
  DEGRADED,         // Reduced functionality but still operational
  RECOVERY,         // Attempting to recover from errors
  MAINTENANCE       // Manual intervention required
};

OperationMode currentMode = OperationMode::NORMAL;

// Timing variables
unsigned long lastMeasurementTime = 0;
unsigned long lastStatusTime = 0;
unsigned long modeChangeTime = 0;
const unsigned long STATUS_INTERVAL = 3000;

// Performance tracking
struct PerformanceMetrics {
  unsigned long totalMeasurements;
  unsigned long validMeasurements;
  unsigned long errorCount;
  unsigned long recoveryCount;
  float uptime;
};

PerformanceMetrics metrics = {0, 0, 0, 0, 0};
unsigned long startTime;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("=== DTS6012M Error Handling Demo ===");
  Serial.println("This example shows robust error handling and recovery");
  Serial.println();
  
  startTime = millis();
  
  // Initialize sensor with comprehensive error checking
  if (!initializeSensorRobust()) {
    Serial.println("CRITICAL: Initial sensor setup failed!");
    currentMode = OperationMode::MAINTENANCE;
  } else {
    Serial.println("Sensor initialized successfully");
    currentMode = OperationMode::NORMAL;
  }
  
  printOperationModeInfo();
  Serial.println("Monitor will show: [Mode] Distance | Quality | Errors | Recovery");
  Serial.println("================================================================");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update performance metrics
  metrics.uptime = (currentTime - startTime) / 1000.0f;
  
  // Handle different operational modes
  switch (currentMode) {
    case OperationMode::NORMAL:
      handleNormalOperation();
      break;
      
    case OperationMode::DEGRADED:
      handleDegradedOperation();
      break;
      
    case OperationMode::RECOVERY:
      handleRecoveryOperation();
      break;
      
    case OperationMode::MAINTENANCE:
      handleMaintenanceMode();
      break;
  }
  
  // Print status periodically
  if (currentTime - lastStatusTime >= STATUS_INTERVAL) {
    printSystemStatus();
    lastStatusTime = currentTime;
  }
  
  // Check for mode transitions
  checkModeTransitions();
  
  delay(10); // Small delay to prevent overwhelming the system
}

bool initializeSensorRobust() {
  Serial.print("Initializing sensor");
  
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print(".");
    
    DTSError result = dtsSensor.begin();
    if (result == DTSError::NONE) {
      Serial.println(" SUCCESS");
      dtsSensor.clearError();
      return true;
    }
    
    Serial.print(" [Attempt ");
    Serial.print(attempt);
    Serial.print(" failed: ");
    Serial.print(static_cast<int>(result));
    Serial.print("]");
    
    if (attempt < 3) {
      delay(1000);
    }
  }
  
  Serial.println(" FAILED");
  return false;
}

void handleNormalOperation() {
  DTSError result = dtsSensor.update();
  metrics.totalMeasurements++;
  
  switch (result) {
    case DTSError::NONE:
      // Successful measurement
      metrics.validMeasurements++;
      printMeasurement();
      errorState.lastError = DTSError::NONE;
      errorState.errorCount = 0; // Reset error count on success
      break;
      
    case DTSError::TIMEOUT:
      handleError(result, "Sensor communication timeout");
      break;
      
    case DTSError::CRC_CHECK_FAILED:
      handleError(result, "Data corruption detected (CRC failure)");
      break;
      
    case DTSError::FRAME_HEADER_INVALID:
      handleError(result, "Invalid frame header");
      break;
      
    case DTSError::FRAME_LENGTH_INVALID:
      handleError(result, "Invalid frame length");
      break;
      
    case DTSError::BUFFER_OVERFLOW:
      handleError(result, "Buffer overflow - data rate too high");
      break;
      
    default:
      handleError(result, "Unknown error");
      break;
  }
}

void handleDegradedOperation() {
  Serial.print("[DEGRADED] ");
  
  // In degraded mode, we're more tolerant of errors
  DTSError result = dtsSensor.update();
  metrics.totalMeasurements++;
  
  if (result == DTSError::NONE) {
    metrics.validMeasurements++;
    
    // Check if we can return to normal operation
    if (dtsSensor.isDataValid() && dtsSensor.getDataQuality() >= DataQuality::FAIR) {
      errorState.errorCount = 0;
      Serial.println("Data quality improved - returning to NORMAL mode");
      changeOperationMode(OperationMode::NORMAL);
    } else {
      printMeasurement();
    }
  } else {
    // Still having errors in degraded mode
    errorState.errorCount++;
    if (errorState.errorCount > 20) {
      Serial.println("Too many errors in degraded mode - entering RECOVERY");
      changeOperationMode(OperationMode::RECOVERY);
    }
  }
}

void handleRecoveryOperation() {
  Serial.print("[RECOVERY] ");
  
  if (!errorState.inRecoveryMode) {
    Serial.println("Starting recovery sequence...");
    errorState.inRecoveryMode = true;
    errorState.recoveryStartTime = millis();
    errorState.recoveryAttempts = 0;
  }
  
  // Try different recovery strategies
  if (millis() - errorState.recoveryStartTime > 5000) { // Every 5 seconds
    errorState.recoveryAttempts++;
    metrics.recoveryCount++;
    
    Serial.print("Recovery attempt #");
    Serial.print(errorState.recoveryAttempts);
    Serial.print(": ");
    
    bool recoverySuccess = false;
    
    switch (errorState.recoveryAttempts) {
      case 1:
        Serial.print("Clearing error state... ");
        dtsSensor.clearError();
        dtsSensor.resetStatistics();
        recoverySuccess = testSensorCommunication();
        break;
        
      case 2:
        Serial.print("Reinitializing sensor... ");
        recoverySuccess = (dtsSensor.begin() == DTSError::NONE);
        break;
        
      case 3:
        Serial.print("Disabling CRC for stability... ");
        dtsSensor.enableCRC(false);
        recoverySuccess = testSensorCommunication();
        break;
        
      case 4:
        Serial.print("Factory reset attempt... ");
        dtsSensor.factoryReset();
        dtsSensor.enableCRC(true); // Re-enable CRC\n        recoverySuccess = (dtsSensor.begin() == DTSError::NONE);
        break;
        
      default:
        Serial.print("Maximum recovery attempts reached ");
        changeOperationMode(OperationMode::MAINTENANCE);
        return;
    }\n    
    if (recoverySuccess) {
      Serial.println("SUCCESS");
      errorState.inRecoveryMode = false;
      errorState.errorCount = 0;
      changeOperationMode(OperationMode::NORMAL);
    } else {
      Serial.println("FAILED");
      errorState.recoveryStartTime = millis(); // Reset timer for next attempt
    }
  }
}

void handleMaintenanceMode() {
  static unsigned long lastMaintenanceMessage = 0;
  
  if (millis() - lastMaintenanceMessage > 10000) { // Every 10 seconds
    Serial.println("[MAINTENANCE] Manual intervention required");
    Serial.println("  - Check sensor power and connections");
    Serial.println("  - Verify wiring and GPIO pin to GND");
    Serial.println("  - Check for electromagnetic interference");
    Serial.println("  - Consider replacing sensor if issues persist");
    Serial.println("  - Reset Arduino to retry automatic recovery");
    
    lastMaintenanceMessage = millis();
  }
}

void handleError(DTSError error, const char* description) {
  errorState.lastError = error;
  errorState.errorTime = millis();
  errorState.errorCount++;
  metrics.errorCount++;
  
  Serial.print("[ERROR] ");
  Serial.print(description);
  Serial.print(" (Code: ");
  Serial.print(static_cast<int>(error));
  Serial.println(")");
  
  // Determine if we should change operation mode
  if (errorState.errorCount > 5) {
    if (currentMode == OperationMode::NORMAL) {
      Serial.println("Multiple errors detected - switching to DEGRADED mode");
      changeOperationMode(OperationMode::DEGRADED);
    } else if (currentMode == OperationMode::DEGRADED && errorState.errorCount > 15) {
      Serial.println("Persistent errors in degraded mode - entering RECOVERY");
      changeOperationMode(OperationMode::RECOVERY);
    }
  }
}

bool testSensorCommunication() {
  // Try to get a few valid measurements
  int validTests = 0;
  for (int i = 0; i < 10; i++) {
    if (dtsSensor.update() == DTSError::NONE) {
      validTests++;
    }
    delay(50);
  }
  return validTests >= 3; // Need at least 3 successful measurements
}

void printMeasurement() {
  DTSMeasurement measurement = dtsSensor.getMeasurement();
  
  if (measurement.primaryDistance_mm != DTS_INVALID_DISTANCE) {
    Serial.print(measurement.primaryDistance_mm);
    Serial.print("mm | ");
    
    switch (measurement.primaryQuality) {
      case DataQuality::EXCELLENT: Serial.print("EXCL"); break;
      case DataQuality::GOOD:      Serial.print("GOOD"); break;
      case DataQuality::FAIR:      Serial.print("FAIR"); break;
      case DataQuality::POOR:      Serial.print("POOR"); break;
      default:                     Serial.print("INVL"); break;
    }
  } else {
    Serial.print("----mm | INVL");
  }
  
  Serial.print(" | Err:");
  Serial.print(errorState.errorCount);
  Serial.print(" | Rec:");
  Serial.println(metrics.recoveryCount);
}

void printSystemStatus() {
  Serial.println();
  Serial.println("=== SYSTEM STATUS ===");
  Serial.print("Mode: ");
  printCurrentMode();
  Serial.print("Uptime: ");
  Serial.print(metrics.uptime, 1);
  Serial.println(" seconds");
  
  Serial.print("Measurements: ");
  Serial.print(metrics.validMeasurements);
  Serial.print("/");
  Serial.print(metrics.totalMeasurements);
  
  if (metrics.totalMeasurements > 0) {
    float successRate = (100.0f * metrics.validMeasurements) / metrics.totalMeasurements;
    Serial.print(" (");
    Serial.print(successRate, 1);
    Serial.println("% success)");
  } else {
    Serial.println();
  }
  
  Serial.print("Error count: ");
  Serial.println(metrics.errorCount);
  Serial.print("Recovery count: ");
  Serial.println(metrics.recoveryCount);
  
  if (errorState.lastError != DTSError::NONE) {
    Serial.print("Last error: ");
    Serial.print(static_cast<int>(errorState.lastError));
    Serial.print(" (");
    Serial.print((millis() - errorState.errorTime) / 1000);
    Serial.println(" seconds ago)");
  }
  
  DTSStatistics stats = dtsSensor.getStatistics();
  if (stats.measurementCount > 0) {
    Serial.print("Sensor stats - Min:");
    Serial.print(stats.minDistance);
    Serial.print("mm Max:");
    Serial.print(stats.maxDistance);
    Serial.print("mm Avg:");
    Serial.print(stats.avgDistance);
    Serial.println("mm");
  }
  
  Serial.println("=====================");
  Serial.println();
}

void changeOperationMode(OperationMode newMode) {
  if (currentMode != newMode) {
    currentMode = newMode;
    modeChangeTime = millis();
    
    Serial.print("Mode changed to: ");
    printCurrentMode();
    Serial.println();
  }
}

void printCurrentMode() {
  switch (currentMode) {
    case OperationMode::NORMAL:     Serial.print("NORMAL"); break;
    case OperationMode::DEGRADED:   Serial.print("DEGRADED"); break;
    case OperationMode::RECOVERY:   Serial.print("RECOVERY"); break;
    case OperationMode::MAINTENANCE: Serial.print("MAINTENANCE"); break;
  }
}

void checkModeTransitions() {
  // Auto-recovery from degraded mode if sensor is working well
  if (currentMode == OperationMode::DEGRADED) {
    if (errorState.errorCount == 0 && dtsSensor.isDataValid()) {
      // If we've had several good measurements, try returning to normal
      static int goodMeasurements = 0;
      goodMeasurements++;
      
      if (goodMeasurements > 10) {
        Serial.println("Sensor stable in degraded mode - returning to NORMAL");
        changeOperationMode(OperationMode::NORMAL);
        goodMeasurements = 0;
      }
    }
  }
}

void printOperationModeInfo() {
  Serial.println();
  Serial.println("Operation Modes:");
  Serial.println("  NORMAL     - Full functionality with strict error checking");
  Serial.println("  DEGRADED   - Relaxed error tolerance, reduced functionality");
  Serial.println("  RECOVERY   - Attempting automatic error recovery");
  Serial.println("  MAINTENANCE- Manual intervention required");
  Serial.println();
}