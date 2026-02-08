# DTS6012M_UART Arduino Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Description

This library provides an interface for the DTS6012M single-point dToF (direct Time-of-Flight) distance sensor module using UART communication.

The DTS6012M is a compact sensor capable of measuring distances up to 20 meters with features like dual-target detection and good ambient light resistance. This library handles the UART protocol (frame parsing, CRC checking) required to communicate with the sensor and retrieve measurement data like distance and signal intensity.

This library is based on the DTS6012M User Manual V1.6 (dated 2024-07-26).

## Features

* Initializes UART communication with the sensor.
* Starts and stops the sensor's continuous measurement stream with `enableSensor()` and `disableSensor()`.
* Parses incoming data frames according to the datasheet protocol.
* Performs Modbus CRC-16 checksum validation for data integrity (can be optionally disabled for performance).
* Supports CRC byte-order modes: `LSB_THEN_MSB` (default), `MSB_THEN_LSB`, and `AUTO` failover.
* In `AUTO` mode, can switch CRC byte order automatically after repeated CRC failures.
* Provides easy-to-use functions to retrieve:
    * Primary Target Distance (mm)
    * Primary Target Intensity
    * Secondary Target Distance (mm)
    * Secondary Target Intensity
    * Sunlight Base Level
    * Correction Values (Primary & Secondary)
* Allows disabling the CRC check via `enableCRC(false)` for potentially faster updates, at the risk of accepting corrupted data.
* Sensor enable/disable control for power management and measurement control.
* Preserves source compatibility with `main`/v1 command symbols (`DTS_CMD_*`) and `sendCommand(byte, ...)`.
* Includes example sketch demonstrating usage with enable/disable functionality.

## Hardware Requirements

* **DTS6012M Sensor Module:** The sensor this library is designed for.
* **Arduino Board:** An Arduino board with at least one available **HardwareSerial** port (e.g., `Serial1`, `Serial2`). Examples include Arduino Mega, Arduino Due, ESP32, STM32-based boards, etc.
    * **Note:** The default sensor baud rate (921600 bps) is generally **too high** for SoftwareSerial libraries. Using a HardwareSerial port is strongly recommended.
* **3.3V Power Supply:** The sensor requires a 3.3V supply for both Pin 1 (3V3_LASER) and Pin 2 (3V3). Ensure your Arduino can supply sufficient current or use an external 3.3V regulator.
* **Jumper Wires:** For making connections.
* **(Optional)** Logic Level Shifter (if connecting 5V Arduino TX to 3.3V sensor RX).

## Software Requirements

* **Arduino IDE:** Version 1.8.10 or later recommended.
* **This Library:** `DTS6012M_UART`

## Installation

1.  **Library Manager:**
    * Open the Arduino IDE.
    * Go to `Sketch` -> `Include Library` -> `Manage Libraries...`
    * Search for `DTS6012M_UART`.
    * Click `Install`.
2.  **Manual Installation:**
    * Download the latest release ZIP file from the repository.
    * In the Arduino IDE, go to `Sketch` -> `Include Library` -> `Add .ZIP Library...`
    * Select the downloaded ZIP file.
    * Alternatively, unzip the file and copy the `DTS6012M_UART` folder into your Arduino `libraries` directory (usually found in your Sketchbook location).
    * Restart the Arduino IDE.

## Wiring (UART Mode)

**Crucially, connect the sensor's GPIO pin (Pin 5) to GND *before* powering on the sensor to select UART mode.**

| DTS6012M Pin | Function        | Arduino Connection                                     | Notes                                                                  |
| :----------- | :-------------- | :----------------------------------------------------- | :--------------------------------------------------------------------- |
| Pin 1        | 3V3_LASER       | **Arduino 3.3V** | Sensor Laser Power                                                     |
| Pin 2        | 3V3             | **Arduino 3.3V** | Sensor Logic Power                                                     |
| Pin 3        | UART_TX/I2C_SDA | **Arduino Hardware RX Pin** (e.g., RX1 Pin 19 on Mega) | Sensor transmits data *to* Arduino                                     |
| Pin 4        | UART_RX/I2C_SCL | **Arduino Hardware TX Pin** (e.g., TX1 Pin 18 on Mega) | Sensor receives data *from* Arduino (Logic Level Shift Needed for 5V Arduinos) |
| Pin 5        | GPIO            | **Arduino GND** | **MUST be connected to GND before power-on for UART mode** |
| Pin 6        | GND             | **Arduino GND** | Common Ground                                                          |

**Logic Level Shifting:** If using a 5V Arduino board (like Uno, Nano), you **must** use a logic level shifter between the Arduino's TX pin (5V) and the sensor's RX pin (Pin 4, 3.3V tolerant) to avoid damaging the sensor. The sensor's TX output (3.3V) might be readable by a 5V Arduino RX pin, but check your Arduino board's specifications. Using a level shifter on both TX/RX lines is the safest approach.

## Basic Usage

```cpp
#include <Arduino.h>
#include "DTS6012M_UART.h" // 1. Include the library

// 2. Select the HardwareSerial port connected to the sensor
HardwareSerial &SensorSerial = Serial1; // Use Serial1, Serial2, etc.

// 3. Create an instance of the sensor library
DTS6012M_UART dtsSensor(SensorSerial);

void setup() {
  Serial.begin(115200); // For printing results
  while (!Serial);

  // 4. Initialize the sensor library (starts Serial1 at 921600 default)
  if (!dtsSensor.begin()) {
    Serial.println("Failed to initialize sensor!");
    while (1); // Halt
  }
  Serial.println("Sensor initialized.");

  // --- Optional: Disable CRC Check ---
  // For maximum performance, you can disable the CRC check.
  // This reduces processing overhead but increases the risk of using corrupted data if transmission errors occur.
  // dtsSensor.enableCRC(false); // CRC is ENABLED by default. Uncomment this line to disable it.

  // --- Optional: CRC Byte-Order Mode ---
  // Default is LSB_THEN_MSB.
  // dtsSensor.setCRCByteOrder(DTSCRCByteOrder::MSB_THEN_LSB);
  // dtsSensor.setCRCByteOrder(DTSCRCByteOrder::AUTO); // starts with LSB_THEN_MSB
  // dtsSensor.setCRCAutoSwitchErrorThreshold(200);    // switch after 200 consecutive CRC failures
  
  // --- Optional: Control sensor enable/disable ---
  // dtsSensor.disableSensor(); // Stop measurements
  // dtsSensor.enableSensor();  // Resume measurements
}

void loop() {
  // 5. Call update() frequently to process incoming data
  bool newData = dtsSensor.update();

  // 6. Check if new data was received
  if (newData) {
    // 7. Get the data
    uint16_t distance = dtsSensor.getDistance(); // in mm
    uint16_t intensity = dtsSensor.getIntensity();

    Serial.print("Distance: ");
    if (distance == 0xFFFF) {
      Serial.print("Invalid/OOR");
    } else {
      Serial.print(distance);
      Serial.print(" mm");
    }
    Serial.print("\t Intensity: ");
    Serial.println(intensity);
  }

  // Do other things...
  delay(10); // Keep loop running reasonably fast
}
```

## API Reference

### Enhanced Configuration

```cpp
// Sensor configuration structure
struct DTSConfig {
  unsigned long baudRate = 921600;        // UART baud rate
  unsigned long timeout_ms = 1000;        // Communication timeout
  bool crcEnabled = true;                  // Enable CRC validation
  uint16_t maxValidDistance_mm = 20000;    // Maximum valid distance
  uint16_t minValidDistance_mm = 30;       // Minimum valid distance  
  uint16_t minIntensityThreshold = 100;    // Minimum signal strength
  DTSCRCByteOrder crcByteOrder = DTSCRCByteOrder::LSB_THEN_MSB; // CRC byte order mode
  uint16_t crcAutoSwitchErrorThreshold = 200; // AUTO mode threshold
};

// Constructor with configuration
DTS6012M_UART sensor(Serial1, config);
```

### Error Handling

```cpp
// Enhanced error codes
enum class DTSError : byte {
  NONE = 0x00,                    // No error
  SERIAL_INIT_FAILED = 0x01,      // Serial port initialization failed
  FRAME_HEADER_INVALID = 0x02,    // Invalid frame header
  FRAME_LENGTH_INVALID = 0x03,    // Invalid frame length
  CRC_CHECK_FAILED = 0x04,        // CRC validation failed
  BUFFER_OVERFLOW = 0x05,         // Buffer overflow detected
  TIMEOUT = 0x06,                 // Communication timeout
  INVALID_COMMAND = 0x07,         // Invalid command parameter
  UNSUPPORTED_OPERATION = 0x08    // Feature not implemented by sensor firmware
};

// Error handling example
DTSError result = sensor.update();
if (result != DTSError::NONE) {
  Serial.print("Error: ");
  Serial.println(static_cast<int>(result));
  // Handle error appropriately
}
```

### CRC Byte-Order Modes

```cpp
enum class DTSCRCByteOrder : byte {
  LSB_THEN_MSB = 0,  // Default behavior
  MSB_THEN_LSB = 1,  // Alternate sensor variants
  AUTO = 2           // Auto-switch after repeated CRC failures
};

// Fixed CRC order
sensor.setCRCByteOrder(DTSCRCByteOrder::LSB_THEN_MSB);
sensor.setCRCByteOrder(DTSCRCByteOrder::MSB_THEN_LSB);

// Auto mode: starts with LSB_THEN_MSB and flips after threshold failures
sensor.setCRCByteOrder(DTSCRCByteOrder::AUTO);
sensor.setCRCAutoSwitchErrorThreshold(200);

// Inspect configured and currently active mode
DTSCRCByteOrder configured = sensor.getCRCByteOrder();
DTSCRCByteOrder active = sensor.getActiveCRCByteOrder();
```

### Data Quality Assessment

```cpp
// Data quality levels
enum class DataQuality : byte {
  EXCELLENT = 0,    // High signal strength, optimal conditions
  GOOD = 1,         // Good signal strength, reliable measurement
  FAIR = 2,         // Moderate signal strength, acceptable
  POOR = 3,         // Low signal strength, use with caution
  INVALID = 4       // Invalid measurement, do not use
};

// Quality assessment example
if (sensor.isDataValid()) {
  DataQuality quality = sensor.getDataQuality();
  switch (quality) {
    case DataQuality::EXCELLENT:
    case DataQuality::GOOD:
      // Use measurement with confidence
      break;
    case DataQuality::FAIR:
      // Use with additional validation
      break;
    case DataQuality::POOR:
    case DataQuality::INVALID:
      // Discard measurement
      break;
  }
}
```

### Measurement Data Structure

```cpp
// Complete measurement data
struct DTSMeasurement {
  uint16_t primaryDistance_mm;      // Primary target distance
  uint16_t primaryIntensity;        // Primary target signal strength
  uint16_t primaryCorrection;       // Primary target correction value
  uint16_t secondaryDistance_mm;    // Secondary target distance
  uint16_t secondaryIntensity;      // Secondary target signal strength
  uint16_t secondaryCorrection;     // Secondary target correction value
  uint16_t sunlightBase;            // Ambient light level
  unsigned long timestamp;          // Measurement timestamp
  DataQuality primaryQuality;       // Primary target quality
  DataQuality secondaryQuality;     // Secondary target quality
  DTSError lastError;               // Last error code
};

// Get complete measurement
DTSMeasurement measurement = sensor.getMeasurement();
```

### Statistics and Analytics

```cpp
// Measurement statistics
struct DTSStatistics {
  uint16_t minDistance;        // Minimum measured distance
  uint16_t maxDistance;        // Maximum measured distance
  uint32_t avgDistance;        // Average distance
  uint16_t measurementCount;   // Total measurements
  uint16_t errorCount;         // Total errors
};

// Get statistics
DTSStatistics stats = sensor.getStatistics();
Serial.print("Success rate: ");
Serial.print(100.0 * (stats.measurementCount - stats.errorCount) / stats.measurementCount);
Serial.println("%");

// Reset statistics
sensor.resetStatistics();
```

### Calibration System

```cpp
// Distance calibration
sensor.setDistanceOffset(10);      // Add 10mm offset
sensor.setDistanceScale(1.05f);    // Apply 5% scaling

// Calibration is applied automatically to all measurements
uint16_t calibratedDistance = sensor.getDistance();
```

### Resetting Library State

```cpp
// Clear calibration tweaks, rolling statistics, and error history
sensor.resetState();

// Compatibility helper: still performs a soft reset but returns UNSUPPORTED_OPERATION
DTSError factoryResult = sensor.factoryReset();
if (factoryResult == DTSError::UNSUPPORTED_OPERATION) {
  Serial.println("Sensor firmware exposes no factory-reset command; local state was cleared instead.");
}
```

### Advanced Control

```cpp
// Sensor control with error handling
DTSError result = sensor.enableSensor();
if (result != DTSError::NONE) {
  // Handle enable failure
}

result = sensor.disableSensor();
if (result != DTSError::NONE) {
  // Handle disable failure
}

// Performance optimization
sensor.enableCRC(false);  // Disable CRC for speed
// Use with caution - reduces reliability

// CRC byte order (default: LSB_THEN_MSB)
sensor.setCRCByteOrder(DTSCRCByteOrder::MSB_THEN_LSB);

// AUTO failover for mixed sensor populations
sensor.setCRCByteOrder(DTSCRCByteOrder::AUTO);
sensor.setCRCAutoSwitchErrorThreshold(200); // Switch after repeated CRC failures
```

### Legacy API Compatibility (main/v1)

```cpp
// Legacy command constants are still available
sensor.sendCommand(DTS_CMD_START_STREAM, NULL, 0);
sensor.sendCommand(DTS_CMD_STOP_STREAM, NULL, 0);

// Legacy bool-style usage still works
if (sensor.begin()) {
  if (sensor.update()) {
    uint16_t distance = sensor.getDistance();
    (void)distance;
  }
}
```

## Examples

The library includes comprehensive examples:

- **BasicRead**: Simple distance measurement
- **DTS6012M_UART_Example**: Enhanced basic usage with all features
- **AdvancedFeatures**: Comprehensive feature demonstration
- **ErrorHandlingDemo**: Robust error handling and recovery

## Testing

Run the comprehensive test suite:

```bash
# Load tests/test_DTS6012M_UART.cpp as Arduino sketch
```

Test coverage includes:
- ✅ Frame parsing and validation
- ✅ CRC byte-order modes and AUTO failover
- ✅ Legacy API compatibility (`DTS_CMD_*`, `sendCommand(byte, ...)`)
- ✅ Error handling and recovery  
- ✅ Data quality assessment
- ✅ Statistics calculations
- ✅ Calibration accuracy

## Performance

### Measurement Rates
- **Standard mode**: 50-100 Hz (CRC enabled)
- **Fast mode**: 100-200 Hz (CRC disabled)

### Memory Usage
- **RAM**: ~2KB (including buffers and statistics)
- **Flash**: ~8KB (including lookup tables)

## Migration from v1.x

The v2.0 library maintains backward compatibility:

```cpp
// v1.x code (still works)
DTS6012M_UART sensor(Serial1);
if (sensor.begin()) {  // Now returns DTSError, but converts to bool
  // ...
}

// v2.0 enhanced code (recommended)
if (sensor.begin() == DTSError::NONE) {
  DTSMeasurement measurement = sensor.getMeasurement();
  if (sensor.isDataValid()) {
    // Use measurement with confidence
  }
}
```

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Changelog

### v2.0.0 (Latest)
- ✨ **Major enhancement release**
- 🛡️ Added comprehensive error handling and recovery
- 📊 Added data quality assessment and validation
- 🔧 Added calibration system with offset and scaling
- 📈 Added statistics tracking and analytics
- ⚡ Added performance optimizations
- 🧪 Added comprehensive test framework
- 📚 Maintained backward compatibility with v1.x
