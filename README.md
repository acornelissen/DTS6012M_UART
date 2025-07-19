# DTS6012M_UART Arduino Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Description

This library provides an interface for the DTS6012M single-point dToF (direct Time-of-Flight) distance sensor module using UART communication.

The DTS6012M is a compact sensor capable of measuring distances up to 20 meters with features like dual-target detection and good ambient light resistance. This library handles the UART protocol (frame parsing, CRC checking) required to communicate with the sensor and retrieve measurement data like distance and signal intensity.

This library is based on the DTS6012M User Manual V1.6 (dated 2024-07-26).

## Version 2.0.0 - Breaking Changes

Version 2.0.0 introduces significant improvements to the library's structure, robustness, and adherence to modern C++ practices. These changes are **not backwards compatible** and require updates to existing code.

### Migration Guide (from v1.x to v2.x)

1.  **Namespace and Class Name:**
    *   The main class `DTS6012M_UART` has been renamed to `UART` and is now inside the `DTS6012M` namespace.
    *   **Update your object declaration:**
        ```cpp
        // Before
        DTS6012M_UART dtsSensor(Serial1);

        // After
        #include "DTS6012M_UART.h"
        DTS6012M::UART dtsSensor(Serial1);
        ```

2.  **Data Reading Logic:**
    *   A new method `isDataNew()` is now the recommended way to check for new measurements. This flag is cleared automatically when you call `getDistance()`.
    *   **Update your data reading loop:**
        ```cpp
        // Before
        if (dtsSensor.update()) {
          uint16_t distance = dtsSensor.getDistance();
          // ...
        }

        // After
        dtsSensor.update(); // Call update() to process serial data
        if (dtsSensor.isDataNew()) { // Check for new data
          uint16_t distance = dtsSensor.getDistance(); // Read data (this clears the flag)
          // ...
        }
        ```

3.  **Debug Output:
    *   A new method `setDebugStream()` allows you to redirect debug and error messages to any `Stream` object (like `Serial`).
    *   **Example:**
        ```cpp
        void setup() {
          Serial.begin(115200);
          dtsSensor.setDebugStream(&Serial); // Enable debug output
          // ...
        }
        ```

## Features

*   Initializes UART communication with the sensor.
*   Starts and stops the sensor's continuous measurement stream with `enableSensor()` and `disableSensor()`.
*   Parses incoming data frames according to the datasheet protocol.
*   Performs Modbus CRC-16 checksum validation for data integrity (can be optionally disabled for performance).
*   Provides easy-to-use functions to retrieve:
    *   Primary Target Distance (mm)
    *   Primary Target Intensity
    *   Secondary Target Distance (mm)
    *   Secondary Target Intensity
    *   Sunlight Base Level
    *   Correction Values (Primary & Secondary)
*   Provides `isDataNew()` method to check for new measurements non-destructively.
*   Optional debug output via `setDebugStream()`.
*   Wrapped in a namespace to prevent conflicts.
*   Uses `constexpr` for better optimization and type safety.

## Hardware Requirements

*   **DTS6012M Sensor Module:** The sensor this library is designed for.
*   **Arduino Board:** An Arduino board with at least one available **HardwareSerial** port (e.g., `Serial1`, `Serial2`). Examples include Arduino Mega, Arduino Due, ESP32, STM32-based boards, etc.
    *   **Note:** The default sensor baud rate (921600 bps) is generally **too high** for SoftwareSerial libraries. Using a HardwareSerial port is strongly recommended.
*   **3.3V Power Supply:** The sensor requires a 3.3V supply for both Pin 1 (3V3_LASER) and Pin 2 (3V3). Ensure your Arduino can supply sufficient current or use an external 3.3V regulator.
*   **Jumper Wires:** For making connections.
*   **(Optional)** Logic Level Shifter (if connecting 5V Arduino TX to 3.3V sensor RX).

## Software Requirements

*   **Arduino IDE:** Version 1.8.10 or later recommended.
*   **This Library:** `DTS6012M_UART`

## Installation

1.  **Library Manager:**
    *   Open the Arduino IDE.
    *   Go to `Sketch` -> `Include Library` -> `Manage Libraries...`
    *   Search for `DTS6012M_UART`.
    *   Click `Install`.
2.  **Manual Installation:**
    *   Download the latest release ZIP file from the repository.
    *   In the Arduino IDE, go to `Sketch` -> `Include Library` -> `Add .ZIP Library...`
    *   Select the downloaded ZIP file.
    *   Alternatively, unzip the file and copy the `DTS6012M_UART` folder into your Arduino `libraries` directory (usually found in your Sketchbook location).
    *   Restart the Arduino IDE.

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
DTS6012M::UART dtsSensor(SensorSerial);

void setup() {
  Serial.begin(115200); // For printing results
  while (!Serial);

  // --- Optional: Enable Debugging ---
  // dtsSensor.setDebugStream(&Serial);

  // 4. Initialize the sensor library (starts Serial1 at 921600 default)
  if (!dtsSensor.begin()) {
    Serial.println("Failed to initialize sensor!");
    while (1); // Halt
  }
  Serial.println("Sensor initialized.");
}

void loop() {
  // 5. Call update() frequently to process incoming serial data
  dtsSensor.update();

  // 6. Check if new data is available
  if (dtsSensor.isDataNew()) {
    // 7. Get the data (this also clears the 'isDataNew' flag)
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