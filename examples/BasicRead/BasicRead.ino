#include <Arduino.h>
#include "DTS6012M_UART.h"

/*
 * Basic DTS6012M Distance Sensor Example
 * 
 * This example demonstrates simple distance measurement using the DTS6012M sensor.
 * Compatible with both v1.x and v2.0 library APIs.
 * 
 * Hardware Connections:
 * - Arduino GND <-> Sensor Pin 6 (GND)
 * - Arduino 3.3V -> Sensor Pin 1 (3V3_LASER)
 * - Arduino 3.3V -> Sensor Pin 2 (3V3)
 * - Arduino GND <-> Sensor Pin 5 (GPIO) ** IMPORTANT: Connect to GND for UART mode **
 * - Arduino RX1 (Pin 19 on Mega) <- Sensor Pin 3 (UART_TX)
 * - Arduino TX1 (Pin 18 on Mega) -> Sensor Pin 4 (UART_RX)
 * 
 * Note: Use logic level shifter for 5V Arduino boards!
 */

// Select the HardwareSerial port connected to the sensor
HardwareSerial &SensorSerial = Serial1;

// Create sensor instance 
DTS6012M_UART dtsSensor(SensorSerial);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("DTS6012M Basic Distance Sensor Example");
  Serial.println("Initializing sensor...");
  
  // Initialize sensor (v1.x compatible syntax)
  if (dtsSensor.begin()) {
    Serial.println("Sensor initialized successfully!");
  } else {
    Serial.println("ERROR: Sensor initialization failed!");
    Serial.println("Check wiring and connections.");
    while (1) delay(100);
  }
  
  Serial.println("Starting measurements...");
  Serial.println("Format: Distance (mm) | Intensity");
  Serial.println("--------------------------------");
}

void loop() {
  // Process sensor data (v1.x compatible syntax)
  if (dtsSensor.update()) {
    // Get measurement data
    uint16_t distance = dtsSensor.getDistance();
    uint16_t intensity = dtsSensor.getIntensity();
    
    // Print results
    Serial.print("Distance: ");
    if (distance == 0xFFFF) {
      Serial.print("----");
    } else {
      Serial.print(distance);
    }
    Serial.print(" mm | Intensity: ");
    Serial.println(intensity);
  }
  
  delay(100); // Print measurements every 100ms
}