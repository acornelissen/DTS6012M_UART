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

  // Optional: Select CRC byte order for sensor variants.
  // Default is MSB_THEN_LSB (per datasheet).
  // dtsSensor.setCRCByteOrder(DTSCRCByteOrder::LSB_THEN_MSB);
  // dtsSensor.setCRCByteOrder(DTSCRCByteOrder::AUTO);
  
  Serial.println("Starting measurements...");
  Serial.println("Format: Distance (mm) | Intensity");
  Serial.println("--------------------------------");
}

void loop() {
  // Call update() every loop so the hardware serial buffer never overflows
  // between polls. At 921600 baud a delay(100) here would let far more than the
  // AVR core's 64-byte RX buffer pile up, silently truncating frames.
  bool newData = dtsSensor.update();

  if (newData) {
    // Get measurement data
    uint16_t distance = dtsSensor.getDistance();
    uint16_t intensity = dtsSensor.getIntensity();

    // Throttle printing (not polling) to ~10 Hz so the serial monitor is readable.
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.print("Distance: ");
      if (distance == 0xFFFF) {
        Serial.print("----");
      } else {
        Serial.print(distance);
      }
      Serial.print(" mm | Intensity: ");
      Serial.println(intensity);
    }
  }
}
