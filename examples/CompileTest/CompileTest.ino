#include <Arduino.h>
#include "DTS6012M_UART.h"

/*
 * Simple compile test for backward compatibility
 */

HardwareSerial &SensorSerial = Serial1;
DTS6012M_UART dtsSensor(SensorSerial);

void setup() {
  Serial.begin(115200);
  
  // Test v1.x style (should work with implicit conversion)
  if (dtsSensor.begin()) {
    Serial.println("V1.x style initialization OK");
  }
  
  // Test v2.0 style  
  DTSResult result = dtsSensor.begin();
  if (result == DTSError::NONE) {
    Serial.println("V2.0 style initialization OK");
  }
  
  // Test bool conversion
  bool success = dtsSensor.begin();
  if (success) {
    Serial.println("Bool conversion OK");
  }
}

void loop() {
  // Test v1.x style update
  if (dtsSensor.update()) {
    uint16_t distance = dtsSensor.getDistance();
    Serial.println(distance);
  }
}