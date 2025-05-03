#include "DTS6012M_UART.h"

/**
 * @brief Constructor for the DTS6012M_UART class.
 * @param serialPort A reference to the HardwareSerial port object (e.g., Serial1) to be used for communication.
 */
DTS6012M_UART::DTS6012M_UART(HardwareSerial &serialPort) : _serial(serialPort) {
  // Initialize private member variables
  _rxBufferIndex = 0;
  _distancePrimary_mm = 0xFFFF; // Initialize distance to invalid value
  _intensityPrimary = 0;
  _correctionPrimary = 0;
  _distanceSecondary_mm = 0xFFFF; // Initialize distance to invalid value
  _intensitySecondary = 0;
  _correctionSecondary = 0;
  _sunlightBase = 0;
  _newDataAvailable = false;
  _lastUpdateTime = 0;
  _crcCheckEnabled = true; // Enable CRC check by default
}

/**
 * @brief Initializes the serial communication and starts the sensor's data stream.
 * @param baudRate The desired baud rate for UART communication (default: 921600).
 * @return true if initialization was successful, false otherwise.
 */
bool DTS6012M_UART::begin(unsigned long baudRate) {
  _serial.begin(baudRate);
  delay(10); // Short delay to allow serial port to stabilize

  // Check if the hardware serial port started correctly
  if (!_serial) {
      // Consider adding error logging if desired: Serial.println("Error: Sensor serial port failed to start.");
      return false;
  }

  // Send the "Start Measurement Stream" command (0x01) to the sensor
  sendCommand(DTS_CMD_START_STREAM, NULL, 0);
  _lastUpdateTime = millis(); // Initialize timeout timer
  return true;
}

/**
 * @brief Processes incoming serial data from the sensor. Needs to be called repeatedly in the main loop.
 * @return true if a new, valid measurement frame was received and parsed during this call, false otherwise.
 */
bool DTS6012M_UART::update() {
  bool frameReceivedAndParsed = false;
  _newDataAvailable = false; // Reset the flag at the beginning of each update cycle

  // Process all available bytes in the serial buffer
  while (_serial.available() > 0) {
    byte incomingByte = _serial.read();
    _lastUpdateTime = millis(); // Update timestamp on receiving any byte

    // --- Simple State Machine for Frame Parsing ---

    // State 1: Waiting for Header Byte (0xA5)
    if (_rxBufferIndex == 0) {
      if (incomingByte == DTS_HEADER) {
        _rxBuffer[_rxBufferIndex++] = incomingByte; // Store header and advance index
      }
      // If not header, ignore the byte and stay in state 1
    }
    // State 2: Receiving Frame Content
    else {
      // Store the byte if buffer is not full
      if (_rxBufferIndex < DTS_RESPONSE_FRAME_LENGTH) {
         _rxBuffer[_rxBufferIndex++] = incomingByte;
      } else {
         // Buffer overflow condition - Should ideally not happen if FRAME_LENGTH is correct
         // Reset buffer to recover
         _rxBufferIndex = 0;
         // Consider logging an error: Serial.println("Error: RX Buffer Overflow!");
         continue; // Skip to next byte
      }

      // Check if the buffer is now full (received expected number of bytes)
      if (_rxBufferIndex >= DTS_RESPONSE_FRAME_LENGTH) {
        // Attempt to parse the complete frame
        if (parseFrame()) {
          frameReceivedAndParsed = true; // Frame was valid
          _newDataAvailable = true;      // Set flag
        } else {
          // Frame was invalid (bad header, length, or CRC)
          // parseFrame already logged errors if debugging enabled
        }
        _rxBufferIndex = 0; // Reset buffer index to wait for the next frame's header
        // If a valid frame was found, we can exit the update early
        if(frameReceivedAndParsed) return true;
      }
    } // End of State 2
  } // End while (_serial.available())

   if (millis() - _lastUpdateTime > 1000) { // Example: 1 second timeout
     Serial.println("Warning: Sensor communication timeout?");
     _lastUpdateTime = millis(); // Reset timer to avoid continuous warnings
     _rxBufferIndex = 0; // Reset buffer state
   }

  return frameReceivedAndParsed; // Return true only if a valid frame was processed in *this* call
}

/**
 * @brief Parses the data in the _rxBuffer. Validates header, length, and CRC. Extracts data if valid.
 * @return true if the frame is valid and data was extracted, false otherwise.
 */
bool DTS6012M_UART::parseFrame() {
   // 1. Validate Header, Device Number, Device Type, and Command Code
   //    We expect responses to the START_STREAM command (0x01)
  if (_rxBuffer[0] != DTS_HEADER ||
      _rxBuffer[1] != DTS_DEVICE_NO ||
      _rxBuffer[2] != DTS_DEVICE_TYPE ||
      _rxBuffer[3] != DTS_CMD_START_STREAM) {
    // Serial.println("Debug: Frame header/ID/CMD mismatch"); // Optional Debugging
    return false; // Invalid frame structure
  }

  // 2. Validate Data Payload Length (Bytes 5 & 6, MSB first)
  uint16_t dataLength = ((uint16_t)_rxBuffer[5] << 8) | _rxBuffer[6];
  if (dataLength != DTS_DATA_LENGTH_EXPECTED) {
    // Serial.print("Debug: Frame length mismatch. Expected: "); Serial.print(DTS_DATA_LENGTH_EXPECTED); // Optional Debugging
    // Serial.print(" Got: "); Serial.println(dataLength); // Optional Debugging
    return false; // Incorrect payload length
  }

  // 3. Validate CRC Checksum (if enabled)
  if (_crcCheckEnabled) {
    // CRC is calculated over the frame *excluding* the last two CRC bytes themselves.
    uint16_t receivedCRC = ((uint16_t)_rxBuffer[DTS_RESPONSE_FRAME_LENGTH - 1] << 8) | _rxBuffer[DTS_RESPONSE_FRAME_LENGTH - 2];
    uint16_t calculatedCRC = calculateCRC16(_rxBuffer, DTS_RESPONSE_FRAME_LENGTH - 2);

    if (calculatedCRC != receivedCRC) {
      // Serial.print("Debug: CRC FAIL! Calc: 0x"); Serial.print(calculatedCRC, HEX); // Optional Debugging
      // Serial.print(" Recv: 0x"); Serial.println(receivedCRC, HEX); // Optional Debugging
      return false; // CRC check failed, data corrupted
    }
  }

  // --- If all checks pass, extract the data ---
  // Data payload starts at index 7 of the full frame buffer.
  const int dataPayloadOffset = 7;

  // Extract values (remembering datasheet specifies LSB first for data fields)
  _distanceSecondary_mm = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SEC_DIST + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_SEC_DIST];
  _correctionSecondary  = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SEC_CORR + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_SEC_CORR];
  _intensitySecondary   = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SEC_INT + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_SEC_INT];
  _distancePrimary_mm   = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_PRI_DIST + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_PRI_DIST];
  _correctionPrimary    = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_PRI_CORR + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_PRI_CORR];
  _intensityPrimary     = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_PRI_INT + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_PRI_INT];
  _sunlightBase         = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SUN_BASE + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_SUN_BASE];

  return true; // Frame successfully parsed and data extracted
}


// --- Data Getter Methods ---

uint16_t DTS6012M_UART::getDistance() {
  // Could add logic here to check _newDataAvailable if strict "only get fresh data" is needed.
  return _distancePrimary_mm;
}

uint16_t DTS6012M_UART::getIntensity() {
  return _intensityPrimary;
}

uint16_t DTS6012M_UART::getSunlightBase() {
  return _sunlightBase;
}

uint16_t DTS6012M_UART::getCorrection() {
    return _correctionPrimary;
}

uint16_t DTS6012M_UART::getSecondaryDistance() {
    return _distanceSecondary_mm;
}

uint16_t DTS6012M_UART::getSecondaryIntensity() {
    return _intensitySecondary;
}

uint16_t DTS6012M_UART::getSecondaryCorrection() {
    return _correctionSecondary;
}

/**
 * @brief Enables or disables the CRC check for incoming frames.
 * @param enable Set to true to enable CRC validation (default), false to disable.
 *               Disabling improves performance but increases the risk of accepting corrupted data.
 */
void DTS6012M_UART::enableCRC(bool enable) {
    _crcCheckEnabled = enable;
}


/**
 * @brief Sends a command frame to the sensor.
 * @param cmd The command byte code (e.g., DTS_CMD_STOP_STREAM).
 * @param dataPayload Pointer to the data bytes to include in the frame (NULL if none).
 * @param payloadLength The number of bytes in dataPayload.
 */
void DTS6012M_UART::sendCommand(byte cmd, const byte *dataPayload, uint16_t payloadLength) {
  // Calculate required frame size: Fixed parts (7) + payload + CRC (2)
  int frameSize = 9 + payloadLength;
  byte frame[frameSize];
  int frameIndex = 0;

  // 1. Build Frame Header & Info
  frame[frameIndex++] = DTS_HEADER;
  frame[frameIndex++] = DTS_DEVICE_NO;
  frame[frameIndex++] = DTS_DEVICE_TYPE;
  frame[frameIndex++] = cmd;
  frame[frameIndex++] = 0x00; // Reserved byte (usually 0x00)
  frame[frameIndex++] = (payloadLength >> 8) & 0xFF; // Length MSB
  frame[frameIndex++] = payloadLength & 0xFF;        // Length LSB

  // 2. Add Data Payload (if provided)
  if (dataPayload != NULL && payloadLength > 0) {
      memcpy(&frame[frameIndex], dataPayload, payloadLength);
      frameIndex += payloadLength;
  }

  // 3. Calculate CRC16 on the frame constructed so far (Header to end of Data)
  uint16_t crc = calculateCRC16(frame, frameIndex);

  // 4. Append CRC16 (LSB first, as per Modbus standard with RefOut=true)
  frame[frameIndex++] = crc & 0xFF;        // CRC LSB
  frame[frameIndex++] = (crc >> 8) & 0xFF; // CRC MSB

  // 5. Send the complete frame over the serial port
  _serial.write(frame, frameIndex);
  _serial.flush(); // Optional: Wait for transmission to complete
}

/**
 * @brief Calculates the Modbus CRC-16 checksum.
 * @param data Pointer to the byte array.
 * @param len The number of bytes in the array to checksum.
 * @return The calculated 16-bit CRC value.
 */
uint16_t DTS6012M_UART::calculateCRC16(const byte *data, int len) {
  // Modbus CRC-16: Polynomial 0x8005 (becomes 0xA001 reversed), Init 0xFFFF, RefIn true, RefOut true, XorOut 0x0000
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i]; // XOR byte into least sig byte of crc
    for (int j = 0; j < 8; j++) { // Process each bit
      if (crc & 0x0001) { // If LSB is 1
        crc = (crc >> 1) ^ 0xA001; // Shift right and XOR with reversed polynomial
      } else {
        crc = crc >> 1; // Just shift right
      }
    }
  }
  return crc; // Result requires no final XOR
}
