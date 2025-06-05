#include "DTS6012M_UART.h"

// Add this for PROGMEM if an AVR microcontroller is used
#if defined(ARDUINO_ARCH_AVR)
#include <avr/pgmspace.h>
#else
// Define pgm_read_word_near for non-AVR architectures to make the code cleaner
#define pgm_read_word_near(addr) (*(const uint16_t *)(addr))
#endif

/**
 * @brief Constructor for the DTS6012M_UART class.
 * @param serialPort A reference to the HardwareSerial port object (e.g., Serial1) to be used for communication.
 */
DTS6012M_UART::DTS6012M_UART(HardwareSerial &serialPort) : _serial(serialPort)
{
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
bool DTS6012M_UART::begin(unsigned long baudRate)
{
  _serial.begin(baudRate);
  delay(10); // Short delay to allow serial port to stabilize

  // Check if the hardware serial port started correctly
  if (!_serial)
  {
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
bool DTS6012M_UART::update()
{
  bool frameReceivedAndParsed = false;
  _newDataAvailable = false; // Reset the flag at the beginning of each update cycle

  // Process all available bytes in the serial buffer
  while (_serial.available() > 0)
  {
    byte incomingByte = _serial.read();
    _lastUpdateTime = millis(); // Update timestamp on receiving any byte

    // --- Simple State Machine for Frame Parsing ---

    // State 1: Waiting for Header Byte (0xA5)
    if (_rxBufferIndex == 0)
    {
      if (incomingByte == DTS_HEADER)
      {
        _rxBuffer[_rxBufferIndex++] = incomingByte; // Store header and advance index
      }
      // If not header, ignore the byte and stay in state 1
    }
    // State 2: Receiving Frame Content
    else
    {
      // Store the byte if buffer is not full
      if (_rxBufferIndex < DTS_RESPONSE_FRAME_LENGTH)
      {
        _rxBuffer[_rxBufferIndex++] = incomingByte;
      }
      else
      {
        // Buffer overflow condition - Should ideally not happen if FRAME_LENGTH is correct
        // Reset buffer to recover
        _rxBufferIndex = 0;
        // Consider logging an error: Serial.println("Error: RX Buffer Overflow!");
        continue; // Skip to next byte
      }

      // Check if the buffer is now full (received expected number of bytes)
      if (_rxBufferIndex >= DTS_RESPONSE_FRAME_LENGTH)
      {
        // Attempt to parse the complete frame
        if (parseFrame())
        {
          frameReceivedAndParsed = true; // Frame was valid
          _newDataAvailable = true;      // Set flag
        }
        else
        {
          // Frame was invalid (bad header, length, or CRC)
          // parseFrame already logged errors if debugging enabled
        }
        _rxBufferIndex = 0; // Reset buffer index to wait for the next frame's header
        // If a valid frame was found, we can exit the update early
        if (frameReceivedAndParsed)
          return true;
      }
    } // End of State 2
  } // End while (_serial.available())

  if (millis() - _lastUpdateTime > 1000)
  { // Example: 1 second timeout
    // Serial.println("Warning: Sensor communication timeout?"); // Keep this commented unless debugging
    _lastUpdateTime = millis(); // Reset timer to avoid continuous warnings
    _rxBufferIndex = 0;         // Reset buffer state
  }

  return frameReceivedAndParsed; // Return true only if a valid frame was processed in *this* call
}

/**
 * @brief Parses the data in the _rxBuffer. Validates header, length, and CRC. Extracts data if valid.
 * @return true if the frame is valid and data was extracted, false otherwise.
 */
bool DTS6012M_UART::parseFrame()
{
  // 1. Validate Header, Device Number, Device Type, and Command Code
  //    We expect responses to the START_STREAM command (0x01)
  if (_rxBuffer[0] != DTS_HEADER ||
      _rxBuffer[1] != DTS_DEVICE_NO ||
      _rxBuffer[2] != DTS_DEVICE_TYPE ||
      _rxBuffer[3] != DTS_CMD_START_STREAM)
  {
    // Serial.println("Debug: Frame header/ID/CMD mismatch"); // Optional Debugging
    return false; // Invalid frame structure
  }

  // 2. Validate Data Payload Length (Bytes 5 & 6, MSB first)
  uint16_t dataLength = ((uint16_t)_rxBuffer[5] << 8) | _rxBuffer[6];
  if (dataLength != DTS_DATA_LENGTH_EXPECTED)
  {
    // Serial.print("Debug: Frame length mismatch. Expected: "); Serial.print(DTS_DATA_LENGTH_EXPECTED); // Optional Debugging
    // Serial.print(" Got: "); Serial.println(dataLength); // Optional Debugging
    return false; // Incorrect payload length
  }

  // 3. Validate CRC Checksum (if enabled)
  if (_crcCheckEnabled)
  {
    // CRC is calculated over the frame *excluding* the last two CRC bytes themselves.
    // The datasheet implies CRC LSB first, then MSB in the frame.
    // So, _rxBuffer[DTS_RESPONSE_FRAME_LENGTH - 2] is LSB, _rxBuffer[DTS_RESPONSE_FRAME_LENGTH - 1] is MSB.
    uint16_t receivedCRC = ((uint16_t)_rxBuffer[DTS_RESPONSE_FRAME_LENGTH - 2] << 8) | _rxBuffer[DTS_RESPONSE_FRAME_LENGTH - 1];
    uint16_t calculatedCRC = calculateCRC16(_rxBuffer, DTS_RESPONSE_FRAME_LENGTH - 2);

    if (calculatedCRC != receivedCRC)
    {
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
  _correctionSecondary = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SEC_CORR + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_SEC_CORR];
  _intensitySecondary = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SEC_INT + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_SEC_INT];
  _distancePrimary_mm = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_PRI_DIST + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_PRI_DIST];
  _correctionPrimary = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_PRI_CORR + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_PRI_CORR];
  _intensityPrimary = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_PRI_INT + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_PRI_INT];
  _sunlightBase = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SUN_BASE + 1] << 8) | _rxBuffer[dataPayloadOffset + DTS_IDX_SUN_BASE];

  return true; // Frame successfully parsed and data extracted
}

// --- Data Getter Methods ---

uint16_t DTS6012M_UART::getDistance()
{
  return _distancePrimary_mm;
}

uint16_t DTS6012M_UART::getIntensity()
{
  return _intensityPrimary;
}

uint16_t DTS6012M_UART::getSunlightBase()
{
  return _sunlightBase;
}

uint16_t DTS6012M_UART::getCorrection()
{
  return _correctionPrimary;
}

uint16_t DTS6012M_UART::getSecondaryDistance()
{
  return _distanceSecondary_mm;
}

uint16_t DTS6012M_UART::getSecondaryIntensity()
{
  return _intensitySecondary;
}

uint16_t DTS6012M_UART::getSecondaryCorrection()
{
  return _correctionSecondary;
}

/**
 * @brief Enables or disables the CRC check for incoming frames.
 * @param enable Set to true to enable CRC validation (default), false to disable.
 * Disabling improves performance but increases the risk of accepting corrupted data.
 */
void DTS6012M_UART::enableCRC(bool enable)
{
  _crcCheckEnabled = enable;
}

/**
 * @brief Sends a command frame to the sensor.
 * @param cmd The command byte code (e.g., DTS_CMD_STOP_STREAM).
 * @param dataPayload Pointer to the data bytes to include in the frame (NULL if none).
 * @param payloadLength The number of bytes in dataPayload.
 */
void DTS6012M_UART::sendCommand(byte cmd, const byte *dataPayload, uint16_t payloadLength)
{
  // Calculate required frame size: Fixed parts (7) + payload + CRC (2)
  int frameSize = 9 + payloadLength;
  byte frame[frameSize]; // Consider dynamic allocation or a max-size buffer if payloadLength can be very large
  int frameIndex = 0;

  // 1. Build Frame Header & Info
  frame[frameIndex++] = DTS_HEADER;
  frame[frameIndex++] = DTS_DEVICE_NO;
  frame[frameIndex++] = DTS_DEVICE_TYPE;
  frame[frameIndex++] = cmd;
  frame[frameIndex++] = 0x00;                        // Reserved byte (usually 0x00)
  frame[frameIndex++] = (payloadLength >> 8) & 0xFF; // Length MSB
  frame[frameIndex++] = payloadLength & 0xFF;        // Length LSB

  // 2. Add Data Payload (if provided)
  if (dataPayload != NULL && payloadLength > 0)
  {
    memcpy(&frame[frameIndex], dataPayload, payloadLength);
    frameIndex += payloadLength;
  }

  // 3. Calculate CRC16 on the frame constructed so far (Header to end of Data)
  uint16_t crc = calculateCRC16(frame, frameIndex);

  // 4. Append CRC16 (LSB first, as per Modbus standard with RefOut=true)
  // The datasheet page 7 describes "CRC-16 校验采用modbus 的校验方式...输出数据反转:是"
  // This typically means the CRC itself is transmitted LSB first.
  frame[frameIndex++] = crc & 0xFF;        // CRC LSB
  frame[frameIndex++] = (crc >> 8) & 0xFF; // CRC MSB

  // 5. Send the complete frame over the serial port
  _serial.write(frame, frameIndex);
  // _serial.flush(); // Optional: Wait for transmission to complete. Generally not needed and can block.
}

/**
 * @brief Calculates the Modbus CRC-16 checksum using a lookup table for speed.
 * @param data Pointer to the byte array.
 * @param len The number of bytes in the array to checksum.
 * @return The calculated 16-bit CRC value.
 */
uint16_t DTS6012M_UART::calculateCRC16(const byte *data, int len)
{
  // CRC-16/MODBUS Lookup Table
  // Polynomial: 0x8005 (becomes 0xA001 when reflected for LSB-first processing)
  // Initial Value: 0xFFFF
  // Reflect Input: true, Reflect Output: true, XOR Output: 0x0000
#if defined(ARDUINO_ARCH_AVR)
  static const uint16_t crc_table[256] PROGMEM = {
#else
  static const uint16_t crc_table[256] = {
#endif
    0x0000,
    0xC0C1,
    0xC181,
    0x0140,
    0xC301,
    0x03C0,
    0x0280,
    0xC241,
    0xC601,
    0x06C0,
    0x0780,
    0xC741,
    0x0500,
    0xC5C1,
    0xC481,
    0x0440,
    0xCC01,
    0x0CC0,
    0x0D80,
    0xCD41,
    0x0F00,
    0xCFC1,
    0xCE81,
    0x0E40,
    0x0A00,
    0xCAC1,
    0xCB81,
    0x0B40,
    0xC901,
    0x09C0,
    0x0880,
    0xC841,
    0xD801,
    0x18C0,
    0x1980,
    0xD941,
    0x1B00,
    0xDBC1,
    0xDA81,
    0x1A40,
    0x1E00,
    0xDEC1,
    0xDF81,
    0x1F40,
    0xDD01,
    0x1DC0,
    0x1C80,
    0xDC41,
    0x1400,
    0xD4C1,
    0xD581,
    0x1540,
    0xD701,
    0x17C0,
    0x1680,
    0xD641,
    0xD201,
    0x12C0,
    0x1380,
    0xD341,
    0x1100,
    0xD1C1,
    0xD081,
    0x1040,
    0xF001,
    0x30C0,
    0x3180,
    0xF141,
    0x3300,
    0xF3C1,
    0xF281,
    0x3240,
    0x3600,
    0xF6C1,
    0xF781,
    0x3740,
    0xF501,
    0x35C0,
    0x3480,
    0xF441,
    0x3C00,
    0xFCC1,
    0xFD81,
    0x3D40,
    0xFF01,
    0x3FC0,
    0x3E80,
    0xFE41,
    0xFA01,
    0x3AC0,
    0x3B80,
    0xFB41,
    0x3900,
    0xF9C1,
    0xF881,
    0x3840,
    0x2800,
    0xE8C1,
    0xE981,
    0x2940,
    0xEB01,
    0x2BC0,
    0x2A80,
    0xEA41,
    0xEE01,
    0x2EC0,
    0x2F80,
    0xEF41,
    0x2D00,
    0xEDC1,
    0xEC81,
    0x2C40,
    0xE401,
    0x24C0,
    0x2580,
    0xE541,
    0x2700,
    0xE7C1,
    0xE681,
    0x2640,
    0x2200,
    0xE2C1,
    0xE381,
    0x2340,
    0xE101,
    0x21C0,
    0x2080,
    0xE041,
    0xA001,
    0x60C0,
    0x6180,
    0xA141,
    0x6300,
    0xA3C1,
    0xA281,
    0x6240,
    0x6600,
    0xA6C1,
    0xA781,
    0x6740,
    0xA501,
    0x65C0,
    0x6480,
    0xA441,
    0x6C00,
    0xACC1,
    0xAD81,
    0x6D40,
    0xAF01,
    0x6FC0,
    0x6E80,
    0xAE41,
    0xAA01,
    0x6AC0,
    0x6B80,
    0xAB41,
    0x6900,
    0xA9C1,
    0xA881,
    0x6840,
    0x7800,
    0xB8C1,
    0xB981,
    0x7940,
    0xBB01,
    0x7BC0,
    0x7A80,
    0xBA41,
    0xBE01,
    0x7EC0,
    0x7F80,
    0xBF41,
    0x7D00,
    0xBDC1,
    0xBC81,
    0x7C40,
    0xB401,
    0x74C0,
    0x7580,
    0xB541,
    0x7700,
    0xB7C1,
    0xB681,
    0x7640,
    0x7200,
    0xB2C1,
    0xB381,
    0x7340,
    0xB101,
    0x71C0,
    0x7080,
    0xB041,
    0x5000,
    0x90C1,
    0x9181,
    0x5140,
    0x9301,
    0x53C0,
    0x5280,
    0x9241,
    0x9601,
    0x56C0,
    0x5780,
    0x9741,
    0x5500,
    0x95C1,
    0x9481,
    0x5440,
    0x9C01,
    0x5CC0,
    0x5D80,
    0x9D41,
    0x5F00,
    0x9FC1,
    0x9E81,
    0x5E40,
    0x5A00,
    0x9AC1,
    0x9B81,
    0x5B40,
    0x9901,
    0x59C0,
    0x5880,
    0x9841,
    0x8801,
    0x48C0,
    0x4980,
    0x8941,
    0x4B00,
    0x8BC1,
    0x8A81,
    0x4A40,
    0x4E00,
    0x8EC1,
    0x8F81,
    0x4F40,
    0x8D01,
    0x4DC0,
    0x4C80,
    0x8C41,
    0x4400,
    0x84C1,
    0x8581,
    0x4540,
    0x8701,
    0x47C0,
    0x4680,
    0x8641,
    0x8201,
    0x42C0,
    0x4380,
    0x8341,
    0x4100,
    0x81C1,
    0x8081,
    0x4040
  };

  uint16_t crc = 0xFFFF; // Initial value

  for (int i = 0; i < len; i++)
  {
    byte index = (byte)(crc ^ data[i]); // LSB of CRC XORed with current data byte
#if defined(ARDUINO_ARCH_AVR)
    crc = (uint16_t)((crc >> 8) ^ pgm_read_word_near(&crc_table[index]));
#else
    crc = (uint16_t)((crc >> 8) ^ crc_table[index]); // For non-AVR, access directly
#endif
  }
  return crc; // No final XOR for standard Modbus CRC
}