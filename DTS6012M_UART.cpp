#include "DTS6012M_UART.h"
#include "crc16_table.h" // Include the CRC table

using namespace DTS6012M;

/**
 * @brief Constructor for the UART class.
 * @param serialPort A reference to the HardwareSerial port object (e.g., Serial1) to be used for communication.
 */
UART::UART(HardwareSerial &serialPort) : _serial(serialPort)
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
  _debugStream = nullptr; // No debug stream by default
}

/**
 * @brief Initializes the serial communication and starts the sensor's data stream.
 * @param baudRate The desired baud rate for UART communication (default: 921600).
 * @return true if initialization was successful, false otherwise.
 */
bool UART::begin(unsigned long baudRate)
{
  _serial.begin(baudRate);
  delay(Timing::SERIAL_STABILIZATION_DELAY_MS); // Short delay to allow serial port to stabilize

  // Check if the hardware serial port started correctly
  if (!_serial)
  {
    if (_debugStream) _debugStream->println("Error: Sensor serial port failed to start.");
    return false;
  }

  // Send the "Start Measurement Stream" command (0x01) to the sensor
  sendCommand(Protocol::CMD_START_STREAM, NULL, 0);
  _lastUpdateTime = millis(); // Initialize timeout timer
  return true;
}

/**
 * @brief Processes incoming serial data from the sensor. Needs to be called repeatedly in the main loop.
 * @return true if a new, valid measurement frame was received and parsed during this call, false otherwise.
 */
bool UART::update()
{
  bool frameReceivedAndParsed = false;
  // Don't reset the flag here. It should only be reset when the user reads the data.
  // _newDataAvailable = false;

  // Process all available bytes in the serial buffer
  while (_serial.available() > 0)
  {
    byte incomingByte = _serial.read();
    _lastUpdateTime = millis(); // Update timestamp on receiving any byte

    // --- Simple State Machine for Frame Parsing ---

    // State 1: Waiting for Header Byte
    if (_rxBufferIndex == 0)
    {
      if (incomingByte == Protocol::HEADER)
      {
        _rxBuffer[_rxBufferIndex++] = incomingByte; // Store header and advance index
      }
      // If not header, ignore the byte and stay in state 1
    }
    // State 2: Receiving Frame Content
    else
    {
      // Store the byte if buffer is not full
      if (_rxBufferIndex < Frame::RESPONSE_FRAME_LENGTH)
      {
        _rxBuffer[_rxBufferIndex++] = incomingByte;
      }
      else
      {
        // Buffer overflow condition
        if (_debugStream) _debugStream->println("Error: RX Buffer Overflow!");
        _rxBufferIndex = 0; // Reset buffer to recover
        continue; // Skip to next byte
      }

      // Check if the buffer is now full
      if (_rxBufferIndex >= Frame::RESPONSE_FRAME_LENGTH)
      {
        // Attempt to parse the complete frame
        if (parseFrame())
        {
          frameReceivedAndParsed = true; // Frame was valid
          _newDataAvailable = true;      // Set flag
        }
        _rxBufferIndex = 0; // Reset buffer index for the next frame
        if (frameReceivedAndParsed) return true; // Exit early on success
      }
    } // End of State 2
  } // End while (_serial.available())

  if (millis() - _lastUpdateTime > Timing::COMMUNICATION_TIMEOUT_MS)
  { // Communication timeout
    if (_debugStream) _debugStream->println("Warning: Sensor communication timeout.");
    _lastUpdateTime = millis(); // Reset timer to avoid continuous warnings
    _rxBufferIndex = 0;         // Reset buffer state
  }

  return frameReceivedAndParsed;
}

/**
 * @brief Parses the data in the _rxBuffer. Validates header, length, and CRC. Extracts data if valid.
 * @return true if the frame is valid and data was extracted, false otherwise.
 */
bool UART::parseFrame()
{
  // 1. Validate Header, Device Number, Device Type, and Command Code
  if (_rxBuffer[0] != Protocol::HEADER ||
      _rxBuffer[1] != Protocol::DEVICE_NO ||
      _rxBuffer[2] != Protocol::DEVICE_TYPE ||
      _rxBuffer[3] != Protocol::CMD_START_STREAM)
  {
    if (_debugStream) _debugStream->println("Debug: Frame header/ID/CMD mismatch");
    return false; // Invalid frame structure
  }

  // 2. Validate Data Payload Length (Bytes 5 & 6, MSB first)
  uint16_t dataLength = ((uint16_t)_rxBuffer[5] << 8) | _rxBuffer[6];
  if (dataLength != Frame::DATA_LENGTH_EXPECTED)
  {
    if (_debugStream) {
      _debugStream->print("Debug: Frame length mismatch. Expected: ");
      _debugStream->print(Frame::DATA_LENGTH_EXPECTED);
      _debugStream->print(" Got: ");
      _debugStream->println(dataLength);
    }
    return false; // Incorrect payload length
  }

  // 3. Validate CRC Checksum (if enabled)
  if (_crcCheckEnabled)
  {
    uint16_t receivedCRC = ((uint16_t)_rxBuffer[Frame::RESPONSE_FRAME_LENGTH - 1] << 8) | _rxBuffer[Frame::RESPONSE_FRAME_LENGTH - 2];
    uint16_t calculatedCRC = calculateCRC16(_rxBuffer, Frame::RESPONSE_FRAME_LENGTH - Frame::CRC_SIZE);

    if (calculatedCRC != receivedCRC)
    {
      if (_debugStream) {
        _debugStream->print("Debug: CRC FAIL! Calc: 0x"); _debugStream->print(calculatedCRC, HEX);
        _debugStream->print(" Recv: 0x"); _debugStream->println(receivedCRC, HEX);
      }
      return false; // CRC check failed
    }
  }

  // --- If all checks pass, extract the data ---
  _distanceSecondary_mm = extractUint16LSB(Frame::DATA_PAYLOAD_OFFSET + PayloadIndex::SEC_DIST);
  _correctionSecondary = extractUint16LSB(Frame::DATA_PAYLOAD_OFFSET + PayloadIndex::SEC_CORR);
  _intensitySecondary = extractUint16LSB(Frame::DATA_PAYLOAD_OFFSET + PayloadIndex::SEC_INT);
  _distancePrimary_mm = extractUint16LSB(Frame::DATA_PAYLOAD_OFFSET + PayloadIndex::PRI_DIST);
  _correctionPrimary = extractUint16LSB(Frame::DATA_PAYLOAD_OFFSET + PayloadIndex::PRI_CORR);
  _intensityPrimary = extractUint16LSB(Frame::DATA_PAYLOAD_OFFSET + PayloadIndex::PRI_INT);
  _sunlightBase = extractUint16LSB(Frame::DATA_PAYLOAD_OFFSET + PayloadIndex::SUN_BASE);

  return true; // Frame successfully parsed
}

// --- Data Getter Methods ---

uint16_t UART::getDistance()
{
  _newDataAvailable = false; // User has read the data
  return _distancePrimary_mm;
}

uint16_t UART::getIntensity()
{
  return _intensityPrimary;
}

uint16_t UART::getSunlightBase()
{
  return _sunlightBase;
}

uint16_t UART::getCorrection()
{
  return _correctionPrimary;
}

uint16_t UART::getSecondaryDistance()
{
  return _distanceSecondary_mm;
}

uint16_t UART::getSecondaryIntensity()
{
  return _intensitySecondary;
}

uint16_t UART::getSecondaryCorrection()
{
  return _correctionSecondary;
}

bool UART::isDataNew() const {
  return _newDataAvailable;
}

/**
 * @brief Enables or disables the CRC check for incoming frames.
 * @param enable Set to true to enable CRC validation (default), false to disable.
 */
void UART::enableCRC(bool enable)
{
  _crcCheckEnabled = enable;
}

/**
 * @brief Enables the sensor by sending the START_STREAM command.
 */
void UART::enableSensor()
{
  sendCommand(Protocol::CMD_START_STREAM, NULL, 0);
}

/**
 * @brief Disables the sensor by sending the STOP_STREAM command.
 */
void UART::disableSensor()
{
  sendCommand(Protocol::CMD_STOP_STREAM, NULL, 0);
}

/**
 * @brief Sets an optional stream for debug logging.
 * @param debugStream Pointer to a Print-derived object (e.g., &Serial) for log output.
 */
void UART::setDebugStream(Stream* debugStream) {
    _debugStream = debugStream;
}

/**
 * @brief Sends a command frame to the sensor.
 * @param cmd The command byte code (e.g., Protocol::CMD_STOP_STREAM).
 * @param dataPayload Pointer to the data bytes to include in the frame (NULL if none).
 * @param payloadLength The number of bytes in dataPayload.
 */
void UART::sendCommand(byte cmd, const byte *dataPayload, uint16_t payloadLength)
{
  if (Frame::HEADER_SIZE + payloadLength + Frame::CRC_SIZE > Frame::MAX_FRAME_SIZE) {
    if (_debugStream) _debugStream->println("Error: Command payload too large!");
    return; // Payload too large
  }

  byte frame[Frame::MAX_FRAME_SIZE];
  int frameIndex = 0;

  // 1. Build Frame Header
  frame[frameIndex++] = Protocol::HEADER;
  frame[frameIndex++] = Protocol::DEVICE_NO;
  frame[frameIndex++] = Protocol::DEVICE_TYPE;
  frame[frameIndex++] = cmd;
  frame[frameIndex++] = 0x00; // Reserved
  frame[frameIndex++] = (payloadLength >> 8) & 0xFF; // Length MSB
  frame[frameIndex++] = payloadLength & 0xFF;        // Length LSB

  // 2. Add Data Payload
  if (dataPayload != NULL && payloadLength > 0)
  {
    memcpy(&frame[frameIndex], dataPayload, payloadLength);
    frameIndex += payloadLength;
  }

  // 3. Calculate and Append CRC16
  uint16_t crc = calculateCRC16(frame, frameIndex);
  frame[frameIndex++] = crc & 0xFF;        // CRC LSB
  frame[frameIndex++] = (crc >> 8) & 0xFF; // CRC MSB

  // 4. Send the frame
  _serial.write(frame, frameIndex);
}

/**
 * @brief Calculates the Modbus CRC-16 checksum using a lookup table.
 * @param data Pointer to the byte array.
 * @param len The number of bytes to checksum.
 * @return The calculated 16-bit CRC value.
 */
uint16_t UART::calculateCRC16(const byte *data, int len)
{
  uint16_t crc = 0xFFFF; // Initial value

  for (int i = 0; i < len; i++)
  {
    byte index = (byte)(crc ^ data[i]);
#if defined(ARDUINO_ARCH_AVR)
    crc = (uint16_t)((crc >> 8) ^ pgm_read_word_near(&crc_table[index]));
#else
    crc = (uint16_t)((crc >> 8) ^ crc_table[index]);
#endif
  }
  return crc;
}

/**
 * @brief Helper function to extract 16-bit value from buffer (LSB first ordering)
 * @param offset The offset in the _rxBuffer to start extraction
 * @return The extracted 16-bit value
 */
uint16_t UART::extractUint16LSB(int offset) const
{
  return ((uint16_t)_rxBuffer[offset + 1] << 8) | _rxBuffer[offset];
}
