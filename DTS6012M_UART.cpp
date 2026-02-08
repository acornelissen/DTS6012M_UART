#include "DTS6012M_UART.h"

// Add this for PROGMEM if an AVR microcontroller is used
#if defined(ARDUINO_ARCH_AVR)
#include <avr/pgmspace.h>
#else
// Define pgm_read_word_near for non-AVR architectures to make the code cleaner
#define pgm_read_word_near(addr) (*(const uint16_t *)(addr))
#endif

// Helper function to convert DTSCommand enum to byte
static inline byte commandToByte(DTSCommand cmd) {
  return static_cast<byte>(cmd);
}

/**
 * @brief Enhanced constructor with configuration support
 * @param serialPort A reference to the HardwareSerial port object (e.g., Serial1)
 * @param config Configuration structure with sensor parameters
 */
DTS6012M_UART::DTS6012M_UART(HardwareSerial &serialPort, const DTSConfig &config) 
    : _serial(serialPort), _config(config)
{
  // Initialize enhanced member variables
  _rxBufferIndex = 0;
  _circularBufferHead = 0;
  _circularBufferTail = 0;
  _lastUpdateTime = 0;
  _lastValidFrameTime = 0;
  _historyIndex = 0;
  _lastError = DTSError::NONE;
  _consecutiveErrors = 0;
  _crcByteOrderMode = _config.crcByteOrder;
  _activeCRCByteOrder = DTSCRCByteOrder::LSB_THEN_MSB;
  _crcErrorStreak = 0;
  _distanceOffset_mm = 0;
  _distanceScale = 1.0f;
  _frameState = FrameState::WAITING_FOR_HEADER;

  setCRCAutoSwitchErrorThreshold(_config.crcAutoSwitchErrorThreshold);
  setCRCByteOrder(_config.crcByteOrder);
  
  // Initialize current measurement with invalid values
  _currentMeasurement = {
    .primaryDistance_mm = DTS_INVALID_DISTANCE,
    .primaryIntensity = DTS_INVALID_INTENSITY,
    .primaryCorrection = 0,
    .secondaryDistance_mm = DTS_INVALID_DISTANCE,
    .secondaryIntensity = DTS_INVALID_INTENSITY,
    .secondaryCorrection = 0,
    .sunlightBase = 0,
    .timestamp = 0,
    .primaryQuality = DataQuality::INVALID,
    .secondaryQuality = DataQuality::INVALID,
    .lastError = DTSError::NONE
  };
  
  // Initialize statistics
  resetStatistics();
  
  // Initialize measurement history
  for (int i = 0; i < DTS_HISTORY_BUFFER_SIZE; i++) {
    _measurementHistory[i] = _currentMeasurement;
  }
}

/**
 * @brief Enhanced initialization with detailed error reporting
 * @param baudRate The desired baud rate (0 = use config default)
 * @return DTSResult for backward compatibility (converts to bool or DTSError)
 */
DTSResult DTS6012M_UART::begin(unsigned long baudRate, int8_t rxPin, int8_t txPin)
{
  // Use provided baud rate or config default
  unsigned long targetBaudRate = (baudRate == 0) ? _config.baudRate : baudRate;
  
  _serial.begin(targetBaudRate, SERIAL_8N1, rxPin, txPin);
  delay(10); // Allow serial port to stabilize

  // Verify serial port initialization
  if (!_serial) {
    recordError(DTSError::SERIAL_INIT_FAILED);
    return DTSResult(DTSError::SERIAL_INIT_FAILED);
  }

  // Reset state and buffers
  resetFrameState();
  resetCRCByteOrderState();
  _lastUpdateTime = millis();
  _lastValidFrameTime = millis();
  
  // Send start stream command
  DTSError result = sendCommand(DTSCommand::START_STREAM, nullptr, 0);
  if (result != DTSError::NONE) {
    recordError(result);
    return DTSResult(result);
  }
  
  _lastError = DTSError::NONE;
  return DTSResult(DTSError::NONE);
}

/**
 * @brief Enhanced update with improved frame synchronization and error handling
 * @return DTSResult for backward compatibility (converts to bool or DTSError)
 */
DTSResult DTS6012M_UART::update()
{
  DTSError result = DTSError::NONE;
  bool newFrameReceived = false;

  // Process all available bytes using circular buffer
  while (_serial.available() > 0) {
    byte incomingByte = _serial.read();
    _lastUpdateTime = millis();
    
    // Store in circular buffer
    _circularBuffer[_circularBufferHead] = incomingByte;
    _circularBufferHead = (_circularBufferHead + 1) % DTS_CIRCULAR_BUFFER_SIZE;
    
    // Prevent buffer overflow by advancing tail if needed
    if (_circularBufferHead == _circularBufferTail) {
      _circularBufferTail = (_circularBufferTail + 1) % DTS_CIRCULAR_BUFFER_SIZE;
    }
  }
  
  // Process circular buffer for complete frames
  result = processCircularBuffer();
  if (result == DTSError::NONE) {
    newFrameReceived = true;
    _lastValidFrameTime = millis();
    _consecutiveErrors = 0;
    _lastError = DTSError::NONE;
  } else if (result != DTSError::TIMEOUT) {
    _consecutiveErrors++;
    recordError(result);
  }
  
  // Check for communication timeout
  if (isTimeout()) {
    recordError(DTSError::TIMEOUT);
    resetFrameState();
    return DTSResult(DTSError::TIMEOUT);
  }
  
  // Return success only if new frame was processed
  return DTSResult(newFrameReceived ? DTSError::NONE : result);
}

/**
 * @brief Enhanced frame parsing with detailed error reporting and data quality assessment
 * @return DTSError::NONE if frame is valid, specific error code otherwise
 */
DTSError DTS6012M_UART::parseFrame()
{
  // 1. Validate frame header structure
  if (_rxBuffer[0] != DTS_HEADER) {
    return DTSError::FRAME_HEADER_INVALID;
  }
  
  if (_rxBuffer[1] != DTS_DEVICE_NO || 
      _rxBuffer[2] != DTS_DEVICE_TYPE ||
      _rxBuffer[3] != commandToByte(DTSCommand::START_STREAM)) {
    return DTSError::FRAME_HEADER_INVALID;
  }

  // 2. Validate data payload length (bytes 5 & 6, MSB first)
  uint16_t dataLength = ((uint16_t)_rxBuffer[5] << 8) | _rxBuffer[6];
  if (dataLength != DTS_DATA_LENGTH_EXPECTED) {
    return DTSError::FRAME_LENGTH_INVALID;
  }

  // 3. Validate CRC checksum (if enabled)
  if (_config.crcEnabled) {
    uint16_t calculatedCRC = calculateCRC16(_rxBuffer, DTS_RESPONSE_FRAME_LENGTH - 2);
    if (!validateFrameCRC(calculatedCRC)) {
      return DTSError::CRC_CHECK_FAILED;
    }
  } else {
    _crcErrorStreak = 0;
  }

  // 4. Extract measurement data (LSB first format)
  constexpr int dataPayloadOffset = 7;
  
  DTSMeasurement newMeasurement;
  newMeasurement.secondaryDistance_mm = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SEC_DIST + 1] << 8) | 
                                        _rxBuffer[dataPayloadOffset + DTS_IDX_SEC_DIST];
  newMeasurement.secondaryCorrection = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SEC_CORR + 1] << 8) | 
                                       _rxBuffer[dataPayloadOffset + DTS_IDX_SEC_CORR];
  newMeasurement.secondaryIntensity = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SEC_INT + 1] << 8) | 
                                      _rxBuffer[dataPayloadOffset + DTS_IDX_SEC_INT];
  
  uint16_t rawPrimaryDistance = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_PRI_DIST + 1] << 8) | 
                                _rxBuffer[dataPayloadOffset + DTS_IDX_PRI_DIST];
  newMeasurement.primaryDistance_mm = applyCalibratedDistance(rawPrimaryDistance);
  
  newMeasurement.primaryCorrection = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_PRI_CORR + 1] << 8) | 
                                     _rxBuffer[dataPayloadOffset + DTS_IDX_PRI_CORR];
  newMeasurement.primaryIntensity = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_PRI_INT + 1] << 8) | 
                                    _rxBuffer[dataPayloadOffset + DTS_IDX_PRI_INT];
  newMeasurement.sunlightBase = ((uint16_t)_rxBuffer[dataPayloadOffset + DTS_IDX_SUN_BASE + 1] << 8) | 
                                _rxBuffer[dataPayloadOffset + DTS_IDX_SUN_BASE];
  
  // 5. Add metadata
  newMeasurement.timestamp = millis();
  newMeasurement.primaryQuality = assessDataQuality(newMeasurement);
  newMeasurement.secondaryQuality = assessDataQuality(newMeasurement); // Could be enhanced for secondary-specific logic
  newMeasurement.lastError = DTSError::NONE;
  
  // 6. Store measurement and update statistics
  _currentMeasurement = newMeasurement;
  
  // Store in history buffer
  _measurementHistory[_historyIndex] = newMeasurement;
  _historyIndex = (_historyIndex + 1) % DTS_HISTORY_BUFFER_SIZE;
  
  // Update statistics
  updateStatistics(newMeasurement);
  
  return DTSError::NONE;
}

// --- Enhanced Data Access Methods ---

DTSMeasurement DTS6012M_UART::getMeasurement() const
{
  return _currentMeasurement;
}

uint16_t DTS6012M_UART::getDistance() const
{
  return _currentMeasurement.primaryDistance_mm;
}

uint16_t DTS6012M_UART::getIntensity() const
{
  return _currentMeasurement.primaryIntensity;
}

bool DTS6012M_UART::isDataValid() const
{
  return (_currentMeasurement.primaryQuality != DataQuality::INVALID &&
          _currentMeasurement.primaryDistance_mm != DTS_INVALID_DISTANCE &&
          _currentMeasurement.primaryIntensity >= _config.minIntensityThreshold);
}

DataQuality DTS6012M_UART::getDataQuality() const
{
  return _currentMeasurement.primaryQuality;
}

// Legacy getters for backward compatibility
uint16_t DTS6012M_UART::getSunlightBase() const
{
  return _currentMeasurement.sunlightBase;
}

uint16_t DTS6012M_UART::getCorrection() const
{
  return _currentMeasurement.primaryCorrection;
}

uint16_t DTS6012M_UART::getSecondaryDistance() const
{
  return _currentMeasurement.secondaryDistance_mm;
}

uint16_t DTS6012M_UART::getSecondaryIntensity() const
{
  return _currentMeasurement.secondaryIntensity;
}

uint16_t DTS6012M_UART::getSecondaryCorrection() const
{
  return _currentMeasurement.secondaryCorrection;
}

// --- Enhanced Control Methods ---

DTSError DTS6012M_UART::configure(const DTSConfig &config)
{
  _config = config;
  setCRCAutoSwitchErrorThreshold(_config.crcAutoSwitchErrorThreshold);
  setCRCByteOrder(_config.crcByteOrder);
  
  // Reinitialize serial if baud rate changed
  if (_serial) {
    _serial.end();
    _serial.begin(_config.baudRate);
    delay(10);
    
    if (!_serial) {
      recordError(DTSError::SERIAL_INIT_FAILED);
      return DTSError::SERIAL_INIT_FAILED;
    }
  }
  
  return DTSError::NONE;
}

void DTS6012M_UART::enableCRC(bool enable)
{
  _config.crcEnabled = enable;
  _crcErrorStreak = 0;
}

void DTS6012M_UART::setCRCByteOrder(DTSCRCByteOrder mode)
{
  _config.crcByteOrder = mode;
  _crcByteOrderMode = mode;

  if (_crcByteOrderMode == DTSCRCByteOrder::MSB_THEN_LSB) {
    _activeCRCByteOrder = DTSCRCByteOrder::MSB_THEN_LSB;
  } else {
    // AUTO starts from LSB_THEN_MSB for backward compatibility.
    _activeCRCByteOrder = DTSCRCByteOrder::LSB_THEN_MSB;
  }

  _crcErrorStreak = 0;
}

DTSCRCByteOrder DTS6012M_UART::getCRCByteOrder() const
{
  return _crcByteOrderMode;
}

DTSCRCByteOrder DTS6012M_UART::getActiveCRCByteOrder() const
{
  return _activeCRCByteOrder;
}

void DTS6012M_UART::setCRCAutoSwitchErrorThreshold(uint16_t threshold)
{
  _config.crcAutoSwitchErrorThreshold = (threshold == 0) ? 1 : threshold;
}

uint16_t DTS6012M_UART::getCRCAutoSwitchErrorThreshold() const
{
  return _config.crcAutoSwitchErrorThreshold;
}

DTSResult DTS6012M_UART::enableSensor()
{
  return DTSResult(sendCommand(DTSCommand::START_STREAM, nullptr, 0));
}

DTSResult DTS6012M_UART::disableSensor()
{
  return DTSResult(sendCommand(DTSCommand::STOP_STREAM, nullptr, 0));
}

DTSError DTS6012M_UART::resetState()
{
  resetStatistics();
  _distanceOffset_mm = 0;
  _distanceScale = 1.0f;
  _lastError = DTSError::NONE;
  _consecutiveErrors = 0;
  resetCRCByteOrderState();
  
  return DTSError::NONE;
}

DTSError DTS6012M_UART::factoryReset()
{
  // Datasheet exposes no factory reset command. Provide a soft-reset and report unsupported.
  resetState();
  return DTSError::UNSUPPORTED_OPERATION;
}

// --- Data Analysis Methods ---

DTSStatistics DTS6012M_UART::getStatistics() const
{
  return _statistics;
}

void DTS6012M_UART::resetStatistics()
{
  _statistics = {
    .minDistance = 65535,
    .maxDistance = 0,
    .avgDistance = 0,
    .measurementCount = 0,
    .errorCount = 0
  };
}

DTSError DTS6012M_UART::getLastError() const
{
  return _lastError;
}

void DTS6012M_UART::clearError()
{
  _lastError = DTSError::NONE;
  _consecutiveErrors = 0;
  _crcErrorStreak = 0;
}

// --- Calibration Methods ---

void DTS6012M_UART::setDistanceOffset(int16_t offset_mm)
{
  _distanceOffset_mm = offset_mm;
}

void DTS6012M_UART::setDistanceScale(float scale_factor)
{
  if (scale_factor > 0.0f) {
    _distanceScale = scale_factor;
  }
}

/**
 * @brief Enhanced command sending with error handling and pre-allocation
 * @param cmd Command from DTSCommand enum
 * @param dataPayload Optional data payload
 * @param payloadLength Size of payload in bytes
 * @return DTSError::NONE on success, error code on failure
 */
DTSError DTS6012M_UART::sendCommand(DTSCommand cmd, const byte *dataPayload, uint16_t payloadLength)
{
  // Validate parameters
  if (payloadLength > (DTS_MAX_COMMAND_FRAME_SIZE - 9)) {
    recordError(DTSError::INVALID_COMMAND);
    return DTSError::INVALID_COMMAND;
  }
  
  if (!_serial) {
    recordError(DTSError::SERIAL_INIT_FAILED);
    return DTSError::SERIAL_INIT_FAILED;
  }
  
  int frameIndex = 0;
  
  // 1. Build frame header using pre-allocated buffer
  _commandFrame[frameIndex++] = DTS_HEADER;
  _commandFrame[frameIndex++] = DTS_DEVICE_NO;
  _commandFrame[frameIndex++] = DTS_DEVICE_TYPE;
  _commandFrame[frameIndex++] = commandToByte(cmd);
  _commandFrame[frameIndex++] = 0x00; // Reserved byte
  _commandFrame[frameIndex++] = (payloadLength >> 8) & 0xFF; // Length MSB
  _commandFrame[frameIndex++] = payloadLength & 0xFF; // Length LSB

  // 2. Add data payload if provided
  if (dataPayload != nullptr && payloadLength > 0) {
    memcpy(&_commandFrame[frameIndex], dataPayload, payloadLength);
    frameIndex += payloadLength;
  }

  // 3. Calculate and append CRC16
  uint16_t crc = calculateCRC16(_commandFrame, frameIndex);
  appendCommandCRC(_commandFrame, frameIndex, crc);

  // 4. Send frame
  size_t bytesWritten = _serial.write(_commandFrame, frameIndex);
  
  // Verify all bytes were sent
  if (bytesWritten != frameIndex) {
    recordError(DTSError::SERIAL_INIT_FAILED);
    return DTSError::SERIAL_INIT_FAILED;
  }
  
  return DTSError::NONE;
}

void DTS6012M_UART::sendCommand(byte cmd, const byte *dataPayload, uint16_t payloadLength)
{
  // Preserve main/v1 behavior for larger payloads that exceed the pre-allocated frame buffer.
  if (payloadLength <= (DTS_MAX_COMMAND_FRAME_SIZE - 9)) {
    (void)sendCommand(static_cast<DTSCommand>(cmd), dataPayload, payloadLength);
    return;
  }

  if (!_serial) {
    recordError(DTSError::SERIAL_INIT_FAILED);
    return;
  }

  const size_t frameSize = static_cast<size_t>(9u + payloadLength);
  byte *frame = static_cast<byte *>(malloc(frameSize));
  if (frame == nullptr) {
    recordError(DTSError::INVALID_COMMAND);
    return;
  }

  int frameIndex = 0;
  frame[frameIndex++] = DTS_HEADER;
  frame[frameIndex++] = DTS_DEVICE_NO;
  frame[frameIndex++] = DTS_DEVICE_TYPE;
  frame[frameIndex++] = cmd;
  frame[frameIndex++] = 0x00;
  frame[frameIndex++] = (payloadLength >> 8) & 0xFF;
  frame[frameIndex++] = payloadLength & 0xFF;

  if (dataPayload != nullptr && payloadLength > 0) {
    memcpy(&frame[frameIndex], dataPayload, payloadLength);
    frameIndex += payloadLength;
  }

  uint16_t crc = calculateCRC16(frame, frameIndex);
  appendCommandCRC(frame, frameIndex, crc);
  _serial.write(frame, frameIndex);
  free(frame);
}

/**
 * @brief Optimized CRC-16 calculation with lookup table
 * @param data Pointer to the byte array
 * @param len The number of bytes in the array to checksum
 * @return The calculated 16-bit CRC value
 */
uint16_t DTS6012M_UART::calculateCRC16(const byte *data, int len) const
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

// --- Enhanced Private Helper Methods ---

DataQuality DTS6012M_UART::assessDataQuality(const DTSMeasurement &measurement) const
{
  // Invalid distance check
  if (measurement.primaryDistance_mm == DTS_INVALID_DISTANCE) {
    return DataQuality::INVALID;
  }
  
  // Range validation
  if (measurement.primaryDistance_mm < _config.minValidDistance_mm ||
      measurement.primaryDistance_mm > _config.maxValidDistance_mm) {
    return DataQuality::POOR;
  }
  
  // Intensity-based quality assessment
  if (measurement.primaryIntensity < _config.minIntensityThreshold) {
    return DataQuality::POOR;
  } else if (measurement.primaryIntensity < (_config.minIntensityThreshold * 2)) {
    return DataQuality::FAIR;
  } else if (measurement.primaryIntensity < (_config.minIntensityThreshold * 4)) {
    return DataQuality::GOOD;
  } else {
    return DataQuality::EXCELLENT;
  }
}

void DTS6012M_UART::updateStatistics(const DTSMeasurement &measurement)
{
  if (measurement.primaryDistance_mm != DTS_INVALID_DISTANCE) {
    _statistics.measurementCount++;
    
    // Update min/max
    if (measurement.primaryDistance_mm < _statistics.minDistance) {
      _statistics.minDistance = measurement.primaryDistance_mm;
    }
    if (measurement.primaryDistance_mm > _statistics.maxDistance) {
      _statistics.maxDistance = measurement.primaryDistance_mm;
    }
    
    // Update running average
    _statistics.avgDistance = ((_statistics.avgDistance * (_statistics.measurementCount - 1)) + 
                              measurement.primaryDistance_mm) / _statistics.measurementCount;
  }
  
  if (measurement.lastError != DTSError::NONE) {
    _statistics.errorCount++;
  }
}

uint16_t DTS6012M_UART::applyCalibratedDistance(uint16_t rawDistance) const
{
  if (rawDistance == DTS_INVALID_DISTANCE) {
    return DTS_INVALID_DISTANCE;
  }
  
  // Apply scaling and offset
  float calibratedDistance = (rawDistance * _distanceScale) + _distanceOffset_mm;
  
  // Clamp to valid range
  if (calibratedDistance < 0) {
    return 0;
  } else if (calibratedDistance > 65534) {
    return 65534;
  } else {
    return static_cast<uint16_t>(calibratedDistance);
  }
}

DTSError DTS6012M_UART::processCircularBuffer()
{
  DTSError lastError = DTSError::TIMEOUT;
  // Look for frame header in circular buffer
  while (_circularBufferTail != _circularBufferHead) {
    byte currentByte = _circularBuffer[_circularBufferTail];
    _circularBufferTail = (_circularBufferTail + 1) % DTS_CIRCULAR_BUFFER_SIZE;
    
    switch (_frameState) {
      case FrameState::WAITING_FOR_HEADER:
        if (currentByte == DTS_HEADER) {
          _rxBuffer[0] = currentByte;
          _rxBufferIndex = 1;
          _frameState = FrameState::RECEIVING_FRAME;
        }
        break;
        
      case FrameState::RECEIVING_FRAME:
        if (_rxBufferIndex < DTS_RESPONSE_FRAME_LENGTH) {
          _rxBuffer[_rxBufferIndex++] = currentByte;
          
          if (_rxBufferIndex >= DTS_RESPONSE_FRAME_LENGTH) {
            _frameState = FrameState::FRAME_COMPLETE;
            DTSError result = parseFrame();
            resetFrameState();
            
            if (result == DTSError::NONE) {
              return DTSError::NONE;
            }
            lastError = result;
          }
        } else {
          // Buffer overflow - reset and try again
          resetFrameState();
          return DTSError::BUFFER_OVERFLOW;
        }
        break;
        
      case FrameState::FRAME_COMPLETE:
        // Should not reach here
        resetFrameState();
        break;
    }
  }
  
  return lastError;
}

void DTS6012M_UART::resetFrameState()
{
  _frameState = FrameState::WAITING_FOR_HEADER;
  _rxBufferIndex = 0;
}

bool DTS6012M_UART::isTimeout() const
{
  return (millis() - _lastValidFrameTime) > _config.timeout_ms;
}

void DTS6012M_UART::recordError(DTSError error)
{
  if (error == DTSError::NONE)
  {
    return;
  }

  _lastError = error;
  _statistics.errorCount++;
}

bool DTS6012M_UART::validateFrameCRC(uint16_t calculatedCRC)
{
  uint16_t receivedCRC = extractFrameCRC(_activeCRCByteOrder);
  if (calculatedCRC == receivedCRC) {
    _crcErrorStreak = 0;
    return true;
  }

  if (_crcByteOrderMode == DTSCRCByteOrder::AUTO) {
    _crcErrorStreak++;
    if (_crcErrorStreak >= _config.crcAutoSwitchErrorThreshold) {
      _activeCRCByteOrder = (_activeCRCByteOrder == DTSCRCByteOrder::LSB_THEN_MSB)
                                ? DTSCRCByteOrder::MSB_THEN_LSB
                                : DTSCRCByteOrder::LSB_THEN_MSB;
      _crcErrorStreak = 0;

      // Retry current frame with the alternate ordering immediately.
      receivedCRC = extractFrameCRC(_activeCRCByteOrder);
      if (calculatedCRC == receivedCRC) {
        return true;
      }
    }
  }

  return false;
}

uint16_t DTS6012M_UART::extractFrameCRC(DTSCRCByteOrder order) const
{
  const int crcLsbIndex = DTS_RESPONSE_FRAME_LENGTH - 2;
  const int crcMsbIndex = DTS_RESPONSE_FRAME_LENGTH - 1;

  if (order == DTSCRCByteOrder::MSB_THEN_LSB) {
    return ((uint16_t)_rxBuffer[crcLsbIndex] << 8) | _rxBuffer[crcMsbIndex];
  }

  // Default and AUTO fallback: LSB then MSB on wire.
  return ((uint16_t)_rxBuffer[crcMsbIndex] << 8) | _rxBuffer[crcLsbIndex];
}

void DTS6012M_UART::appendCommandCRC(byte *buffer, int &index, uint16_t crc) const
{
  if (_activeCRCByteOrder == DTSCRCByteOrder::MSB_THEN_LSB) {
    buffer[index++] = (crc >> 8) & 0xFF; // CRC MSB
    buffer[index++] = crc & 0xFF;        // CRC LSB
    return;
  }

  buffer[index++] = crc & 0xFF;         // CRC LSB
  buffer[index++] = (crc >> 8) & 0xFF;  // CRC MSB
}

void DTS6012M_UART::resetCRCByteOrderState()
{
  setCRCAutoSwitchErrorThreshold(_config.crcAutoSwitchErrorThreshold);
  setCRCByteOrder(_config.crcByteOrder);
}
