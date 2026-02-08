#ifndef DTS6012M_UART_H
#define DTS6012M_UART_H

#include <Arduino.h>

#if defined(DTS6012M_TEST_MODE)
/**
 * @brief Lightweight HardwareSerial stub used exclusively during unit tests.
 * It provides the minimal interface the library relies on, avoiding MCU-specific constructors.
 */
class HardwareSerial : public Stream {
public:
  virtual void begin(unsigned long) {}
  virtual void begin(unsigned long, uint16_t) {}
  virtual void end() {}
  virtual int available() override = 0;
  virtual int read() override = 0;
  virtual int peek() override = 0;
  virtual void flush() override {}
  virtual size_t write(uint8_t) override = 0;
  virtual size_t write(const uint8_t *buffer, size_t size) override {
    size_t written = 0;
    for (size_t i = 0; i < size; ++i) {
      written += write(buffer[i]);
    }
    return written;
  }
  virtual int availableForWrite() { return 0; }
  virtual operator bool() const { return true; }
};
#else
#include <HardwareSerial.h>
#endif

// --- Enhanced Constants and Types ---
// Protocol constants (using constexpr for compile-time evaluation)
constexpr byte DTS_HEADER = 0xA5;
constexpr byte DTS_DEVICE_NO = 0x03;
constexpr byte DTS_DEVICE_TYPE = 0x20;

// Enhanced command codes with enum class for type safety
enum class DTSCommand : byte {
  START_STREAM = 0x01,
  STOP_STREAM = 0x02,
  GET_VERSION = 0x0A,
  SET_BAUD = 0x10,
  GET_BAUD = 0x11,
  SET_I2C_ADDR = 0x12,
  GET_I2C_ADDR = 0x13,
  SET_FRAME_RATE = 0x1A,
  GET_FRAME_RATE = 0x1B
};

// Error codes for better error reporting
enum class DTSError : byte {
  NONE = 0x00,
  SERIAL_INIT_FAILED = 0x01,
  FRAME_HEADER_INVALID = 0x02,
  FRAME_LENGTH_INVALID = 0x03,
  CRC_CHECK_FAILED = 0x04,
  BUFFER_OVERFLOW = 0x05,
  TIMEOUT = 0x06,
  INVALID_COMMAND = 0x07,
  UNSUPPORTED_OPERATION = 0x08
};

// Data quality indicators
enum class DataQuality : byte {
  EXCELLENT = 0,
  GOOD = 1,
  FAIR = 2,
  POOR = 3,
  INVALID = 4
};

// CRC byte ordering options for the 2-byte CRC field
enum class DTSCRCByteOrder : byte {
  LSB_THEN_MSB = 0,
  MSB_THEN_LSB = 1,
  AUTO = 2
};

// Sensor configuration structure
struct DTSConfig {
  unsigned long baudRate = 921600;
  unsigned long timeout_ms = 1000;
  bool crcEnabled = true;
  uint16_t maxValidDistance_mm = 20000;
  uint16_t minValidDistance_mm = 30;
  uint16_t minIntensityThreshold = 100;
  DTSCRCByteOrder crcByteOrder = DTSCRCByteOrder::LSB_THEN_MSB;
  uint16_t crcAutoSwitchErrorThreshold = 200;
};


// Response frame structure constants (using constexpr)
constexpr int DTS_RESPONSE_FRAME_LENGTH = 23;
constexpr int DTS_DATA_LENGTH_EXPECTED = 14;
constexpr int DTS_MAX_COMMAND_FRAME_SIZE = 32;
constexpr int DTS_CIRCULAR_BUFFER_SIZE = 64;

// Indices within the 14-byte data payload (LSB first ordering)
constexpr int DTS_IDX_SEC_DIST = 0;
constexpr int DTS_IDX_SEC_CORR = 2;
constexpr int DTS_IDX_SEC_INT = 4;
constexpr int DTS_IDX_PRI_DIST = 6;
constexpr int DTS_IDX_PRI_CORR = 8;
constexpr int DTS_IDX_PRI_INT = 10;
constexpr int DTS_IDX_SUN_BASE = 12;

// Invalid value constants
constexpr uint16_t DTS_INVALID_DISTANCE = 0xFFFF;
constexpr uint16_t DTS_INVALID_INTENSITY = 0x0000;

// History buffer size for data logging
constexpr int DTS_HISTORY_BUFFER_SIZE = 10;

// Backward compatibility wrapper for DTSError to bool conversion
struct DTSResult {
  DTSError error;
  
  DTSResult(DTSError err) : error(err) {}
  
  // Implicit conversion to bool for v1.x compatibility
  operator bool() const { return error == DTSError::NONE; }
  
  // Explicit conversion to DTSError for v2.0 usage
  operator DTSError() const { return error; }
  
  // Comparison operators
  bool operator==(DTSError other) const { return error == other; }
  bool operator!=(DTSError other) const { return error != other; }
};


// Enhanced measurement data structure with timestamp and quality
struct DTSMeasurement {
  uint16_t primaryDistance_mm;
  uint16_t primaryIntensity;
  uint16_t primaryCorrection;
  uint16_t secondaryDistance_mm;
  uint16_t secondaryIntensity;
  uint16_t secondaryCorrection;
  uint16_t sunlightBase;
  unsigned long timestamp;
  DataQuality primaryQuality;
  DataQuality secondaryQuality;
  DTSError lastError;
};

// Statistics structure for data analysis
struct DTSStatistics {
  uint16_t minDistance;
  uint16_t maxDistance;
  uint32_t avgDistance;
  uint16_t measurementCount;
  uint16_t errorCount;
};

class DTS6012M_UART {
public:
  // --- Enhanced Public Methods ---

  /**
   * @brief Constructor with optional configuration
   * @param serialPort Reference to HardwareSerial port (e.g., Serial1, Serial2)
   * @param config Optional configuration structure
   * @example DTS6012M_UART sensor(Serial1, {.baudRate = 460800, .crcEnabled = false});
   */
  DTS6012M_UART(HardwareSerial &serialPort, const DTSConfig &config = DTSConfig());

  /**
   * @brief Initialize sensor with enhanced error reporting
   * @param baudRate UART baud rate (default from config)
   * @return DTSResult that converts to bool (v1.x) or DTSError (v2.0)
   * @example 
   *   // v1.x style: if (sensor.begin()) { ... }
   *   // v2.0 style: if (sensor.begin() == DTSError::NONE) { ... }
   */
  DTSResult begin(unsigned long baudRate = 0, int8_t rxPin = -1, int8_t txPin = -1);

  /**
   * @brief Process incoming data with enhanced frame synchronization
   * @return DTSResult that converts to bool (v1.x) or DTSError (v2.0)
   * @note Call frequently in main loop for optimal performance
   */
  DTSResult update();

  // --- Enhanced Data Access Methods ---
  
  /**
   * @brief Get complete measurement data with quality indicators
   * @return DTSMeasurement structure with all sensor data and metadata
   */
  DTSMeasurement getMeasurement() const;
  
  /**
   * @brief Get primary target distance with validation
   * @return Distance in mm, DTS_INVALID_DISTANCE if invalid
   */
  uint16_t getDistance() const;
  
  /**
   * @brief Get primary target intensity
   * @return Intensity value, DTS_INVALID_INTENSITY if invalid
   */
  uint16_t getIntensity() const;
  
  /**
   * @brief Check if current measurement data is valid
   * @return true if data passes quality checks
   */
  bool isDataValid() const;
  
  /**
   * @brief Get data quality assessment for primary target
   * @return DataQuality enum indicating measurement reliability
   */
  DataQuality getDataQuality() const;
  
  // Legacy getters maintained for backward compatibility
  uint16_t getSunlightBase() const;
  uint16_t getCorrection() const;
  uint16_t getSecondaryDistance() const;
  uint16_t getSecondaryIntensity() const;
  uint16_t getSecondaryCorrection() const;

  // --- Enhanced Control Methods ---

  /**
   * @brief Send command with enhanced error handling
   * @param cmd Command from DTSCommand enum
   * @param dataPayload Optional data payload
   * @param payloadLength Size of payload in bytes
   * @return DTSError::NONE on success, error code on failure
   */
  DTSError sendCommand(DTSCommand cmd, const byte *dataPayload = nullptr, uint16_t payloadLength = 0);

  /**
   * @brief Configure sensor parameters
   * @param config New configuration structure
   * @return DTSError::NONE on success, error code on failure
   */
  DTSError configure(const DTSConfig &config);

  /**
   * @brief Enable/disable CRC validation with immediate effect
   * @param enable true to enable CRC checking
   */
  void enableCRC(bool enable);

  /**
   * @brief Set CRC byte-order mode used for validation and command transmission.
   * AUTO starts in LSB_THEN_MSB mode and switches after repeated CRC failures.
   */
  void setCRCByteOrder(DTSCRCByteOrder mode);

  /**
   * @brief Get configured CRC byte-order mode.
   */
  DTSCRCByteOrder getCRCByteOrder() const;

  /**
   * @brief Get active CRC byte-order currently used on the wire.
   * In AUTO mode this can differ from getCRCByteOrder().
   */
  DTSCRCByteOrder getActiveCRCByteOrder() const;

  /**
   * @brief Set consecutive CRC error threshold used by AUTO mode before switching order.
   * @param threshold Minimum 1. Lower values switch faster.
   */
  void setCRCAutoSwitchErrorThreshold(uint16_t threshold);

  /**
   * @brief Get AUTO mode consecutive CRC error threshold.
   */
  uint16_t getCRCAutoSwitchErrorThreshold() const;

  /**
   * @brief Start measurement stream
   * @return DTSResult that converts to bool (v1.x) or DTSError (v2.0)
   */
  DTSResult enableSensor();

  /**
   * @brief Stop measurement stream  
   * @return DTSResult that converts to bool (v1.x) or DTSError (v2.0)
   */
  DTSResult disableSensor();

  /**
   * @brief Clear library-side calibration, statistics, and error history.
   * @return DTSError::NONE upon completion
   */
  DTSError resetState();

  /**
   * @brief Compatibility shim for firmware that previously expected a factory reset command.
   * Returns DTSError::UNSUPPORTED_OPERATION because the datasheet exposes no such command.
   */
  DTSError factoryReset();

  // --- Data Logging and Analysis ---
  
  /**
   * @brief Get measurement statistics since last reset
   * @return DTSStatistics structure with analysis data
   */
  DTSStatistics getStatistics() const;
  
  /**
   * @brief Reset statistics counters
   */
  void resetStatistics();
  
  /**
   * @brief Get last error code
   * @return Most recent DTSError encountered
   */
  DTSError getLastError() const;
  
  /**
   * @brief Clear error status
   */
  void clearError();


  // --- Calibration Methods ---
  
  /**
   * @brief Set distance offset calibration
   * @param offset_mm Offset to add to all distance measurements
   */
  void setDistanceOffset(int16_t offset_mm);
  
  /**
   * @brief Set distance scaling factor
   * @param scale_factor Multiplier for distance measurements (1.0 = no scaling)
   */
  void setDistanceScale(float scale_factor);

private:
  // --- Enhanced Private Members ---
  HardwareSerial &_serial;
  DTSConfig _config;
  
  // Improved buffer management with circular buffer
  byte _rxBuffer[DTS_RESPONSE_FRAME_LENGTH];
  byte _circularBuffer[DTS_CIRCULAR_BUFFER_SIZE];
  int _rxBufferIndex;
  int _circularBufferHead;
  int _circularBufferTail;
  unsigned long _lastUpdateTime;
  unsigned long _lastValidFrameTime;
  
  // Pre-allocated command frame buffer
  byte _commandFrame[DTS_MAX_COMMAND_FRAME_SIZE];
  
  // Enhanced measurement data with history
  DTSMeasurement _currentMeasurement;
  DTSMeasurement _measurementHistory[DTS_HISTORY_BUFFER_SIZE];
  int _historyIndex;
  
  // Statistics and error tracking
  DTSStatistics _statistics;
  DTSError _lastError;
  uint16_t _consecutiveErrors;
  DTSCRCByteOrder _crcByteOrderMode;
  DTSCRCByteOrder _activeCRCByteOrder;
  uint16_t _crcErrorStreak;
  
  // Calibration parameters
  int16_t _distanceOffset_mm;
  float _distanceScale;
  
  // Frame synchronization state
  enum class FrameState : byte {
    WAITING_FOR_HEADER,
    RECEIVING_FRAME,
    FRAME_COMPLETE
  } _frameState;


  // --- Enhanced Private Helper Methods ---

  /**
   * @brief Parse and validate complete frame with enhanced error reporting
   * @return DTSError::NONE if valid, specific error code if invalid
   */
  DTSError parseFrame();

  /**
   * @brief Fast CRC-16 calculation with lookup table
   * @param data Pointer to data buffer
   * @param len Length of data to checksum
   * @return Calculated CRC-16 value
   */
  uint16_t calculateCRC16(const byte *data, int len) const;

  /**
   * @brief Validate measurement data quality
   * @param measurement Reference to measurement structure
   * @return DataQuality assessment
   */
  DataQuality assessDataQuality(const DTSMeasurement &measurement) const;

  /**
   * @brief Update statistics with new measurement
   * @param measurement New measurement data
   */
  void updateStatistics(const DTSMeasurement &measurement);

  /**
   * @brief Apply calibration to raw distance measurement
   * @param rawDistance Raw distance value from sensor
   * @return Calibrated distance value
   */
  uint16_t applyCalibratedDistance(uint16_t rawDistance) const;

  /**
   * @brief Process circular buffer for frame synchronization
   * @return DTSError::NONE if frame found, error code otherwise
   */
  DTSError processCircularBuffer();

  /**
   * @brief Reset frame parsing state machine
   */
  void resetFrameState();

  /**
   * @brief Check for communication timeout
   * @return true if timeout detected
   */
  bool isTimeout() const;

  /**
   * @brief Record an error for diagnostics/statistics.
   */
  void recordError(DTSError error);

  /**
   * @brief Validate CRC field according to active byte order and AUTO policy.
   * @param calculatedCRC Locally computed CRC over frame payload
   * @return true if CRC is valid (possibly after AUTO switch/retry)
   */
  bool validateFrameCRC(uint16_t calculatedCRC);

  /**
   * @brief Reconstruct frame CRC according to provided byte ordering.
   */
  uint16_t extractFrameCRC(DTSCRCByteOrder order) const;

  /**
   * @brief Append CRC bytes to outgoing command according to active order.
   */
  void appendCommandCRC(byte *buffer, int &index, uint16_t crc) const;

  /**
   * @brief Reset runtime CRC byte-order tracking state.
   */
  void resetCRCByteOrderState();
};

#endif // DTS6012M_UART_H
