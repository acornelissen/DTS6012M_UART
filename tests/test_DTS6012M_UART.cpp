#include <Arduino.h>
#ifndef DTS6012M_TEST_MODE
#define DTS6012M_TEST_MODE
#endif
#include "DTS6012M_UART.h"

// Mock Serial class for testing
class MockSerial : public HardwareSerial {
private:
  byte _mockBuffer[256];
  int _mockBufferIndex;
  int _mockReadIndex;
  bool _isAvailable;

public:
  // Records the pins from the most recent begin(): -1 when begun without pins.
  int lastRxPin;
  int lastTxPin;

  MockSerial() : _mockBufferIndex(0), _mockReadIndex(0), _isAvailable(true),
                 lastRxPin(-1), lastTxPin(-1) {}

  void begin(unsigned long) override {
    _isAvailable = true;
    lastRxPin = -1;
    lastTxPin = -1;
  }

  void begin(unsigned long, int8_t rxPin, int8_t txPin) override {
    _isAvailable = true;
    lastRxPin = rxPin;
    lastTxPin = txPin;
  }

  void end() override {
    _isAvailable = false;
  }
  
  operator bool() const override {
    return _isAvailable;
  }
  
  int available() override {
    return _mockBufferIndex - _mockReadIndex;
  }

  int availableForWrite() override {
    return 256 - _mockBufferIndex;
  }
  
  int read() override {
    if (_mockReadIndex < _mockBufferIndex) {
      return _mockBuffer[_mockReadIndex++];
    }
    return -1;
  }

  int peek() override {
    if (_mockReadIndex < _mockBufferIndex) {
      return _mockBuffer[_mockReadIndex];
    }
    return -1;
  }
  
  size_t write(uint8_t data) override {
    if (_mockBufferIndex < 256) {
      _mockBuffer[_mockBufferIndex++] = data;
      return 1;
    }
    return 0;
  }
  
  size_t write(const uint8_t *buffer, size_t size) override {
    size_t written = 0;
    for (size_t i = 0; i < size && _mockBufferIndex < 256; i++) {
      _mockBuffer[_mockBufferIndex++] = buffer[i];
      written++;
    }
    return written;
  }
  
  // Test helper methods
  void mockIncomingData(const byte *data, int length) {
    _mockReadIndex = 0;
    _mockBufferIndex = 0;
    for (int i = 0; i < length && i < 256; i++) {
      _mockBuffer[i] = data[i];
    }
    _mockBufferIndex = length;
  }
  
  void resetMock() {
    _mockBufferIndex = 0;
    _mockReadIndex = 0;
  }
  
  byte* getSentData() { return _mockBuffer; }
  int getSentDataLength() { return _mockBufferIndex; }
};

// Mock that emulates the sensor's request/response behavior for sendOneShot().
// A programmed response is queued into the RX FIFO only when the matching
// command frame is written, so the initial buffer flush in sendOneShot() does
// not consume it. This mirrors real hardware and avoids the shared TX/RX buffer
// echo that the simpler MockSerial exhibits.
class OneShotMockSerial : public HardwareSerial {
private:
  byte _rx[128];
  int _rxHead;
  int _rxTail;
  byte _response[64];
  int _responseLen;
  byte _respondToCmd;

public:
  // Counts command frames written for each command byte, so tests can assert
  // whether the stream was (re)started.
  int startStreamWrites;
  int stopStreamWrites;

  OneShotMockSerial() : _rxHead(0), _rxTail(0), _responseLen(0), _respondToCmd(0),
                        startStreamWrites(0), stopStreamWrites(0) {}

  // Program the response delivered when a frame for command `cmd` is written.
  void setResponse(byte cmd, const byte *data, int len) {
    _respondToCmd = cmd;
    _responseLen = (len < static_cast<int>(sizeof(_response))) ? len : static_cast<int>(sizeof(_response));
    for (int i = 0; i < _responseLen; i++) {
      _response[i] = data[i];
    }
  }

  int available() override { return _rxTail - _rxHead; }
  int read() override { return (_rxHead < _rxTail) ? _rx[_rxHead++] : -1; }
  int peek() override { return (_rxHead < _rxTail) ? _rx[_rxHead] : -1; }
  size_t write(uint8_t) override { return 1; }

  size_t write(const uint8_t *buffer, size_t size) override {
    // A full command frame starts with the header byte; buffer[3] is the command.
    if (size >= 4 && buffer[0] == DTS_HEADER) {
      if (buffer[3] == static_cast<byte>(DTSCommand::START_STREAM)) startStreamWrites++;
      if (buffer[3] == static_cast<byte>(DTSCommand::STOP_STREAM)) stopStreamWrites++;
      if (buffer[3] == _respondToCmd) {
        for (int i = 0; i < _responseLen && _rxTail < static_cast<int>(sizeof(_rx)); i++) {
          _rx[_rxTail++] = _response[i];
        }
      }
    }
    return size;
  }

  operator bool() const override { return true; }
};

// Mock whose block write always reports one byte short, to exercise short-write
// error reporting on the legacy large-payload command path.
class ShortWriteMockSerial : public HardwareSerial {
public:
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  size_t write(uint8_t) override { return 0; }
  size_t write(const uint8_t *, size_t size) override { return size > 0 ? size - 1 : 0; }
  operator bool() const override { return true; }
};

// Test framework
class TestFramework {
private:
  int _testCount;
  int _passedTests;
  int _failedTests;
  
public:
  TestFramework() : _testCount(0), _passedTests(0), _failedTests(0) {}
  
  void assertTrue(bool condition, const char* testName) {
    _testCount++;
    if (condition) {
      Serial.print("PASS: ");
      Serial.println(testName);
      _passedTests++;
    } else {
      Serial.print("FAIL: ");
      Serial.println(testName);
      _failedTests++;
    }
  }
  
  void assertEqual(int expected, int actual, const char* testName) {
    _testCount++;
    if (expected == actual) {
      Serial.print("PASS: ");
      Serial.println(testName);
      _passedTests++;
    } else {
      Serial.print("FAIL: ");
      Serial.print(testName);
      Serial.print(" - Expected: ");
      Serial.print(expected);
      Serial.print(", Got: ");
      Serial.println(actual);
      _failedTests++;
    }
  }
  
  void printSummary() {
    Serial.println("==========================================");
    Serial.print("Test Summary: ");
    Serial.print(_passedTests);
    Serial.print(" passed, ");
    Serial.print(_failedTests);
    Serial.print(" failed out of ");
    Serial.print(_testCount);
    Serial.println(" total tests");
    Serial.println("==========================================");
  }

  int failedCount() const { return _failedTests; }
};

// Global test framework instance
TestFramework testFramework;
MockSerial mockSerial;

// Test helper functions
uint16_t calculateTestCRC16(const byte *data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void updateFrameCRC(byte *frame, DTSCRCByteOrder order = DTSCRCByteOrder::MSB_THEN_LSB) {
  uint16_t crc = calculateTestCRC16(frame, 21);

  if (order == DTSCRCByteOrder::MSB_THEN_LSB) {
    frame[21] = (crc >> 8) & 0xFF;  // CRC MSB
    frame[22] = crc & 0xFF;         // CRC LSB
    return;
  }

  frame[21] = crc & 0xFF;         // CRC LSB
  frame[22] = (crc >> 8) & 0xFF;  // CRC MSB
}

void createValidFrame(byte *frame, DTSCRCByteOrder order = DTSCRCByteOrder::MSB_THEN_LSB) {
  // Create a valid DTS6012M frame for testing
  frame[0] = 0xA5;  // Header
  frame[1] = 0x03;  // Device No
  frame[2] = 0x20;  // Device Type
  frame[3] = 0x01;  // Command (START_STREAM)
  frame[4] = 0x00;  // Reserved
  frame[5] = 0x00;  // Length MSB
  frame[6] = 0x0E;  // Length LSB (14 bytes)
  
  // Data payload (14 bytes) - example measurement data
  frame[7] = 0x00; frame[8] = 0x00;   // Secondary Distance (0)
  frame[9] = 0x00; frame[10] = 0x00;  // Temperature Code (0)
  frame[11] = 0x00; frame[12] = 0x00; // Secondary Intensity (0)
  frame[13] = 0xE8; frame[14] = 0x03; // Primary Distance (1000mm)
  frame[15] = 0x00; frame[16] = 0x00; // Primary Correction (0)
  frame[17] = 0x64; frame[18] = 0x00; // Primary Intensity (100)
  frame[19] = 0x32; frame[20] = 0x00; // Sunlight Base (50)
  
  updateFrameCRC(frame, order);
}

// Individual test functions
void testConstructorAndConfiguration() {
  Serial.println("Running Constructor and Configuration Tests...");
  
  // Test default configuration
  DTSConfig defaultConfig;
  DTS6012M_UART sensor1(mockSerial, defaultConfig);
  testFramework.assertTrue(true, "Constructor with default config");
  
  // Test custom configuration
  DTSConfig customConfig = {
    .baudRate = 460800,
    .timeout_ms = 2000,
    .crcEnabled = false,
    .maxValidDistance_mm = 15000,
    .minValidDistance_mm = 50,
    .minIntensityThreshold = 200
  };
  DTS6012M_UART sensor2(mockSerial, customConfig);
  testFramework.assertTrue(true, "Constructor with custom config");
}

void testInitialization() {
  Serial.println("Running Initialization Tests...");
  
  DTS6012M_UART sensor(mockSerial);
  
  // Test successful initialization
  DTSError result = sensor.begin();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "Successful initialization");
  
  // Test that start command was sent
  testFramework.assertTrue(mockSerial.getSentDataLength() > 0, "Start command sent during initialization");
}

void testFrameParsing() {
  Serial.println("Running Frame Parsing Tests...");
  
  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();
  
  // Test valid frame parsing
  byte validFrame[23];
  createValidFrame(validFrame);
  mockSerial.mockIncomingData(validFrame, 23);
  
  DTSError result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "Valid frame parsing");
  
  // Test data extraction
  uint16_t distance = sensor.getDistance();
  testFramework.assertEqual(1000, distance, "Distance extraction from valid frame");
  
  uint16_t intensity = sensor.getIntensity();
  testFramework.assertEqual(100, intensity, "Intensity extraction from valid frame");
}

void testErrorHandling() {
  Serial.println("Running Error Handling Tests...");
  
  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  mockSerial.resetMock();
  
  // Test invalid header
  byte invalidFrame[23];
  createValidFrame(invalidFrame);
  invalidFrame[1] = 0xFF; // Invalid device number to trigger header check failure
  updateFrameCRC(invalidFrame);
  mockSerial.mockIncomingData(invalidFrame, 23);
  
  DTSError result = sensor.update();
  testFramework.assertTrue(result != DTSError::NONE, "Invalid header detection");
  DTSStatistics stats = sensor.getStatistics();
  testFramework.assertEqual(1, stats.errorCount, "Error count increments on header failure");
  
  // Test invalid length
  createValidFrame(invalidFrame);
  invalidFrame[6] = 0xFF; // Invalid length
  updateFrameCRC(invalidFrame);
  mockSerial.mockIncomingData(invalidFrame, 23);
  
  result = sensor.update();
  testFramework.assertTrue(result != DTSError::NONE, "Invalid length detection");
  stats = sensor.getStatistics();
  testFramework.assertEqual(2, stats.errorCount, "Error count increments on subsequent failures");
}

void testCRCByteOrderModes() {
  Serial.println("Running CRC Byte Order Mode Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  byte frame[23];

  // Default mode (MSB->LSB per datasheet) should accept MSB->LSB CRC.
  createValidFrame(frame, DTSCRCByteOrder::MSB_THEN_LSB);
  mockSerial.mockIncomingData(frame, 23);
  DTSError result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "Default mode accepts MSB->LSB CRC");

  // Fixed LSB->MSB mode should accept LSB->MSB CRC.
  sensor.setCRCByteOrder(DTSCRCByteOrder::LSB_THEN_MSB);
  createValidFrame(frame, DTSCRCByteOrder::LSB_THEN_MSB);
  mockSerial.mockIncomingData(frame, 23);
  result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "LSB->MSB mode accepts LSB->MSB CRC");

  // Fixed LSB->MSB mode should reject MSB->LSB CRC.
  createValidFrame(frame, DTSCRCByteOrder::MSB_THEN_LSB);
  mockSerial.mockIncomingData(frame, 23);
  result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::CRC_CHECK_FAILED), static_cast<int>(result), "LSB->MSB mode rejects MSB->LSB CRC");
}

void testAutoCRCByteOrderSwitch() {
  Serial.println("Running AUTO CRC Byte Order Switching Tests...");

  DTSConfig config = {
    .baudRate = 921600,
    .timeout_ms = 1000,
    .crcEnabled = true,
    .maxValidDistance_mm = 18000,
    .minValidDistance_mm = 20,
    .minIntensityThreshold = 100,
    .crcByteOrder = DTSCRCByteOrder::AUTO,
    .crcAutoSwitchErrorThreshold = 3
  };

  DTS6012M_UART sensor(mockSerial, config);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  testFramework.assertEqual(static_cast<int>(DTSCRCByteOrder::MSB_THEN_LSB),
                            static_cast<int>(sensor.getActiveCRCByteOrder()),
                            "AUTO mode starts with MSB->LSB");

  byte lsbFirstFrame[23];
  createValidFrame(lsbFirstFrame, DTSCRCByteOrder::LSB_THEN_MSB);

  // First two frames should fail while AUTO mode is still trying MSB->LSB.
  mockSerial.mockIncomingData(lsbFirstFrame, 23);
  DTSError result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::CRC_CHECK_FAILED), static_cast<int>(result), "AUTO frame #1 fails before switch");

  mockSerial.mockIncomingData(lsbFirstFrame, 23);
  result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::CRC_CHECK_FAILED), static_cast<int>(result), "AUTO frame #2 fails before switch");

  // Third consecutive failure reaches threshold, flips order, and retries same frame.
  mockSerial.mockIncomingData(lsbFirstFrame, 23);
  result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "AUTO frame #3 triggers switch and succeeds");

  testFramework.assertEqual(static_cast<int>(DTSCRCByteOrder::LSB_THEN_MSB),
                            static_cast<int>(sensor.getActiveCRCByteOrder()),
                            "AUTO mode switches to LSB->MSB");
}

void testDataQuality() {
  Serial.println("Running Data Quality Tests...");
  
  DTSConfig config = {
    .baudRate = 921600,
    .timeout_ms = 1000,
    .crcEnabled = true,
    .maxValidDistance_mm = 18000,
    .minValidDistance_mm = 20,
    .minIntensityThreshold = 100
  };
  
  DTS6012M_UART sensor(mockSerial, config);
  sensor.begin();
  mockSerial.resetMock();
  
  // Test good quality data
  byte validFrame[23];
  createValidFrame(validFrame);
  validFrame[17] = 0xC8; validFrame[18] = 0x00; // High intensity (200)
  updateFrameCRC(validFrame);
  mockSerial.mockIncomingData(validFrame, 23);
  
  sensor.update();
  DataQuality quality = sensor.getDataQuality();
  testFramework.assertTrue(quality == DataQuality::GOOD || quality == DataQuality::EXCELLENT, "Good data quality detection");
  
  // Test data validation
  bool isValid = sensor.isDataValid();
  testFramework.assertTrue(isValid, "Data validation for good data");
}

void testStatistics() {
  Serial.println("Running Statistics Tests...");
  
  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  mockSerial.resetMock();
  
  // Reset statistics
  sensor.resetStatistics();
  DTSStatistics stats = sensor.getStatistics();
  testFramework.assertEqual(0, stats.measurementCount, "Statistics reset - measurement count");
  
  // Process multiple measurements
  byte validFrame[23];
  createValidFrame(validFrame);
  
  // First measurement: 1000mm
  mockSerial.mockIncomingData(validFrame, 23);
  sensor.update();
  
  // Second measurement: 1500mm
  validFrame[13] = 0xDC; validFrame[14] = 0x05; // 1500mm
  updateFrameCRC(validFrame);
  mockSerial.mockIncomingData(validFrame, 23);
  sensor.update();
  
  stats = sensor.getStatistics();
  testFramework.assertEqual(2, stats.measurementCount, "Statistics - measurement count after 2 measurements");
  testFramework.assertEqual(1000, stats.minDistance, "Statistics - minimum distance");
  testFramework.assertEqual(1500, stats.maxDistance, "Statistics - maximum distance");
}

void testCalibration() {
  Serial.println("Running Calibration Tests...");
  
  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  mockSerial.resetMock();
  
  // Test distance offset
  sensor.setDistanceOffset(10); // Add 10mm offset
  
  byte validFrame[23];
  createValidFrame(validFrame); // 1000mm base measurement
  mockSerial.mockIncomingData(validFrame, 23);
  sensor.update();
  
  uint16_t distance = sensor.getDistance();
  testFramework.assertEqual(1010, distance, "Distance offset calibration");
  
  // Test distance scaling
  sensor.setDistanceOffset(0);   // Reset offset
  sensor.setDistanceScale(1.1f); // Scale by 1.1x
  
  mockSerial.mockIncomingData(validFrame, 23);
  sensor.update();
  
  distance = sensor.getDistance();
  testFramework.assertEqual(1100, distance, "Distance scale calibration");
}

void testCommandSending() {
  Serial.println("Running Command Sending Tests...");
  
  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  mockSerial.resetMock();
  
  // Test enable sensor command
  DTSError result = sensor.enableSensor();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "Enable sensor command");
  testFramework.assertTrue(mockSerial.getSentDataLength() > 0, "Enable command sent data");
  
  mockSerial.resetMock();
  
  // Test disable sensor command
  result = sensor.disableSensor();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "Disable sensor command");
  testFramework.assertTrue(mockSerial.getSentDataLength() > 0, "Disable command sent data");

  // Verify CRC byte order in command frames.
  const byte *frame = mockSerial.getSentData();
  int frameLen = mockSerial.getSentDataLength();
  uint16_t crc = calculateTestCRC16(frame, frameLen - 2);
  testFramework.assertEqual(static_cast<int>((crc >> 8) & 0xFF), static_cast<int>(frame[frameLen - 2]), "Command CRC default MSB byte");
  testFramework.assertEqual(static_cast<int>(crc & 0xFF), static_cast<int>(frame[frameLen - 1]), "Command CRC default LSB byte");

  mockSerial.resetMock();
  sensor.setCRCByteOrder(DTSCRCByteOrder::LSB_THEN_MSB);
  result = sensor.enableSensor();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "Enable sensor command with LSB->MSB CRC order");
  frame = mockSerial.getSentData();
  frameLen = mockSerial.getSentDataLength();
  crc = calculateTestCRC16(frame, frameLen - 2);
  testFramework.assertEqual(static_cast<int>(crc & 0xFF), static_cast<int>(frame[frameLen - 2]), "Command CRC LSB byte in LSB->MSB mode");
  testFramework.assertEqual(static_cast<int>((crc >> 8) & 0xFF), static_cast<int>(frame[frameLen - 1]), "Command CRC MSB byte in LSB->MSB mode");
}

void testLegacyCommandApiCompatibility() {
  Serial.println("Running Legacy Command API Compatibility Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  mockSerial.resetMock();

  // Legacy symbols should remain valid and route through command transmission.
  sensor.sendCommand(DTS_CMD_STOP_STREAM, NULL, 0);
  testFramework.assertTrue(mockSerial.getSentDataLength() > 0, "Legacy sendCommand(byte,...) sends data");

  const byte *frame = mockSerial.getSentData();
  testFramework.assertEqual(static_cast<int>(DTS_CMD_STOP_STREAM), static_cast<int>(frame[3]), "Legacy command byte preserved in frame");

  // Legacy API should still support payloads larger than the fixed pre-allocated command buffer.
  mockSerial.resetMock();
  byte largePayload[24];
  for (int i = 0; i < 24; ++i) {
    largePayload[i] = static_cast<byte>(i);
  }
  sensor.sendCommand(DTS_CMD_SET_FRAME_RATE, largePayload, 24);
  testFramework.assertEqual(33, mockSerial.getSentDataLength(), "Legacy sendCommand supports payload > 23 bytes");
}

void testNewDataAvailable() {
  Serial.println("Running newDataAvailable Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  // No data yet — should return false
  testFramework.assertTrue(!sensor.newDataAvailable(), "No data available initially");

  // Feed a valid frame
  byte frame[23];
  createValidFrame(frame);
  mockSerial.mockIncomingData(frame, 23);
  sensor.update();

  // First call should return true
  testFramework.assertTrue(sensor.newDataAvailable(), "Data available after update");

  // Second call should return false (edge-triggered)
  testFramework.assertTrue(!sensor.newDataAvailable(), "Data flag cleared after read");
}

void testHasSecondaryTarget() {
  Serial.println("Running hasSecondaryTarget Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  // Default frame has secondary distance/intensity = 0
  byte frame[23];
  createValidFrame(frame);
  mockSerial.mockIncomingData(frame, 23);
  sensor.update();
  testFramework.assertTrue(!sensor.hasSecondaryTarget(), "No secondary target with zero distance");

  // Set secondary distance = 500mm and intensity = 30
  createValidFrame(frame);
  frame[7] = 0xF4; frame[8] = 0x01;   // Secondary distance 500 (LSB first)
  frame[11] = 0x1E; frame[12] = 0x00; // Secondary intensity 30
  updateFrameCRC(frame);
  mockSerial.mockIncomingData(frame, 23);
  sensor.update();
  testFramework.assertTrue(sensor.hasSecondaryTarget(), "Secondary target detected");
}

void testFilteredDistance() {
  Serial.println("Running getFilteredDistance Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  // Fewer than 3 valid samples — should return INVALID
  uint16_t filtered = sensor.getFilteredDistance();
  testFramework.assertEqual(static_cast<int>(DTS_INVALID_DISTANCE), static_cast<int>(filtered), "Filtered distance invalid with empty history");

  // Feed 5 frames with distances: 1000, 1200, 800, 1100, 900
  byte frame[23];
  uint16_t distances[] = {1000, 1200, 800, 1100, 900};
  for (int i = 0; i < 5; i++) {
    createValidFrame(frame);
    frame[13] = distances[i] & 0xFF;
    frame[14] = (distances[i] >> 8) & 0xFF;
    updateFrameCRC(frame);
    mockSerial.mockIncomingData(frame, 23);
    sensor.update();
  }

  // Sorted: 800, 900, 1000, 1100, 1200 → median = 1000
  filtered = sensor.getFilteredDistance();
  testFramework.assertEqual(1000, static_cast<int>(filtered), "Filtered distance returns median");
}

void testPartialFrameDelivery() {
  Serial.println("Running Partial Frame Delivery Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  byte frame[23];
  createValidFrame(frame);

  // Deliver first 10 bytes
  mockSerial.mockIncomingData(frame, 10);
  DTSError result = sensor.update();
  testFramework.assertTrue(result != DTSError::NONE, "Partial frame (10 bytes) does not succeed");

  // Deliver remaining 13 bytes
  mockSerial.mockIncomingData(frame + 10, 13);
  result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "Frame completes after remaining bytes arrive");

  uint16_t distance = sensor.getDistance();
  testFramework.assertEqual(1000, static_cast<int>(distance), "Correct distance from split-delivered frame");
}

void testResetAndFactoryBehavior() {
  Serial.println("Running Reset/Factory Tests...");
  
  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  mockSerial.resetMock();
  
  byte validFrame[23];
  createValidFrame(validFrame);
  mockSerial.mockIncomingData(validFrame, 23);
  sensor.update(); // increment measurement count
  
  // Introduce an error to increment error count
  byte invalidFrame[23];
  createValidFrame(invalidFrame);
  invalidFrame[0] = 0x00;
  mockSerial.mockIncomingData(invalidFrame, 23);
  sensor.update();
  
  sensor.resetState();
  DTSStatistics stats = sensor.getStatistics();
  testFramework.assertEqual(0, stats.measurementCount, "resetState clears measurement count");
  testFramework.assertEqual(0, stats.errorCount, "resetState clears error count");
  
  DTSError result = sensor.factoryReset();
  testFramework.assertEqual(static_cast<int>(DTSError::UNSUPPORTED_OPERATION), static_cast<int>(result), "factoryReset reports unsupported");
}

void testDrainFreshestFrame() {
  Serial.println("Running Drain Freshest Frame Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();   // zero the statistics so measurementCount is a clean drop counter
  mockSerial.resetMock();

  // Three back-to-back valid frames with distinct primary distances.
  uint16_t distances[3] = {1000, 1500, 2000};
  byte batch[69];
  for (int i = 0; i < 3; i++) {
    byte *f = batch + i * 23;
    createValidFrame(f);
    f[13] = distances[i] & 0xFF;
    f[14] = (distances[i] >> 8) & 0xFF;
    updateFrameCRC(f);
  }
  mockSerial.mockIncomingData(batch, 69);

  // A single update() must drain the whole batch.
  DTSError result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "Drain: single update() succeeds with batched frames");

  // Freshest (third) frame must win, not the oldest.
  testFramework.assertEqual(2000, static_cast<int>(sensor.getDistance()), "Drain: getMeasurement returns freshest (3rd) frame");

  // All three frames must be parsed — none silently discarded from the buffer.
  DTSStatistics stats = sensor.getStatistics();
  testFramework.assertEqual(3, static_cast<int>(stats.measurementCount), "Drain: all 3 batched frames parsed, none lost");
}

void testSendOneShotTruncatedResponse() {
  Serial.println("Running One-Shot Truncated Response Tests...");

  OneShotMockSerial mock;
  DTSConfig config;
  config.crcEnabled = false;   // isolate the truncation path from CRC validation
  DTS6012M_UART sensor(mock, config);
  sensor.begin();

  // Response claims dataLen=2 (needs 7+2+2=11 bytes) but only 8 arrive.
  byte resp[8] = {0xA5, 0x03, 0x20, 0x1B, 0x00, 0x00, 0x02, 0xAA};
  mock.setResponse(static_cast<byte>(DTSCommand::GET_FRAME_RATE), resp, 8);

  uint16_t fps = 0;
  DTSError result = sensor.getFrameRate(fps, 50);
  testFramework.assertEqual(static_cast<int>(DTSError::TIMEOUT), static_cast<int>(result), "Truncated one-shot response returns TIMEOUT, not success");
}

void testGetFrameRateUnparseable() {
  Serial.println("Running getFrameRate Unparseable Response Tests...");

  OneShotMockSerial mock;
  DTSConfig config;
  config.crcEnabled = false;
  DTS6012M_UART sensor(mock, config);
  sensor.begin();

  // Complete frame but dataLen=0 — no frame-rate value present to parse.
  byte resp[9] = {0xA5, 0x03, 0x20, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00};
  mock.setResponse(static_cast<byte>(DTSCommand::GET_FRAME_RATE), resp, 9);

  uint16_t fps = 0;
  DTSError result = sensor.getFrameRate(fps, 50);
  testFramework.assertTrue(result != DTSError::NONE, "getFrameRate reports error when dataLen < 2");
}

void testGetFrameRateSuccess() {
  Serial.println("Running getFrameRate Success Tests...");

  OneShotMockSerial mock;
  DTSConfig config;
  config.crcEnabled = false;
  DTS6012M_UART sensor(mock, config);
  sensor.begin();

  // Valid response: dataLen=2, frame rate 10 fps (0x000A, MSB first in payload).
  byte resp[11] = {0xA5, 0x03, 0x20, 0x1B, 0x00, 0x00, 0x02, 0x00, 0x0A, 0x00, 0x00};
  mock.setResponse(static_cast<byte>(DTSCommand::GET_FRAME_RATE), resp, 11);

  uint16_t fps = 0;
  DTSError result = sensor.getFrameRate(fps, 50);
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "getFrameRate succeeds on a valid response");
  testFramework.assertEqual(10, static_cast<int>(fps), "getFrameRate parses the frame-rate value");
}

void testSetFrameRateLargeAck() {
  Serial.println("Running setFrameRate Large-ACK Tests...");

  OneShotMockSerial mock;
  DTSConfig config;
  config.crcEnabled = false;
  DTS6012M_UART sensor(mock, config);
  sensor.begin();

  // ACK carries dataLen=8 → 17-byte frame, larger than the old 16-byte buffer.
  // A correctly sized buffer must read it in full and report success, not a
  // false TIMEOUT from the truncation guard.
  byte resp[17] = {0xA5, 0x03, 0x20, 0x1A, 0x00, 0x00, 0x08,
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  mock.setResponse(static_cast<byte>(DTSCommand::SET_FRAME_RATE), resp, 17);

  DTSError result = sensor.setFrameRate(10);
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "setFrameRate accepts an ACK larger than 16 bytes");
}

void testWriteIICRegisterLargeAck() {
  Serial.println("Running writeIICRegister Large-ACK Tests...");

  OneShotMockSerial mock;
  DTSConfig config;
  config.crcEnabled = false;
  DTS6012M_UART sensor(mock, config);
  sensor.begin();

  byte resp[17] = {0xA5, 0x03, 0x20, 0x03, 0x00, 0x00, 0x08,
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  mock.setResponse(static_cast<byte>(DTSCommand::WRITE_IIC_REG), resp, 17);

  byte data[2] = {0xAB, 0xCD};
  DTSError result = sensor.writeIICRegister(0x10, data, 2);
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "writeIICRegister accepts an ACK larger than 16 bytes");
}

void testUpdateNoNewData() {
  Serial.println("Running update() No-New-Data Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  // No bytes available, but well within the timeout window (fresh from begin()).
  // This must be reported as the benign NO_NEW_DATA, not a comms TIMEOUT.
  DTSError result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NO_NEW_DATA), static_cast<int>(result), "update() reports NO_NEW_DATA when no frame is available (not TIMEOUT)");

  // Must still read as false for the v1.x bool API (backward compatible).
  DTSResult boolResult = sensor.update();
  testFramework.assertTrue(!boolResult, "update() NO_NEW_DATA converts to false for the bool API");

  // No error should be recorded for the benign no-data case.
  DTSStatistics stats = sensor.getStatistics();
  testFramework.assertEqual(0, static_cast<int>(stats.errorCount), "update() NO_NEW_DATA records no error");
}

void testUpdateRealTimeout() {
  Serial.println("Running update() Real-Timeout Tests...");

  DTSConfig config;
  config.timeout_ms = 1;   // 1 ms window so a short delay is a genuine timeout
  DTS6012M_UART sensor(mockSerial, config);
  sensor.begin();
  mockSerial.resetMock();

  delay(5);   // exceed the timeout window with no incoming frames
  DTSError result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::TIMEOUT), static_cast<int>(result), "update() still reports TIMEOUT after a real comms stall");
}

void testOneShotAutoCRCSwitch() {
  Serial.println("Running One-Shot AUTO CRC Switch Tests...");

  DTSConfig config;
  config.crcEnabled = true;
  config.crcByteOrder = DTSCRCByteOrder::AUTO;   // starts at MSB_THEN_LSB
  OneShotMockSerial mock;
  DTS6012M_UART sensor(mock, config);
  sensor.begin();

  // Valid GET_FRAME_RATE response (dataLen=2, fps=10) but with an LSB-ordered CRC.
  byte resp[11] = {0xA5, 0x03, 0x20, 0x1B, 0x00, 0x00, 0x02, 0x00, 0x0A, 0x00, 0x00};
  uint16_t crc = calculateTestCRC16(resp, 9);   // CRC over 7 header + 2 data bytes
  resp[9] = crc & 0xFF;          // LSB first
  resp[10] = (crc >> 8) & 0xFF;  // MSB second
  mock.setResponse(static_cast<byte>(DTSCommand::GET_FRAME_RATE), resp, 11);

  uint16_t fps = 0;
  DTSError result = sensor.getFrameRate(fps, 50);
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "One-shot AUTO CRC accepts LSB-ordered response by switching order");
  testFramework.assertEqual(10, static_cast<int>(fps), "One-shot AUTO CRC parses fps after switching");
  testFramework.assertEqual(static_cast<int>(DTSCRCByteOrder::LSB_THEN_MSB), static_cast<int>(sensor.getActiveCRCByteOrder()), "One-shot AUTO adopts LSB->MSB order after success");
}

void testEsp32PinPersistence() {
  Serial.println("Running ESP32 Pin Persistence Tests...");

  MockSerial mock;   // local instance records begin() pins
  DTSConfig config;
  DTS6012M_UART sensor(mock, config);

  // begin() with explicit RX/TX pins must forward them.
  sensor.begin(0, 16, 17);
  testFramework.assertEqual(16, mock.lastRxPin, "begin() forwards RX pin");
  testFramework.assertEqual(17, mock.lastTxPin, "begin() forwards TX pin");

  // A configure() that changes the baud rate re-opens the port; it must reuse
  // the stored pins, not silently drop back to core defaults.
  DTSConfig changed = config;
  changed.baudRate = 115200;
  sensor.configure(changed);
  testFramework.assertEqual(16, mock.lastRxPin, "configure() re-begin preserves RX pin");
  testFramework.assertEqual(17, mock.lastTxPin, "configure() re-begin preserves TX pin");
}

void testCircularBufferOverflowReported() {
  Serial.println("Running Circular Buffer Overflow Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  // Feed more bytes than the 128-byte ring can hold (127 usable) in one update().
  byte flood[130];
  for (int i = 0; i < 130; i++) flood[i] = 0x00;   // no valid header among them
  mockSerial.mockIncomingData(flood, 130);

  sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::BUFFER_OVERFLOW), static_cast<int>(sensor.getLastError()), "Ring overflow is reported via getLastError()");
  DTSStatistics stats = sensor.getStatistics();
  testFramework.assertTrue(stats.errorCount >= 1, "Ring overflow increments the error count");
}

void testConsecutiveErrorsCounter() {
  Serial.println("Running getConsecutiveErrors Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  testFramework.assertEqual(0, static_cast<int>(sensor.getConsecutiveErrors()), "Consecutive errors start at zero");

  // Two bad frames (invalid device number) in a row.
  byte bad[23];
  createValidFrame(bad);
  bad[1] = 0xFF;
  updateFrameCRC(bad);
  mockSerial.mockIncomingData(bad, 23);
  sensor.update();
  mockSerial.mockIncomingData(bad, 23);
  sensor.update();
  testFramework.assertEqual(2, static_cast<int>(sensor.getConsecutiveErrors()), "Consecutive errors count two bad frames in a row");

  // A valid frame resets the counter.
  byte good[23];
  createValidFrame(good);
  mockSerial.mockIncomingData(good, 23);
  sensor.update();
  testFramework.assertEqual(0, static_cast<int>(sensor.getConsecutiveErrors()), "Consecutive errors reset after a valid frame");
}

void testLegacyLargePayloadShortWriteReported() {
  Serial.println("Running Legacy Large-Payload Short-Write Tests...");

  ShortWriteMockSerial mock;
  DTS6012M_UART sensor(mock);
  sensor.clearError();

  // Payload > 23 bytes takes the malloc path; the mock reports a short write.
  byte payload[24];
  for (int i = 0; i < 24; i++) payload[i] = static_cast<byte>(i);
  sensor.sendCommand(DTS_CMD_SET_FRAME_RATE, payload, 24);

  testFramework.assertEqual(static_cast<int>(DTSError::SERIAL_INIT_FAILED), static_cast<int>(sensor.getLastError()), "Legacy large-payload short write is reported");
}

void testRawZeroDistanceStaysInvalidWithOffset() {
  Serial.println("Running Raw-Zero Distance Calibration Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  // Host geometry offset, as configured by the MRF2 firmware (400 mm).
  sensor.setDistanceOffset(400);

  // A no-return frame reporting raw distance 0 must stay invalid: applying
  // the offset to it would fabricate a plausible-looking 400 mm reading.
  byte frame[23];
  createValidFrame(frame);
  frame[13] = 0x00; frame[14] = 0x00; // Primary distance = 0 (no return)
  updateFrameCRC(frame);
  mockSerial.mockIncomingData(frame, 23);
  sensor.update();
  testFramework.assertEqual(static_cast<int>(DTS_INVALID_DISTANCE),
                            static_cast<int>(sensor.getDistance()),
                            "Raw 0 distance stays invalid despite offset");

  // A genuine reading still gets the offset applied.
  createValidFrame(frame); // 1000 mm
  mockSerial.mockIncomingData(frame, 23);
  sensor.update();
  testFramework.assertEqual(1400, static_cast<int>(sensor.getDistance()),
                            "Real reading still calibrated with offset");
}

void testAutoCRCDetectionSurvivesHostRecovery() {
  Serial.println("Running AUTO CRC Detection Across Host Recovery Tests...");

  DTSConfig config = {
    .baudRate = 921600,
    .timeout_ms = 1000,
    .crcEnabled = true,
    .maxValidDistance_mm = 18000,
    .minValidDistance_mm = 20,
    .minIntensityThreshold = 100,
    .crcByteOrder = DTSCRCByteOrder::AUTO,
    .crcAutoSwitchErrorThreshold = 3
  };

  DTS6012M_UART sensor(mockSerial, config);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  byte lsbFirstFrame[23];
  createValidFrame(lsbFirstFrame, DTSCRCByteOrder::LSB_THEN_MSB);

  // Two CRC failures, then the host's error-recovery path runs (the MRF2
  // firmware calls clearError() + resetState() every few CRC errors). If
  // recovery wipes the byte-order streak, AUTO detection can never reach its
  // threshold on a LSB-first sensor variant and the host recovery-loops
  // forever.
  mockSerial.mockIncomingData(lsbFirstFrame, 23);
  sensor.update();
  mockSerial.mockIncomingData(lsbFirstFrame, 23);
  sensor.update();

  sensor.clearError();
  sensor.resetState();

  // Third consecutive CRC failure must still reach the threshold and switch.
  mockSerial.mockIncomingData(lsbFirstFrame, 23);
  DTSError result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result),
                            "AUTO switch threshold survives host clearError/resetState");
  testFramework.assertEqual(static_cast<int>(DTSCRCByteOrder::LSB_THEN_MSB),
                            static_cast<int>(sensor.getActiveCRCByteOrder()),
                            "AUTO mode switched after host recovery");
}

void testMedianHistoryClearedByResetAndDisable() {
  Serial.println("Running Median History Reset Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  byte frame[23];
  createValidFrame(frame); // 1000 mm
  for (int i = 0; i < 5; i++) {
    mockSerial.mockIncomingData(frame, 23);
    sensor.update();
  }
  testFramework.assertEqual(1000, static_cast<int>(sensor.getFilteredDistance()),
                            "Median filter primed with history");

  // Host recovery resets state: stale history must not survive to bias the
  // first readings after the sensor comes back (previous-subject median).
  sensor.resetState();
  testFramework.assertEqual(static_cast<int>(DTS_INVALID_DISTANCE),
                            static_cast<int>(sensor.getFilteredDistance()),
                            "resetState clears median history");

  // Re-prime, then standby via disableSensor: same requirement on wake.
  for (int i = 0; i < 5; i++) {
    mockSerial.mockIncomingData(frame, 23);
    sensor.update();
  }
  testFramework.assertEqual(1000, static_cast<int>(sensor.getFilteredDistance()),
                            "Median filter re-primed");
  sensor.disableSensor();
  testFramework.assertEqual(static_cast<int>(DTS_INVALID_DISTANCE),
                            static_cast<int>(sensor.getFilteredDistance()),
                            "disableSensor clears median history");
}

void testTimeoutLivelockRecovery() {
  Serial.println("Running Timeout Livelock Recovery Tests...");

  DTSConfig config;
  config.timeout_ms = 1;   // tiny window: every poll during reassembly is "timed out"
  DTS6012M_UART sensor(mockSerial, config);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  // Enter a genuine timeout with no data.
  delay(3);
  DTSError r = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::TIMEOUT), static_cast<int>(r),
                            "Livelock: enters TIMEOUT after a stall");

  // Sensor resumes but delivers a frame in <23-byte chunks, one per update(),
  // while still inside the timeout window. The old code wiped the ring on every
  // timed-out update() and could never reassemble; the fix leaves an actively
  // arriving partial frame intact so it completes and the driver recovers.
  byte frame[23];
  createValidFrame(frame);
  DTSError last = DTSError::TIMEOUT;
  for (int off = 0; off < 23; off += 5) {
    int n = (off + 5 <= 23) ? 5 : (23 - off);
    delay(2);   // stay past the 1 ms timeout window on each call
    mockSerial.mockIncomingData(frame + off, n);
    last = sensor.update();
  }
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(last),
                            "Livelock: recovers once the chunked frame completes");
  testFramework.assertEqual(1000, static_cast<int>(sensor.getDistance()),
                            "Livelock: recovered frame parsed correctly");
}

void testTimeoutCountedInStatistics() {
  Serial.println("Running Timeout Accounting Tests...");

  DTSConfig config;
  config.timeout_ms = 1;
  DTS6012M_UART sensor(mockSerial, config);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  delay(3);
  sensor.update();   // first timed-out call: edge of the stall episode
  DTSStatistics stats = sensor.getStatistics();
  testFramework.assertEqual(1, static_cast<int>(stats.errorCount),
                            "Timeout is counted in errorCount");
  testFramework.assertEqual(1, static_cast<int>(sensor.getConsecutiveErrors()),
                            "Timeout bumps consecutiveErrors");

  // A second timed-out call in the same stall episode must NOT keep incrementing
  // (one count per episode, not one per call).
  delay(3);
  sensor.update();
  stats = sensor.getStatistics();
  testFramework.assertEqual(1, static_cast<int>(stats.errorCount),
                            "Timeout counted once per stall episode, not per call");
}

void testEnableSensorRefreshesTimeoutClock() {
  Serial.println("Running enableSensor Timeout-Clock Tests...");

  DTSConfig config;
  config.timeout_ms = 5;
  DTS6012M_UART sensor(mockSerial, config);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  // Standby for longer than the timeout window, then re-enable.
  sensor.disableSensor();
  delay(10);
  sensor.enableSensor();

  // The first update() after re-enable must NOT report a spurious TIMEOUT from a
  // healthy sensor — enableSensor() refreshes the timeout clock like begin() does.
  DTSError r = sensor.update();
  testFramework.assertTrue(r != DTSError::TIMEOUT,
                           "enableSensor refreshes the timeout clock (no false TIMEOUT)");
}

void testBatchedParseErrorIsCounted() {
  Serial.println("Running Batched Parse-Error Accounting Tests...");

  DTS6012M_UART sensor(mockSerial);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  // One corrupt frame (bad device number) immediately followed by a valid frame,
  // delivered together in a single update() drain. The corrupt frame must still
  // be counted even though the good frame that follows it parses successfully.
  byte batch[46];
  createValidFrame(batch);
  batch[1] = 0xFF;                 // corrupt the first frame's device number
  updateFrameCRC(batch);
  createValidFrame(batch + 23);    // second frame is valid (1000 mm)
  mockSerial.mockIncomingData(batch, 46);

  DTSError r = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(r),
                            "Batched drain returns NONE (a good frame parsed)");
  testFramework.assertEqual(1000, static_cast<int>(sensor.getDistance()),
                            "Freshest good frame wins in a mixed batch");
  DTSStatistics stats = sensor.getStatistics();
  testFramework.assertEqual(1, static_cast<int>(stats.errorCount),
                            "Corrupt frame batched with a good frame is still counted");
}

void testOneShotRespectsDisabledStream() {
  Serial.println("Running One-Shot Stream-State Restore Tests...");

  OneShotMockSerial mock;
  DTSConfig config;
  config.crcEnabled = false;
  DTS6012M_UART sensor(mock, config);
  sensor.begin();

  // Valid GET_FRAME_RATE response so the one-shot succeeds.
  byte resp[11] = {0xA5, 0x03, 0x20, 0x1B, 0x00, 0x00, 0x02, 0x00, 0x0A, 0x00, 0x00};
  mock.setResponse(static_cast<byte>(DTSCommand::GET_FRAME_RATE), resp, 11);

  // Host puts the sensor in standby, then issues a one-shot query.
  sensor.disableSensor();
  int startsBefore = mock.startStreamWrites;
  uint16_t fps = 0;
  sensor.getFrameRate(fps, 50);

  // The one-shot must NOT have re-enabled the stream the host disabled.
  testFramework.assertEqual(startsBefore, mock.startStreamWrites,
                            "One-shot does not restart a host-disabled stream");

  // While enabled, the same one-shot SHOULD restore the stream.
  sensor.enableSensor();
  startsBefore = mock.startStreamWrites;
  sensor.getFrameRate(fps, 50);
  testFramework.assertTrue(mock.startStreamWrites > startsBefore,
                           "One-shot restarts the stream when the host had it enabled");
}

void testStaleDataInvalidatedOnTimeout() {
  Serial.println("Running Stale-Data-On-Timeout Tests...");

  DTSConfig config;
  config.timeout_ms = 1;
  DTS6012M_UART sensor(mockSerial, config);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  // Prime with several valid frames so the getters and median filter are live.
  byte frame[23];
  createValidFrame(frame); // 1000 mm
  for (int i = 0; i < 5; i++) {
    mockSerial.mockIncomingData(frame, 23);
    sensor.update();
  }
  testFramework.assertTrue(sensor.isDataValid(), "Primed data reads as valid");
  testFramework.assertEqual(1000, static_cast<int>(sensor.getDistance()), "Primed distance present");
  testFramework.assertEqual(1000, static_cast<int>(sensor.getFilteredDistance()), "Primed median present");

  // Sensor goes silent past the timeout window.
  delay(3);
  DTSError r = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::TIMEOUT), static_cast<int>(r), "Stall reports TIMEOUT");

  // The last good frame must no longer be presented as current/valid.
  testFramework.assertTrue(!sensor.isDataValid(), "isDataValid() false after comms loss");
  testFramework.assertEqual(static_cast<int>(DTS_INVALID_DISTANCE), static_cast<int>(sensor.getDistance()),
                            "getDistance() returns sentinel after comms loss");
  testFramework.assertEqual(static_cast<int>(DTS_INVALID_DISTANCE), static_cast<int>(sensor.getFilteredDistance()),
                            "getFilteredDistance() returns sentinel after comms loss");
}

void testSendOneShotRejectsOversizedLength() {
  Serial.println("Running One-Shot Oversized-Length Tests...");

  OneShotMockSerial mock;
  DTSConfig config;
  config.crcEnabled = false;
  DTS6012M_UART sensor(mock, config);
  sensor.begin();

  // Response header claims a corrupted, huge data length (0x8000 = 32768) but
  // only a few bytes follow. The frame-size math must not overflow (it is
  // computed in a 32-bit type; on 16-bit-int AVR the old int math wrapped
  // negative and bypassed the truncation guard, driving an OOB read). The
  // truncation guard must fire and report TIMEOUT, never success-with-garbage.
  byte resp[9] = {0xA5, 0x03, 0x20, 0x1B, 0x00, 0x80, 0x00, 0xAA, 0xBB};
  mock.setResponse(static_cast<byte>(DTSCommand::GET_FRAME_RATE), resp, 9);

  uint16_t fps = 0;
  DTSError r = sensor.getFrameRate(fps, 50);
  testFramework.assertEqual(static_cast<int>(DTSError::TIMEOUT), static_cast<int>(r),
                            "Oversized response length returns TIMEOUT, not garbage success");
}

// Main test runner
void runAllTests() {
  Serial.println("==========================================");
  Serial.println("Starting DTS6012M_UART Library Tests");
  Serial.println("==========================================");
  
  testConstructorAndConfiguration();
  testInitialization();
  testFrameParsing();
  testErrorHandling();
  testCRCByteOrderModes();
  testAutoCRCByteOrderSwitch();
  testDataQuality();
  testStatistics();
  testCalibration();
  testCommandSending();
  testLegacyCommandApiCompatibility();
  testNewDataAvailable();
  testHasSecondaryTarget();
  testFilteredDistance();
  testPartialFrameDelivery();
  testResetAndFactoryBehavior();
  testDrainFreshestFrame();
  testSendOneShotTruncatedResponse();
  testGetFrameRateUnparseable();
  testGetFrameRateSuccess();
  testSetFrameRateLargeAck();
  testWriteIICRegisterLargeAck();
  testUpdateNoNewData();
  testUpdateRealTimeout();
  testOneShotAutoCRCSwitch();
  testEsp32PinPersistence();
  testCircularBufferOverflowReported();
  testConsecutiveErrorsCounter();
  testLegacyLargePayloadShortWriteReported();
  testRawZeroDistanceStaysInvalidWithOffset();
  testAutoCRCDetectionSurvivesHostRecovery();
  testMedianHistoryClearedByResetAndDisable();
  testTimeoutLivelockRecovery();
  testTimeoutCountedInStatistics();
  testEnableSensorRefreshesTimeoutClock();
  testBatchedParseErrorIsCounted();
  testOneShotRespectsDisabledStream();
  testStaleDataInvalidatedOnTimeout();
  testSendOneShotRejectsOversizedLength();

  testFramework.printSummary();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  delay(1000); // Allow time for serial monitor to connect
  
  runAllTests();
}

void loop() {
  // Tests run once in setup()
}

#ifndef ARDUINO
int main() {
  setup();
  // Non-zero exit on any failure so CI (and TESTING.md's claim) actually hold.
  return testFramework.failedCount() > 0 ? 1 : 0;
}
#endif
