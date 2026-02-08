#include <Arduino.h>
#define DTS6012M_TEST_MODE
#include "DTS6012M_UART.h"

// Mock Serial class for testing
class MockSerial : public HardwareSerial {
private:
  byte _mockBuffer[256];
  int _mockBufferIndex;
  int _mockReadIndex;
  bool _isAvailable;
  
public:
  MockSerial() : _mockBufferIndex(0), _mockReadIndex(0), _isAvailable(true) {}
  
  void begin(unsigned long) override {
    _isAvailable = true;
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

void updateFrameCRC(byte *frame, DTSCRCByteOrder order = DTSCRCByteOrder::LSB_THEN_MSB) {
  uint16_t crc = calculateTestCRC16(frame, 21);

  if (order == DTSCRCByteOrder::MSB_THEN_LSB) {
    frame[21] = (crc >> 8) & 0xFF;  // CRC MSB
    frame[22] = crc & 0xFF;         // CRC LSB
    return;
  }

  frame[21] = crc & 0xFF;         // CRC LSB
  frame[22] = (crc >> 8) & 0xFF;  // CRC MSB
}

void createValidFrame(byte *frame, DTSCRCByteOrder order = DTSCRCByteOrder::LSB_THEN_MSB) {
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
  frame[9] = 0x00; frame[10] = 0x00;  // Secondary Correction (0)
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

  // Default mode should accept LSB->MSB CRC.
  createValidFrame(frame, DTSCRCByteOrder::LSB_THEN_MSB);
  mockSerial.mockIncomingData(frame, 23);
  DTSError result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "Default mode accepts LSB->MSB CRC");

  // Fixed MSB->LSB mode should accept MSB->LSB CRC.
  sensor.setCRCByteOrder(DTSCRCByteOrder::MSB_THEN_LSB);
  createValidFrame(frame, DTSCRCByteOrder::MSB_THEN_LSB);
  mockSerial.mockIncomingData(frame, 23);
  result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "MSB->LSB mode accepts MSB->LSB CRC");

  // Fixed MSB->LSB mode should reject LSB->MSB CRC.
  createValidFrame(frame, DTSCRCByteOrder::LSB_THEN_MSB);
  mockSerial.mockIncomingData(frame, 23);
  result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::CRC_CHECK_FAILED), static_cast<int>(result), "MSB->LSB mode rejects LSB->MSB CRC");
}

void testAutoCRCByteOrderSwitch() {
  Serial.println("Running AUTO CRC Byte Order Switching Tests...");

  DTSConfig config = {
    .baudRate = 921600,
    .timeout_ms = 1000,
    .crcEnabled = true,
    .maxValidDistance_mm = 20000,
    .minValidDistance_mm = 30,
    .minIntensityThreshold = 100,
    .crcByteOrder = DTSCRCByteOrder::AUTO,
    .crcAutoSwitchErrorThreshold = 3
  };

  DTS6012M_UART sensor(mockSerial, config);
  sensor.begin();
  sensor.resetState();
  mockSerial.resetMock();

  testFramework.assertEqual(static_cast<int>(DTSCRCByteOrder::LSB_THEN_MSB),
                            static_cast<int>(sensor.getActiveCRCByteOrder()),
                            "AUTO mode starts with LSB->MSB");

  byte msbFirstFrame[23];
  createValidFrame(msbFirstFrame, DTSCRCByteOrder::MSB_THEN_LSB);

  // First two frames should fail while AUTO mode is still trying LSB->MSB.
  mockSerial.mockIncomingData(msbFirstFrame, 23);
  DTSError result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::CRC_CHECK_FAILED), static_cast<int>(result), "AUTO frame #1 fails before switch");

  mockSerial.mockIncomingData(msbFirstFrame, 23);
  result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::CRC_CHECK_FAILED), static_cast<int>(result), "AUTO frame #2 fails before switch");

  // Third consecutive failure reaches threshold, flips order, and retries same frame.
  mockSerial.mockIncomingData(msbFirstFrame, 23);
  result = sensor.update();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "AUTO frame #3 triggers switch and succeeds");

  testFramework.assertEqual(static_cast<int>(DTSCRCByteOrder::MSB_THEN_LSB),
                            static_cast<int>(sensor.getActiveCRCByteOrder()),
                            "AUTO mode switches to MSB->LSB");
}

void testDataQuality() {
  Serial.println("Running Data Quality Tests...");
  
  DTSConfig config = {
    .baudRate = 921600,
    .timeout_ms = 1000,
    .crcEnabled = true,
    .maxValidDistance_mm = 20000,
    .minValidDistance_mm = 30,
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
  testFramework.assertEqual(static_cast<int>(crc & 0xFF), static_cast<int>(frame[frameLen - 2]), "Command CRC default LSB byte");
  testFramework.assertEqual(static_cast<int>((crc >> 8) & 0xFF), static_cast<int>(frame[frameLen - 1]), "Command CRC default MSB byte");

  mockSerial.resetMock();
  sensor.setCRCByteOrder(DTSCRCByteOrder::MSB_THEN_LSB);
  result = sensor.enableSensor();
  testFramework.assertEqual(static_cast<int>(DTSError::NONE), static_cast<int>(result), "Enable sensor command with MSB->LSB CRC order");
  frame = mockSerial.getSentData();
  frameLen = mockSerial.getSentDataLength();
  crc = calculateTestCRC16(frame, frameLen - 2);
  testFramework.assertEqual(static_cast<int>((crc >> 8) & 0xFF), static_cast<int>(frame[frameLen - 2]), "Command CRC MSB byte in MSB->LSB mode");
  testFramework.assertEqual(static_cast<int>(crc & 0xFF), static_cast<int>(frame[frameLen - 1]), "Command CRC LSB byte in MSB->LSB mode");
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
  testResetAndFactoryBehavior();
  
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
  return 0;
}
#endif
