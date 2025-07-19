#ifndef DTS6012M_UART_H
#define DTS6012M_UART_H

#include <Arduino.h>
#include <HardwareSerial.h> // Ensure this is included

namespace DTS6012M {

// --- Constants based on datasheet ---
namespace Protocol {
  // Protocol constants
  constexpr byte HEADER = 0xA5;
  constexpr byte DEVICE_NO = 0x03;
  constexpr byte DEVICE_TYPE = 0x20; // Example type from datasheet

  // Command codes
  constexpr byte CMD_START_STREAM = 0x01;
  constexpr byte CMD_STOP_STREAM = 0x02;
  // Add other command codes if needed (e.g., 0x0A, 0x10, 0x1A, etc.)
  // constexpr byte CMD_GET_VERSION = 0x0A;
  // constexpr byte CMD_SET_BAUD = 0x10;
  // constexpr byte CMD_GET_BAUD = 0x11;
  // constexpr byte CMD_SET_I2C_ADDR = 0x12;
  // constexpr byte CMD_GET_I2C_ADDR = 0x13;
  // constexpr byte CMD_SET_FRAME_RATE = 0x1A;
  // constexpr byte CMD_GET_FRAME_RATE = 0x1B;
}

namespace Frame {
  // Response frame structure constants for start_stream command (0x01)
  constexpr int RESPONSE_FRAME_LENGTH = 23; // Full frame size: Header(1)+DevNo(1)+DevType(1)+CMD(1)+Res(1)+Len(2)+Data(14)+CRC(2)
  constexpr int DATA_LENGTH_EXPECTED = 14;  // Expected data payload size

  // Frame structure constants
  constexpr int HEADER_SIZE = 7;      // Header to end of length field
  constexpr int CRC_SIZE = 2;         // CRC field size
  constexpr int DATA_PAYLOAD_OFFSET = 7;    // Offset to data payload in frame
  constexpr int MAX_FRAME_SIZE = 64;        // Maximum supported frame size
}

namespace Timing {
  // Timing constants
  constexpr unsigned long SERIAL_STABILIZATION_DELAY_MS = 10;  // Delay after serial.begin()
  constexpr unsigned long COMMUNICATION_TIMEOUT_MS = 1000;     // Communication timeout
}

namespace PayloadIndex {
  // Indices within the 14-byte data payload (LSB first ordering)
  constexpr int SEC_DIST = 0; // Secondary Target Distance (2 bytes)
  constexpr int SEC_CORR = 2; // Secondary Target Correction (2 bytes)
  constexpr int SEC_INT = 4;  // Secondary Target Intensity (2 bytes)
  constexpr int PRI_DIST = 6; // Primary Target Distance (2 bytes)
  constexpr int PRI_CORR = 8; // Primary Target Correction (2 bytes)
  constexpr int PRI_INT = 10; // Primary Target Intensity (2 bytes)
  constexpr int SUN_BASE = 12;// Sunlight Base (2 bytes)
}

class UART {
public:
  // --- Public Methods ---

  // Constructor: Takes a reference to the HardwareSerial port object (e.g., Serial1, Serial2)
  UART(HardwareSerial &serialPort);

  // Initialization: Starts serial communication at the specified baud rate
  // Sends the 'Start Stream' command automatically.
  // Returns true on success, false if the serial port failed to start.
  bool begin(unsigned long baudRate = 921600); // Default baud rate from datasheet

  // Update: Processes incoming serial data. Call this frequently in your loop().
  // Returns true if a new, valid measurement frame was received and parsed, false otherwise.
  bool update();

  // --- Data Getters ---
  // These return the values from the last successfully parsed measurement frame.

  uint16_t getDistance();           // Primary target distance (mm). Returns 0xFFFF if no target detected or invalid.
  uint16_t getIntensity();          // Primary target intensity/signal strength.
  uint16_t getSunlightBase();       // Ambient sunlight base level.
  uint16_t getCorrection();         // Primary target correction value (meaning may vary).
  uint16_t getSecondaryDistance();  // Secondary target distance (mm). Returns 0xFFFF if no target detected or invalid.
  uint16_t getSecondaryIntensity(); // Secondary target intensity/signal strength.
  uint16_t getSecondaryCorrection();// Secondary target correction value.
  
  // Check if new data is available since the last read.
  bool isDataNew() const;

  // --- Advanced Methods ---

  // Send Command: Allows sending other commands defined in the datasheet (e.g., stop stream, set baud).
  // cmd: The command code (e.g., Protocol::CMD_STOP_STREAM).
  // dataPayload: Pointer to byte array containing data for the command (if any).
  // payloadLength: Number of bytes in dataPayload.
  void sendCommand(byte cmd, const byte *dataPayload = NULL, uint16_t payloadLength = 0);

  // Enable/Disable CRC Check: Controls whether the CRC checksum is validated on incoming frames.
  // Disabling improves performance but risks accepting corrupted data. Enabled by default.
  void enableCRC(bool enable);

  // Enable Sensor: Starts the measurement stream from the sensor.
  void enableSensor();

  // Disable Sensor: Stops the measurement stream from the sensor.
  void disableSensor();

  // Set an optional stream for debug logging
  void setDebugStream(Stream* debugStream);

private:
  // --- Private Members ---
  HardwareSerial &_serial; // Reference to the hardware serial port instance
  Stream* _debugStream = nullptr; // Optional stream for debug output

  byte _rxBuffer[Frame::RESPONSE_FRAME_LENGTH]; // Buffer to hold incoming frame bytes
  int _rxBufferIndex;                        // Current position in the rx buffer
  unsigned long _lastUpdateTime;             // Timestamp of the last byte received (for potential timeout logic)

  // Variables to store the latest valid measurement data
  uint16_t _distancePrimary_mm;
  uint16_t _intensityPrimary;
  uint16_t _correctionPrimary;
  uint16_t _distanceSecondary_mm;
  uint16_t _intensitySecondary;
  uint16_t _correctionSecondary;
  uint16_t _sunlightBase;
  volatile bool _newDataAvailable; // Flag to indicate new data is ready
  bool _crcCheckEnabled;  // Flag to control CRC validation


  // --- Private Helper Methods ---

  // parseFrame: Validates and extracts data from the completed buffer.
  // Returns true if the frame is valid (correct format, length, CRC), false otherwise.
  bool parseFrame();

  // calculateCRC16: Computes the Modbus CRC-16 checksum for given data.
  uint16_t calculateCRC16(const byte *data, int len);

  // extractUint16LSB: Helper function to extract 16-bit value from buffer (LSB first)
  uint16_t extractUint16LSB(int offset) const;
};

} // namespace DTS6012M

#endif // DTS6012M_UART_H
