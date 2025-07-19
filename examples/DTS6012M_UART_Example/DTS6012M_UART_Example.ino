#include <Arduino.h>
#include "DTS6012M_UART.h" // Include the library header file

/*
 * Hardware Connections:
 * - Arduino GND <-> Sensor Pin 6 (GND)
 * - Arduino 3.3V -> Sensor Pin 1 (3V3_LASER)
 * - Arduino 3.3V -> Sensor Pin 2 (3V3)
 * - Arduino GND <-> Sensor Pin 5 (GPIO)  ** IMPORTANT: Connect GPIO to GND to enable UART mode **
 * - Arduino RX1 (e.g., Pin 19 on Mega) <- Sensor Pin 3 (UART_TX)
 * - Arduino TX1 (e.g., Pin 18 on Mega) -> Sensor Pin 4 (UART_RX)
 *
 * Note on Serial Ports:
 * It is highly recommended to use a HardwareSerial port (Serial1, Serial2, etc.)
 * because the sensor's default baud rate of 921600 is too fast for most
 * SoftwareSerial implementations, which can lead to unreliable communication.
 */

// Select the HardwareSerial port connected to the sensor
HardwareSerial &SensorSerial = Serial1; // Change to Serial2 etc. if needed

// Create an instance of the sensor library, using the new namespace
DTS6012M::UART dtsSensor(SensorSerial);

// --- Timing constants ---
const unsigned long PRINT_INTERVAL_MS = 100;  // How often to print results (milliseconds)
const unsigned long TOGGLE_INTERVAL_MS = 5000; // Toggle sensor every 5 seconds

// --- Timing variables ---
unsigned long lastPrintTime = 0;
unsigned long lastToggleTime = 0;
bool sensorEnabled = true;

void setup() {
  // Initialize the Serial Monitor for output/debugging
  Serial.begin(115200); // Use a standard baud rate for the monitor
  while (!Serial);      // Wait for Serial Monitor to open (for some boards)
  Serial.println("--- DTS6012M UART Library Example ---");

  // --- Optional: Enable Debugging ---
  // Uncomment the following line to get detailed debug information from the library
  // printed to the Serial Monitor. This is useful for troubleshooting.
  // dtsSensor.setDebugStream(&Serial);

  Serial.println("Attempting to initialize sensor...");

  // Initialize the sensor library. This starts the chosen hardware serial port
  // at 921600 baud (default) and sends the start stream command.
  if (dtsSensor.begin()) {
    Serial.println("Sensor initialization successful.");
  } else {
    Serial.println("ERROR: Sensor initialization failed! Check wiring, port selection, and power.");
    // Halt execution if sensor fails to initialize
    while (1) {
      delay(100);
    }
  }
  Serial.println("-------------------------------------");
}

void loop() {
  // --- Demo: Enable/Disable sensor every 5 seconds ---
  unsigned long currentTime = millis();
  if (currentTime - lastToggleTime >= TOGGLE_INTERVAL_MS) {
    lastToggleTime = currentTime;
    sensorEnabled = !sensorEnabled;
    
    if (sensorEnabled) {
      Serial.println("=== ENABLING SENSOR ===");
      dtsSensor.enableSensor();
    } else {
      Serial.println("=== DISABLING SENSOR ===");
      dtsSensor.disableSensor();
    }
  }

  // *** Crucial Step: Call the library's update() function frequently ***
  // This function reads incoming serial data and parses complete frames.
  dtsSensor.update();

  // Check if new data is available using the isDataNew() method.
  // This is more explicit than checking the return value of update() in the main loop.
  if (dtsSensor.isDataNew() && (currentTime - lastPrintTime >= PRINT_INTERVAL_MS)) {
    lastPrintTime = currentTime; // Update the last print time

    // Get the latest data. Calling getDistance() also clears the isDataNew() flag.
    uint16_t distance_mm = dtsSensor.getDistance();
    uint16_t intensity = dtsSensor.getIntensity();
    // uint16_t sunlight = dtsSensor.getSunlightBase(); // Uncomment if needed
    // uint16_t sec_distance_mm = dtsSensor.getSecondaryDistance(); // Uncomment if needed
    // uint16_t sec_intensity = dtsSensor.getSecondaryIntensity(); // Uncomment if needed

    // Print the primary target data
    Serial.print("Primary Dist: ");
    if (distance_mm == 0xFFFF) {
      // 0xFFFF indicates an invalid reading or no target detected
      Serial.print("---- mm");
    } else {
      Serial.print(distance_mm);
      Serial.print(" mm");
    }

    Serial.print("\t Intensity: ");
    Serial.print(intensity);

    // Uncomment to also print secondary target data:
    // uint16_t sec_distance_mm = dtsSensor.getSecondaryDistance();
    // uint16_t sec_intensity = dtsSensor.getSecondaryIntensity();
    // Serial.print("\t | Secondary: ");
    // Serial.print((sec_distance_mm == 0xFFFF) ? "---- mm" : String(sec_distance_mm) + " mm");
    // Serial.print(" ("); Serial.print(sec_intensity); Serial.print(")");

    Serial.println(); // New line for next reading

  } // End of printing block

  // The main loop should run as fast as possible to ensure the serial buffer
  // is processed promptly. Avoid using long delay() calls here.

} // End of loop()
