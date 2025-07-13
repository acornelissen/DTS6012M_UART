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
 * Note: Use a HardwareSerial port (Serial1, Serial2, etc.) as the default
 * baud rate (921600) is too high for SoftwareSerial.
 */

// Select the HardwareSerial port connected to the sensor
HardwareSerial &SensorSerial = Serial1; // Change to Serial2 etc. if needed

// Create an instance of the sensor library, passing the chosen serial port
DTS6012M_UART dtsSensor(SensorSerial);

// --- Timing for printing data and demo ---
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 100; // How often to print results (milliseconds)

// --- Enable/Disable demo timing ---
unsigned long lastToggleTime = 0;
const unsigned long TOGGLE_INTERVAL_MS = 5000; // Toggle sensor every 5 seconds
bool sensorEnabled = true;

void setup() {
  // Initialize the Serial Monitor for output/debugging
  Serial.begin(115200); // Use a standard baud rate for the monitor
  while (!Serial);      // Wait for Serial Monitor to open (optional)
  Serial.println("--- DTS6012M UART Library Example ---");
  Serial.println("Attempting to initialize sensor...");

  // Initialize the sensor library. This starts Serial1 (or your chosen port)
  // at 921600 baud (default) and sends the start stream command.
  if (dtsSensor.begin()) {
    Serial.println("Sensor initialization successful.");
  } else {
    Serial.println("ERROR: Sensor initialization failed! Check wiring and Serial port selection.");
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
  // It returns true if a new valid measurement was parsed in this call.
  bool newDataReceived = dtsSensor.update();

  // Check if new data is available AND if enough time has passed since the last print
  if (newDataReceived && (currentTime - lastPrintTime >= PRINT_INTERVAL_MS)) {
    lastPrintTime = currentTime; // Update the last print time

    // Get the latest data using the library's getter functions
    uint16_t distance_mm = dtsSensor.getDistance();
    uint16_t intensity = dtsSensor.getIntensity();
    // uint16_t sunlight = dtsSensor.getSunlightBase(); // Uncomment if needed
    // uint16_t sec_distance_mm = dtsSensor.getSecondaryDistance(); // Uncomment if needed
    // uint16_t sec_intensity = dtsSensor.getSecondaryIntensity(); // Uncomment if needed

    // Print the primary target data
    Serial.print("Primary Dist: ");
    if (distance_mm == 0xFFFF) {
      // 0xFFFF often indicates an invalid reading or no target detected
      Serial.print("---- mm");
    } else {
      Serial.print(distance_mm);
      Serial.print(" mm");
    }

    Serial.print("\t Intensity: ");
    Serial.print(intensity);

    // Example of printing secondary target data (optional)
    /*
    Serial.print("\t | Secondary Dist: ");
    if (sec_distance_mm == 0xFFFF) {
       Serial.print("---- mm");
    } else {
       Serial.print(sec_distance_mm);
       Serial.print(" mm");
    }
    Serial.print("\t Intensity: ");
    Serial.print(sec_intensity);
    */

    Serial.println(); // New line for next reading

  } // End of printing block

  // --- Your other application code can go here ---
  // The loop should run quickly to allow dtsSensor.update() to be called frequently.
  // Avoid long delays in the main loop if possible.
  // delay(1); // A very small delay might be acceptable if needed

} // End of loop()
