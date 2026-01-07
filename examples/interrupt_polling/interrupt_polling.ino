/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * M5IOE1 Interrupt Polling Example
 *
 * This example demonstrates how to use the M5IOE1 library in POLLING mode
 * without a physical interrupt pin. The library periodically polls the
 * interrupt status registers to detect GPIO changes.
 *
 * Hardware Connections:
 * - M5IOE1 SDA -> GPIO 38 (default, configurable)
 * - M5IOE1 SCL -> GPIO 39 (default, configurable)
 * - IO1 (M5IOE1) -> Button or switch (connect to GND for active LOW)
 *
 * Features demonstrated:
 * - Initializing M5IOE1 in polling mode (no INT pin)
 * - Attaching interrupt callback to pin 1 (IO1) with FALLING edge trigger
 * - Reading device information (UID, version)
 * - Using attachInterrupt() callback for event handling
 * - Polling interval configuration
 */

#include <M5IOE1.h>

// M5IOE1 device instance
M5IOE1 ioe1;

// I2C configuration
#define I2C_SDA_PIN 38
#define I2C_SCL_PIN 39
#define I2C_FREQ 400000

// M5IOE1 I2C address (default 0x6F)
#define I2C_ADDR M5IOE1_DEFAULT_ADDR

// Pin definitions
#define IOE1_PIN_1 M5IOE1_PIN_1  // IO1 on M5IOE1 (pin index 0)

// Counter for interrupt events
volatile int interruptCounter = 0;

// Callback function for pin 1 falling edge interrupt
void IRAM_ATTR pin1FallingCallback() {
    interruptCounter++;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n========================================");
    Serial.println("M5IOE1 Interrupt Polling Example");
    Serial.println("========================================\n");

    // Set log level to INFO (default)
    M5IOE1::setLogLevel(M5IOE1_LOG_LEVEL_INFO);

    // Initialize M5IOE1 in POLLING mode (no physical INT pin)
    // When intPin is not provided (or set to -1), only POLLING and DISABLED modes are supported
    Serial.println("Initializing M5IOE1 in polling mode...");

    if (!ioe1.begin(&Wire, I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ, M5IOE1_INT_MODE_POLLING)) {
        Serial.println("ERROR: Failed to initialize M5IOE1!");
        Serial.println("Please check:");
        Serial.println("  - I2C connections (SDA=" + String(I2C_SDA_PIN) + ", SCL=" + String(I2C_SCL_PIN) + ")");
        Serial.println("  - M5IOE1 I2C address (0x" + String(I2C_ADDR, HEX) + ")");
        Serial.println("  - Power supply to M5IOE1");
        while (1) { delay(1000); }
    }

    Serial.println("M5IOE1 initialized successfully!\n");

    // Read and display device information
    uint16_t uid;
    uint8_t version;

    if (ioe1.getUID(&uid)) {
        Serial.println("Device UID: 0x" + String(uid, HEX));
    }

    if (ioe1.getVersion(&version)) {
        Serial.println("Firmware Version: " + String(version));
    }

    uint16_t refVoltage;
    if (ioe1.getRefVoltage(&refVoltage)) {
        Serial.println("Reference Voltage: " + String(refVoltage) + " mV");
    }

    Serial.println();

    // Configure IO1 as input with pull-up
    Serial.println("Configuring IO1 as input with internal pull-up...");
    ioe1.pinMode(IOE1_PIN_1, INPUT_PULLUP);
    Serial.println("IO1 configured successfully!\n");

    // Attach interrupt to pin 1 with FALLING edge trigger
    // In polling mode, the library will periodically check the interrupt status
    // and call this callback when a falling edge is detected on IO1
    Serial.println("Attaching interrupt callback to IO1 (FALLING edge)...");
    ioe1.attachInterrupt(IOE1_PIN_1, pin1FallingCallback, FALLING);
    Serial.println("Interrupt attached successfully!\n");

    // Set polling interval to 1 second (1000ms)
    // Default is 5000ms. Shorter intervals = faster response but more CPU usage.
    Serial.println("Setting polling interval to 1.0 second...");
    if (ioe1.setPollingInterval(1.0f)) {
        Serial.println("Polling interval configured successfully!\n");
    } else {
        Serial.println("WARNING: Failed to set polling interval\n");
    }

    Serial.println("========================================");
    Serial.println("Setup complete! Ready for interrupts.");
    Serial.println("========================================\n");

    Serial.println("Instructions:");
    Serial.println("  - Connect a button between IO1 and GND");
    Serial.println("  - Press the button to trigger FALLING edge interrupt");
    Serial.println("  - The library polls interrupt status every 1 second");
    Serial.println("  - Watch for interrupt events in the serial monitor\n");
}

void loop() {
    // Store current counter value to detect changes
    static int lastCounter = 0;

    // Check if interrupt counter changed
    if (interruptCounter != lastCounter) {
        Serial.println(">>> INTERRUPT TRIGGERED! Count: " + String(interruptCounter) + " <<<");
        lastCounter = interruptCounter;

        // Read current state of IO1
        int pinState = ioe1.digitalRead(IOE1_PIN_1);
        Serial.println("    IO1 state: " + String(pinState == LOW ? "LOW (pressed)" : "HIGH (released)"));

        // Read and display interrupt status register
        uint16_t status = ioe1.getInterruptStatus();
        Serial.println("    Interrupt status register: 0b" + String(status, BIN) + "\n");

        // Clear the interrupt for this pin
        ioe1.clearInterrupt(IOE1_PIN_1);
    }

    // Optional: Manual polling check (the library does this automatically in background)
    // This is just for demonstration - the library handles polling automatically
    static unsigned long lastStatusCheck = 0;
    if (millis() - lastStatusCheck > 5000) {
        lastStatusCheck = millis();

        // Display current polling mode info
        Serial.println("--- Status Update ---");
        Serial.println("    Total interrupts: " + String(interruptCounter));
        Serial.println("    IO1 current state: " + String(ioe1.digitalRead(IOE1_PIN_1) == LOW ? "LOW" : "HIGH"));
        Serial.println("    Polling mode: ACTIVE (background task)\n");
    }

    delay(100);  // Small delay to prevent excessive CPU usage
}
