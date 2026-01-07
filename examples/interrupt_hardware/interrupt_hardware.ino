/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * M5IOE1 Hardware Interrupt Example
 *
 * This example demonstrates how to use the M5IOE1 library in HARDWARE
 * interrupt mode with a physical INT pin connected to GPIO1 of the host MCU.
 *
 * Hardware Connections:
 * - M5IOE1 SDA -> GPIO 38 (default, configurable)
 * - M5IOE1 SCL -> GPIO 39 (default, configurable)
 * - M5IOE1 INT -> GPIO 1 (host MCU interrupt pin)
 * - IO1 (M5IOE1) -> Button or switch (connect to GND for active LOW)
 *
 * Features demonstrated:
 * - Initializing M5IOE1 with hardware interrupt pin (INT_PIN)
 * - Using HARDWARE interrupt mode for instant response
 * - Attaching multiple interrupt callbacks to different pins
 * - Using attachInterruptArg() for callback with custom data
 * - Interrupt enable/disable control
 * - Reading interrupt status registers
 */

#include <M5IOE1.h>

// M5IOE1 device instance
M5IOE1 ioe1;

// I2C configuration
#define I2C_SDA_PIN 38
#define I2C_SCL_PIN 39
#define I2C_FREQ 400000

// Physical interrupt pin on host MCU (GPIO1 as default)
// Connect this pin to M5IOE1's INT pin
#define INT_PIN 1

// M5IOE1 I2C address (default 0x6F)
#define I2C_ADDR M5IOE1_DEFAULT_ADDR

// Pin definitions for M5IOE1 GPIOs
#define IOE1_PIN_1 M5IOE1_PIN_1  // IO1 on M5IOE1 (pin index 0)
#define IOE1_PIN_2 M5IOE1_PIN_2  // IO2 on M5IOE1 (pin index 1)

// Counter for interrupt events
volatile int pin1Counter = 0;
volatile int pin2Counter = 0;

// Custom data structure for callback with argument
struct ButtonData {
    const char* name;
    int* counter;
    uint8_t pin;
};

ButtonData button1Data = {"Button 1", &pin1Counter, IOE1_PIN_1};
ButtonData button2Data = {"Button 2", &pin2Counter, IOE1_PIN_2};

// Simple callback for pin 1 (without argument)
void IRAM_ATTR pin1Callback() {
    pin1Counter++;
}

// Callback with custom data argument for pin 2
void IRAM_ATTR pin2CallbackWithArg(void* arg) {
    ButtonData* data = static_cast<ButtonData*>(arg);
    if (data) {
        (*(data->counter))++;
        // Note: Avoid Serial prints in ISR context
        // This is just demonstration - in real apps, use flags and handle in loop
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n========================================");
    Serial.println("M5IOE1 Hardware Interrupt Example");
    Serial.println("========================================\n");

    // Set log level to INFO
    M5IOE1::setLogLevel(M5IOE1_LOG_LEVEL_INFO);

    // Initialize M5IOE1 with hardware interrupt pin
    // When intPin is provided, HARDWARE mode is the default and most efficient
    Serial.println("Initializing M5IOE1 in HARDWARE interrupt mode...");
    Serial.println("  I2C: SDA=" + String(I2C_SDA_PIN) + ", SCL=" + String(I2C_SCL_PIN));
    Serial.println("  INT: GPIO " + String(INT_PIN) + "\n");

    if (!ioe1.begin(&Wire, I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ,
                    INT_PIN, M5IOE1_INT_MODE_HARDWARE)) {
        Serial.println("ERROR: Failed to initialize M5IOE1!");
        Serial.println("Please check:");
        Serial.println("  - I2C connections (SDA=" + String(I2C_SDA_PIN) + ", SCL=" + String(I2C_SCL_PIN) + ")");
        Serial.println("  - INT pin connection (M5IOE1 INT -> GPIO " + String(INT_PIN) + ")");
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

    // Configure IO1 and IO2 as inputs with pull-up
    Serial.println("Configuring IO1 and IO2 as inputs with internal pull-up...");
    ioe1.pinMode(IOE1_PIN_1, INPUT_PULLUP);
    ioe1.pinMode(IOE1_PIN_2, INPUT_PULLUP);
    Serial.println("Pins configured successfully!\n");

    // Attach interrupt to pin 1 with FALLING edge trigger (simple callback)
    Serial.println("Attaching FALLING edge interrupt to IO1...");
    ioe1.attachInterrupt(IOE1_PIN_1, pin1Callback, FALLING);
    Serial.println("IO1 interrupt attached (simple callback)!\n");

    // Attach interrupt to pin 2 with RISING edge trigger (callback with argument)
    Serial.println("Attaching RISING edge interrupt to IO2...");
    ioe1.attachInterruptArg(IOE1_PIN_2, pin2CallbackWithArg, &button2Data, RISING);
    Serial.println("IO2 interrupt attached (callback with argument)!\n");

    Serial.println("========================================");
    Serial.println("Setup complete! Ready for interrupts.");
    Serial.println("========================================\n");

    Serial.println("Instructions:");
    Serial.println("  - Connect buttons between IO1/IO2 and GND");
    Serial.println("  - Press IO1 button for FALLING edge interrupt");
    Serial.println("  - Press IO2 button for RISING edge interrupt");
    Serial.println("  - Hardware interrupts provide instant response!");
    Serial.println("\nAvailable commands:");
    Serial.println("  'e' - Enable interrupts on IO1");
    Serial.println("  'd' - Disable interrupts on IO1");
    Serial.println("  's' - Show interrupt status");
    Serial.println("  'c' - Clear interrupt status\n");
}

void loop() {
    // Store current counter values to detect changes
    static int lastPin1Counter = 0;
    static int lastPin2Counter = 0;

    // Check if pin 1 interrupt counter changed
    if (pin1Counter != lastPin1Counter) {
        Serial.println(">>> IO1 INTERRUPT TRIGGERED! <<<");
        Serial.println("    Count: " + String(pin1Counter));
        Serial.println("    Edge: FALLING");

        // Read current state of IO1
        int pinState = ioe1.digitalRead(IOE1_PIN_1);
        Serial.println("    IO1 state: " + String(pinState == LOW ? "LOW" : "HIGH"));

        // Read and display interrupt status register
        uint16_t status = ioe1.getInterruptStatus();
        Serial.println("    INT status: 0b" + String(status, BIN));

        // Clear the interrupt for this pin
        ioe1.clearInterrupt(IOE1_PIN_1);
        Serial.println("    Interrupt cleared\n");

        lastPin1Counter = pin1Counter;
    }

    // Check if pin 2 interrupt counter changed
    if (pin2Counter != lastPin2Counter) {
        Serial.println(">>> IO2 INTERRUPT TRIGGERED! <<<");
        Serial.println("    Count: " + String(pin2Counter));
        Serial.println("    Edge: RISING");
        Serial.println("    Callback: with argument (ButtonData)");

        // Read current state of IO2
        int pinState = ioe1.digitalRead(IOE1_PIN_2);
        Serial.println("    IO2 state: " + String(pinState == HIGH ? "HIGH" : "LOW"));

        // Read and display interrupt status register
        uint16_t status = ioe1.getInterruptStatus();
        Serial.println("    INT status: 0b" + String(status, BIN));

        // Clear the interrupt for this pin
        ioe1.clearInterrupt(IOE1_PIN_2);
        Serial.println("    Interrupt cleared\n");

        lastPin2Counter = pin2Counter;
    }

    // Handle serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        switch (cmd) {
            case 'e':
                Serial.println("Enabling interrupts on IO1...");
                ioe1.enableInterrupt(IOE1_PIN_1);
                Serial.println("IO1 interrupts enabled\n");
                break;

            case 'd':
                Serial.println("Disabling interrupts on IO1...");
                ioe1.disableInterrupt(IOE1_PIN_1);
                Serial.println("IO1 interrupts disabled\n");
                break;

            case 's':
                {
                    uint16_t status = ioe1.getInterruptStatus();
                    Serial.println("Interrupt Status:");
                    Serial.println("  Register: 0b" + String(status, BIN));
                    Serial.println("  IO1 count: " + String(pin1Counter));
                    Serial.println("  IO2 count: " + String(pin2Counter));
                    Serial.println("  IO1 state: " + String(ioe1.digitalRead(IOE1_PIN_1) == LOW ? "LOW" : "HIGH"));
                    Serial.println("  IO2 state: " + String(ioe1.digitalRead(IOE1_PIN_2) == LOW ? "LOW" : "HIGH") + "\n");
                }
                break;

            case 'c':
                Serial.println("Clearing all interrupt flags...");
                for (uint8_t i = 0; i < 14; i++) {
                    ioe1.clearInterrupt(i);
                }
                Serial.println("All interrupts cleared\n");
                break;

            case '\n':
            case '\r':
                // Ignore newlines
                break;

            default:
                Serial.println("Unknown command: " + String(cmd));
                Serial.println("Available: e (enable), d (disable), s (status), c (clear)\n");
                break;
        }
    }

    delay(10);  // Small delay
}
