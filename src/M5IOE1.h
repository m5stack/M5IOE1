/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef _M5IOE1_H_
#define _M5IOE1_H_

#include "M5IOE1_i2c_compat.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef ARDUINO
// Arduino: FreeRTOS headers included via Arduino framework
#else
// ESP-IDF specific includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#endif

// ============================
// IO Pin Definitions
// ============================
#define M5IOE1_PIN_NC    -1
#define M5IOE1_PIN_1      0
#define M5IOE1_PIN_2      1
#define M5IOE1_PIN_3      2
#define M5IOE1_PIN_4      3
#define M5IOE1_PIN_5      4
#define M5IOE1_PIN_6      5
#define M5IOE1_PIN_7      6
#define M5IOE1_PIN_8      7
#define M5IOE1_PIN_9      8
#define M5IOE1_PIN_10     9
#define M5IOE1_PIN_11    10
#define M5IOE1_PIN_12    11
#define M5IOE1_PIN_13    12
#define M5IOE1_PIN_14    13

// ============================
// Device Constants
// ============================
#define M5IOE1_DEFAULT_ADDR     0x6F
#define M5IOE1_MAX_GPIO_PINS    14
#define M5IOE1_MAX_ADC_CHANNELS 4
#define M5IOE1_MAX_PWM_CHANNELS 4
#define M5IOE1_MAX_LED_COUNT    32
#define M5IOE1_RTC_RAM_SIZE     32

// ============================
// I2C Frequency Constants
// ============================
#define M5IOE1_I2C_FREQ_100K    100000
#define M5IOE1_I2C_FREQ_400K    400000
#define M5IOE1_I2C_FREQ_DEFAULT M5IOE1_I2C_FREQ_100K

// ============================
// Register Addresses
// ============================
// System
#define M5IOE1_REG_UID_L            0x00    // R     [7:0] UID Low Byte
#define M5IOE1_REG_UID_H            0x01    // R     [15:8] UID High Byte
#define M5IOE1_REG_REV              0x02    // R     [7:0] SW7-SW0 Version
// GPIO
#define M5IOE1_REG_GPIO_MODE_L      0x03    // R/W   [7:0] Mode P8-P1
#define M5IOE1_REG_GPIO_MODE_H      0x04    // R/W   [7:6] Res | [5:0] Mode P14-P9
#define M5IOE1_REG_GPIO_OUT_L       0x05    // R/W   [7:0] Out P8-P1
#define M5IOE1_REG_GPIO_OUT_H       0x06    // R/W   [7:6] Res | [5:0] Out P14-P9
#define M5IOE1_REG_GPIO_IN_L        0x07    // R     [7:0] In P8-P1
#define M5IOE1_REG_GPIO_IN_H        0x08    // R     [7:6] Res | [5:0] In P14-P9
#define M5IOE1_REG_GPIO_PU_L        0x09    // R/W   [7:0] PU P8-P1
#define M5IOE1_REG_GPIO_PU_H        0x0A    // R/W   [7:6] Res | [5:0] PU P14-P9
#define M5IOE1_REG_GPIO_PD_L        0x0B    // R/W   [7:0] PD P8-P1
#define M5IOE1_REG_GPIO_PD_H        0x0C    // R/W   [7:6] Res | [5:0] PD P14-P9
#define M5IOE1_REG_GPIO_IE_L        0x0D    // R/W   [7:0] IE P8-P1
#define M5IOE1_REG_GPIO_IE_H        0x0E    // R/W   [7:6] Res | [5:0] IE P14-P9
#define M5IOE1_REG_GPIO_IP_L        0x0F    // R/W   [7:0] IP P8-P1
#define M5IOE1_REG_GPIO_IP_H        0x10    // R/W   [7:6] Res | [5:0] IP P14-P9
#define M5IOE1_REG_GPIO_IS_L        0x11    // R     [7:0] IS P8-P1
#define M5IOE1_REG_GPIO_IS_H        0x12    // R     [7:6] Res | [5:0] IS P14-P9
#define M5IOE1_REG_GPIO_DRV_L       0x13    // R/W   [7:0] Drive P8-P1
#define M5IOE1_REG_GPIO_DRV_H       0x14    // R/W   [7:6] Res | [5:0] Drive P14-P9
// ADC
#define M5IOE1_REG_ADC_CTRL         0x15    // R/W   [7] BUSY | [6] START | [2:0] Channel
#define M5IOE1_REG_ADC_DATA_L       0x16    // R     [7:0] ADC Data Low
#define M5IOE1_REG_ADC_DATA_H       0x17    // R     [3:0] ADC Data High
// Temperature
#define M5IOE1_REG_TEMP_CTRL        0x18    // R/W   [7] TBUSY | [6] TSTART
#define M5IOE1_REG_TEMP_DATA_L      0x19    // R     [7:0] Temp Data Low
#define M5IOE1_REG_TEMP_DATA_H      0x1A    // R     [3:0] Temp Data High
// PWM
#define M5IOE1_REG_PWM1_DUTY_L      0x1B    // R/W   [7:0] PWM1 Duty Low
#define M5IOE1_REG_PWM1_DUTY_H      0x1C    // R/W   [7] EN | [6] POL | [3:0] PWM1 Duty High
#define M5IOE1_REG_PWM2_DUTY_L      0x1D    // R/W   [7:0] PWM2 Duty Low
#define M5IOE1_REG_PWM2_DUTY_H      0x1E    // R/W   [7] EN | [6] POL | [3:0] PWM2 Duty High
#define M5IOE1_REG_PWM3_DUTY_L      0x1F    // R/W   [7:0] PWM3 Duty Low
#define M5IOE1_REG_PWM3_DUTY_H      0x20    // R/W   [7] EN | [6] POL | [3:0] PWM3 Duty High
#define M5IOE1_REG_PWM4_DUTY_L      0x21    // R/W   [7:0] PWM4 Duty Low
#define M5IOE1_REG_PWM4_DUTY_H      0x22    // R/W   [7] EN | [6] POL | [3:0] PWM4 Duty High
// System Config
#define M5IOE1_REG_I2C_CFG          0x23    // R/W   [6] INT_PU/PD | [5] WAKE | [4] SPD | [3:0] SLEEP
#define M5IOE1_REG_LED_CFG          0x24    // R/W   [6] REFRESH | [5:0] LED Num
#define M5IOE1_REG_PWM_FREQ_L       0x25    // R/W   [7:0] PWM Freq Low
#define M5IOE1_REG_PWM_FREQ_H       0x26    // R/W   [7:0] PWM Freq High
#define M5IOE1_REG_REF_VOLTAGE_L    0x27    // R     [7:0] Ref Voltage Low
#define M5IOE1_REG_REF_VOLTAGE_H    0x28    // R     [15:8] Ref Voltage High
#define M5IOE1_REG_FACTORY_RESET    0x29    // W     [7:0] Reset Key
// Data areas
#define M5IOE1_REG_LED_RAM_START    0x30    // R/W   NeoPixel RGB565 Data (32 LEDs x 2B)
#define M5IOE1_REG_LED_RAM_END      0x6F
#define M5IOE1_REG_RTC_RAM_START    0x70    // R/W   RTC Retention RAM (32B)
#define M5IOE1_REG_RTC_RAM_END      0x8F
// Extended
#define M5IOE1_REG_AW8737A_PULSE    0x90    // R/W   [7] REFRESH | [6:5] NUM | [4:0] GPIO

// ============================
// Bit Definitions
// ============================
// ADC Control
#define M5IOE1_ADC_CH_MASK          0x07
#define M5IOE1_ADC_START            (1 << 6)
#define M5IOE1_ADC_BUSY             (1 << 7)
// Temperature Control
#define M5IOE1_TEMP_START           (1 << 6)
#define M5IOE1_TEMP_BUSY            (1 << 7)
// PWM Control
#define M5IOE1_PWM_POLARITY         (1 << 6)
#define M5IOE1_PWM_ENABLE           (1 << 7)
// I2C Config
#define M5IOE1_I2C_SLEEP_MASK       0x0F
#define M5IOE1_I2C_SPEED_400K       (1 << 4)
#define M5IOE1_I2C_WAKE_RISING      (1 << 5)
#define M5IOE1_I2C_PULL_OFF         (1 << 6)
// LED Config
#define M5IOE1_LED_NUM_MASK         0x3F
#define M5IOE1_LED_REFRESH          (1 << 6)
// Factory Reset
#define M5IOE1_FACTORY_RESET_KEY    0x3A
// AW8737A Pulse
#define M5IOE1_AW8737A_GPIO_MASK    0x1F
#define M5IOE1_AW8737A_NUM_SHIFT    5
#define M5IOE1_AW8737A_NUM_MASK     0x03
#define M5IOE1_AW8737A_REFRESH      (1 << 7)

// ============================
// ADC Channel Definitions (IO pins that support ADC)
// ============================
#define M5IOE1_ADC_CH1              1   // IO2 (pin index 1)
#define M5IOE1_ADC_CH2              2   // IO4 (pin index 3)
#define M5IOE1_ADC_CH3              3   // IO5 (pin index 4)
#define M5IOE1_ADC_CH4              4   // IO7 (pin index 6)

// ============================
// PWM Channel Definitions (IO pins that support PWM)
// ============================
#define M5IOE1_PWM_CH1              0   // IO9 (pin index 8)
#define M5IOE1_PWM_CH2              1   // IO8 (pin index 7)
#define M5IOE1_PWM_CH3              2   // IO11 (pin index 10)
#define M5IOE1_PWM_CH4              3   // IO10 (pin index 9)

// ============================
// GPIO Mode Definitions (Arduino-compatible)
// ============================
#ifndef INPUT
#define INPUT           0x00
#endif
#ifndef OUTPUT
#define OUTPUT          0x01
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP    0x02
#endif
#ifndef INPUT_PULLDOWN
#define INPUT_PULLDOWN  0x03
#endif

// ============================
// GPIO Level Definitions
// ============================
#ifndef LOW
#define LOW             0
#endif
#ifndef HIGH
#define HIGH            1
#endif

// ============================
// Interrupt Mode Definitions
// ============================
#ifndef RISING
#define RISING          0x01
#endif
#ifndef FALLING
#define FALLING         0x02
#endif

// ============================
// Pull Mode Definitions
// ============================
#define M5IOE1_PULL_NONE    0x00
#define M5IOE1_PULL_UP      0x01
#define M5IOE1_PULL_DOWN    0x02

// ============================
// Drive Mode Definitions
// ============================
#define M5IOE1_DRIVE_PUSHPULL   0x00
#define M5IOE1_DRIVE_OPENDRAIN  0x01

// ============================
// AW8737A PULSE Refresh Types
// ============================
typedef enum {
    M5IOE1_AW8737A_REFRESH_WAIT = 0,    // No refresh, wait for next trigger
    M5IOE1_AW8737A_REFRESH_NOW = 1      // Refresh and execute immediately
} m5ioe1_aw8737a_refresh_t;

// ============================
// AW8737A PULSE NUM Types
// ============================
typedef enum {
    M5IOE1_AW8737A_PULSE_NUM_0 = 0,     // 0 pulse
    M5IOE1_AW8737A_PULSE_NUM_1 = 1,     // 1 pulse
    M5IOE1_AW8737A_PULSE_NUM_2 = 2,     // 2 pulses
    M5IOE1_AW8737A_PULSE_NUM_3 = 3      // 3 pulses
} m5ioe1_aw8737a_pulse_num_t;

// ============================
// Interrupt Handling Mode
// ============================
typedef enum {
    M5IOE1_INT_MODE_DISABLED = 0,   // Interrupt handling disabled
    M5IOE1_INT_MODE_POLLING,        // Polling mode (default)
    M5IOE1_INT_MODE_HARDWARE        // Hardware interrupt mode
} m5ioe1_int_mode_t;

// ============================
// Log Level Definitions
// ============================
typedef enum {
    M5IOE1_LOG_LEVEL_NONE = 0,      // No log output
    M5IOE1_LOG_LEVEL_ERROR,         // Error messages only
    M5IOE1_LOG_LEVEL_WARN,          // Warning and error messages
    M5IOE1_LOG_LEVEL_INFO,          // Info, warning and error messages (default)
    M5IOE1_LOG_LEVEL_DEBUG,         // Debug, info, warning and error messages
    M5IOE1_LOG_LEVEL_VERBOSE        // All messages including verbose
} m5ioe1_log_level_t;

// ============================
// Configuration Type for Validation
// ============================
typedef enum {
    M5IOE1_CONFIG_GPIO_INPUT = 0,   // GPIO input mode
    M5IOE1_CONFIG_GPIO_OUTPUT,      // GPIO output mode
    M5IOE1_CONFIG_GPIO_INTERRUPT,   // GPIO interrupt mode
    M5IOE1_CONFIG_ADC,              // ADC function
    M5IOE1_CONFIG_PWM,              // PWM function
    M5IOE1_CONFIG_NEOPIXEL,         // NeoPixel LED function (IO14 only)
    M5IOE1_CONFIG_I2C_SLEEP         // I2C sleep mode configuration
} m5ioe1_config_type_t;

// ============================
// RGB Color Structure
// ============================
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} m5ioe1_rgb_t;

// ============================
// Configuration Validation Result
// ============================
typedef struct {
    bool valid;
    char error_msg[64];
    uint8_t conflicting_pin;
} m5ioe1_validation_t;

// ============================
// Snapshot Verification Result
// ============================
typedef struct {
    bool consistent;            // true if all cached values match hardware registers
    bool gpio_mismatch;         // true if GPIO registers don't match cache
    bool pwm_mismatch;          // true if PWM registers don't match cache
    bool adc_mismatch;          // true if ADC registers don't match cache
    uint16_t expected_mode;     // cached GPIO mode register value
    uint16_t actual_mode;       // actual GPIO mode register value
    uint16_t expected_output;   // cached GPIO output register value
    uint16_t actual_output;     // actual GPIO output register value
} m5ioe1_snapshot_verify_t;

// ============================
// Callback Types
// ============================
typedef void (*m5ioe1_callback_t)(void);
typedef void (*m5ioe1_callback_arg_t)(void*);

// ============================
// M5IOE1 Class
// ============================
class M5IOE1 {
public:
    M5IOE1();
    ~M5IOE1();

    // ========================
    // Initialization
    // ========================
#ifdef ARDUINO
    /**
     * @brief Initialize the device without hardware interrupt pin (Arduino)
     * @note Without intPin: Only POLLING and DISABLED modes are supported
     * @note POLLING mode: Periodically reads GPIO_IS registers (0x11-0x12). Non-zero indicates one or more pins triggered
     * @param wire Pointer to TwoWire instance
     * @param addr I2C address (default 0x6F)
     * @param sda SDA pin (default -1, uses default I2C pins)
     * @param scl SCL pin (default -1, uses default I2C pins)
     * @param speed I2C speed in Hz (default 100000)
     * @param mode Interrupt mode: M5IOE1_INT_MODE_POLLING or M5IOE1_INT_MODE_DISABLED
     * @return true if successful
     */
    bool begin(TwoWire *wire, uint8_t addr = M5IOE1_DEFAULT_ADDR,
               uint8_t sda = -1, uint8_t scl = -1, uint32_t speed = 100000,
               m5ioe1_int_mode_t mode = M5IOE1_INT_MODE_POLLING);

    /**
     * @brief Initialize with hardware interrupt pin (Arduino)
     * @note With intPin: Supports HARDWARE, POLLING, and DISABLED modes
     * @note HARDWARE mode: Configures intPin as input without internal pull-up. Waits for falling edge, then reads GPIO_IS registers (0x11-0x12)
     * @note POLLING mode: Periodically reads GPIO_IS registers (0x11-0x12). Non-zero indicates one or more pins triggered
     * @note DISABLED mode: No interrupt handling
     * @param wire Pointer to TwoWire instance
     * @param addr I2C address (default 0x6F)
     * @param sda SDA pin (default -1, uses default I2C pins)
     * @param scl SCL pin (default -1, uses default I2C pins)
     * @param speed I2C speed in Hz (default 100000)
     * @param intPin Hardware interrupt pin. Must be valid GPIO for HARDWARE mode
     * @param mode Interrupt mode: M5IOE1_INT_MODE_HARDWARE (default), M5IOE1_INT_MODE_POLLING, or M5IOE1_INT_MODE_DISABLED
     * @return true if successful
     */
    bool begin(TwoWire *wire, uint8_t addr = M5IOE1_DEFAULT_ADDR,
                uint8_t sda = -1, uint8_t scl = -1, uint32_t speed = 100000,
                int8_t intPin = -1, m5ioe1_int_mode_t mode = M5IOE1_INT_MODE_HARDWARE);
#else // ESP-IDF
    // =====================================================
    // Type 1A: Self-created I2C bus, no hardware interrupt
    // =====================================================
    /**
     * @brief Initialize with self-created I2C bus, no hardware interrupt (ESP-IDF)
     * @param port I2C port number (I2C_NUM_0 or I2C_NUM_1)
     * @param addr I2C address (default 0x6F)
     * @param sda SDA pin (default 21)
     * @param scl SCL pin (default 22)
     * @param speed I2C speed in Hz (only 100000 or 400000 supported)
     * @param mode Interrupt mode: M5IOE1_INT_MODE_POLLING or M5IOE1_INT_MODE_DISABLED
     * @return true if successful
     */
    bool begin(i2c_port_t port = I2C_NUM_0, uint8_t addr = M5IOE1_DEFAULT_ADDR,
               int sda = 21, int scl = 22, uint32_t speed = M5IOE1_I2C_FREQ_100K,
               m5ioe1_int_mode_t mode = M5IOE1_INT_MODE_POLLING);

    // =====================================================
    // Type 1B: Self-created I2C bus, with hardware interrupt
    // =====================================================
    /**
     * @brief Initialize with self-created I2C bus, with hardware interrupt (ESP-IDF)
     * @param port I2C port number
     * @param addr I2C address
     * @param sda SDA pin
     * @param scl SCL pin
     * @param speed I2C speed in Hz
     * @param intPin Hardware interrupt GPIO pin
     * @param mode Interrupt mode: M5IOE1_INT_MODE_HARDWARE, POLLING, or DISABLED
     * @return true if successful
     */
    bool begin(i2c_port_t port, uint8_t addr, int sda, int scl, uint32_t speed,
               int intPin, m5ioe1_int_mode_t mode = M5IOE1_INT_MODE_HARDWARE);

    // =====================================================
    // Type 2A: Existing i2c_master_bus_handle_t, no hardware interrupt
    // =====================================================
    /**
     * @brief Initialize with existing i2c_master_bus handle (ESP-IDF native driver)
     * @param bus Existing i2c_master_bus_handle_t
     * @param addr I2C address (default 0x6F)
     * @param speed I2C speed in Hz (for device handle creation)
     * @param mode Interrupt mode: M5IOE1_INT_MODE_POLLING or M5IOE1_INT_MODE_DISABLED
     * @return true if successful
     */
    bool begin(i2c_master_bus_handle_t bus, uint8_t addr = M5IOE1_DEFAULT_ADDR,
               uint32_t speed = M5IOE1_I2C_FREQ_100K,
               m5ioe1_int_mode_t mode = M5IOE1_INT_MODE_POLLING);

    // =====================================================
    // Type 2B: Existing i2c_master_bus_handle_t, with hardware interrupt
    // =====================================================
    /**
     * @brief Initialize with existing i2c_master_bus handle, with hardware interrupt
     * @param bus Existing i2c_master_bus_handle_t
     * @param addr I2C address
     * @param speed I2C speed in Hz
     * @param intPin Hardware interrupt GPIO pin
     * @param mode Interrupt mode
     * @return true if successful
     */
    bool begin(i2c_master_bus_handle_t bus, uint8_t addr, uint32_t speed,
               int intPin, m5ioe1_int_mode_t mode = M5IOE1_INT_MODE_HARDWARE);

    // =====================================================
    // Type 3A: Existing i2c_bus_handle_t, no hardware interrupt
    // =====================================================
    /**
     * @brief Initialize with existing i2c_bus handle (esp-idf-lib component)
     * @param bus Existing i2c_bus_handle_t
     * @param addr I2C address (default 0x6F)
     * @param speed I2C speed in Hz (for device handle creation)
     * @param mode Interrupt mode: M5IOE1_INT_MODE_POLLING or M5IOE1_INT_MODE_DISABLED
     * @return true if successful
     */
    bool begin(i2c_bus_handle_t bus, uint8_t addr = M5IOE1_DEFAULT_ADDR,
               uint32_t speed = M5IOE1_I2C_FREQ_100K,
               m5ioe1_int_mode_t mode = M5IOE1_INT_MODE_POLLING);

    // =====================================================
    // Type 3B: Existing i2c_bus_handle_t, with hardware interrupt
    // =====================================================
    /**
     * @brief Initialize with existing i2c_bus handle, with hardware interrupt
     * @param bus Existing i2c_bus_handle_t
     * @param addr I2C address
     * @param speed I2C speed in Hz
     * @param intPin Hardware interrupt GPIO pin
     * @param mode Interrupt mode
     * @return true if successful
     */
    bool begin(i2c_bus_handle_t bus, uint8_t addr, uint32_t speed,
               int intPin, m5ioe1_int_mode_t mode = M5IOE1_INT_MODE_HARDWARE);
#endif

    /**
     * @brief Set interrupt handling mode
     * @param mode Interrupt mode (DISABLED, POLLING, HARDWARE)
     * @param pollingIntervalMs Polling interval in ms (for polling mode, default 5000ms)
     * @return true if successful
     */
    bool setInterruptMode(m5ioe1_int_mode_t mode, uint32_t pollingIntervalMs = 5000);

    /**
     * @brief Set polling interval for POLLING mode
     * @param seconds Polling interval in seconds (range: 0.001 to 3600, supports float)
     * @return true if successful, false if out of range
     * @note If already in POLLING mode, the task will be restarted with new interval
     */
    bool setPollingInterval(float seconds);

    /**
     * @brief Set global log level for M5IOE1 library
     * @param level Log level (ERROR, WARN, INFO, DEBUG, VERBOSE)
     * @note For ESP-IDF: Uses esp_log_level_set() to control ESP_LOGx macros
     * @note For Arduino: Controls Serial.printf output filtering
     * @note Default level is M5IOE1_LOG_LEVEL_INFO
     */
    static void setLogLevel(m5ioe1_log_level_t level);

    /**
     * @brief Get current global log level
     * @return Current log level
     */
    static m5ioe1_log_level_t getLogLevel();

    // ========================
    // Device Information
    // ========================
    bool getUID(uint16_t* uid);
    bool getVersion(uint8_t* version);
    bool getRefVoltage(uint16_t* voltage_mv);

    // ========================
    // GPIO Functions (Arduino-style)
    // ========================
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t value);
    int digitalRead(uint8_t pin);

    // ========================
    // Advanced GPIO Functions
    // ========================
    bool setPullMode(uint8_t pin, uint8_t pullMode);
    bool setDriveMode(uint8_t pin, uint8_t driveMode);
    bool getInputState(uint8_t pin, uint8_t* state);

    // ========================
    // Interrupt Functions
    // ========================
    void attachInterrupt(uint8_t pin, m5ioe1_callback_t callback, uint8_t mode);
    void attachInterruptArg(uint8_t pin, m5ioe1_callback_arg_t callback, void* arg, uint8_t mode);
    void detachInterrupt(uint8_t pin);
    void enableInterrupt(uint8_t pin);
    void disableInterrupt(uint8_t pin);
    uint16_t getInterruptStatus();
    bool clearInterrupt(uint8_t pin);

    // ========================
    // ADC Functions
    // ========================
    /**
     * @brief Read ADC value
     * @param channel ADC channel (1-4)
     * @param result Pointer to store 12-bit result
     * @return true if successful
     */
    bool analogRead(uint8_t channel, uint16_t* result);
    bool isAdcBusy();
    bool disableAdc();

    // ========================
    // Temperature Sensor
    // ========================
    bool readTemperature(uint16_t* temperature);
    bool isTemperatureBusy();

    // ========================
    // PWM Functions
    // ========================
    /**
     * @brief Set PWM frequency (shared by all channels)
     * @param frequency Frequency in Hz
     * @return true if successful
     */
    bool setPwmFrequency(uint16_t frequency);
    bool getPwmFrequency(uint16_t* frequency);

    /**
     * @brief Set PWM duty cycle (percentage)
     * @param channel PWM channel (0-3)
     * @param duty Duty cycle percentage (0-100)
     * @param polarity PWM polarity (false=normal, true=inverted)
     * @param enable Enable PWM output
     * @return true if successful
     */
    bool setPwmDuty(uint8_t channel, uint8_t duty, bool polarity = false, bool enable = true);

    /**
     * @brief Set PWM duty cycle (12-bit value)
     * @param channel PWM channel (0-3)
     * @param duty12 12-bit duty value (0-4095)
     * @param polarity PWM polarity
     * @param enable Enable PWM output
     * @return true if successful
     */
    bool setPwmDuty12bit(uint8_t channel, uint16_t duty12, bool polarity = false, bool enable = true);
    bool getPwmDuty(uint8_t channel, uint8_t* duty, bool* polarity, bool* enable);

    // ========================
    // NeoPixel LED Functions
    // ========================
    bool setLedCount(uint8_t count);
    bool setLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
    bool setLedColor(uint8_t index, m5ioe1_rgb_t color);
    bool refreshLeds();
    bool disableLeds();

    // ========================
    // AW8737A Pulse Functions
    // ========================
    /**
     * @brief Set AW8737A pulse output configuration
     * @param pin GPIO pin number (0-13)
     * @param pulseNum Number of pulses (0-3)
     * @param refresh Refresh control (WAIT or NOW)
     * @return true if successful
     * @note This will automatically configure the pin as output
     * @note If using open-drain output, external pull-up is required
     * @note When refresh=NOW, there will be a 20ms delay after execution
     */
    bool setAw8737aPulse(uint8_t pin, m5ioe1_aw8737a_pulse_num_t pulseNum, 
                         m5ioe1_aw8737a_refresh_t refresh = M5IOE1_AW8737A_REFRESH_NOW);
    
    /**
     * @brief Trigger AW8737A pulse refresh
     * @return true if successful
     * @note Call setAw8737aPulse with refresh=WAIT first, then call this to trigger
     * @note There will be a 20ms delay after execution
     */
    bool refreshAw8737aPulse();

    // ========================
    // RTC RAM Functions
    // ========================
    bool writeRtcRam(uint8_t offset, const uint8_t* data, uint8_t length);
    bool readRtcRam(uint8_t offset, uint8_t* data, uint8_t length);

    // ========================
    // System Configuration
    // ========================
    bool setI2cConfig(uint8_t sleepTime, bool speed400k = false, bool wakeRising = false, bool pullOff = false);
    bool factoryReset();

    // ========================
    // State Snapshot Functions
    // ========================
    void setAutoSnapshot(bool enable);
    bool isAutoSnapshotEnabled() const;
    bool updateSnapshot();

    // ========================
    // Debug Functions
    // ========================
    bool getModeReg(uint16_t* reg);
    bool getOutputReg(uint16_t* reg);
    bool getInputReg(uint16_t* reg);
    bool getPullUpReg(uint16_t* reg);
    bool getPullDownReg(uint16_t* reg);
    bool getDriveReg(uint16_t* reg);

    // ========================
    // Configuration Validation
    // ========================
    /**
     * @brief Validate pin configuration before applying
     * @param pin Pin number (0-13)
     * @param configType Configuration type (m5ioe1_config_type_t)
     * @param enable true to enable config, false to disable
     * @return Validation result with error details if invalid
     */
    m5ioe1_validation_t validateConfig(uint8_t pin, m5ioe1_config_type_t configType, bool enable = true);

    // ========================
    // Snapshot Verification
    // ========================
    /**
     * @brief Verify that cached state matches actual hardware registers
     * @return Verification result with mismatch details
     */
    m5ioe1_snapshot_verify_t verifySnapshot();

    // ========================
    // Cached State Query Functions
    // ========================
    /**
     * @brief Get cached PWM frequency
     * @param frequency Output: PWM frequency in Hz
     * @return true if cache is valid
     */
    bool getCachedPwmFrequency(uint16_t* frequency);

    /**
     * @brief Get cached PWM channel state
     * @param channel PWM channel (0-3)
     * @param duty12 Output: 12-bit duty value
     * @param polarity Output: polarity setting
     * @param enabled Output: enable state
     * @return true if cache is valid
     */
    bool getCachedPwmState(uint8_t channel, uint16_t* duty12, bool* polarity, bool* enabled);

    /**
     * @brief Get cached ADC state
     * @param activeChannel Output: current ADC channel (0=disabled, 1-4=channel)
     * @param busy Output: conversion status
     * @param lastValue Output: last conversion result
     * @return true if cache is valid
     */
    bool getCachedAdcState(uint8_t* activeChannel, bool* busy, uint16_t* lastValue);

    /**
     * @brief Get cached GPIO pin state
     * @param pin Pin number (0-13)
     * @param isOutput Output: true if pin is output
     * @param level Output: current level (output or input)
     * @param pull Output: pull mode (0=none, 1=up, 2=down)
     * @return true if cache is valid
     */
    bool getCachedPinState(uint8_t pin, bool* isOutput, uint8_t* level, uint8_t* pull);

    /**
     * @brief Enable or disable default interrupt logging when no callback is registered
     * @param enable true to enable logging, false to disable (default)
     */
    void enableDefaultInterruptLog(bool enable);

private:
    // Device state
    uint8_t _addr;
    bool _initialized;
    bool _autoSnapshot;
    bool _enableDefaultIsrLog;
    uint32_t _requestedSpeed;  // User requested I2C speed (for 400K switch)

    // Interrupt mode
    m5ioe1_int_mode_t _intMode;
    int8_t _intPin;
    uint32_t _pollingInterval;

#ifdef ARDUINO
    TwoWire *_wire;
    uint8_t _sda;   // SDA pin number for I2C re-initialization
    uint8_t _scl;   // SCL pin number for I2C re-initialization
#else
    // I2C driver type selection
    m5ioe1_i2c_driver_t _i2cDriverType;
    
    // I2C handles (only one pair is used based on driver type)
    // For M5IOE1_I2C_DRIVER_SELF_CREATED: uses _i2c_master_bus + _i2c_master_dev
    // For M5IOE1_I2C_DRIVER_MASTER: uses _i2c_master_bus + _i2c_master_dev
    // For M5IOE1_I2C_DRIVER_BUS: uses _i2c_bus + _i2c_device
    i2c_master_bus_handle_t _i2c_master_bus;  // ESP-IDF native driver / self-created
    i2c_master_dev_handle_t _i2c_master_dev;  // ESP-IDF native driver / self-created
    i2c_bus_handle_t _i2c_bus;                 // esp-idf-lib component
    i2c_bus_device_handle_t _i2c_device;       // esp-idf-lib component
    
    // I2C management flags
    bool _busExternal;  // true if bus handle is provided externally
    
    // I2C pins for self-created bus (for frequency switching)
    int _sda;
    int _scl;
    i2c_port_t _port;
    
    // Interrupt handling
    TaskHandle_t _pollTask;
    QueueHandle_t _intrQueue;
#endif

    // Interrupt callbacks
    struct {
        m5ioe1_callback_t callback;
        m5ioe1_callback_arg_t callbackArg;
        void* arg;
        bool enabled;
        bool rising;
    } _callbacks[M5IOE1_MAX_GPIO_PINS];

    // Cached pin states
    struct {
        bool isOutput;
        uint8_t outputLevel;
        uint8_t inputLevel;
        uint8_t pull;       // 0:none, 1:up, 2:down
        uint8_t drive;      // 0:push-pull, 1:open-drain
        bool intrEnabled;
        bool intrRising;
    } _pinStates[M5IOE1_MAX_GPIO_PINS];
    bool _pinStatesValid;

    // Cached PWM states
    struct {
        uint16_t duty12;
        bool enabled;
        bool polarity;
    } _pwmStates[M5IOE1_MAX_PWM_CHANNELS];
    uint16_t _pwmFrequency;
    bool _pwmStatesValid;

    // Cached ADC state
    struct {
        uint8_t activeChannel;
        bool busy;
        uint16_t lastValue;
    } _adcState;
    bool _adcStateValid;

    // Cached I2C config state (for sleep mode detection)
    struct {
        uint8_t sleepTime;      // 0=disabled, 1-15=sleep time
        bool speed400k;         // I2C speed mode
        bool wakeRising;        // Wake edge mode
        bool pullOff;           // Internal pull-up off
    } _i2cConfig;
    bool _i2cConfigValid;

    // NeoPixel state
    uint8_t _ledCount;
    bool _ledEnabled;

    // ========================
    // Internal Helper Functions
    // ========================
    bool _writeReg(uint8_t reg, uint8_t value);
    bool _writeReg16(uint8_t reg, uint16_t value);
    bool _readReg(uint8_t reg, uint8_t* value);
    bool _readReg16(uint8_t reg, uint16_t* value);
    bool _writeBytes(uint8_t reg, const uint8_t* data, uint8_t len);
    bool _readBytes(uint8_t reg, uint8_t* data, uint8_t len);

    bool _isValidPin(uint8_t pin);
    bool _isAdcPin(uint8_t pin);
    bool _isPwmPin(uint8_t pin);
    uint8_t _getAdcChannel(uint8_t pin);
    uint8_t _getPwmChannel(uint8_t pin);

    void _clearPinStates();
    void _clearPwmStates();
    void _clearAdcState();
    bool _snapshotPinStates();
    bool _snapshotPwmStates();
    bool _snapshotAdcState();
    void _autoSnapshotUpdate();

    bool _initDevice();
    void _handleInterrupt();

    // I2C frequency validation and switching
    bool _isValidI2cFrequency(uint32_t speed);
    bool _switchTo400K();

    // Interrupt mutex pairs check
    static bool _pinsConflict(uint8_t a, uint8_t b);
    bool _hasConflictingInterrupt(uint8_t pin);

    // Configuration validation helpers
    bool _getInterruptMutexPin(uint8_t pin, uint8_t* mutexPin);
    bool _isNeopixelPin(uint8_t pin);
    bool _hasActiveInterrupt(uint8_t pin);
    bool _hasActiveAdc(uint8_t pin);
    bool _hasActivePwm(uint8_t pin);
    bool _hasI2cSleepEnabled();
    bool _isLedEnabled();

    // I2C config snapshot
    void _clearI2cConfig();
    bool _snapshotI2cConfig();

#ifdef ARDUINO
    // Arduino specific
    bool _setupPollingArduino();
    void _cleanupPollingArduino();
    static void _pollTaskArduino(void* arg);
#else
    // ESP-IDF specific
    static void _pollTaskFunc(void* arg);
    static void IRAM_ATTR _isrHandler(void* arg);
    bool _setupHardwareInterrupt();
    bool _setupPolling();
    void _cleanupInterrupt();
#endif
};

#endif // _M5IOE1_H_
