/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef _M5IOE1_H_
#define _M5IOE1_H_

#include "M5IOE1_i2c_compat.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef ARDUINO
// Arduino：FreeRTOS 头文件通过 Arduino 框架包含
// Arduino: FreeRTOS headers included via Arduino framework
#else
// ESP-IDF 专用包含文件
// ESP-IDF specific includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#endif

// ============================
// IO 引脚定义
// IO Pin Definitions
// ============================
typedef enum {
    M5IOE1_PIN_NC = -1,
    M5IOE1_PIN_1 = 0,
    M5IOE1_PIN_2 = 1,
    M5IOE1_PIN_3 = 2,
    M5IOE1_PIN_4 = 3,
    M5IOE1_PIN_5 = 4,
    M5IOE1_PIN_6 = 5,
    M5IOE1_PIN_7 = 6,
    M5IOE1_PIN_8 = 7,
    M5IOE1_PIN_9 = 8,
    M5IOE1_PIN_10 = 9,
    M5IOE1_PIN_11 = 10,
    M5IOE1_PIN_12 = 11,
    M5IOE1_PIN_13 = 12,
    M5IOE1_PIN_14 = 13
} m5ioe1_pin_t;

// ============================
// 设备常量
// Device Constants
// ============================
#define M5IOE1_DEFAULT_ADDR     0x6F
#define M5IOE1_MAX_GPIO_PINS    14
#define M5IOE1_MAX_ADC_CHANNELS 4
#define M5IOE1_MAX_PWM_CHANNELS 4
#define M5IOE1_MAX_LED_COUNT    32
#define M5IOE1_RTC_RAM_SIZE     32

// ============================
// I2C 频率常量
// I2C Frequency Constants
// ============================
#define M5IOE1_I2C_FREQ_100K    100000
#define M5IOE1_I2C_FREQ_400K    400000
#define M5IOE1_I2C_FREQ_DEFAULT M5IOE1_I2C_FREQ_100K

// ============================
// 寄存器地址
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
// 位定义
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
#define M5IOE1_I2C_SPEED_400K_BIT   (1 << 4)
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
// ADC 通道定义（支持 ADC 的 IO 引脚）
// ADC Channel Definitions (IO pins that support ADC)
// ============================
#define M5IOE1_ADC_CH1              1   // IO2 (pin index 1)
#define M5IOE1_ADC_CH2              2   // IO4 (pin index 3)
#define M5IOE1_ADC_CH3              3   // IO5 (pin index 4)
#define M5IOE1_ADC_CH4              4   // IO7 (pin index 6)

// ============================
// PWM 通道定义（支持 PWM 的 IO 引脚）
// PWM Channel Definitions (IO pins that support PWM)
// ============================
#define M5IOE1_PWM_CH1              0   // IO9 (pin index 8)
#define M5IOE1_PWM_CH2              1   // IO8 (pin index 7)
#define M5IOE1_PWM_CH3              2   // IO11 (pin index 10)
#define M5IOE1_PWM_CH4              3   // IO10 (pin index 9)

// ============================
// GPIO 电平定义
// GPIO Level Definitions
// ============================
#ifndef LOW
#define LOW             0x0
#endif
#ifndef HIGH
#define HIGH            0x1
#endif

// ============================
// GPIO 模式定义（Arduino 兼容）
// GPIO Mode Definitions (Arduino-compatible)
// ============================
#ifndef INPUT
#define INPUT             0x01
#endif
#ifndef OUTPUT
#define OUTPUT            0x03
#endif
#ifndef PULLUP
#define PULLUP            0x04
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP      0x05
#endif
#ifndef PULLDOWN
#define PULLDOWN          0x08
#endif
#ifndef INPUT_PULLDOWN
#define INPUT_PULLDOWN    0x09
#endif
#ifndef OPEN_DRAIN
#define OPEN_DRAIN        0x10
#endif
#ifndef OUTPUT_OPEN_DRAIN
#define OUTPUT_OPEN_DRAIN 0x13
#endif
#ifndef ANALOG
#define ANALOG            0xC0
#endif

// ============================
// 中断模式定义
// Interrupt Mode Definitions
// ============================
#ifndef DISABLED
#define DISABLED  0x00
#endif
#ifndef RISING
#define RISING    0x01
#endif
#ifndef FALLING
#define FALLING   0x02
#endif
#ifndef CHANGE
#define CHANGE    0x03
#endif
#ifndef ONLOW
#define ONLOW     0x04
#endif
#ifndef ONHIGH
#define ONHIGH    0x05
#endif
#ifndef ONLOW_WE
#define ONLOW_WE  0x0C
#endif
#ifndef ONHIGH_WE
#define ONHIGH_WE 0x0D
#endif

// ============================
// 上拉/下拉模式定义
// Pull Mode Definitions
// ============================
#define M5IOE1_PULL_NONE    0x00
#define M5IOE1_PULL_UP      0x01
#define M5IOE1_PULL_DOWN    0x02

// ============================
// 驱动模式定义
// Drive Mode Definitions
// ============================
#define M5IOE1_DRIVE_PUSHPULL   0x00
#define M5IOE1_DRIVE_OPENDRAIN  0x01

// ============================
// AW8737A 脉冲刷新类型
// AW8737A PULSE Refresh Types
// ============================
typedef enum {
    M5IOE1_AW8737A_REFRESH_WAIT = 0,    // 不刷新，等待下一次触发
                                        // No refresh, wait for next trigger
    M5IOE1_AW8737A_REFRESH_NOW = 1      // 刷新并立即执行
                                        // Refresh and execute immediately
} m5ioe1_aw8737a_refresh_t;

// ============================
// AW8737A 脉冲数量类型
// AW8737A PULSE NUM Types
// ============================
typedef enum {
    M5IOE1_AW8737A_PULSE_NUM_0 = 0,     // 0 个脉冲
                                        // 0 pulse
    M5IOE1_AW8737A_PULSE_NUM_1 = 1,     // 1 个脉冲
                                        // 1 pulse
    M5IOE1_AW8737A_PULSE_NUM_2 = 2,     // 2 个脉冲
                                        // 2 pulses
    M5IOE1_AW8737A_PULSE_NUM_3 = 3      // 3 个脉冲
                                        // 3 pulses
} m5ioe1_aw8737a_pulse_num_t;

// ============================
// 中断处理模式
// Interrupt Handling Mode
// ============================
typedef enum {
    M5IOE1_INT_MODE_DISABLED = 0,   // 中断处理已禁用
                                    // Interrupt handling disabled
    M5IOE1_INT_MODE_POLLING,        // 轮询模式（默认）
                                    // Polling mode (default)
    M5IOE1_INT_MODE_HARDWARE        // 硬件中断模式
                                    // Hardware interrupt mode
} m5ioe1_int_mode_t;

// ============================
// I2C 速度定义
// I2C Speed Definitions
// ============================
typedef enum {
    M5IOE1_I2C_SPEED_100K = 0,      // 100KHz 标准模式
                                    // 100KHz standard mode
    M5IOE1_I2C_SPEED_400K = 1       // 400KHz 快速模式
                                    // 400KHz fast mode
} m5ioe1_i2c_speed_t;

// ============================
// I2C 唤醒边沿定义
// I2C Wake Edge Definitions
// ============================
typedef enum {
    M5IOE1_WAKE_EDGE_FALLING = 0,   // 下降沿唤醒（默认）
                                    // Falling edge wake (default)
    M5IOE1_WAKE_EDGE_RISING = 1     // 上升沿唤醒
                                    // Rising edge wake
} m5ioe1_wake_edge_t;

// ============================
// I2C 内部上拉定义
// I2C Internal Pull-up Definitions
// ============================
typedef enum {
    M5IOE1_PULL_ENABLED = 0,        // 内部上拉启用（默认）
                                    // Internal pull-up enabled (default)
    M5IOE1_PULL_DISABLED = 1        // 内部上拉禁用
                                    // Internal pull-up disabled
} m5ioe1_pull_config_t;

// ============================
// 日志级别定义
// Log Level Definitions
// ============================
typedef enum {
    M5IOE1_LOG_LEVEL_NONE = 0,      // 无日志输出
                                    // No log output
    M5IOE1_LOG_LEVEL_ERROR,         // 仅错误消息
                                    // Error messages only
    M5IOE1_LOG_LEVEL_WARN,          // 警告和错误消息
                                    // Warning and error messages
    M5IOE1_LOG_LEVEL_INFO,          // 信息、警告和错误消息（默认）
                                    // Info, warning and error messages (default)
    M5IOE1_LOG_LEVEL_DEBUG,         // 调试、信息、警告和错误消息
                                    // Debug, info, warning and error messages
    M5IOE1_LOG_LEVEL_VERBOSE        // 所有消息包括详细输出
                                    // All messages including verbose
} m5ioe1_log_level_t;

// ============================
// 用于验证的配置类型
// Configuration Type for Validation
// ============================
typedef enum {
    M5IOE1_CONFIG_GPIO_INPUT = 0,   // GPIO 输入模式
                                    // GPIO input mode
    M5IOE1_CONFIG_GPIO_OUTPUT,      // GPIO 输出模式
                                    // GPIO output mode
    M5IOE1_CONFIG_GPIO_INTERRUPT,   // GPIO 中断模式
                                    // GPIO interrupt mode
    M5IOE1_CONFIG_ADC,              // ADC 功能
                                    // ADC function
    M5IOE1_CONFIG_PWM,              // PWM 功能
                                    // PWM function
    M5IOE1_CONFIG_NEOPIXEL,         // NeoPixel LED 功能（仅 IO14）
                                    // NeoPixel LED function (IO14 only)
    M5IOE1_CONFIG_I2C_SLEEP         // I2C 睡眠模式配置
                                    // I2C sleep mode configuration
} m5ioe1_config_type_t;

// ============================
// RGB 颜色结构
// RGB Color Structure
// ============================
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} m5ioe1_rgb_t;

// ============================
// 配置验证结果
// Configuration Validation Result
// ============================
typedef struct {
    bool valid;
    char error_msg[64];
    uint8_t conflicting_pin;
} m5ioe1_validation_t;

// ============================
// 快照验证结果
// Snapshot Verification Result
// ============================
typedef struct {
    bool consistent;            // 如果所有缓存值与硬件寄存器匹配则为 true
                                // true if all cached values match hardware registers
    bool gpio_mismatch;         // 如果 GPIO 寄存器与缓存不匹配则为 true
                                // true if GPIO registers don't match cache
    bool pwm_mismatch;          // 如果 PWM 寄存器与缓存不匹配则为 true
                                // true if PWM registers don't match cache
    bool adc_mismatch;          // 如果 ADC 寄存器与缓存不匹配则为 true
                                // true if ADC registers don't match cache
    uint16_t expected_mode;     // 缓存的 GPIO 模式寄存器值
                                // cached GPIO mode register value
    uint16_t actual_mode;       // 实际的 GPIO 模式寄存器值
                                // actual GPIO mode register value
    uint16_t expected_output;   // 缓存的 GPIO 输出寄存器值
                                // cached GPIO output register value
    uint16_t actual_output;     // 实际的 GPIO 输出寄存器值
                                // actual GPIO output register value
} m5ioe1_snapshot_verify_t;

// ============================
// 回调类型
// Callback Types
// ============================
typedef void (*m5ioe1_callback_t)(void);
typedef void (*m5ioe1_callback_arg_t)(void*);

// ============================
// M5IOE1 类
// M5IOE1 Class
// ============================
class M5IOE1 {
public:
    M5IOE1();
    ~M5IOE1();

    // ========================
    // 初始化
    // Initialization
    // ========================
#ifdef ARDUINO
    /**
     * @brief Initialize the M5IOE1 device (Arduino)
     * @note Without intPin (or intPin=-1): Only POLLING and DISABLED modes are supported
     * @note With intPin >= 0: Supports HARDWARE, POLLING, and DISABLED modes
     * @note HARDWARE mode: Configures intPin as input without internal pull-up. Waits for falling edge, then reads GPIO_IS registers (0x11-0x12)
     * @note POLLING mode: Periodically reads GPIO_IS registers (0x11-0x12). Non-zero indicates one or more pins triggered
     * @note DISABLED mode: No interrupt handling
     * @param wire Pointer to TwoWire instance
     * @param addr I2C address (default 0x6F)
     * @param sda SDA pin (default -1, uses default I2C pins)
     * @param scl SCL pin (default -1, uses default I2C pins)
     * @param speed I2C speed in Hz (default 100000)
     * @param intPin Hardware interrupt pin (-1 = no hardware interrupt, default -1)
     * @param intMode Interrupt mode (default M5IOE1_INT_MODE_POLLING)
     * @return true if successful
     */
    bool begin(TwoWire *wire, uint8_t addr = M5IOE1_DEFAULT_ADDR,
               uint8_t sda = -1, uint8_t scl = -1, uint32_t speed = 100000,
               int8_t intPin = -1, m5ioe1_int_mode_t intMode = M5IOE1_INT_MODE_POLLING);
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
     * @param intMode Interrupt mode: M5IOE1_INT_MODE_POLLING or M5IOE1_INT_MODE_DISABLED
     * @return true if successful
     */
    bool begin(i2c_port_t port = I2C_NUM_0, uint8_t addr = M5IOE1_DEFAULT_ADDR,
               int sda = 21, int scl = 22, uint32_t speed = M5IOE1_I2C_FREQ_100K,
               m5ioe1_int_mode_t intMode = M5IOE1_INT_MODE_POLLING);

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
     * @param intMode Interrupt mode: M5IOE1_INT_MODE_HARDWARE, POLLING, or DISABLED
     * @return true if successful
     */
    bool begin(i2c_port_t port, uint8_t addr, int sda, int scl, uint32_t speed,
               int intPin, m5ioe1_int_mode_t intMode = M5IOE1_INT_MODE_HARDWARE);

    // =====================================================
    // Type 2A: Existing i2c_master_bus_handle_t, no hardware interrupt
    // =====================================================
    /**
     * @brief Initialize with existing i2c_master_bus handle (ESP-IDF native driver)
     * @param bus Existing i2c_master_bus_handle_t
     * @param addr I2C address (default 0x6F)
     * @param speed I2C speed in Hz (for device handle creation)
     * @param intMode Interrupt mode: M5IOE1_INT_MODE_POLLING or M5IOE1_INT_MODE_DISABLED
     * @return true if successful
     */
    bool begin(i2c_master_bus_handle_t bus, uint8_t addr = M5IOE1_DEFAULT_ADDR,
               uint32_t speed = M5IOE1_I2C_FREQ_100K,
               m5ioe1_int_mode_t intMode = M5IOE1_INT_MODE_POLLING);

    // =====================================================
    // Type 2B: Existing i2c_master_bus_handle_t, with hardware interrupt
    // =====================================================
    /**
     * @brief Initialize with existing i2c_master_bus handle, with hardware interrupt
     * @param bus Existing i2c_master_bus_handle_t
     * @param addr I2C address
     * @param speed I2C speed in Hz
     * @param intPin Hardware interrupt GPIO pin
     * @param intMode Interrupt mode
     * @return true if successful
     */
    bool begin(i2c_master_bus_handle_t bus, uint8_t addr, uint32_t speed,
               int intPin, m5ioe1_int_mode_t intMode = M5IOE1_INT_MODE_HARDWARE);

    // =====================================================
    // Type 3A: Existing i2c_bus_handle_t, no hardware interrupt
    // =====================================================
    /**
     * @brief Initialize with existing i2c_bus handle (esp-idf-lib component)
     * @param bus Existing i2c_bus_handle_t
     * @param addr I2C address (default 0x6F)
     * @param speed I2C speed in Hz (for device handle creation)
     * @param intMode Interrupt mode: M5IOE1_INT_MODE_POLLING or M5IOE1_INT_MODE_DISABLED
     * @return true if successful
     */
    bool begin(i2c_bus_handle_t bus, uint8_t addr = M5IOE1_DEFAULT_ADDR,
               uint32_t speed = M5IOE1_I2C_FREQ_100K,
               m5ioe1_int_mode_t intMode = M5IOE1_INT_MODE_POLLING);

    // =====================================================
    // Type 3B: Existing i2c_bus_handle_t, with hardware interrupt
    // =====================================================
    /**
     * @brief Initialize with existing i2c_bus handle, with hardware interrupt
     * @param bus Existing i2c_bus_handle_t
     * @param addr I2C address
     * @param speed I2C speed in Hz
     * @param intPin Hardware interrupt GPIO pin
     * @param intMode Interrupt mode
     * @return true if successful
     */
    bool begin(i2c_bus_handle_t bus, uint8_t addr, uint32_t speed,
               int intPin, m5ioe1_int_mode_t intMode = M5IOE1_INT_MODE_HARDWARE);
#endif

    /**
     * @brief Set interrupt handling mode
     * @param intMode Interrupt mode (DISABLED, POLLING, HARDWARE)
     * @param pollingIntervalMs Polling interval in ms (for polling mode, default 5000ms)
     * @return true if successful
     */
    bool setInterruptMode(m5ioe1_int_mode_t intMode, uint32_t pollingIntervalMs = 5000);

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
    // 设备信息
    // Device Information
    // ========================
    bool getUID(uint16_t* uid);
    bool getVersion(uint8_t* version);
    bool getRefVoltage(uint16_t* voltage_mv);

    // ========================
    // GPIO 功能（Arduino 风格）
    // GPIO Functions (Arduino-style)
    // ========================
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t value);
    int digitalRead(uint8_t pin);

    // ========================
    // 高级 GPIO 功能
    // Advanced GPIO Functions
    // ========================
    bool setPullMode(uint8_t pin, uint8_t pullMode);
    bool setDriveMode(uint8_t pin, uint8_t driveMode);
    bool getInputState(uint8_t pin, uint8_t* state);

    // ========================
    // 中断功能
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
    // ADC 功能
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
    // 温度传感器
    // Temperature Sensor
    // ========================
    bool readTemperature(uint16_t* temperature);
    bool isTemperatureBusy();

    // ========================
    // PWM 功能
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
    bool getPwmDuty(uint8_t channel, uint8_t* duty, bool* polarity, bool* enable);

    /**
     * @brief Set PWM duty cycle (12-bit value)
     * @param channel PWM channel (0-3)
     * @param duty12 12-bit duty value (0-4095)
     * @param polarity PWM polarity
     * @param enable Enable PWM output
     * @return true if successful
     */
    bool setPwmDuty12bit(uint8_t channel, uint16_t duty12, bool polarity = false, bool enable = true);
    bool getPwmDuty12bit(uint8_t channel, uint16_t* duty12, bool* polarity, bool* enable);

    /**
     * @brief Arduino-compatible analogWrite function (PWM output)
     *        Arduino 兼容的 analogWrite 函数（PWM 输出）
     * @param channel PWM channel (0-3): M5IOE1_PWM_CH1/CH2/CH3/CH4
     *               PWM 通道（0-3）：M5IOE1_PWM_CH1/CH2/CH3/CH4
     *               CH1=IO9, CH2=IO8, CH3=IO11, CH4=IO10
     * @param value PWM duty cycle (0-255, 8-bit Arduino standard)
     *              PWM 占空比（0-255，8-bit Arduino 标准）
     *              0 = 0% duty, 127 = 50% duty, 255 = 100% duty
     * @return true if successful
     * @note This function scales 8-bit value to 12-bit internally
     *       此函数内部将 8-bit 值缩放到 12-bit
     * @note Value 0 turns off PWM output
     *       值为 0 时关闭 PWM 输出
     */
    bool analogWrite(uint8_t channel, uint8_t value);

    // ========================
    // NeoPixel LED 功能
    // NeoPixel LED Functions
    // ========================
    bool setLedCount(uint8_t count);
    bool setLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
    bool setLedColor(uint8_t index, m5ioe1_rgb_t color);
    bool refreshLeds();
    bool disableLeds();

    // ========================
    // AW8737A 脉冲功能
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
    // RTC RAM 功能
    // RTC RAM Functions
    // ========================
    bool writeRtcRAM(uint8_t offset, const uint8_t* data, uint8_t length);
    bool readRtcRAM(uint8_t offset, uint8_t* data, uint8_t length);

    // ========================
    // 系统配置
    // System Configuration
    // ========================

    /**
     * @brief Set all I2C configuration at once / 一次性设置所有 I2C 配置
     * @param sleepTime Sleep timeout (0=disabled, 1-15=timeout value)
     * @param sleepTime 休眠超时（0=禁用，1-15=超时值）
     * @param speed I2C speed (M5IOE1_I2C_SPEED_100K or M5IOE1_I2C_SPEED_400K)
     * @param speed I2C 速度（M5IOE1_I2C_SPEED_100K 或 M5IOE1_I2C_SPEED_400K）
     * @param wakeEdge Wake edge (M5IOE1_WAKE_EDGE_FALLING or M5IOE1_WAKE_EDGE_RISING)
     * @param wakeEdge 唤醒边沿（M5IOE1_WAKE_EDGE_FALLING 或 M5IOE1_WAKE_EDGE_RISING）
     * @param pullConfig Pull-up config (M5IOE1_PULL_ENABLED or M5IOE1_PULL_DISABLED)
     * @param pullConfig 上拉配置（M5IOE1_PULL_ENABLED 或 M5IOE1_PULL_DISABLED）
     * @return true if successful
     */
    bool setI2cConfig(uint8_t sleepTime,
                      m5ioe1_i2c_speed_t speed = M5IOE1_I2C_SPEED_100K,
                      m5ioe1_wake_edge_t wakeEdge = M5IOE1_WAKE_EDGE_FALLING,
                      m5ioe1_pull_config_t pullConfig = M5IOE1_PULL_ENABLED);

    /**
     * @brief Switch I2C communication speed / 切换 I2C 通讯速度
     * @param speed Target speed (M5IOE1_I2C_SPEED_100K or M5IOE1_I2C_SPEED_400K)
     * @param speed 目标速度 (M5IOE1_I2C_SPEED_100K 或 M5IOE1_I2C_SPEED_400K)
     * @return true if successful, false otherwise
     * @note This function will configure both the device and host I2C bus
     * @note 此函数会同时配置设备和主机 I2C 总线
     */
    bool switchI2cSpeed(m5ioe1_i2c_speed_t speed);

    /**
     * @brief Get current I2C speed setting / 获取当前 I2C 速度设置
     * @param speed Pointer to store the speed value / 存储速度值的指针
     * @return true if successful, false if read failed
     * @note Reads from device register and updates internal cache
     * @note 从设备寄存器读取并更新内部缓存
     */
    bool getI2cSpeed(m5ioe1_i2c_speed_t* speed);

    /**
     * @brief Set I2C sleep timeout / 设置 I2C 休眠超时
     * @param sleepTime Sleep timeout value (0=disabled, 1-15=timeout)
     * @param sleepTime 休眠超时值（0=禁用，1-15=超时）
     * @return true if successful
     * @note Sleep time formula: T = sleepTime * base_time
     * @note 休眠时间公式：T = sleepTime * 基础时间
     */
    bool setI2cSleepTime(uint8_t sleepTime);

    /**
     * @brief Get current I2C sleep timeout / 获取当前 I2C 休眠超时
     * @param sleepTime Pointer to store the sleep timeout value / 存储休眠超时值的指针
     * @return true if successful, false if read failed
     * @note Reads from device register and updates internal cache
     * @note 从设备寄存器读取并更新内部缓存
     */
    bool getI2cSleepTime(uint8_t* sleepTime);

    /**
     * @brief Set I2C wake edge / 设置 I2C 唤醒边沿
     * @param edge Wake edge (M5IOE1_WAKE_EDGE_FALLING or M5IOE1_WAKE_EDGE_RISING)
     * @param edge 唤醒边沿（M5IOE1_WAKE_EDGE_FALLING 或 M5IOE1_WAKE_EDGE_RISING）
     * @return true if successful
     */
    bool setI2cWakeEdge(m5ioe1_wake_edge_t edge);

    /**
     * @brief Get current I2C wake edge / 获取当前 I2C 唤醒边沿
     * @param edge Pointer to store the wake edge setting / 存储唤醒边沿设置的指针
     * @return true if successful, false if read failed
     * @note Reads from device register and updates internal cache
     * @note 从设备寄存器读取并更新内部缓存
     */
    bool getI2cWakeEdge(m5ioe1_wake_edge_t* edge);

    /**
     * @brief Set I2C internal pull-up configuration / 设置 I2C 内部上拉配置
     * @param config Pull-up config (M5IOE1_PULL_ENABLED or M5IOE1_PULL_DISABLED)
     * @param config 上拉配置（M5IOE1_PULL_ENABLED 或 M5IOE1_PULL_DISABLED）
     * @return true if successful
     */
    bool setI2cPullConfig(m5ioe1_pull_config_t config);

    /**
     * @brief Get current I2C internal pull-up configuration / 获取当前 I2C 内部上拉配置
     * @param config Pointer to store the pull-up configuration / 存储上拉配置的指针
     * @return true if successful, false if read failed
     * @note Reads from device register and updates internal cache
     * @note 从设备寄存器读取并更新内部缓存
     */
    bool getI2cPullConfig(m5ioe1_pull_config_t* config);

    /**
     * @brief Factory reset the device / 恢复出厂设置
     * @return true if successful
     */
    bool factoryReset();

    // ========================
    // 自动唤醒功能
    // Auto Wake Feature
    // ========================
    /**
     * @brief Enable/disable automatic wake signal before I2C operations
     *        启用/禁用 I2C 操作前的自动唤醒信号
     * @param enable true to enable auto-wake, false to disable
     * @note When IOE1 enters sleep mode (I2C sleep timeout), it needs a
     *       START signal on SDA to wake up. This feature automatically
     *       sends the wake signal when needed.
     *       当 IOE1 进入睡眠模式（I2C 睡眠超时）后，需要在 SDA 上发送
     *       START 信号来唤醒。此功能会在需要时自动发送唤醒信号。
     * @note Even without enabling this option, communication will likely
     *       succeed in most cases, as the first I2C transaction itself
     *       can wake the device. Enable this for guaranteed reliability.
     *       即使不启用此选项，通讯在大多数情况下也能成功，因为第一次
     *       I2C 传输本身就能唤醒设备。启用此选项可确保可靠性。
     */
    void setAutoWakeEnable(bool enable);

    /**
     * @brief Check if auto wake is enabled / 检查自动唤醒是否启用
     * @return true if enabled
     */
    bool isAutoWakeEnabled() const;

    /**
     * @brief Manually send wake signal to IOE1 / 手动发送唤醒信号到 IOE1
     * @return true if successful
     */
    bool sendWakeSignal();

    // ========================
    // 状态快照功能
    // State Snapshot Functions
    // ========================
    void setAutoSnapshot(bool enable);
    bool isAutoSnapshotEnabled() const;
    bool updateSnapshot();

    // ========================
    // 调试功能
    // Debug Functions
    // ========================
    bool getModeReg(uint16_t* reg);
    bool getOutputReg(uint16_t* reg);
    bool getInputReg(uint16_t* reg);
    bool getPullUpReg(uint16_t* reg);
    bool getPullDownReg(uint16_t* reg);
    bool getDriveReg(uint16_t* reg);

    // ========================
    // 配置验证
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
    // 快照验证
    // Snapshot Verification
    // ========================
    /**
     * @brief Verify that cached state matches actual hardware registers
     * @return Verification result with mismatch details
     */
    m5ioe1_snapshot_verify_t verifySnapshot();

    // ========================
    // 缓存状态查询函数
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
    // 设备状态
    // Device state
    uint8_t _addr;
    bool _initialized;
    bool _autoSnapshot;
    bool _enableDefaultIsrLog;
    uint32_t _requestedSpeed;  // 用户请求的 I2C 速度（用于 400K 切换）
                            // User requested I2C speed (for 400K switch)

    // 自动唤醒状态
    // Auto wake state
    bool _autoWakeEnabled;      // 自动唤醒是否启用
                                // Whether auto wake is enabled
    uint32_t _lastCommTime;     // 上次通信时间（毫秒）
                                // Last communication time (milliseconds)

    // 中断模式
    // Interrupt mode
    m5ioe1_int_mode_t _intMode;
    int8_t _intPin;
    uint32_t _pollingInterval;

#ifdef ARDUINO
    TwoWire *_wire;
    uint8_t _sda;   // SDA 引脚编号，用于 I2C 重新初始化
                    // SDA pin number for I2C re-initialization
    uint8_t _scl;   // SCL 引脚编号，用于 I2C 重新初始化
                    // SCL pin number for I2C re-initialization
#else
    // I2C 驱动类型选择
    // I2C driver type selection
    m5ioe1_i2c_driver_t _i2cDriverType;

    // I2C 句柄（根据驱动类型仅使用一对）
    // I2C handles (only one pair is used based on driver type)
    // M5IOE1_I2C_DRIVER_SELF_CREATED: 使用 _i2c_master_bus + _i2c_master_dev
    // uses _i2c_master_bus + _i2c_master_dev
    // M5IOE1_I2C_DRIVER_MASTER: 使用 _i2c_master_bus + _i2c_master_dev
    // uses _i2c_master_bus + _i2c_master_dev
    // M5IOE1_I2C_DRIVER_BUS: 使用 _i2c_bus + _i2c_device
    // uses _i2c_bus + _i2c_device
    i2c_master_bus_handle_t _i2c_master_bus;  // ESP-IDF 原生驱动/自创建
                                            // ESP-IDF native driver / self-created
    i2c_master_dev_handle_t _i2c_master_dev;  // ESP-IDF 原生驱动/自创建
                                            // ESP-IDF native driver / self-created
    i2c_bus_handle_t _i2c_bus;                 // esp-idf-lib 组件
                                            // esp-idf-lib component
    i2c_bus_device_handle_t _i2c_device;       // esp-idf-lib 组件
                                            // esp-idf-lib component

    // I2C 管理标志
    // I2C management flags
    bool _busExternal;  // 如果总线句柄由外部提供则为 true
                        // true if bus handle is provided externally

    // 自创建总线的 I2C 引脚（用于频率切换）
    // I2C pins for self-created bus (for frequency switching)
    int _sda;
    int _scl;
    i2c_port_t _port;

    // 中断处理
    // Interrupt handling
    TaskHandle_t _pollTask;
    QueueHandle_t _intrQueue;
#endif

    // 中断回调
    // Interrupt callbacks
    struct {
        m5ioe1_callback_t callback;
        m5ioe1_callback_arg_t callbackArg;
        void* arg;
        bool enabled;
        bool rising;
    } _callbacks[M5IOE1_MAX_GPIO_PINS];

    // 缓存的引脚状态
    // Cached pin states
    struct {
        bool isOutput;
        uint8_t outputLevel;
        uint8_t inputLevel;
        uint8_t pull;       // 0:无, 1:上拉, 2:下拉
                            // 0:none, 1:up, 2:down
        uint8_t drive;      // 0:推挽, 1:开漏
                            // 0:push-pull, 1:open-drain
        bool intrEnabled;
        bool intrRising;
    } _pinStates[M5IOE1_MAX_GPIO_PINS];
    bool _pinStatesValid;

    // 缓存的 PWM 状态
    // Cached PWM states
    struct {
        uint16_t duty12;
        bool enabled;
        bool polarity;
    } _pwmStates[M5IOE1_MAX_PWM_CHANNELS];
    uint16_t _pwmFrequency;
    bool _pwmStatesValid;

    // 缓存的 ADC 状态
    // Cached ADC state
    struct {
        uint8_t activeChannel;
        bool busy;
        uint16_t lastValue;
    } _adcState;
    bool _adcStateValid;

    // 缓存的 I2C 配置状态（用于睡眠模式检测）
    // Cached I2C config state (for sleep mode detection)
    struct {
        uint8_t sleepTime;      // 0=禁用, 1-15=睡眠时间
                                // 0=disabled, 1-15=sleep time
        bool speed400k;         // I2C 速度模式
                                // I2C speed mode
        bool wakeRising;        // 唤醒边沿模式
                                // Wake edge mode
        bool pullOff;           // 内部上拉关闭
                                // Internal pull-up off
    } _i2cConfig;
    bool _i2cConfigValid;

    // NeoPixel 状态
    // NeoPixel state
    uint8_t _ledCount;
    bool _ledEnabled;

    // ========================
    // 内部辅助函数
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

    // 自动唤醒检查
    // Auto wake check
    void _checkAutoWake();

    // I2C 频率验证
    // I2C frequency validation
    bool _isValidI2cFrequency(uint32_t speed);

    // 中断互斥对检查
    // Interrupt mutex pairs check
    static bool _pinsConflict(uint8_t a, uint8_t b);
    bool _hasConflictingInterrupt(uint8_t pin);

    // 配置验证辅助函数
    // Configuration validation helpers
    bool _getInterruptMutexPin(uint8_t pin, uint8_t* mutexPin);
    bool _isNeopixelPin(uint8_t pin);
    bool _hasActiveInterrupt(uint8_t pin);
    bool _hasActiveAdc(uint8_t pin);
    bool _hasActivePwm(uint8_t pin);
    bool _hasI2cSleepEnabled();
    bool _isLedEnabled();

    // I2C 配置快照
    // I2C config snapshot
    void _clearI2cConfig();
    bool _snapshotI2cConfig();

#ifdef ARDUINO
    // Arduino 专用
    // Arduino specific
    bool _setupPollingArduino();
    void _cleanupPollingArduino();
    static void _pollTaskArduino(void* arg);
#else
    // ESP-IDF 专用
    // ESP-IDF specific
    static void _pollTaskFunc(void* arg);
    static void IRAM_ATTR _isrHandler(void* arg);
    bool _setupHardwareInterrupt();
    bool _setupPolling();
    void _cleanupInterrupt();
#endif
};

#endif // _M5IOE1_H_
