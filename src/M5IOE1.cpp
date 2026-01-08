/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#include "M5IOE1.h"
#include <string.h>

static const char* TAG = "M5IOE1";

#ifdef ARDUINO
    #include <Arduino.h>
    #define M5IOE1_DELAY_MS(ms) delay(ms)
    #define M5IOE1_GET_TIME_MS() millis()

    // Arduino 日志级别控制
    // Arduino log level control
    static m5ioe1_log_level_t _m5ioe1_log_level = M5IOE1_LOG_LEVEL_INFO;
    
    #define M5IOE1_LOG_I(tag, fmt, ...) do { \
        if (_m5ioe1_log_level >= M5IOE1_LOG_LEVEL_INFO) { \
            Serial.printf("[%s] " fmt "\r\n", tag, ##__VA_ARGS__); \
        } \
    } while(0)
    
    #define M5IOE1_LOG_W(tag, fmt, ...) do { \
        if (_m5ioe1_log_level >= M5IOE1_LOG_LEVEL_WARN) { \
            Serial.printf("[%s] WARN: " fmt "\r\n", tag, ##__VA_ARGS__); \
        } \
    } while(0)
    
    #define M5IOE1_LOG_E(tag, fmt, ...) do { \
        if (_m5ioe1_log_level >= M5IOE1_LOG_LEVEL_ERROR) { \
            Serial.printf("[%s] ERROR: " fmt "\r\n", tag, ##__VA_ARGS__); \
        } \
    } while(0)
#else
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include "driver/gpio.h"
    #define M5IOE1_DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))
    #define M5IOE1_GET_TIME_MS() (xTaskGetTickCount() * portTICK_PERIOD_MS)
    #define M5IOE1_LOG_I(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
    #define M5IOE1_LOG_W(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)
    #define M5IOE1_LOG_E(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)
    
    // ESP-IDF 平台日志级别控制
    // ESP-IDF platform log level control
    static m5ioe1_log_level_t _m5ioe1_current_log_level = M5IOE1_LOG_LEVEL_INFO;
#endif

// ============================
// 全局日志级别控制
// Global Log Level Control
// ============================

void M5IOE1::setLogLevel(m5ioe1_log_level_t level) {
#ifdef ARDUINO
    _m5ioe1_log_level = level;
#else
    _m5ioe1_current_log_level = level;
    
    // 将 M5IOE1 日志级别映射到 ESP-IDF 日志级别
    // Map M5IOE1 log level to ESP-IDF log level
    esp_log_level_t esp_level;
    switch (level) {
        case M5IOE1_LOG_LEVEL_NONE:
            esp_level = ESP_LOG_NONE;
            break;
        case M5IOE1_LOG_LEVEL_ERROR:
            esp_level = ESP_LOG_ERROR;
            break;
        case M5IOE1_LOG_LEVEL_WARN:
            esp_level = ESP_LOG_WARN;
            break;
        case M5IOE1_LOG_LEVEL_INFO:
            esp_level = ESP_LOG_INFO;
            break;
        case M5IOE1_LOG_LEVEL_DEBUG:
            esp_level = ESP_LOG_DEBUG;
            break;
        case M5IOE1_LOG_LEVEL_VERBOSE:
            esp_level = ESP_LOG_VERBOSE;
            break;
        default:
            esp_level = ESP_LOG_INFO;
            break;
    }
    
    esp_log_level_set(TAG, esp_level);
#endif
}

m5ioe1_log_level_t M5IOE1::getLogLevel() {
#ifdef ARDUINO
    return _m5ioe1_log_level;
#else
    return _m5ioe1_current_log_level;
#endif
}

void M5IOE1::enableDefaultInterruptLog(bool enable) {
    _enableDefaultIsrLog = enable;
}

// ============================
// 构造函数
// 析构函数
// ============================

M5IOE1::M5IOE1() {
    _addr = M5IOE1_DEFAULT_ADDR;
    _initialized = false;
    _autoSnapshot = true;
    _enableDefaultIsrLog = false;
    _requestedSpeed = M5IOE1_I2C_FREQ_DEFAULT;
    _autoWakeEnabled = false;
    _lastCommTime = 0;
    _intMode = M5IOE1_INT_MODE_DISABLED;
    _intPin = -1;
    _pollingInterval = 5000;
    _pinStatesValid = false;
    _pwmStatesValid = false;
    _pwmFrequency = 0;
    _adcStateValid = false;
    _i2cConfigValid = false;
    _ledCount = 0;
    _ledEnabled = false;

#ifdef ARDUINO
    _wire = nullptr;
    _sda = 0;
    _scl = 0;
#else
    _i2cDriverType = M5IOE1_I2C_DRIVER_NONE;
    _i2c_master_bus = nullptr;
    _i2c_master_dev = nullptr;
    _i2c_bus = nullptr;
    _i2c_device = nullptr;
    _busExternal = false;
    _sda = -1;
    _scl = -1;
    _port = I2C_NUM_0;
    _pollTask = nullptr;
    _intrQueue = nullptr;
#endif

    memset(_callbacks, 0, sizeof(_callbacks));
    _clearPinStates();
    _clearPwmStates();
    _clearAdcState();
    _clearI2cConfig();
}

M5IOE1::~M5IOE1() {
#ifdef ARDUINO
    _cleanupPollingArduino();
#else
    _cleanupInterrupt();

    // 根据驱动类型进行清理
    // Cleanup based on driver type
    switch (_i2cDriverType) {
        case M5IOE1_I2C_DRIVER_SELF_CREATED:
            // 自创建：先删除设备，再删除总线
            // Self-created: delete device first, then bus
            if (_i2c_master_dev) {
                i2c_master_bus_rm_device(_i2c_master_dev);
                _i2c_master_dev = nullptr;
            }
            if (_i2c_master_bus) {
                i2c_del_master_bus(_i2c_master_bus);
                _i2c_master_bus = nullptr;
            }
            break;

        case M5IOE1_I2C_DRIVER_MASTER:
            // 外部 i2c_master：总是删除我们创建的设备句柄，但不删除总线
            // External i2c_master: always delete the device handle we created, but not the bus
            if (_i2c_master_dev) {
                i2c_master_bus_rm_device(_i2c_master_dev);
                _i2c_master_dev = nullptr;
            }
            break;

        case M5IOE1_I2C_DRIVER_BUS:
            // 外部 i2c_bus：总是删除我们创建的设备句柄，但不删除总线
            // External i2c_bus: always delete the device handle we created, but not the bus
            if (_i2c_device) {
                i2c_bus_device_delete(&_i2c_device);
                _i2c_device = nullptr;
            }
            break;

        default:
            break;
    }
#endif
}

// ============================
// 初始化函数
// Initialization Functions
// ============================

#ifdef ARDUINO

bool M5IOE1::begin(TwoWire *wire, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t speed, m5ioe1_int_mode_t mode) {
    _wire = wire;
    _addr = addr;
    _sda = sda;   // 保存 SDA 引脚用于 I2C 重新初始化
    _scl = scl;   // 保存 SCL 引脚用于 I2C 重新初始化
    _intPin = -1;

    // 验证 I2C 频率 - M5IOE1 仅支持 100KHz 或 400KHz
    // Validate I2C frequency - M5IOE1 only supports 100KHz or 400KHz
    if (!_isValidI2cFrequency(speed)) {
        M5IOE1_LOG_W(TAG, "Invalid I2C frequency: %lu Hz. M5IOE1 only supports 100KHz or 400KHz. Falling back to 100KHz.", speed);
        _requestedSpeed = M5IOE1_I2C_FREQ_100K;
    } else {
        _requestedSpeed = speed;
    }

    // 始终以 100KHz 开始 - M5IOE1 在上电/复位后默认为 100KHz
    // Always start with 100KHz - M5IOE1 defaults to 100KHz after power-on/reset
    M5IOE1_LOG_I(TAG, "Initializing M5IOE1 with 100KHz (device default)");

    // 在开始新的 I2C 会话之前结束之前的会话（修复 ESP_ERR_INVALID_STATE）
    // End any previous I2C session before starting new one (fixes ESP_ERR_INVALID_STATE)
    // 注意：ESP32 Arduino Wire 库需要足够的延时
    // Note: ESP32 Arduino Wire library needs sufficient delay
    _wire->end();
    M5IOE1_DELAY_MS(50);

    // 初始化 I2C 总线并检查返回值
    // Initialize I2C bus and check return value
    if (!_wire->begin(sda, scl, M5IOE1_I2C_FREQ_100K)) {
        M5IOE1_LOG_E(TAG, "Failed to initialize I2C bus (SDA=%d, SCL=%d)", sda, scl);
        return false;
    }

    // 给 I2C 总线时间在初始化后稳定
    // Give the I2C bus time to stabilize after initialization
    M5IOE1_DELAY_MS(100);

    // 尝试唤醒设备 - 发送 I2C START 信号
    // Try to wake up the device - send I2C START signal
    // M5IOE1 可能处于睡眠状态，需要先唤醒
    // M5IOE1 may be in sleep mode, need to wake up first
    M5IOE1_I2C_SEND_WAKE(_wire, _addr);
    M5IOE1_DELAY_MS(10);

    // 步骤1: 先尝试100K通信
    // Step 1: Try 100K communication first
    if (!_initDevice()) {
        // 步骤2: 100K失败，尝试400K
        // Step 2: 100K failed, try 400K
        M5IOE1_LOG_W(TAG, "Failed at 100KHz, trying 400KHz...");

        _wire->end();
        M5IOE1_DELAY_MS(50);

        if (!_wire->begin(sda, scl, M5IOE1_I2C_FREQ_400K)) {
            M5IOE1_LOG_E(TAG, "Failed to initialize I2C bus at 400KHz");
            return false;
        }
        M5IOE1_DELAY_MS(100);

        M5IOE1_I2C_SEND_WAKE(_wire, _addr);
        M5IOE1_DELAY_MS(10);

        if (!_initDevice()) {
            // 步骤3: 都失败，初始化失败
            // Step 3: Both failed, initialization failed
            M5IOE1_LOG_E(TAG, "Failed at both 100KHz and 400KHz");
            return false;
        }
    }

    // 步骤4: 通信成功，设置为已初始化
    // Step 4: Communication succeeded, set as initialized
    _initialized = true;

    // 步骤5: 强制配置I2C（用户请求的频率 + 关闭休眠）
    // Step 5: Force configure I2C (user requested speed + disable sleep)
    m5ioe1_i2c_speed_t targetSpeed = (_requestedSpeed == M5IOE1_I2C_FREQ_400K)
        ? M5IOE1_I2C_SPEED_400K : M5IOE1_I2C_SPEED_100K;

    // 使用setI2cConfig一次性配置：sleepTime=0, 用户速度, 默认唤醒边沿, 默认上拉
    // Use setI2cConfig to configure at once: sleepTime=0, user speed, default wake edge, default pull
    if (!setI2cConfig(0, targetSpeed, M5IOE1_WAKE_EDGE_FALLING, M5IOE1_PULL_ENABLED)) {
        M5IOE1_LOG_W(TAG, "Failed to set I2C config");
    }

    // 步骤6: 切换主机I2C总线到用户请求的速度
    // Step 6: Switch host I2C bus to user requested speed
    _wire->end();
    M5IOE1_DELAY_MS(10);
    if (!_wire->begin(sda, scl, _requestedSpeed)) {
        M5IOE1_LOG_E(TAG, "Failed to switch host to %lu Hz", _requestedSpeed);
        return false;
    }
    M5IOE1_DELAY_MS(50);

    // 步骤7: 快照
    // Step 7: Snapshot
    _snapshotPinStates();
    _snapshotPwmStates();
    _snapshotAdcState();
    _snapshotI2cConfig();

    M5IOE1_LOG_I(TAG, "M5IOE1 initialized at address 0x%02X (I2C: %lu Hz)", _addr, _requestedSpeed);

    // 如果未禁用，设置中断模式
    // Set interrupt mode if not disabled
    if (mode != M5IOE1_INT_MODE_DISABLED) {
        setInterruptMode(mode);
    }

    return true;
}

bool M5IOE1::begin(TwoWire *wire, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t speed, int8_t intPin, m5ioe1_int_mode_t mode) {
    if (!begin(wire, addr, sda, scl, speed, M5IOE1_INT_MODE_DISABLED)) return false;
    
    _intPin = intPin;
    if (_intPin >= 0) {
        return setInterruptMode(mode);
    }
    return true;
}

#else // ESP-IDF

// =====================================================
// Type 1A: Self-created I2C bus, no hardware interrupt
// =====================================================
bool M5IOE1::begin(i2c_port_t port, uint8_t addr, int sda, int scl, uint32_t speed, m5ioe1_int_mode_t mode) {
    _addr = addr;
    _busExternal = false;
    _i2cDriverType = M5IOE1_I2C_DRIVER_SELF_CREATED;
    _intPin = -1;
    _port = port;
    _sda = sda;
    _scl = scl;

    // 验证 I2C 频率 - M5IOE1 仅支持 100KHz 或 400KHz
    // Validate I2C frequency - M5IOE1 only supports 100KHz or 400KHz
    if (!_isValidI2cFrequency(speed)) {
        M5IOE1_LOG_W(TAG, "Invalid I2C frequency: %lu Hz. M5IOE1 only supports 100KHz or 400KHz. Falling back to 100KHz.", speed);
        _requestedSpeed = M5IOE1_I2C_FREQ_100K;
    } else {
        _requestedSpeed = speed;
    }

    // 始终以 100KHz 开始 - M5IOE1 在上电/复位后默认为 100KHz
    // Always start with 100KHz - M5IOE1 defaults to 100KHz after power-on/reset
    M5IOE1_LOG_I(TAG, "Initializing M5IOE1 with 100KHz (device default)");

    // 使用 ESP-IDF 原生驱动创建 I2C 主总线
    // Create I2C master bus using ESP-IDF native driver
    i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = (gpio_num_t)sda,
        .scl_io_num = (gpio_num_t)scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &_i2c_master_bus);
    if (ret != ESP_OK) {
        M5IOE1_LOG_E(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return false;
    }

    // 在 100KHz 创建设备句柄
    // Create device handle at 100KHz
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _addr,
        .scl_speed_hz = M5IOE1_I2C_FREQ_100K,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    // 在 100KHz 创建设备句柄
    // Create device handle at 100KHz
    ret = i2c_master_bus_add_device(_i2c_master_bus, &dev_config, &_i2c_master_dev);
    if (ret != ESP_OK) {
        M5IOE1_LOG_E(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(_i2c_master_bus);
        _i2c_master_bus = nullptr;
        return false;
    }

    // 尝试唤醒设备 - 发送 I2C START 信号
    // Try to wake up the device - send I2C START signal
    // M5IOE1 可能处于睡眠状态，需要先唤醒
    // M5IOE1 may be in sleep mode, need to wake up first
    M5IOE1_I2C_MASTER_SEND_WAKE(_i2c_master_bus, _addr);
    M5IOE1_DELAY_MS(10);

    // 步骤1: 先尝试100K通信
    // Step 1: Try 100K communication first
    if (!_initDevice()) {
        // 步骤2: 100K失败，尝试400K
        // Step 2: 100K failed, try 400K
        M5IOE1_LOG_W(TAG, "Failed at 100KHz, trying 400KHz...");

        // 删除当前100K设备句柄
        // Remove current 100K device handle
        i2c_master_bus_rm_device(_i2c_master_dev);
        _i2c_master_dev = nullptr;

        // 以400K重新创建设备句柄
        // Recreate device handle at 400K
        i2c_device_config_t dev_config_400k = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = _addr,
            .scl_speed_hz = M5IOE1_I2C_FREQ_400K,
            .scl_wait_us = 0,
            .flags = {
                .disable_ack_check = false,
            },
        };

        ret = i2c_master_bus_add_device(_i2c_master_bus, &dev_config_400k, &_i2c_master_dev);
        if (ret != ESP_OK) {
            M5IOE1_LOG_E(TAG, "Failed to add I2C device at 400KHz: %s", esp_err_to_name(ret));
            i2c_del_master_bus(_i2c_master_bus);
            _i2c_master_bus = nullptr;
            return false;
        }

        M5IOE1_I2C_MASTER_SEND_WAKE(_i2c_master_bus, _addr);
        M5IOE1_DELAY_MS(10);

        if (!_initDevice()) {
            // 步骤3: 都失败，初始化失败
            // Step 3: Both failed, initialization failed
            M5IOE1_LOG_E(TAG, "Failed at both 100KHz and 400KHz");
            i2c_master_bus_rm_device(_i2c_master_dev);
            i2c_del_master_bus(_i2c_master_bus);
            _i2c_master_dev = nullptr;
            _i2c_master_bus = nullptr;
            return false;
        }
    }

    // 步骤4: 通信成功，设置为已初始化
    // Step 4: Communication succeeded, set as initialized
    _initialized = true;

    // 步骤5: 强制配置I2C（用户请求的频率 + 关闭休眠）
    // Step 5: Force configure I2C (user requested speed + disable sleep)
    m5ioe1_i2c_speed_t targetSpeed = (_requestedSpeed == M5IOE1_I2C_FREQ_400K)
        ? M5IOE1_I2C_SPEED_400K : M5IOE1_I2C_SPEED_100K;

    if (!setI2cConfig(0, targetSpeed, M5IOE1_WAKE_EDGE_FALLING, M5IOE1_PULL_ENABLED)) {
        M5IOE1_LOG_W(TAG, "Failed to set I2C config");
    }

    // 步骤6: 切换设备句柄到用户请求的速度
    // Step 6: Switch device handle to user requested speed
    i2c_master_bus_rm_device(_i2c_master_dev);
    _i2c_master_dev = nullptr;

    i2c_device_config_t dev_config_target = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _addr,
        .scl_speed_hz = _requestedSpeed,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    ret = i2c_master_bus_add_device(_i2c_master_bus, &dev_config_target, &_i2c_master_dev);
    if (ret != ESP_OK) {
        M5IOE1_LOG_E(TAG, "Failed to switch device to %lu Hz: %s", _requestedSpeed, esp_err_to_name(ret));
        i2c_del_master_bus(_i2c_master_bus);
        _i2c_master_bus = nullptr;
        return false;
    }

    // 步骤7: 快照
    // Step 7: Snapshot
    _snapshotPinStates();
    _snapshotPwmStates();
    _snapshotAdcState();
    _snapshotI2cConfig();

    M5IOE1_LOG_I(TAG, "M5IOE1 initialized at address 0x%02X (I2C: %lu Hz)", _addr, _requestedSpeed);

    // 如果未禁用，设置中断模式
    // Set interrupt mode if not disabled
    if (mode != M5IOE1_INT_MODE_DISABLED) {
        setInterruptMode(mode);
    }

    return true;
}

// =====================================================
// Type 1B: Self-created I2C bus, with hardware interrupt
// =====================================================
bool M5IOE1::begin(i2c_port_t port, uint8_t addr, int sda, int scl, uint32_t speed, int intPin, m5ioe1_int_mode_t mode) {
    if (!begin(port, addr, sda, scl, speed, M5IOE1_INT_MODE_DISABLED)) {
        return false;
    }
    
    _intPin = intPin;
    if (_intPin >= 0) {
        return setInterruptMode(mode);
    }
    return true;
}

// =====================================================
// Type 2A: Existing i2c_master_bus_handle_t, no hardware interrupt
// =====================================================
bool M5IOE1::begin(i2c_master_bus_handle_t bus, uint8_t addr, uint32_t speed, m5ioe1_int_mode_t mode) {
    _addr = addr;
    _busExternal = true;
    _i2cDriverType = M5IOE1_I2C_DRIVER_MASTER;
    _intPin = -1;
    _i2c_master_bus = bus;
    _sda = -1;  // 外部总线未知
                // Unknown for external bus
    _scl = -1;

    // 验证 I2C 频率
    // Validate I2C frequency
    if (!_isValidI2cFrequency(speed)) {
        M5IOE1_LOG_W(TAG, "Invalid I2C frequency: %lu Hz. Falling back to 100KHz.", speed);
        _requestedSpeed = M5IOE1_I2C_FREQ_100K;
    } else {
        _requestedSpeed = speed;
    }

    M5IOE1_LOG_I(TAG, "Initializing M5IOE1 with 100KHz (device default)");

    // 在 100KHz 创建设备句柄
    // Create device handle at 100KHz
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _addr,
        .scl_speed_hz = M5IOE1_I2C_FREQ_100K,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    // 在 100KHz 创建设备句柄
    // Create device handle at 100KHz
    esp_err_t ret = i2c_master_bus_add_device(_i2c_master_bus, &dev_config, &_i2c_master_dev);
    if (ret != ESP_OK) {
        M5IOE1_LOG_E(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return false;
    }

    // 尝试唤醒设备 - 发送 I2C START 信号
    // Try to wake up the device - send I2C START signal
    // M5IOE1 可能处于睡眠状态，需要先唤醒
    // M5IOE1 may be in sleep mode, need to wake up first
    M5IOE1_I2C_MASTER_SEND_WAKE(_i2c_master_bus, _addr);
    M5IOE1_DELAY_MS(10);

    // 步骤1: 先尝试100K通信
    // Step 1: Try 100K communication first
    if (!_initDevice()) {
        // 步骤2: 100K失败，尝试400K
        // Step 2: 100K failed, try 400K
        M5IOE1_LOG_W(TAG, "Failed at 100KHz, trying 400KHz...");

        // 删除当前100K设备句柄
        // Remove current 100K device handle
        i2c_master_bus_rm_device(_i2c_master_dev);
        _i2c_master_dev = nullptr;

        // 以400K重新创建设备句柄
        // Recreate device handle at 400K
        i2c_device_config_t dev_config_400k = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = _addr,
            .scl_speed_hz = M5IOE1_I2C_FREQ_400K,
            .scl_wait_us = 0,
            .flags = {
                .disable_ack_check = false,
            },
        };

        ret = i2c_master_bus_add_device(_i2c_master_bus, &dev_config_400k, &_i2c_master_dev);
        if (ret != ESP_OK) {
            M5IOE1_LOG_E(TAG, "Failed to add I2C device at 400KHz: %s", esp_err_to_name(ret));
            return false;
        }

        M5IOE1_I2C_MASTER_SEND_WAKE(_i2c_master_bus, _addr);
        M5IOE1_DELAY_MS(10);

        if (!_initDevice()) {
            // 步骤3: 都失败，初始化失败
            // Step 3: Both failed, initialization failed
            M5IOE1_LOG_E(TAG, "Failed at both 100KHz and 400KHz");
            i2c_master_bus_rm_device(_i2c_master_dev);
            _i2c_master_dev = nullptr;
            return false;
        }
    }

    // 步骤4: 通信成功，设置为已初始化
    // Step 4: Communication succeeded, set as initialized
    _initialized = true;

    // 步骤5: 强制配置I2C（用户请求的频率 + 关闭休眠）
    // Step 5: Force configure I2C (user requested speed + disable sleep)
    m5ioe1_i2c_speed_t targetSpeed = (_requestedSpeed == M5IOE1_I2C_FREQ_400K)
        ? M5IOE1_I2C_SPEED_400K : M5IOE1_I2C_SPEED_100K;

    if (!setI2cConfig(0, targetSpeed, M5IOE1_WAKE_EDGE_FALLING, M5IOE1_PULL_ENABLED)) {
        M5IOE1_LOG_W(TAG, "Failed to set I2C config");
    }

    // 步骤6: 切换设备句柄到用户请求的速度
    // Step 6: Switch device handle to user requested speed
    i2c_master_bus_rm_device(_i2c_master_dev);
    _i2c_master_dev = nullptr;

    i2c_device_config_t dev_config_target = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _addr,
        .scl_speed_hz = _requestedSpeed,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    ret = i2c_master_bus_add_device(_i2c_master_bus, &dev_config_target, &_i2c_master_dev);
    if (ret != ESP_OK) {
        M5IOE1_LOG_E(TAG, "Failed to switch device to %lu Hz: %s", _requestedSpeed, esp_err_to_name(ret));
        return false;
    }

    // 步骤7: 快照
    // Step 7: Snapshot
    _snapshotPinStates();
    _snapshotPwmStates();
    _snapshotAdcState();
    _snapshotI2cConfig();

    M5IOE1_LOG_I(TAG, "M5IOE1 initialized at address 0x%02X (I2C: %lu Hz)", _addr, _requestedSpeed);

    if (mode != M5IOE1_INT_MODE_DISABLED) {
        setInterruptMode(mode);
    }

    return true;
}

// =====================================================
// Type 2B: Existing i2c_master_bus_handle_t, with hardware interrupt
// =====================================================
bool M5IOE1::begin(i2c_master_bus_handle_t bus, uint8_t addr, uint32_t speed, int intPin, m5ioe1_int_mode_t mode) {
    if (!begin(bus, addr, speed, M5IOE1_INT_MODE_DISABLED)) {
        return false;
    }
    
    _intPin = intPin;
    if (_intPin >= 0) {
        return setInterruptMode(mode);
    }
    return true;
}

// =====================================================
// Type 3A: Existing i2c_bus_handle_t, no hardware interrupt
// =====================================================
bool M5IOE1::begin(i2c_bus_handle_t bus, uint8_t addr, uint32_t speed, m5ioe1_int_mode_t mode) {
    _addr = addr;
    _busExternal = true;
    _i2cDriverType = M5IOE1_I2C_DRIVER_BUS;
    _intPin = -1;
    _i2c_bus = bus;
    _sda = -1;  // 外部总线未知
                // Unknown for external bus
    _scl = -1;

    // 验证 I2C 频率
    // Validate I2C frequency
    if (!_isValidI2cFrequency(speed)) {
        M5IOE1_LOG_W(TAG, "Invalid I2C frequency: %lu Hz. Falling back to 100KHz.", speed);
        _requestedSpeed = M5IOE1_I2C_FREQ_100K;
    } else {
        _requestedSpeed = speed;
    }

    M5IOE1_LOG_I(TAG, "Initializing M5IOE1 with 100KHz (device default)");

    // 在 100KHz 创建设备句柄
    // Create device handle at 100KHz
    _i2c_device = i2c_bus_device_create(_i2c_bus, _addr, M5IOE1_I2C_FREQ_100K);
    if (_i2c_device == nullptr) {
        M5IOE1_LOG_E(TAG, "Failed to create I2C device");
        return false;
    }

    // 尝试唤醒设备 - 发送 I2C START 信号
    // Try to wake up the device - send I2C START signal
    // M5IOE1 可能处于睡眠状态，需要先唤醒
    // M5IOE1 may be in sleep mode, need to wake up first
    M5IOE1_I2C_SEND_WAKE(_i2c_device, M5IOE1_REG_HW_REV);
    M5IOE1_DELAY_MS(10);

    // 步骤1: 先尝试100K通信
    // Step 1: Try 100K communication first
    if (!_initDevice()) {
        // 步骤2: 100K失败，尝试400K
        // Step 2: 100K failed, try 400K
        M5IOE1_LOG_W(TAG, "Failed at 100KHz, trying 400KHz...");

        // 删除当前100K设备句柄
        // Remove current 100K device handle
        i2c_bus_device_delete(&_i2c_device);
        _i2c_device = nullptr;

        // 以400K重新创建设备句柄
        // Recreate device handle at 400K
        _i2c_device = i2c_bus_device_create(_i2c_bus, _addr, M5IOE1_I2C_FREQ_400K);
        if (_i2c_device == nullptr) {
            M5IOE1_LOG_E(TAG, "Failed to create I2C device at 400KHz");
            return false;
        }

        M5IOE1_I2C_SEND_WAKE(_i2c_device, M5IOE1_REG_HW_REV);
        M5IOE1_DELAY_MS(10);

        if (!_initDevice()) {
            // 步骤3: 都失败，初始化失败
            // Step 3: Both failed, initialization failed
            M5IOE1_LOG_E(TAG, "Failed at both 100KHz and 400KHz");
            i2c_bus_device_delete(&_i2c_device);
            _i2c_device = nullptr;
            return false;
        }
    }

    // 步骤4: 通信成功，设置为已初始化
    // Step 4: Communication succeeded, set as initialized
    _initialized = true;

    // 步骤5: 强制配置I2C（用户请求的频率 + 关闭休眠）
    // Step 5: Force configure I2C (user requested speed + disable sleep)
    m5ioe1_i2c_speed_t targetSpeed = (_requestedSpeed == M5IOE1_I2C_FREQ_400K)
        ? M5IOE1_I2C_SPEED_400K : M5IOE1_I2C_SPEED_100K;

    if (!setI2cConfig(0, targetSpeed, M5IOE1_WAKE_EDGE_FALLING, M5IOE1_PULL_ENABLED)) {
        M5IOE1_LOG_W(TAG, "Failed to set I2C config");
    }

    // 步骤6: 切换设备句柄到用户请求的速度
    // Step 6: Switch device handle to user requested speed
    i2c_bus_device_delete(&_i2c_device);
    _i2c_device = nullptr;

    _i2c_device = i2c_bus_device_create(_i2c_bus, _addr, _requestedSpeed);
    if (_i2c_device == nullptr) {
        M5IOE1_LOG_E(TAG, "Failed to switch device to %lu Hz", _requestedSpeed);
        return false;
    }

    // 步骤7: 快照
    // Step 7: Snapshot
    _snapshotPinStates();
    _snapshotPwmStates();
    _snapshotAdcState();
    _snapshotI2cConfig();

    M5IOE1_LOG_I(TAG, "M5IOE1 initialized at address 0x%02X (I2C: %lu Hz)", _addr, _requestedSpeed);

    if (mode != M5IOE1_INT_MODE_DISABLED) {
        setInterruptMode(mode);
    }

    return true;
}

// =====================================================
// Type 3B: Existing i2c_bus_handle_t, with hardware interrupt
// =====================================================
bool M5IOE1::begin(i2c_bus_handle_t bus, uint8_t addr, uint32_t speed, int intPin, m5ioe1_int_mode_t mode) {
    if (!begin(bus, addr, speed, M5IOE1_INT_MODE_DISABLED)) {
        return false;
    }
    
    _intPin = intPin;
    if (_intPin >= 0) {
        return setInterruptMode(mode);
    }
    return true;
}

#endif

bool M5IOE1::setInterruptMode(m5ioe1_int_mode_t mode, uint32_t pollingIntervalMs) {
    _intMode = mode;
    _pollingInterval = pollingIntervalMs;

#ifdef ARDUINO
    _cleanupPollingArduino();

    if (mode == M5IOE1_INT_MODE_POLLING) {
        return _setupPollingArduino();
    }
    // Arduino 上的硬件中断模式需要 attachInterrupt
    // 这更复杂且特定于平台
    // Hardware interrupt mode on Arduino would require attachInterrupt
    // which is more complex and platform-specific
#else
    _cleanupInterrupt();

    if (mode == M5IOE1_INT_MODE_POLLING) {
        return _setupPolling();
    } else if (mode == M5IOE1_INT_MODE_HARDWARE) {
        if (_intPin < 0) {
            M5IOE1_LOG_E(TAG, "Hardware interrupt mode requires interrupt pin");
            return false;
        }
        return _setupHardwareInterrupt();
    }
#endif

    return true;
}

bool M5IOE1::setPollingInterval(float seconds) {
    if (seconds < 0.001f || seconds > 3600.0f) {
        M5IOE1_LOG_E(TAG, "Invalid polling interval: %.3f seconds (valid range: 0.001-3600)", seconds);
        return false;
    }
    
    uint32_t intervalMs = (uint32_t)(seconds * 1000.0f);
    _pollingInterval = intervalMs;
    
    // 如果当前处于轮询模式，使用新间隔重新启动
    // If currently in polling mode, restart with new interval
    if (_intMode == M5IOE1_INT_MODE_POLLING) {
#ifdef ARDUINO
        _cleanupPollingArduino();
        return _setupPollingArduino();
#else
        _cleanupInterrupt();
        return _setupPolling();
#endif
    }
    
    M5IOE1_LOG_I(TAG, "Polling interval set to %.3f seconds (%u ms)", seconds, intervalMs);
    return true;
}

// ============================
// 设备信息
// Device Information
// ============================

bool M5IOE1::getUID(uint16_t* uid) {
    if (uid == nullptr || !_initialized) return false;
    return _readReg16(M5IOE1_REG_UID_L, uid);
}

bool M5IOE1::getVersion(uint8_t* version) {
    if (version == nullptr || !_initialized) return false;
    return _readReg(M5IOE1_REG_REV, version);
}

bool M5IOE1::getRefVoltage(uint16_t* voltage_mv) {
    if (voltage_mv == nullptr || !_initialized) return false;
    return _readReg16(M5IOE1_REG_REF_VOLTAGE_L, voltage_mv);
}

// ============================
// GPIO 功能
// GPIO Functions
// ============================

void M5IOE1::pinMode(uint8_t pin, uint8_t mode) {
    if (!_isValidPin(pin) || !_initialized) {
        M5IOE1_LOG_E(TAG, "Invalid pin or not initialized");
        return;
    }

    uint16_t modeReg = 0, puReg = 0, pdReg = 0, drvReg = 0;

    if (!_readReg16(M5IOE1_REG_GPIO_MODE_L, &modeReg)) return;
    if (!_readReg16(M5IOE1_REG_GPIO_PU_L, &puReg)) return;
    if (!_readReg16(M5IOE1_REG_GPIO_PD_L, &pdReg)) return;
    if (!_readReg16(M5IOE1_REG_GPIO_DRV_L, &drvReg)) return;

    switch (mode) {
        case INPUT:  // 0x01
            modeReg &= ~(1 << pin);
            puReg &= ~(1 << pin);
            pdReg &= ~(1 << pin);
            _pinStates[pin].isOutput = false;
            _pinStates[pin].pull = 0;
            break;
        case PULLUP:  // 0x04 - 仅上拉
        case INPUT_PULLUP:  // 0x05
            modeReg &= ~(1 << pin);
            puReg |= (1 << pin);
            pdReg &= ~(1 << pin);
            _pinStates[pin].isOutput = false;
            _pinStates[pin].pull = 1;
            break;
        case PULLDOWN:  // 0x08 - 仅下拉
        case INPUT_PULLDOWN:  // 0x09
            modeReg &= ~(1 << pin);
            puReg &= ~(1 << pin);
            pdReg |= (1 << pin);
            _pinStates[pin].isOutput = false;
            _pinStates[pin].pull = 2;
            break;
        case OUTPUT:  // 0x03
            // 如果此引脚上启用了 PWM 则禁用
            // Disable PWM if enabled on this pin
            if (_isPwmPin(pin)) {
                uint8_t ch = _getPwmChannel(pin);
                uint8_t regL = M5IOE1_REG_PWM1_DUTY_L + ch * 2;
                uint16_t pwmData = 0;
                if (_readReg16(regL, &pwmData)) {
                    if (pwmData & ((uint16_t)M5IOE1_PWM_ENABLE << 8)) {
                        pwmData &= ~((uint16_t)M5IOE1_PWM_ENABLE << 8);
                        _writeReg16(regL, pwmData);
                    }
                }
            }
            modeReg |= (1 << pin);
            puReg &= ~(1 << pin);
            pdReg &= ~(1 << pin);
            drvReg &= ~(1 << pin);  // 推挽
                                // Push-pull
            _pinStates[pin].isOutput = true;
            _pinStates[pin].drive = 0;
            break;
        case OPEN_DRAIN:  // 0x10
        case OUTPUT_OPEN_DRAIN:  // 0x13
            // 如果此引脚上启用了 PWM 则禁用
            // Disable PWM if enabled on this pin
            if (_isPwmPin(pin)) {
                uint8_t ch = _getPwmChannel(pin);
                uint8_t regL = M5IOE1_REG_PWM1_DUTY_L + ch * 2;
                uint16_t pwmData = 0;
                if (_readReg16(regL, &pwmData)) {
                    if (pwmData & ((uint16_t)M5IOE1_PWM_ENABLE << 8)) {
                        pwmData &= ~((uint16_t)M5IOE1_PWM_ENABLE << 8);
                        _writeReg16(regL, pwmData);
                    }
                }
            }
            modeReg |= (1 << pin);
            puReg &= ~(1 << pin);
            pdReg &= ~(1 << pin);
            drvReg |= (1 << pin);  // 开漏
                                // Open-drain
            _pinStates[pin].isOutput = true;
            _pinStates[pin].drive = 1;
            break;
        case ANALOG:  // 0xC0
            // 模拟模式 - 设置为输入，无上拉下拉
            // Analog mode - set as input, no pull-up/down
            modeReg &= ~(1 << pin);
            puReg &= ~(1 << pin);
            pdReg &= ~(1 << pin);
            _pinStates[pin].isOutput = false;
            _pinStates[pin].pull = 0;
            break;
        default:
            M5IOE1_LOG_E(TAG, "Invalid mode: %d", mode);
            return;
    }

    _writeReg16(M5IOE1_REG_GPIO_PU_L, puReg);
    _writeReg16(M5IOE1_REG_GPIO_PD_L, pdReg);
    _writeReg16(M5IOE1_REG_GPIO_DRV_L, drvReg);
    _writeReg16(M5IOE1_REG_GPIO_MODE_L, modeReg);

    _autoSnapshotUpdate();
}

void M5IOE1::digitalWrite(uint8_t pin, uint8_t value) {
    if (!_isValidPin(pin) || !_initialized) {
        M5IOE1_LOG_E(TAG, "Invalid pin or not initialized");
        return;
    }

    uint16_t outReg = 0;
    if (!_readReg16(M5IOE1_REG_GPIO_OUT_L, &outReg)) return;

    if (value) {
        outReg |= (1 << pin);
    } else {
        outReg &= ~(1 << pin);
    }

    _writeReg16(M5IOE1_REG_GPIO_OUT_L, outReg);
    _pinStates[pin].outputLevel = value ? 1 : 0;
}

int M5IOE1::digitalRead(uint8_t pin) {
    if (!_isValidPin(pin) || !_initialized) {
        M5IOE1_LOG_E(TAG, "Invalid pin or not initialized");
        return -1;
    }

    uint16_t inReg = 0;
    if (!_readReg16(M5IOE1_REG_GPIO_IN_L, &inReg)) return -1;

    return (inReg & (1 << pin)) ? HIGH : LOW;
}

// ============================
// 高级 GPIO 功能
// Advanced GPIO Functions
// ============================

bool M5IOE1::setPullMode(uint8_t pin, uint8_t pullMode) {
    if (!_isValidPin(pin) || !_initialized) return false;

    uint16_t puReg = 0, pdReg = 0;
    if (!_readReg16(M5IOE1_REG_GPIO_PU_L, &puReg)) return false;
    if (!_readReg16(M5IOE1_REG_GPIO_PD_L, &pdReg)) return false;

    puReg &= ~(1 << pin);
    pdReg &= ~(1 << pin);

    if (pullMode == M5IOE1_PULL_UP) {
        puReg |= (1 << pin);
        _pinStates[pin].pull = 1;
    } else if (pullMode == M5IOE1_PULL_DOWN) {
        pdReg |= (1 << pin);
        _pinStates[pin].pull = 2;
    } else {
        _pinStates[pin].pull = 0;
    }

    bool ok = _writeReg16(M5IOE1_REG_GPIO_PU_L, puReg) &&
              _writeReg16(M5IOE1_REG_GPIO_PD_L, pdReg);
    if (ok) _autoSnapshotUpdate();
    return ok;
}

bool M5IOE1::setDriveMode(uint8_t pin, uint8_t driveMode) {
    if (!_isValidPin(pin) || !_initialized) return false;

    uint16_t drvReg = 0;
    if (!_readReg16(M5IOE1_REG_GPIO_DRV_L, &drvReg)) return false;

    if (driveMode == M5IOE1_DRIVE_OPENDRAIN) {
        drvReg |= (1 << pin);
        _pinStates[pin].drive = 1;
    } else {
        drvReg &= ~(1 << pin);
        _pinStates[pin].drive = 0;
    }

    bool ok = _writeReg16(M5IOE1_REG_GPIO_DRV_L, drvReg);
    if (ok) _autoSnapshotUpdate();
    return ok;
}

bool M5IOE1::getInputState(uint8_t pin, uint8_t* state) {
    if (!_isValidPin(pin) || state == nullptr || !_initialized) return false;

    int val = digitalRead(pin);
    if (val < 0) return false;

    *state = (uint8_t)val;
    return true;
}

// ============================
// 中断功能
// Interrupt Functions
// ============================

void M5IOE1::attachInterrupt(uint8_t pin, m5ioe1_callback_t callback, uint8_t mode) {
    if (!_isValidPin(pin) || callback == nullptr || !_initialized) return;

    // 检查冲突的中断
    // Check for conflicting interrupts
    if (_hasConflictingInterrupt(pin)) {
        M5IOE1_LOG_E(TAG, "Interrupt conflict on pin %d", pin);
        return;
    }

    _callbacks[pin].callback = callback;
    _callbacks[pin].callbackArg = nullptr;
    _callbacks[pin].arg = nullptr;
    _callbacks[pin].enabled = true;
    _callbacks[pin].rising = (mode == RISING);

    // 将引脚配置为输入
    // Configure pin as input
    uint16_t modeReg = 0;
    _readReg16(M5IOE1_REG_GPIO_MODE_L, &modeReg);
    modeReg &= ~(1 << pin);
    _writeReg16(M5IOE1_REG_GPIO_MODE_L, modeReg);

    // 配置中断
    // Configure interrupt
    uint16_t ieReg = 0, itReg = 0;
    _readReg16(M5IOE1_REG_GPIO_IE_L, &ieReg);
    _readReg16(M5IOE1_REG_GPIO_IP_L, &itReg);

    ieReg |= (1 << pin);
    if (mode == RISING) {
        itReg |= (1 << pin);
    } else {
        itReg &= ~(1 << pin);
    }

    _writeReg16(M5IOE1_REG_GPIO_IE_L, ieReg);
    _writeReg16(M5IOE1_REG_GPIO_IP_L, itReg);

    _pinStates[pin].intrEnabled = true;
    _pinStates[pin].intrRising = (mode == RISING);
}

void M5IOE1::attachInterruptArg(uint8_t pin, m5ioe1_callback_arg_t callback, void* arg, uint8_t mode) {
    if (!_isValidPin(pin) || callback == nullptr || !_initialized) return;

    if (_hasConflictingInterrupt(pin)) {
        M5IOE1_LOG_E(TAG, "Interrupt conflict on pin %d", pin);
        return;
    }

    _callbacks[pin].callback = nullptr;
    _callbacks[pin].callbackArg = callback;
    _callbacks[pin].arg = arg;
    _callbacks[pin].enabled = true;
    _callbacks[pin].rising = (mode == RISING);

    // 将引脚配置为输入
    // Configure pin as input
    uint16_t modeReg = 0;
    _readReg16(M5IOE1_REG_GPIO_MODE_L, &modeReg);
    modeReg &= ~(1 << pin);
    _writeReg16(M5IOE1_REG_GPIO_MODE_L, modeReg);

    // 配置中断
    // Configure interrupt
    uint16_t ieReg = 0, itReg = 0;
    _readReg16(M5IOE1_REG_GPIO_IE_L, &ieReg);
    _readReg16(M5IOE1_REG_GPIO_IP_L, &itReg);

    ieReg |= (1 << pin);
    if (mode == RISING) {
        itReg |= (1 << pin);
    } else {
        itReg &= ~(1 << pin);
    }

    _writeReg16(M5IOE1_REG_GPIO_IE_L, ieReg);
    _writeReg16(M5IOE1_REG_GPIO_IP_L, itReg);

    _pinStates[pin].intrEnabled = true;
    _pinStates[pin].intrRising = (mode == RISING);
}

void M5IOE1::detachInterrupt(uint8_t pin) {
    if (!_isValidPin(pin) || !_initialized) return;

    uint16_t ieReg = 0;
    _readReg16(M5IOE1_REG_GPIO_IE_L, &ieReg);
    ieReg &= ~(1 << pin);
    _writeReg16(M5IOE1_REG_GPIO_IE_L, ieReg);

    _callbacks[pin].callback = nullptr;
    _callbacks[pin].callbackArg = nullptr;
    _callbacks[pin].arg = nullptr;
    _callbacks[pin].enabled = false;
    _pinStates[pin].intrEnabled = false;
}

void M5IOE1::enableInterrupt(uint8_t pin) {
    if (!_isValidPin(pin)) return;
    _callbacks[pin].enabled = true;
}

void M5IOE1::disableInterrupt(uint8_t pin) {
    if (!_isValidPin(pin)) return;
    _callbacks[pin].enabled = false;
}

uint16_t M5IOE1::getInterruptStatus() {
    if (!_initialized) return 0;

    uint16_t status = 0;
    _readReg16(M5IOE1_REG_GPIO_IS_L, &status);
    return status;
}

bool M5IOE1::clearInterrupt(uint8_t pin) {
    if (!_isValidPin(pin) || !_initialized) return false;

    uint16_t isReg = 0;
    if (!_readReg16(M5IOE1_REG_GPIO_IS_L, &isReg)) return false;
    isReg &= ~(1 << pin);
    return _writeReg16(M5IOE1_REG_GPIO_IS_L, isReg);
}

// ============================
// ADC 功能
// ADC Functions
// ============================

bool M5IOE1::analogRead(uint8_t channel, uint16_t* result) {
    if (result == nullptr || channel < 1 || channel > 4 || !_initialized) return false;

    // 开始转换
    // Start conversion
    uint8_t ctrl = (channel & M5IOE1_ADC_CH_MASK) | M5IOE1_ADC_START;
    if (!_writeReg(M5IOE1_REG_ADC_CTRL, ctrl)) return false;

    // 等待完成
    // Wait for completion
    uint8_t reg = 0;
    int tries = 0;
    do {
        M5IOE1_DELAY_MS(1);
        if (!_readReg(M5IOE1_REG_ADC_CTRL, &reg)) return false;
        tries++;
    } while ((reg & M5IOE1_ADC_BUSY) && tries < 20);

    if (reg & M5IOE1_ADC_BUSY) return false;

    bool ok = _readReg16(M5IOE1_REG_ADC_DATA_L, result);
    if (ok) _autoSnapshotUpdate();
    return ok;
}

bool M5IOE1::isAdcBusy() {
    if (!_initialized) return false;

    uint8_t ctrl = 0;
    if (_readReg(M5IOE1_REG_ADC_CTRL, &ctrl)) {
        return (ctrl & M5IOE1_ADC_BUSY) != 0;
    }
    return false;
}

bool M5IOE1::disableAdc() {
    if (!_initialized) return false;
    bool ok = _writeReg(M5IOE1_REG_ADC_CTRL, 0);
    if (ok) _autoSnapshotUpdate();
    return ok;
}

// ============================
// 温度传感器
// Temperature Sensor
// ============================

bool M5IOE1::readTemperature(uint16_t* temperature) {
    if (temperature == nullptr || !_initialized) return false;

    if (!_writeReg(M5IOE1_REG_TEMP_CTRL, M5IOE1_TEMP_START)) return false;

    uint8_t ctrl = 0;
    int tries = 0;
    do {
        M5IOE1_DELAY_MS(1);
        if (!_readReg(M5IOE1_REG_TEMP_CTRL, &ctrl)) return false;
        tries++;
    } while ((ctrl & M5IOE1_TEMP_BUSY) && tries < 20);

    if (ctrl & M5IOE1_TEMP_BUSY) return false;

    return _readReg16(M5IOE1_REG_TEMP_DATA_L, temperature);
}

bool M5IOE1::isTemperatureBusy() {
    if (!_initialized) return false;

    uint8_t ctrl = 0;
    if (_readReg(M5IOE1_REG_TEMP_CTRL, &ctrl)) {
        return (ctrl & M5IOE1_TEMP_BUSY) != 0;
    }
    return false;
}

// ============================
// PWM 功能
// PWM Functions
// ============================

bool M5IOE1::setPwmFrequency(uint16_t frequency) {
    if (!_initialized) return false;
    bool ok = _writeReg16(M5IOE1_REG_PWM_FREQ_L, frequency);
    if (ok) {
        _pwmFrequency = frequency;
        _autoSnapshotUpdate();
    }
    return ok;
}

bool M5IOE1::getPwmFrequency(uint16_t* frequency) {
    if (frequency == nullptr || !_initialized) return false;

    if (!_readReg16(M5IOE1_REG_PWM_FREQ_L, frequency)) return false;

    // 更新缓存
    // Update cache
    _pwmFrequency = *frequency;

    return true;
}

bool M5IOE1::setPwmDuty(uint8_t channel, uint8_t duty, bool polarity, bool enable) {
    if (channel > 3 || duty > 100 || !_initialized) return false;

    // 将百分比转换为 12 位 (0-4095)
    // Convert percentage to 12-bit (0-4095)
    uint16_t duty12 = (uint16_t)((duty * 0x0FFF) / 100);
    return setPwmDuty12bit(channel, duty12, polarity, enable);
}

bool M5IOE1::getPwmDuty(uint8_t channel, uint8_t* duty, bool* polarity, bool* enable) {
    if (channel > 3 || duty == nullptr || polarity == nullptr || enable == nullptr || !_initialized) {
        return false;
    }

    uint8_t regL = M5IOE1_REG_PWM1_DUTY_L + (channel * 2);
    uint16_t data = 0;
    if (!_readReg16(regL, &data)) return false;

    uint16_t duty12 = data & 0x0FFF;
    *duty = (uint8_t)((duty12 * 100) / 0x0FFF);
    *polarity = (data & ((uint16_t)M5IOE1_PWM_POLARITY << 8)) != 0;
    *enable = (data & ((uint16_t)M5IOE1_PWM_ENABLE << 8)) != 0;

    // 更新缓存
    // Update cache
    _pwmStates[channel].duty12 = duty12;
    _pwmStates[channel].polarity = *polarity;
    _pwmStates[channel].enabled = *enable;

    return true;
}

bool M5IOE1::setPwmDuty12bit(uint8_t channel, uint16_t duty12, bool polarity, bool enable) {
    if (channel > 3 || duty12 > 0x0FFF || !_initialized) return false;

    // 获取对应的引脚
    // Get corresponding pin
    uint8_t pin = (channel == 0) ? 8 : (channel == 1) ? 7 : (channel == 2) ? 10 : 9;

    uint8_t regL = M5IOE1_REG_PWM1_DUTY_L + (channel * 2);

    uint8_t dataL = (uint8_t)(duty12 & 0xFF);
    uint8_t dataH = (uint8_t)((duty12 >> 8) & 0x0F);
    if (polarity) dataH |= M5IOE1_PWM_POLARITY;
    if (enable) dataH |= M5IOE1_PWM_ENABLE;

    uint8_t buf[2] = {dataL, dataH};
    if (!_writeBytes(regL, buf, 2)) return false;

    if (enable) {
        // 将引脚设置为输出模式
        // Set pin to output mode
        uint16_t modeReg = 0;
        if (_readReg16(M5IOE1_REG_GPIO_MODE_L, &modeReg)) {
            modeReg |= (1 << pin);
            _writeReg16(M5IOE1_REG_GPIO_MODE_L, modeReg);
        }
    }

    _pwmStates[channel].duty12 = duty12;
    _pwmStates[channel].polarity = polarity;
    _pwmStates[channel].enabled = enable;

    _autoSnapshotUpdate();
    return true;
}

bool M5IOE1::getPwmDuty12bit(uint8_t channel, uint16_t* duty12, bool* polarity, bool* enable) {
    if (channel > 3 || duty12 == nullptr || polarity == nullptr || enable == nullptr || !_initialized) {
        return false;
    }

    uint8_t regL = M5IOE1_REG_PWM1_DUTY_L + (channel * 2);
    uint16_t data = 0;
    if (!_readReg16(regL, &data)) {
        return false;
    }

    *duty12 = data & 0x0FFF;
    *polarity = (data & ((uint16_t)M5IOE1_PWM_POLARITY << 8)) != 0;
    *enable = (data & ((uint16_t)M5IOE1_PWM_ENABLE << 8)) != 0;

    // 更新缓存
    // Update cache
    _pwmStates[channel].duty12 = *duty12;
    _pwmStates[channel].polarity = *polarity;
    _pwmStates[channel].enabled = *enable;

    return true;
}

// ============================
// Arduino 兼容 analogWrite
// Arduino-compatible analogWrite
// ============================

bool M5IOE1::analogWrite(uint8_t channel, uint8_t value) {
    if (channel > 3 || !_initialized) {
        M5IOE1_LOG_E(TAG, "Invalid channel or not initialized: ch=%d", channel);
        return false;
    }

    // 值为 0 时关闭 PWM 输出
    // Turn off PWM when value is 0
    if (value == 0) {
        return setPwmDuty12bit(channel, 0, false, false);
    }

    // 将 8-bit 值 (0-255) 缩放到 12-bit (0-4095)
    // Scale 8-bit value (0-255) to 12-bit (0-4095)
    // 公式：duty12 = (value * 4095) / 255 = value * 16 + value / 16
    // Formula: duty12 = (value * 4095) / 255 = value * 16 + value / 16
    uint16_t duty12 = (uint16_t)value * 16 + (uint16_t)value / 16;

    // 设置 PWM，默认启用输出，正常极性
    // Set PWM with output enabled, normal polarity
    return setPwmDuty12bit(channel, duty12, false, true);
}

// ============================
// NeoPixel LED 功能
// NeoPixel LED Functions
// ============================

bool M5IOE1::setLedCount(uint8_t count) {
    if (count > M5IOE1_MAX_LED_COUNT || !_initialized) return false;
    bool ok = _writeReg(M5IOE1_REG_LED_CFG, count & M5IOE1_LED_NUM_MASK);
    if (ok) {
        _ledCount = count;
        _ledEnabled = (count > 0);
    }
    return ok;
}

bool M5IOE1::setLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= M5IOE1_MAX_LED_COUNT || !_initialized) return false;

    // 转换为 RGB565
    // Convert to RGB565
    uint16_t r5 = (r >> 3) & 0x1F;
    uint16_t g6 = (g >> 2) & 0x3F;
    uint16_t b5 = (b >> 3) & 0x1F;
    uint16_t rgb565 = (r5 << 11) | (g6 << 5) | b5;

    uint8_t regAddr = M5IOE1_REG_LED_RAM_START + (index * 2);
    uint8_t data[2] = {(uint8_t)((rgb565 >> 8) & 0xFF), (uint8_t)(rgb565 & 0xFF)};

    return _writeBytes(regAddr, data, 2);
}

bool M5IOE1::setLedColor(uint8_t index, m5ioe1_rgb_t color) {
    return setLedColor(index, color.r, color.g, color.b);
}

bool M5IOE1::refreshLeds() {
    if (!_initialized) return false;

    uint8_t cfg = 0;
    if (!_readReg(M5IOE1_REG_LED_CFG, &cfg)) return false;

    cfg |= M5IOE1_LED_REFRESH;
    return _writeReg(M5IOE1_REG_LED_CFG, cfg);
}

bool M5IOE1::disableLeds() {
    if (!_initialized) return false;
    bool ok = _writeReg(M5IOE1_REG_LED_CFG, 0);
    if (ok) {
        _ledCount = 0;
        _ledEnabled = false;
    }
    return ok;
}

// ============================
// AW8737A 脉冲功能
// AW8737A Pulse Functions
// ============================

bool M5IOE1::setAw8737aPulse(uint8_t pin, m5ioe1_aw8737a_pulse_num_t pulseNum, 
                             m5ioe1_aw8737a_refresh_t refresh) {
    if (!_initialized) {
        M5IOE1_LOG_E(TAG, "Device not initialized");
        return false;
    }

    // 验证引脚范围 (0-13)
    // Validate pin range (0-13)
    if (pin >= M5IOE1_MAX_GPIO_PINS) {
        M5IOE1_LOG_E(TAG, "Invalid pin number: %d (valid range: 0-%d)", pin, M5IOE1_MAX_GPIO_PINS - 1);
        return false;
    }

    // 验证脉冲编号 (0-3)
    // Validate pulse number (0-3)
    if (pulseNum > M5IOE1_AW8737A_PULSE_NUM_3) {
        M5IOE1_LOG_E(TAG, "Invalid pulse number: %d (valid range: 0-3)", pulseNum);
        return false;
    }

    // 构建寄存器值
    // [7] REFRESH | [6:5] NUM[1:0] | [4:0] GPIO[4:0]
    // Build register value
    // [7] REFRESH | [6:5] NUM[1:0] | [4:0] GPIO[4:0]
    uint8_t regValue = 0;
    regValue |= (pin & M5IOE1_AW8737A_GPIO_MASK);                                    // 位[4:0]: GPIO 选择
                                                                                      // Bits[4:0]: GPIO selection
    regValue |= ((pulseNum & M5IOE1_AW8737A_NUM_MASK) << M5IOE1_AW8737A_NUM_SHIFT); // 位[6:5]: 脉冲编号
                                                                                      // Bits[6:5]: Pulse number
    if (refresh == M5IOE1_AW8737A_REFRESH_NOW) {
        regValue |= M5IOE1_AW8737A_REFRESH;                                          // 位[7]: 刷新标志
                                                                                      // Bit[7]: Refresh flag
    }

    bool ok = _writeReg(M5IOE1_REG_AW8737A_PULSE, regValue);
    if (!ok) {
        M5IOE1_LOG_E(TAG, "Failed to set AW8737A pulse config");
        return false;
    }

    M5IOE1_LOG_I(TAG, "AW8737A pulse set: pin=%d, num=%d, refresh=%d (reg=0x%02X)", 
                 pin, pulseNum, refresh, regValue);

    // 如果设置了 REFRESH 位 (REFRESH_NOW)，等待 20ms，因为它会影响 I2C 通信
    // If REFRESH bit was set (REFRESH_NOW), wait 20ms as it affects I2C communication
    if (refresh == M5IOE1_AW8737A_REFRESH_NOW) {
        M5IOE1_DELAY_MS(20);
    }

    return true;
}

bool M5IOE1::refreshAw8737aPulse() {
    if (!_initialized) {
        M5IOE1_LOG_E(TAG, "Device not initialized");
        return false;
    }

    // 读取当前寄存器值
    // Read current register value
    uint8_t regValue = 0;
    if (!_readReg(M5IOE1_REG_AW8737A_PULSE, &regValue)) {
        M5IOE1_LOG_E(TAG, "Failed to read AW8737A pulse register");
        return false;
    }

    // 将位 7 设置为 1
    // Set bit 7 to 1
    regValue |= M5IOE1_AW8737A_REFRESH;

    if (!_writeReg(M5IOE1_REG_AW8737A_PULSE, regValue)) {
        M5IOE1_LOG_E(TAG, "Failed to refresh AW8737A pulse");
        return false;
    }

    M5IOE1_LOG_I(TAG, "AW8737A pulse refresh triggered (reg=0x%02X)", regValue);

    // 写入位 7 后等待 20ms，因为它会影响 I2C 通信
    // Wait 20ms after writing bit 7, as it affects I2C communication
    M5IOE1_DELAY_MS(20);

    return true;
}

// ============================
// RTC RAM 功能
// RTC RAM Functions
// ============================

bool M5IOE1::writeRtcRAM(uint8_t offset, const uint8_t* data, uint8_t length) {
    if (data == nullptr || offset >= M5IOE1_RTC_RAM_SIZE ||
        length == 0 || (offset + length) > M5IOE1_RTC_RAM_SIZE || !_initialized) {
        return false;
    }

    uint8_t regAddr = M5IOE1_REG_RTC_RAM_START + offset;
    return _writeBytes(regAddr, data, length);
}

bool M5IOE1::readRtcRAM(uint8_t offset, uint8_t* data, uint8_t length) {
    if (data == nullptr || offset >= M5IOE1_RTC_RAM_SIZE ||
        length == 0 || (offset + length) > M5IOE1_RTC_RAM_SIZE || !_initialized) {
        return false;
    }

    uint8_t regAddr = M5IOE1_REG_RTC_RAM_START + offset;
    return _readBytes(regAddr, data, length);
}

// ============================
// 系统配置
// System Configuration
// ============================

bool M5IOE1::setI2cConfig(uint8_t sleepTime, m5ioe1_i2c_speed_t speed,
                          m5ioe1_wake_edge_t wakeEdge, m5ioe1_pull_config_t pullConfig) {
    if (sleepTime > 15 || !_initialized) return false;

    uint8_t cfg = (sleepTime & M5IOE1_I2C_SLEEP_MASK);
    if (speed == M5IOE1_I2C_SPEED_400K) cfg |= M5IOE1_I2C_SPEED_400K_BIT;
    if (wakeEdge == M5IOE1_WAKE_EDGE_RISING) cfg |= M5IOE1_I2C_WAKE_RISING;
    if (pullConfig == M5IOE1_PULL_DISABLED) cfg |= M5IOE1_I2C_PULL_OFF;

    bool ok = _writeReg(M5IOE1_REG_I2C_CFG, cfg);
    if (ok) {
        _i2cConfig.sleepTime = sleepTime;
        _i2cConfig.speed400k = (speed == M5IOE1_I2C_SPEED_400K);
        _i2cConfig.wakeRising = (wakeEdge == M5IOE1_WAKE_EDGE_RISING);
        _i2cConfig.pullOff = (pullConfig == M5IOE1_PULL_DISABLED);
        _i2cConfigValid = true;

        // 若配置了休眠时间，自动启用 autoWake 防止数据丢失
        // If sleep time is configured, auto-enable autoWake to prevent data loss
        if (sleepTime > 0 && !_autoWakeEnabled) {
            setAutoWakeEnable(true);
            M5IOE1_LOG_W(TAG, "I2C sleep enabled (sleepTime=%d), auto-wake automatically enabled", sleepTime);
        }
    }
    return ok;
}

bool M5IOE1::setI2cSleepTime(uint8_t sleepTime) {
    if (sleepTime > 15 || !_initialized) return false;

    uint8_t cfg = 0;
    if (!_readReg(M5IOE1_REG_I2C_CFG, &cfg)) return false;

    // 清除 SLEEP 位 [3:0]，设置新值
    // Clear SLEEP bits [3:0], set new value
    cfg = (cfg & ~M5IOE1_I2C_SLEEP_MASK) | (sleepTime & M5IOE1_I2C_SLEEP_MASK);

    bool ok = _writeReg(M5IOE1_REG_I2C_CFG, cfg);
    if (ok) {
        _i2cConfig.sleepTime = sleepTime;
        M5IOE1_LOG_I(TAG, "I2C sleep time set to %d", sleepTime);

        // 若配置了休眠时间，自动启用 autoWake 防止数据丢失
        // If sleep time is configured, auto-enable autoWake to prevent data loss
        if (sleepTime > 0 && !_autoWakeEnabled) {
            setAutoWakeEnable(true);
            M5IOE1_LOG_W(TAG, "I2C sleep enabled, auto-wake automatically enabled");
        }
    }
    return ok;
}

bool M5IOE1::getI2cSleepTime(uint8_t* sleepTime) {
    if (sleepTime == nullptr || !_initialized) return false;

    uint8_t cfg = 0;
    if (!_readReg(M5IOE1_REG_I2C_CFG, &cfg)) return false;

    // 更新缓存
    // Update cache
    _i2cConfig.sleepTime = cfg & M5IOE1_I2C_SLEEP_MASK;
    _i2cConfigValid = true;

    *sleepTime = _i2cConfig.sleepTime;
    return true;
}

bool M5IOE1::setI2cWakeEdge(m5ioe1_wake_edge_t edge) {
    if (!_initialized) return false;

    uint8_t cfg = 0;
    if (!_readReg(M5IOE1_REG_I2C_CFG, &cfg)) return false;

    // 设置或清除 WAKE 位 [5]
    // Set or clear WAKE bit [5]
    if (edge == M5IOE1_WAKE_EDGE_RISING) {
        cfg |= M5IOE1_I2C_WAKE_RISING;
    } else {
        cfg &= ~M5IOE1_I2C_WAKE_RISING;
    }

    bool ok = _writeReg(M5IOE1_REG_I2C_CFG, cfg);
    if (ok) {
        _i2cConfig.wakeRising = (edge == M5IOE1_WAKE_EDGE_RISING);
        M5IOE1_LOG_I(TAG, "I2C wake edge set to %s",
                     edge == M5IOE1_WAKE_EDGE_RISING ? "rising" : "falling");
    }
    return ok;
}

bool M5IOE1::getI2cWakeEdge(m5ioe1_wake_edge_t* edge) {
    if (edge == nullptr || !_initialized) return false;

    uint8_t cfg = 0;
    if (!_readReg(M5IOE1_REG_I2C_CFG, &cfg)) return false;

    // 更新缓存
    // Update cache
    _i2cConfig.wakeRising = (cfg & M5IOE1_I2C_WAKE_RISING) != 0;
    _i2cConfigValid = true;

    *edge = _i2cConfig.wakeRising ? M5IOE1_WAKE_EDGE_RISING : M5IOE1_WAKE_EDGE_FALLING;
    return true;
}

bool M5IOE1::setI2cPullConfig(m5ioe1_pull_config_t config) {
    if (!_initialized) return false;

    uint8_t cfg = 0;
    if (!_readReg(M5IOE1_REG_I2C_CFG, &cfg)) return false;

    // 设置或清除 INT_PU/PD 位 [6]
    // Set or clear INT_PU/PD bit [6]
    if (config == M5IOE1_PULL_DISABLED) {
        cfg |= M5IOE1_I2C_PULL_OFF;
    } else {
        cfg &= ~M5IOE1_I2C_PULL_OFF;
    }

    bool ok = _writeReg(M5IOE1_REG_I2C_CFG, cfg);
    if (ok) {
        _i2cConfig.pullOff = (config == M5IOE1_PULL_DISABLED);
        M5IOE1_LOG_I(TAG, "I2C internal pull-up %s",
                     config == M5IOE1_PULL_DISABLED ? "disabled" : "enabled");
    }
    return ok;
}

bool M5IOE1::getI2cPullConfig(m5ioe1_pull_config_t* config) {
    if (config == nullptr || !_initialized) return false;

    uint8_t cfg = 0;
    if (!_readReg(M5IOE1_REG_I2C_CFG, &cfg)) return false;

    // 更新缓存
    // Update cache
    _i2cConfig.pullOff = (cfg & M5IOE1_I2C_PULL_OFF) != 0;
    _i2cConfigValid = true;

    *config = _i2cConfig.pullOff ? M5IOE1_PULL_DISABLED : M5IOE1_PULL_ENABLED;
    return true;
}

bool M5IOE1::factoryReset() {
    if (!_initialized) return false;

    bool ok = _writeReg(M5IOE1_REG_FACTORY_RESET, M5IOE1_FACTORY_RESET_KEY);
    if (ok) {
        M5IOE1_DELAY_MS(100);
        _initialized = false;
        _pinStatesValid = false;
        _pwmStatesValid = false;
        _adcStateValid = false;
        _clearPinStates();
        _clearPwmStates();
        _clearAdcState();
        M5IOE1_LOG_W(TAG, "Factory reset complete. Call begin() to reinitialize.");
    }
    return ok;
}

// ============================
// 自动唤醒功能
// Auto Wake Feature
// ============================

void M5IOE1::setAutoWakeEnable(bool enable) {
    _autoWakeEnabled = enable;
    if (enable) {
        _lastCommTime = M5IOE1_GET_TIME_MS();
    }
    M5IOE1_LOG_I(TAG, "Auto wake %s", enable ? "enabled" : "disabled");
}

bool M5IOE1::isAutoWakeEnabled() const {
    return _autoWakeEnabled;
}

bool M5IOE1::sendWakeSignal() {
#ifdef ARDUINO
    M5IOE1_I2C_SEND_WAKE(_wire, _addr);
    return true;
#else
    switch (_i2cDriverType) {
        case M5IOE1_I2C_DRIVER_SELF_CREATED:
        case M5IOE1_I2C_DRIVER_MASTER:
            return M5IOE1_I2C_MASTER_SEND_WAKE(_i2c_master_bus, _addr) == ESP_OK;
        case M5IOE1_I2C_DRIVER_BUS:
            return M5IOE1_I2C_SEND_WAKE(_i2c_device, M5IOE1_REG_HW_REV) == ESP_OK;
        default:
            return false;
    }
#endif
}

void M5IOE1::_checkAutoWake() {
    if (!_autoWakeEnabled || !_i2cConfigValid || _i2cConfig.sleepTime == 0) return;

    uint32_t now = M5IOE1_GET_TIME_MS();
    uint32_t elapsed = now - _lastCommTime;

    // If more than 1 second since last communication, send wake signal
    // 如果距离上次通信超过1秒，发送唤醒信号
    if (elapsed >= 1000) {
        sendWakeSignal();
        M5IOE1_DELAY_MS(10);
    }
    _lastCommTime = now;
}

// ============================
// 状态快照功能
// State Snapshot Functions
// ============================

void M5IOE1::setAutoSnapshot(bool enable) {
    _autoSnapshot = enable;
}

bool M5IOE1::isAutoSnapshotEnabled() const {
    return _autoSnapshot;
}

bool M5IOE1::updateSnapshot() {
    if (!_initialized) return false;

    bool gpio = _snapshotPinStates();
    bool pwm = _snapshotPwmStates();
    bool adc = _snapshotAdcState();

    return gpio && pwm && adc;
}

// ============================
// 调试功能
// Debug Functions
// ============================

bool M5IOE1::getModeReg(uint16_t* reg) {
    if (reg == nullptr || !_initialized) return false;
    return _readReg16(M5IOE1_REG_GPIO_MODE_L, reg);
}

bool M5IOE1::getOutputReg(uint16_t* reg) {
    if (reg == nullptr || !_initialized) return false;
    return _readReg16(M5IOE1_REG_GPIO_OUT_L, reg);
}

bool M5IOE1::getInputReg(uint16_t* reg) {
    if (reg == nullptr || !_initialized) return false;
    return _readReg16(M5IOE1_REG_GPIO_IN_L, reg);
}

bool M5IOE1::getPullUpReg(uint16_t* reg) {
    if (reg == nullptr || !_initialized) return false;
    return _readReg16(M5IOE1_REG_GPIO_PU_L, reg);
}

bool M5IOE1::getPullDownReg(uint16_t* reg) {
    if (reg == nullptr || !_initialized) return false;
    return _readReg16(M5IOE1_REG_GPIO_PD_L, reg);
}

bool M5IOE1::getDriveReg(uint16_t* reg) {
    if (reg == nullptr || !_initialized) return false;
    return _readReg16(M5IOE1_REG_GPIO_DRV_L, reg);
}

// ============================
// 配置验证
// Configuration Validation
// ============================

m5ioe1_validation_t M5IOE1::validateConfig(uint8_t pin, m5ioe1_config_type_t configType, bool enable) {
    m5ioe1_validation_t result = {false, {0}, 0xFF};

    // 基本验证
    // Basic validation
    if (!_isValidPin(pin)) {
        snprintf(result.error_msg, sizeof(result.error_msg), "Invalid pin %d", pin);
        return result;
    }

    if (!_initialized) {
        snprintf(result.error_msg, sizeof(result.error_msg), "Not initialized");
        return result;
    }

    // 如果禁用，无需冲突检查
    // If disabling, no conflict check needed
    if (!enable) {
        result.valid = true;
        return result;
    }

    uint8_t mutexPin = 0xFF;

    switch (configType) {
        case M5IOE1_CONFIG_GPIO_INPUT:
        case M5IOE1_CONFIG_GPIO_OUTPUT:
            // 检查引脚是否用于特殊功能
            // Check if pin is being used for special functions
            if (_hasActivePwm(pin)) {
                snprintf(result.error_msg, sizeof(result.error_msg),
                         "Pin %d is used for PWM. Disable first (e.g. setPwmDuty)", pin);
                return result;
            }
            if (_hasActiveAdc(pin)) {
                snprintf(result.error_msg, sizeof(result.error_msg),
                         "Pin %d is used for ADC. Disable first (disableAdc())", pin);
                return result;
            }
            if (_isNeopixelPin(pin) && _isLedEnabled()) {
                snprintf(result.error_msg, sizeof(result.error_msg),
                         "Pin %d (IO14) used for NeoPixel; call disableLeds()", pin);
                return result;
            }
            break;

        case M5IOE1_CONFIG_GPIO_INTERRUPT:
            // 检查中断互斥约束
            // Check interrupt mutex constraint
            if (_getInterruptMutexPin(pin, &mutexPin)) {
                if (_hasActiveInterrupt(mutexPin)) {
                    snprintf(result.error_msg, sizeof(result.error_msg),
                             "Interrupt conflict: IO%d and IO%d are mutex", pin + 1, mutexPin + 1);
                    result.conflicting_pin = mutexPin;
                    return result;
                }
            }
            // 检查 IO5 的 I2C 睡眠模式冲突
            // Check I2C sleep mode conflict for IO5
            if (pin == 4 && _hasI2cSleepEnabled()) {  // IO5 is pin index 4
                snprintf(result.error_msg, sizeof(result.error_msg),
                         "IO5 interrupt disabled when I2C sleep enabled");
                return result;
            }
            break;

        case M5IOE1_CONFIG_ADC:
            // 检查引脚是否支持 ADC
            // Check if pin supports ADC
            if (!_isAdcPin(pin)) {
                snprintf(result.error_msg, sizeof(result.error_msg),
                         "Pin %d does not support ADC", pin);
                return result;
            }
            // 检查引脚是否配置为输出
            // Check if pin is configured as output
            if (_pinStatesValid && _pinStates[pin].isOutput) {
                snprintf(result.error_msg, sizeof(result.error_msg),
                         "Pin %d is configured as output", pin);
                return result;
            }
            break;

        case M5IOE1_CONFIG_PWM:
            // 检查引脚是否支持 PWM
            // Check if pin supports PWM
            if (!_isPwmPin(pin)) {
                snprintf(result.error_msg, sizeof(result.error_msg),
                         "Pin %d does not support PWM", pin);
                return result;
            }
            break;

        case M5IOE1_CONFIG_NEOPIXEL:
            // NeoPixel 仅在 IO14 上工作（引脚索引 13）
            // NeoPixel only works on IO14 (pin index 13)
            if (!_isNeopixelPin(pin)) {
                snprintf(result.error_msg, sizeof(result.error_msg),
                         "NeoPixel only supported on IO14 (pin 13)");
                return result;
            }
            // 检查引脚是否有活动中断
            // Check if pin has active interrupt
            if (_hasActiveInterrupt(pin)) {
                snprintf(result.error_msg, sizeof(result.error_msg),
                         "IO14 has active interrupt, conflicts with NeoPixel");
                return result;
            }
            // 检查中断互斥（IO10 和 IO14 是互斥的）
            // Check interrupt mutex (IO10 and IO14 are mutex)
            if (_getInterruptMutexPin(pin, &mutexPin)) {
                if (_hasActiveInterrupt(mutexPin)) {
                    snprintf(result.error_msg, sizeof(result.error_msg),
                             "IO10 has interrupt enabled, mutex with IO14 NeoPixel");
                    result.conflicting_pin = mutexPin;
                    return result;
                }
            }
            break;

        case M5IOE1_CONFIG_I2C_SLEEP:
            // I2C 睡眠模式禁用 IO5 中断
            // I2C sleep mode disables IO5 interrupt
            if (_hasActiveInterrupt(4)) {  // IO5 is pin index 4
                snprintf(result.error_msg, sizeof(result.error_msg),
                         "I2C sleep mode will disable IO5 interrupt");
                result.conflicting_pin = 4;
                return result;
            }
            break;

        default:
            snprintf(result.error_msg, sizeof(result.error_msg), "Unknown config type");
            return result;
    }

    result.valid = true;
    return result;
}

// ============================
// 内部辅助函数
// Internal Helper Functions
// ============================

bool M5IOE1::_writeReg(uint8_t reg, uint8_t value) {
    _checkAutoWake();
#ifdef ARDUINO
    return M5IOE1_I2C_WRITE_BYTE(_wire, _addr, reg, value);
#else
    switch (_i2cDriverType) {
        case M5IOE1_I2C_DRIVER_SELF_CREATED:
        case M5IOE1_I2C_DRIVER_MASTER:
            return M5IOE1_I2C_MASTER_WRITE_BYTE(_i2c_master_dev, reg, value) == ESP_OK;
        case M5IOE1_I2C_DRIVER_BUS:
            return M5IOE1_I2C_WRITE_BYTE(_i2c_device, reg, value) == ESP_OK;
        default:
            return false;
    }
#endif
}

bool M5IOE1::_writeReg16(uint8_t reg, uint16_t value) {
    _checkAutoWake();
#ifdef ARDUINO
    return M5IOE1_I2C_WRITE_REG16(_wire, _addr, reg, value);
#else
    switch (_i2cDriverType) {
        case M5IOE1_I2C_DRIVER_SELF_CREATED:
        case M5IOE1_I2C_DRIVER_MASTER:
            return M5IOE1_I2C_MASTER_WRITE_REG16(_i2c_master_dev, reg, value) == ESP_OK;
        case M5IOE1_I2C_DRIVER_BUS:
            return M5IOE1_I2C_WRITE_REG16(_i2c_device, reg, value) == ESP_OK;
        default:
            return false;
    }
#endif
}

bool M5IOE1::_readReg(uint8_t reg, uint8_t* value) {
    _checkAutoWake();
#ifdef ARDUINO
    return M5IOE1_I2C_READ_BYTE(_wire, _addr, reg, value);
#else
    switch (_i2cDriverType) {
        case M5IOE1_I2C_DRIVER_SELF_CREATED:
        case M5IOE1_I2C_DRIVER_MASTER:
            return M5IOE1_I2C_MASTER_READ_BYTE(_i2c_master_dev, reg, value) == ESP_OK;
        case M5IOE1_I2C_DRIVER_BUS:
            return M5IOE1_I2C_READ_BYTE(_i2c_device, reg, value) == ESP_OK;
        default:
            return false;
    }
#endif
}

bool M5IOE1::_readReg16(uint8_t reg, uint16_t* value) {
    _checkAutoWake();
#ifdef ARDUINO
    return M5IOE1_I2C_READ_REG16(_wire, _addr, reg, value);
#else
    switch (_i2cDriverType) {
        case M5IOE1_I2C_DRIVER_SELF_CREATED:
        case M5IOE1_I2C_DRIVER_MASTER:
            return M5IOE1_I2C_MASTER_READ_REG16(_i2c_master_dev, reg, value) == ESP_OK;
        case M5IOE1_I2C_DRIVER_BUS:
            return M5IOE1_I2C_READ_REG16(_i2c_device, reg, value) == ESP_OK;
        default:
            return false;
    }
#endif
}

bool M5IOE1::_writeBytes(uint8_t reg, const uint8_t* data, uint8_t len) {
    _checkAutoWake();
#ifdef ARDUINO
    return M5IOE1_I2C_WRITE_BYTES(_wire, _addr, reg, len, data);
#else
    switch (_i2cDriverType) {
        case M5IOE1_I2C_DRIVER_SELF_CREATED:
        case M5IOE1_I2C_DRIVER_MASTER:
            return M5IOE1_I2C_MASTER_WRITE_BYTES(_i2c_master_dev, reg, len, data) == ESP_OK;
        case M5IOE1_I2C_DRIVER_BUS:
            return M5IOE1_I2C_WRITE_BYTES(_i2c_device, reg, len, data) == ESP_OK;
        default:
            return false;
    }
#endif
}

bool M5IOE1::_readBytes(uint8_t reg, uint8_t* data, uint8_t len) {
    _checkAutoWake();
#ifdef ARDUINO
    return M5IOE1_I2C_READ_BYTES(_wire, _addr, reg, len, data);
#else
    switch (_i2cDriverType) {
        case M5IOE1_I2C_DRIVER_SELF_CREATED:
        case M5IOE1_I2C_DRIVER_MASTER:
            return M5IOE1_I2C_MASTER_READ_BYTES(_i2c_master_dev, reg, len, data) == ESP_OK;
        case M5IOE1_I2C_DRIVER_BUS:
            return M5IOE1_I2C_READ_BYTES(_i2c_device, reg, len, data) == ESP_OK;
        default:
            return false;
    }
#endif
}

bool M5IOE1::_isValidPin(uint8_t pin) {
    return pin < M5IOE1_MAX_GPIO_PINS;
}

bool M5IOE1::_isAdcPin(uint8_t pin) {
    // ADC 引脚：IO2(1), IO4(3), IO5(4), IO7(6)
    // ADC pins: IO2(1), IO4(3), IO5(4), IO7(6)
    return (pin == 1 || pin == 3 || pin == 4 || pin == 6);
}

bool M5IOE1::_isPwmPin(uint8_t pin) {
    // PWM 引脚：IO8(7), IO9(8), IO10(9), IO11(10)
    // PWM pins: IO8(7), IO9(8), IO10(9), IO11(10)
    return (pin == 7 || pin == 8 || pin == 9 || pin == 10);
}

uint8_t M5IOE1::_getAdcChannel(uint8_t pin) {
    switch (pin) {
        case 1: return 1;  // IO2
        case 3: return 2;  // IO4
        case 4: return 3;  // IO5
        case 6: return 4;  // IO7
        default: return 0;
    }
}

uint8_t M5IOE1::_getPwmChannel(uint8_t pin) {
    switch (pin) {
        case 8: return 0;  // IO9 -> PWM1
        case 7: return 1;  // IO8 -> PWM2
        case 10: return 2; // IO11 -> PWM3
        case 9: return 3;  // IO10 -> PWM4
        default: return 0;
    }
}

void M5IOE1::_clearPinStates() {
    memset(_pinStates, 0, sizeof(_pinStates));
    _pinStatesValid = false;
}

void M5IOE1::_clearPwmStates() {
    memset(_pwmStates, 0, sizeof(_pwmStates));
    _pwmFrequency = 0;
    _pwmStatesValid = false;
}

void M5IOE1::_clearAdcState() {
    memset(&_adcState, 0, sizeof(_adcState));
    _adcStateValid = false;
}

bool M5IOE1::_snapshotPinStates() {
    if (!_initialized) return false;

    uint16_t modeReg = 0, outReg = 0, inReg = 0, puReg = 0, pdReg = 0, drvReg = 0, ieReg = 0, itReg = 0;

    if (!_readReg16(M5IOE1_REG_GPIO_MODE_L, &modeReg)) return false;
    if (!_readReg16(M5IOE1_REG_GPIO_OUT_L, &outReg)) return false;
    if (!_readReg16(M5IOE1_REG_GPIO_IN_L, &inReg)) return false;
    if (!_readReg16(M5IOE1_REG_GPIO_PU_L, &puReg)) return false;
    if (!_readReg16(M5IOE1_REG_GPIO_PD_L, &pdReg)) return false;
    if (!_readReg16(M5IOE1_REG_GPIO_DRV_L, &drvReg)) return false;
    if (!_readReg16(M5IOE1_REG_GPIO_IE_L, &ieReg)) return false;
    if (!_readReg16(M5IOE1_REG_GPIO_IP_L, &itReg)) return false;

    for (uint8_t pin = 0; pin < M5IOE1_MAX_GPIO_PINS; pin++) {
        _pinStates[pin].isOutput = (modeReg & (1 << pin)) != 0;
        _pinStates[pin].outputLevel = (outReg & (1 << pin)) ? 1 : 0;
        _pinStates[pin].inputLevel = (inReg & (1 << pin)) ? 1 : 0;
        _pinStates[pin].pull = (puReg & (1 << pin)) ? 1 : ((pdReg & (1 << pin)) ? 2 : 0);
        _pinStates[pin].drive = (drvReg & (1 << pin)) ? 1 : 0;
        _pinStates[pin].intrEnabled = (ieReg & (1 << pin)) != 0;
        _pinStates[pin].intrRising = (itReg & (1 << pin)) != 0;
    }

    _pinStatesValid = true;
    return true;
}

bool M5IOE1::_snapshotPwmStates() {
    if (!_initialized) return false;

    if (!_readReg16(M5IOE1_REG_PWM_FREQ_L, &_pwmFrequency)) return false;

    for (uint8_t ch = 0; ch < M5IOE1_MAX_PWM_CHANNELS; ch++) {
        uint8_t regL = M5IOE1_REG_PWM1_DUTY_L + (ch * 2);
        uint16_t data = 0;
        if (!_readReg16(regL, &data)) return false;

        _pwmStates[ch].duty12 = data & 0x0FFF;
        _pwmStates[ch].enabled = (data & ((uint16_t)M5IOE1_PWM_ENABLE << 8)) != 0;
        _pwmStates[ch].polarity = (data & ((uint16_t)M5IOE1_PWM_POLARITY << 8)) != 0;
    }

    _pwmStatesValid = true;
    return true;
}

bool M5IOE1::_snapshotAdcState() {
    if (!_initialized) return false;

    uint8_t ctrl = 0;
    if (!_readReg(M5IOE1_REG_ADC_CTRL, &ctrl)) return false;

    _adcState.activeChannel = ctrl & M5IOE1_ADC_CH_MASK;
    _adcState.busy = (ctrl & M5IOE1_ADC_BUSY) != 0;

    if (!_adcState.busy) {
        if (!_readReg16(M5IOE1_REG_ADC_DATA_L, &_adcState.lastValue)) {
            _adcState.lastValue = 0;
        }
    }

    _adcStateValid = true;
    return true;
}

void M5IOE1::_autoSnapshotUpdate() {
    if (_autoSnapshot && _initialized) {
        _snapshotPinStates();
        _snapshotPwmStates();
        _snapshotAdcState();
    }
}

bool M5IOE1::_isValidI2cFrequency(uint32_t speed) {
    return (speed == M5IOE1_I2C_FREQ_100K || speed == M5IOE1_I2C_FREQ_400K);
}

bool M5IOE1::getI2cSpeed(m5ioe1_i2c_speed_t* speed) {
    if (speed == nullptr || !_initialized) return false;

    uint8_t cfg = 0;
    if (!_readReg(M5IOE1_REG_I2C_CFG, &cfg)) return false;

    // 更新缓存
    // Update cache
    _i2cConfig.speed400k = (cfg & M5IOE1_I2C_SPEED_400K_BIT) != 0;
    _i2cConfigValid = true;

    *speed = _i2cConfig.speed400k ? M5IOE1_I2C_SPEED_400K : M5IOE1_I2C_SPEED_100K;
    return true;
}

bool M5IOE1::switchI2cSpeed(m5ioe1_i2c_speed_t speed) {
    if (!_initialized) {
        M5IOE1_LOG_E(TAG, "Cannot switch I2C speed: device not initialized");
        return false;
    }

    uint32_t targetFreq = (speed == M5IOE1_I2C_SPEED_400K) ? M5IOE1_I2C_FREQ_400K : M5IOE1_I2C_FREQ_100K;
    
    // 如果目标速度与当前速度相同，直接返回成功
    // If target speed is same as current, return success directly
    if (targetFreq == _requestedSpeed) {
        M5IOE1_LOG_I(TAG, "I2C speed already at %lu Hz, no change needed", targetFreq);
        return true;
    }

    // 步骤 1：读取当前 I2C 配置
    // Step 1: Read current I2C config
    uint8_t i2cCfg = 0;
    if (!_readReg(M5IOE1_REG_I2C_CFG, &i2cCfg)) {
        M5IOE1_LOG_E(TAG, "Failed to read I2C config register");
        return false;
    }

    // 步骤 2：根据目标速度设置或清除 400KHz 模式位
    // Step 2: Set or clear 400KHz mode bit based on target speed
    if (speed == M5IOE1_I2C_SPEED_400K) {
        i2cCfg |= M5IOE1_I2C_SPEED_400K_BIT;
    } else {
        i2cCfg &= ~M5IOE1_I2C_SPEED_400K_BIT;
    }
    
    if (!_writeReg(M5IOE1_REG_I2C_CFG, i2cCfg)) {
        M5IOE1_LOG_E(TAG, "Failed to write I2C config register");
        return false;
    }

    M5IOE1_LOG_I(TAG, "M5IOE1 I2C config set to %lu Hz mode", targetFreq);

    // 步骤 3：短暂延迟以允许设备处理配置更改
    // Step 3: Small delay to allow device to process the configuration change
    M5IOE1_DELAY_MS(5);

    // 步骤 4：将主机 I2C 总线切换到目标速度
    // Step 4: Switch host I2C bus to target speed
#ifdef ARDUINO
    if (_wire != nullptr) {
        // 必须使用 Wire.end() + Wire.begin() 在 ESP32 上正确切换 I2C 频率
        // Must use Wire.end() + Wire.begin() to properly switch I2C frequency on ESP32
        _wire->end();
        M5IOE1_DELAY_MS(10);
        if (!_wire->begin(_sda, _scl, targetFreq)) {
            M5IOE1_LOG_E(TAG, "Failed to re-initialize I2C bus at %lu Hz", targetFreq);
            // 尝试恢复到原来的速度
            // Try to recover with original speed
            uint32_t originalFreq = (speed == M5IOE1_I2C_SPEED_400K) ? M5IOE1_I2C_FREQ_100K : M5IOE1_I2C_FREQ_400K;
            _wire->begin(_sda, _scl, originalFreq);
            // 恢复设备配置
            // Revert device config
            if (speed == M5IOE1_I2C_SPEED_400K) {
                i2cCfg &= ~M5IOE1_I2C_SPEED_400K_BIT;
            } else {
                i2cCfg |= M5IOE1_I2C_SPEED_400K_BIT;
            }
            _writeReg(M5IOE1_REG_I2C_CFG, i2cCfg);
            return false;
        }
        M5IOE1_DELAY_MS(10);
        M5IOE1_LOG_I(TAG, "Host I2C bus switched to %lu Hz", targetFreq);
    }
#else
    // ESP-IDF：处理不同的驱动类型
    // ESP-IDF: Handle different driver types
    esp_err_t ret;
    
    switch (_i2cDriverType) {
        case M5IOE1_I2C_DRIVER_SELF_CREATED:
        case M5IOE1_I2C_DRIVER_MASTER:
            // 对于 i2c_master 驱动：删除设备并以新速度重新添加
            // For i2c_master driver: remove device and add with new speed
            if (_i2c_master_dev != nullptr) {
                ret = i2c_master_bus_rm_device(_i2c_master_dev);
                if (ret != ESP_OK) {
                    M5IOE1_LOG_E(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
                    return false;
                }
                _i2c_master_dev = nullptr;

                // 以目标速度重新创建设备句柄
                // Recreate device handle with target speed
                i2c_device_config_t dev_config = {
                    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                    .device_address = _addr,
                    .scl_speed_hz = targetFreq,
                    .scl_wait_us = 0,
                    .flags = {
                        .disable_ack_check = false,
                    },
                };
                
                ret = i2c_master_bus_add_device(_i2c_master_bus, &dev_config, &_i2c_master_dev);
                if (ret != ESP_OK) {
                    M5IOE1_LOG_E(TAG, "Failed to add I2C device at %lu Hz: %s", targetFreq, esp_err_to_name(ret));
                    // 尝试恢复到原来的速度
                    // Try to recover with original speed
                    uint32_t originalFreq = (speed == M5IOE1_I2C_SPEED_400K) ? M5IOE1_I2C_FREQ_100K : M5IOE1_I2C_FREQ_400K;
                    dev_config.scl_speed_hz = originalFreq;
                    i2c_master_bus_add_device(_i2c_master_bus, &dev_config, &_i2c_master_dev);
                    return false;
                }
                M5IOE1_LOG_I(TAG, "I2C master device recreated at %lu Hz", targetFreq);
            }
            break;
            
        case M5IOE1_I2C_DRIVER_BUS:
            // 对于 i2c_bus 驱动：删除设备并以新速度创建
            // For i2c_bus driver: delete device and create with new speed
            if (_i2c_device != nullptr) {
                ret = i2c_bus_device_delete(&_i2c_device);
                if (ret != ESP_OK) {
                    M5IOE1_LOG_E(TAG, "Failed to delete I2C device: %s", esp_err_to_name(ret));
                    return false;
                }

                // 以目标速度重新创建设备句柄
                // Recreate device handle with target speed
                _i2c_device = i2c_bus_device_create(_i2c_bus, _addr, targetFreq);
                if (_i2c_device == nullptr) {
                    M5IOE1_LOG_E(TAG, "Failed to create I2C device at %lu Hz", targetFreq);
                    // 尝试恢复到原来的速度
                    // Try to recover with original speed
                    uint32_t originalFreq = (speed == M5IOE1_I2C_SPEED_400K) ? M5IOE1_I2C_FREQ_100K : M5IOE1_I2C_FREQ_400K;
                    _i2c_device = i2c_bus_device_create(_i2c_bus, _addr, originalFreq);
                    return false;
                }
                M5IOE1_LOG_I(TAG, "I2C bus device recreated at %lu Hz", targetFreq);
            }
            break;
            
        default:
            M5IOE1_LOG_E(TAG, "Unknown I2C driver type");
            return false;
    }
#endif

    // 步骤 5：验证通信仍然有效
    // Step 5: Verify communication still works
    uint16_t uid = 0;
    if (!_readReg16(M5IOE1_REG_UID_L, &uid)) {
        M5IOE1_LOG_E(TAG, "Communication failed after switching to %lu Hz, reverting", targetFreq);

        // 恢复设备配置
        // Revert device config
        if (speed == M5IOE1_I2C_SPEED_400K) {
            i2cCfg &= ~M5IOE1_I2C_SPEED_400K_BIT;
        } else {
            i2cCfg |= M5IOE1_I2C_SPEED_400K_BIT;
        }
        uint32_t originalFreq = (speed == M5IOE1_I2C_SPEED_400K) ? M5IOE1_I2C_FREQ_100K : M5IOE1_I2C_FREQ_400K;
        
#ifdef ARDUINO
        if (_wire != nullptr) {
            _wire->end();
            M5IOE1_DELAY_MS(10);
            _wire->begin(_sda, _scl, originalFreq);
            M5IOE1_DELAY_MS(10);
        }
#else
        switch (_i2cDriverType) {
            case M5IOE1_I2C_DRIVER_SELF_CREATED:
            case M5IOE1_I2C_DRIVER_MASTER:
                if (_i2c_master_dev != nullptr) {
                    i2c_master_bus_rm_device(_i2c_master_dev);
                    i2c_device_config_t dev_config = {
                        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                        .device_address = _addr,
                        .scl_speed_hz = originalFreq,
                        .scl_wait_us = 0,
                        .flags = { .disable_ack_check = false },
                    };
                    i2c_master_bus_add_device(_i2c_master_bus, &dev_config, &_i2c_master_dev);
                }
                break;
            case M5IOE1_I2C_DRIVER_BUS:
                if (_i2c_device != nullptr) {
                    i2c_bus_device_delete(&_i2c_device);
                    _i2c_device = i2c_bus_device_create(_i2c_bus, _addr, originalFreq);
                }
                break;
            default:
                break;
        }
#endif
        _writeReg(M5IOE1_REG_I2C_CFG, i2cCfg);
        return false;
    }

    // 更新 I2C 配置缓存和请求速度
    // Update I2C config cache and requested speed
    _i2cConfig.speed400k = (speed == M5IOE1_I2C_SPEED_400K);
    _requestedSpeed = targetFreq;

    M5IOE1_LOG_I(TAG, "Successfully switched to %lu Hz I2C mode", targetFreq);
    return true;
}

bool M5IOE1::_initDevice() {
    // 读取设备信息以验证通信
    // Read device info to verify communication
    uint16_t uid = 0;
    uint8_t version = 0;

    if (!_readReg16(M5IOE1_REG_UID_L, &uid)) {
        M5IOE1_LOG_E(TAG, "Failed to read device UID");
        return false;
    }

    if (!_readReg(M5IOE1_REG_REV, &version)) {
        M5IOE1_LOG_E(TAG, "Failed to read device version");
        return false;
    }

    M5IOE1_LOG_I(TAG, "Device UID: 0x%04X, FW Version: %d", uid, version);
    return true;
}

void M5IOE1::_handleInterrupt() {
    uint16_t status = getInterruptStatus();

    for (uint8_t pin = 0; pin < M5IOE1_MAX_GPIO_PINS; pin++) {
        if ((status & (1 << pin)) && _callbacks[pin].enabled) {
            if (_enableDefaultIsrLog) {
                M5IOE1_LOG_I(TAG, "Pin %d triggered by %s edge", pin, _callbacks[pin].rising ? "RISING" : "FALLING");
            }

            if (_callbacks[pin].callbackArg != nullptr) {
                _callbacks[pin].callbackArg(_callbacks[pin].arg);
            } else if (_callbacks[pin].callback != nullptr) {
                _callbacks[pin].callback();
            }
            clearInterrupt(pin);
        }
    }
}

bool M5IOE1::_pinsConflict(uint8_t a, uint8_t b) {
    // 中断冲突对（1-based IO 编号）：
    // Interrupt conflict pairs (1-based IO numbers):
    // (1,6), (2,3), (7,12), (8,9), (10,14), (11,13)
    uint8_t A = a + 1, B = b + 1;

    auto eq = [](uint8_t x, uint8_t y, uint8_t m, uint8_t n) {
        return (x == m && y == n) || (x == n && y == m);
    };

    return eq(A, B, 1, 6) || eq(A, B, 2, 3) || eq(A, B, 7, 12) ||
           eq(A, B, 8, 9) || eq(A, B, 10, 14) || eq(A, B, 11, 13);
}

bool M5IOE1::_hasConflictingInterrupt(uint8_t pin) {
    for (uint8_t i = 0; i < M5IOE1_MAX_GPIO_PINS; i++) {
        if (i != pin && _pinStates[i].intrEnabled && _pinsConflict(pin, i)) {
            return true;
        }
    }
    return false;
}

// ============================
// 配置验证辅助函数
// Configuration Validation Helpers
// ============================

bool M5IOE1::_getInterruptMutexPin(uint8_t pin, uint8_t* mutexPin) {
    if (mutexPin == nullptr) return false;

    // 中断互斥对（0-based 引脚索引）：
    // Interrupt mutex pairs (0-based pin indices):
    // IO1(0) <-> IO6(5)
    // IO2(1) <-> IO3(2)
    // IO7(6) <-> IO12(11)
    // IO8(7) <-> IO9(8)
    // IO10(9) <-> IO14(13)
    // IO11(10) <-> IO13(12)

    static const uint8_t mutexPairs[][2] = {
        {0, 5},   // IO1 <-> IO6
        {1, 2},   // IO2 <-> IO3
        {6, 11},  // IO7 <-> IO12
        {7, 8},   // IO8 <-> IO9
        {9, 13},  // IO10 <-> IO14
        {10, 12}  // IO11 <-> IO13
    };

    for (size_t i = 0; i < sizeof(mutexPairs) / sizeof(mutexPairs[0]); i++) {
        if (pin == mutexPairs[i][0]) {
            *mutexPin = mutexPairs[i][1];
            return true;
        }
        if (pin == mutexPairs[i][1]) {
            *mutexPin = mutexPairs[i][0];
            return true;
        }
    }

    return false;
}

bool M5IOE1::_isNeopixelPin(uint8_t pin) {
    // NeoPixel LED 功能仅在 IO14 上可用（引脚索引 13）
    // NeoPixel LED function only available on IO14 (pin index 13)
    return (pin == 13);
}

bool M5IOE1::_hasActiveInterrupt(uint8_t pin) {
    if (!_isValidPin(pin)) return false;
    if (!_pinStatesValid) return false;
    return _pinStates[pin].intrEnabled;
}

bool M5IOE1::_hasActiveAdc(uint8_t pin) {
    if (!_isAdcPin(pin)) return false;
    if (!_adcStateValid) return false;

    // 检查 ADC 当前是否正在使用此引脚的通道
    // Check if the ADC is currently using this pin's channel
    uint8_t channel = _getAdcChannel(pin);
    return (_adcState.activeChannel == channel);
}

bool M5IOE1::_hasActivePwm(uint8_t pin) {
    if (!_isPwmPin(pin)) return false;
    if (!_pwmStatesValid) return false;

    uint8_t channel = _getPwmChannel(pin);
    return _pwmStates[channel].enabled;
}

bool M5IOE1::_hasI2cSleepEnabled() {
    if (!_i2cConfigValid) return false;
    return (_i2cConfig.sleepTime > 0);
}

bool M5IOE1::_isLedEnabled() {
    return _ledEnabled;
}

// ============================
// I2C 配置快照
// I2C Config Snapshot
// ============================

void M5IOE1::_clearI2cConfig() {
    memset(&_i2cConfig, 0, sizeof(_i2cConfig));
    _i2cConfigValid = false;
}

bool M5IOE1::_snapshotI2cConfig() {
    if (!_initialized) return false;

    uint8_t cfg = 0;
    if (!_readReg(M5IOE1_REG_I2C_CFG, &cfg)) return false;

    _i2cConfig.sleepTime = cfg & M5IOE1_I2C_SLEEP_MASK;
    _i2cConfig.speed400k = (cfg & M5IOE1_I2C_SPEED_400K_BIT) != 0;
    _i2cConfig.wakeRising = (cfg & M5IOE1_I2C_WAKE_RISING) != 0;
    _i2cConfig.pullOff = (cfg & M5IOE1_I2C_PULL_OFF) != 0;
    _i2cConfigValid = true;

    // 同时快照 LED 配置
    // Also snapshot LED config
    uint8_t ledCfg = 0;
    if (_readReg(M5IOE1_REG_LED_CFG, &ledCfg)) {
        _ledCount = ledCfg & M5IOE1_LED_NUM_MASK;
        _ledEnabled = (_ledCount > 0);
    }

    return true;
}

// ============================
// 快照验证
// Snapshot Verification
// ============================

m5ioe1_snapshot_verify_t M5IOE1::verifySnapshot() {
    m5ioe1_snapshot_verify_t result = {true, false, false, false, 0, 0, 0, 0};

    if (!_initialized) {
        result.consistent = false;
        return result;
    }

    // 验证 GPIO 寄存器
    // Verify GPIO registers
    if (_pinStatesValid) {
        uint16_t actualMode = 0, actualOutput = 0;

        if (_readReg16(M5IOE1_REG_GPIO_MODE_L, &actualMode) &&
            _readReg16(M5IOE1_REG_GPIO_OUT_L, &actualOutput)) {

            // 从缓存构建期望值
            // Build expected values from cache
            uint16_t expectedMode = 0, expectedOutput = 0;
            for (uint8_t i = 0; i < M5IOE1_MAX_GPIO_PINS; i++) {
                if (_pinStates[i].isOutput) {
                    expectedMode |= (1 << i);
                }
                if (_pinStates[i].outputLevel) {
                    expectedOutput |= (1 << i);
                }
            }

            result.expected_mode = expectedMode;
            result.actual_mode = actualMode;
            result.expected_output = expectedOutput;
            result.actual_output = actualOutput;

            if (expectedMode != actualMode || expectedOutput != actualOutput) {
                result.gpio_mismatch = true;
                result.consistent = false;
            }
        } else {
            result.consistent = false;
        }
    }

    // 验证 PWM 寄存器
    // Verify PWM registers
    if (_pwmStatesValid) {
        uint16_t actualFreq = 0;
        if (_readReg16(M5IOE1_REG_PWM_FREQ_L, &actualFreq)) {
            if (actualFreq != _pwmFrequency) {
                result.pwm_mismatch = true;
                result.consistent = false;
            }
        }

        for (uint8_t ch = 0; ch < M5IOE1_MAX_PWM_CHANNELS; ch++) {
            uint8_t regL = M5IOE1_REG_PWM1_DUTY_L + (ch * 2);
            uint16_t data = 0;
            if (_readReg16(regL, &data)) {
                uint16_t actualDuty = data & 0x0FFF;
                bool actualEnabled = (data & ((uint16_t)M5IOE1_PWM_ENABLE << 8)) != 0;
                bool actualPolarity = (data & ((uint16_t)M5IOE1_PWM_POLARITY << 8)) != 0;

                if (actualDuty != _pwmStates[ch].duty12 ||
                    actualEnabled != _pwmStates[ch].enabled ||
                    actualPolarity != _pwmStates[ch].polarity) {
                    result.pwm_mismatch = true;
                    result.consistent = false;
                }
            }
        }
    }

    // 验证 ADC 寄存器
    // Verify ADC registers
    if (_adcStateValid) {
        uint8_t ctrl = 0;
        if (_readReg(M5IOE1_REG_ADC_CTRL, &ctrl)) {
            uint8_t actualChannel = ctrl & M5IOE1_ADC_CH_MASK;
            if (actualChannel != _adcState.activeChannel) {
                result.adc_mismatch = true;
                result.consistent = false;
            }
        }
    }

    return result;
}

// ============================
// 缓存状态查询函数
// Cached State Query Functions
// ============================

bool M5IOE1::getCachedPwmFrequency(uint16_t* frequency) {
    if (frequency == nullptr || !_pwmStatesValid) return false;
    *frequency = _pwmFrequency;
    return true;
}

bool M5IOE1::getCachedPwmState(uint8_t channel, uint16_t* duty12, bool* polarity, bool* enabled) {
    if (channel > 3 || duty12 == nullptr || polarity == nullptr || enabled == nullptr) {
        return false;
    }
    if (!_pwmStatesValid) return false;

    *duty12 = _pwmStates[channel].duty12;
    *polarity = _pwmStates[channel].polarity;
    *enabled = _pwmStates[channel].enabled;
    return true;
}

bool M5IOE1::getCachedAdcState(uint8_t* activeChannel, bool* busy, uint16_t* lastValue) {
    if (activeChannel == nullptr || busy == nullptr || lastValue == nullptr) {
        return false;
    }
    if (!_adcStateValid) return false;

    *activeChannel = _adcState.activeChannel;
    *busy = _adcState.busy;
    *lastValue = _adcState.lastValue;
    return true;
}

bool M5IOE1::getCachedPinState(uint8_t pin, bool* isOutput, uint8_t* level, uint8_t* pull) {
    if (!_isValidPin(pin) || isOutput == nullptr || level == nullptr || pull == nullptr) {
        return false;
    }
    if (!_pinStatesValid) return false;

    *isOutput = _pinStates[pin].isOutput;
    *level = _pinStates[pin].isOutput ? _pinStates[pin].outputLevel : _pinStates[pin].inputLevel;
    *pull = _pinStates[pin].pull;
    return true;
}

// ============================
// 平台特定函数
// Platform-Specific Functions
// ============================

#ifdef ARDUINO

static M5IOE1* _arduinoPollingInstance = nullptr;
static TaskHandle_t _arduinoPollingTask = nullptr;

void M5IOE1::_pollTaskArduino(void* arg) {
    M5IOE1* self = static_cast<M5IOE1*>(arg);

    while (true) {
        uint16_t status = self->getInterruptStatus();
        if (status != 0) {
            self->_handleInterrupt();
        }
        vTaskDelay(pdMS_TO_TICKS(self->_pollingInterval));
    }
}

bool M5IOE1::_setupPollingArduino() {
    _arduinoPollingInstance = this;

    BaseType_t ok = xTaskCreatePinnedToCore(
        _pollTaskArduino, "m5ioe1_poll", 4096, this, 5, &_arduinoPollingTask, tskNO_AFFINITY
    );
    return ok == pdPASS;
}

void M5IOE1::_cleanupPollingArduino() {
    if (_arduinoPollingTask) {
        vTaskDelete(_arduinoPollingTask);
        _arduinoPollingTask = nullptr;
    }
    _arduinoPollingInstance = nullptr;
}

#else // ESP-IDF

void M5IOE1::_pollTaskFunc(void* arg) {
    M5IOE1* self = static_cast<M5IOE1*>(arg);

    while (true) {
        ESP_LOGI(TAG, "Polling...");
        uint16_t status = self->getInterruptStatus();
        if (status != 0) {
            self->_handleInterrupt();
        }
        vTaskDelay(pdMS_TO_TICKS(self->_pollingInterval));
    }
}

void IRAM_ATTR M5IOE1::_isrHandler(void* arg) {
    M5IOE1* self = static_cast<M5IOE1*>(arg);
    if (self && self->_intrQueue) {
        uint32_t evt = 1;
        BaseType_t hpw = pdFALSE;
        xQueueSendFromISR(self->_intrQueue, &evt, &hpw);
        if (hpw == pdTRUE) portYIELD_FROM_ISR();
    }
}

bool M5IOE1::_setupHardwareInterrupt() {
    if (_intPin < 0) return false;

    _intrQueue = xQueueCreate(8, sizeof(uint32_t));
    if (_intrQueue == nullptr) return false;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << _intPin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL
    };

    if (gpio_config(&io_conf) != ESP_OK) {
        vQueueDelete(_intrQueue);
        _intrQueue = nullptr;
        return false;
    }

    if (gpio_install_isr_service(ESP_INTR_FLAG_IRAM) != ESP_OK &&
        gpio_install_isr_service(ESP_INTR_FLAG_IRAM) != ESP_ERR_INVALID_STATE) {
        vQueueDelete(_intrQueue);
        _intrQueue = nullptr;
        return false;
    }

    if (gpio_isr_handler_add((gpio_num_t)_intPin, _isrHandler, this) != ESP_OK) {
        vQueueDelete(_intrQueue);
        _intrQueue = nullptr;
        return false;
    }

    // 创建任务以处理中断
    // Create task to handle interrupts
    xTaskCreatePinnedToCore(
        [](void* arg) {
            M5IOE1* self = static_cast<M5IOE1*>(arg);
            uint32_t evt;
            while (true) {
                if (xQueueReceive(self->_intrQueue, &evt, portMAX_DELAY) == pdTRUE) {
                    gpio_intr_disable((gpio_num_t)self->_intPin);
                    self->_handleInterrupt();
                    gpio_intr_enable((gpio_num_t)self->_intPin);
                }
            }
        },
        "m5ioe1_intr", 4096, this, configMAX_PRIORITIES - 2, &_pollTask, tskNO_AFFINITY
    );

    return true;
}

bool M5IOE1::_setupPolling() {
    BaseType_t ok = xTaskCreatePinnedToCore(
        _pollTaskFunc, "m5ioe1_poll", 4096, this, 5, &_pollTask, tskNO_AFFINITY
    );
    return ok == pdPASS;
}

void M5IOE1::_cleanupInterrupt() {
    if (_pollTask) {
        vTaskDelete(_pollTask);
        _pollTask = nullptr;
    }
    if (_intrQueue) {
        vQueueDelete(_intrQueue);
        _intrQueue = nullptr;
    }
    if (_intPin >= 0) {
        // 移除 GPIO ISR handler
        // Remove GPIO ISR handler
        // 如果返回错误（如"GPIO isr service is not installed"）可忽略
        // The returned error (e.g., "GPIO isr service is not installed") can be ignored
        // This error indicates that the ISR service is not currently installed and does not need to be removed, which is expected behavior
        esp_err_t err = gpio_isr_handler_remove((gpio_num_t)_intPin);
        if (err != ESP_OK) {
            M5IOE1_LOG_W(TAG, "gpio_isr_handler_remove failed (can be ignored if ISR service was not installed): %s", esp_err_to_name(err));
        }
    }
}

#endif
