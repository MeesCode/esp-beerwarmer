#include "ina219.h"
#include "esp_log.h"

static const char *TAG = "INA219";

// Register addresses
#define REG_CONFIG       0x00
#define REG_SHUNT_VOLT   0x01
#define REG_BUS_VOLTAGE  0x02
#define REG_POWER        0x03
#define REG_CURRENT      0x04
#define REG_CALIBRATION  0x05

// Configuration for 2A max, 20mOhm shunt
#define CONFIG_WORD  0x01FF  // 32V bus, +/-40mV gain, 12-bit, continuous
#define CAL_VALUE    0x7FF8  // Calibration for 2A range
#define CURRENT_LSB  0.00006257f  // 62.57 uA per LSB

#define I2C_TIMEOUT_MS 100

static i2c_master_dev_handle_t dev_handle = NULL;
static bool ina219_initialized = false;

static esp_err_t write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };
    return i2c_master_transmit(dev_handle, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, buf, 2, I2C_TIMEOUT_MS);
    if (ret == ESP_OK) {
        *value = (buf[0] << 8) | buf[1];
    }
    return ret;
}

esp_err_t ina219_init(i2c_master_bus_handle_t bus_handle)
{
    esp_err_t ret;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA219_I2C_ADDR,
        .scl_speed_hz = 100000,
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = write_reg(REG_CALIBRATION, CAL_VALUE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Calibration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = write_reg(REG_CONFIG, CONFIG_WORD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ina219_initialized = true;
    ESP_LOGI(TAG, "Initialized successfully");
    return ESP_OK;
}

bool ina219_available(void)
{
    return ina219_initialized;
}

float ina219_read_voltage(void)
{
    if (!ina219_initialized) return 0.0f;
    uint16_t raw;
    if (read_reg(REG_BUS_VOLTAGE, &raw) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read voltage");
        return 0.0f;
    }
    // Bits [15:3] = voltage, LSB = 4mV
    return (float)(raw >> 3) * 0.004f;
}

float ina219_read_current(void)
{
    if (!ina219_initialized) return 0.0f;
    uint16_t raw;
    if (read_reg(REG_CURRENT, &raw) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current");
        return 0.0f;
    }
    return (float)(int16_t)raw * CURRENT_LSB;
}

void ina219_read_all(float *voltage, float *current, float *power)
{
    *voltage = ina219_read_voltage();
    *current = ina219_read_current();
    if (power) {
        *power = *voltage * *current;
    }
}
