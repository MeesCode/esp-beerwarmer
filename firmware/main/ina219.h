#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#define INA219_I2C_ADDR 0x40

/**
 * @brief Initialize the INA219 current/voltage sensor
 * @param bus_handle I2C bus handle to use
 * @return ESP_OK on success
 */
esp_err_t ina219_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Read the bus voltage
 * @return Voltage in Volts
 */
float ina219_read_voltage(void);

/**
 * @brief Read the current
 * @return Current in Amperes
 */
float ina219_read_current(void);

/**
 * @brief Read both voltage and current, calculate power
 * @param voltage Pointer to store voltage (V)
 * @param current Pointer to store current (A)
 * @param power Pointer to store power (W), can be NULL
 */
void ina219_read_all(float *voltage, float *current, float *power);
