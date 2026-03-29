#define EXAMPLE_ONEWIRE_BUS_GPIO    0
#define EXAMPLE_ONEWIRE_MAX_DS18B20 2

#include "onewire_bus.h"
#include "ds18b20.h"
#include "esp_log.h"
#include <math.h>

#include "temp_sensor.h"

static const char *TAG = "TEMP_SENSOR";
static ds18b20_device_handle_t ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];
static int ds18b20_device_num = 0;

bool temp_sensor_available(void)
{
    return ds18b20_device_num > 0;
}

esp_err_t init_temp_sensor(void)
{
    // install 1-wire bus
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };

    esp_err_t ret = onewire_new_bus_rmt(&bus_config, &rmt_config, &bus);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create 1-Wire bus: %s", esp_err_to_name(ret));
        return ret;
    }

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ret = onewire_new_device_iter(bus, &iter);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create device iterator: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Device iterator created, start searching...");
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK) {
                ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", ds18b20_device_num, next_onewire_device.address);
                ds18b20_device_num++;
            } else {
                ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        } else if (search_result != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "1-Wire search failed: %s, aborting search", esp_err_to_name(search_result));
            break;
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    onewire_del_device_iter(iter);
    ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);

    if (ds18b20_device_num == 0) {
        ESP_LOGW(TAG, "No temperature sensors found - temperature features disabled");
    }

    return ESP_OK;
}

float get_temp(void)
{
    if (ds18b20_device_num == 0) {
        return NAN;
    }

    float temperature = NAN;
    for (int i = 0; i < ds18b20_device_num; i++) {
        esp_err_t ret = ds18b20_trigger_temperature_conversion(ds18b20s[i]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "DS18B20[%d] conversion failed: %s", i, esp_err_to_name(ret));
            continue;
        }
        ret = ds18b20_get_temperature(ds18b20s[i], &temperature);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "DS18B20[%d] read failed: %s", i, esp_err_to_name(ret));
            temperature = NAN;
            continue;
        }
        ESP_LOGI(TAG, "temperature read from DS18B20[%d]: %.2fC", i, temperature);
    }
    return temperature;
}
