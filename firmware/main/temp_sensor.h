#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <stdbool.h>
#include "esp_err.h"

esp_err_t init_temp_sensor(void);
float get_temp(void);
bool temp_sensor_available(void);

#endif // TEMP_SENSOR_H
