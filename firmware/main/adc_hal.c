

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_types.h"
#include "esp_log.h"

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t cali_handle;

void init_adc(){
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t channelConfig = {
        .atten = ADC_ATTEN_DB_12,  // 150mV - 2450mV
        .bitwidth = ADC_BITWIDTH_12,  // resolution 12 bit
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &channelConfig);

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_0,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
}

uint16_t get_adc(){

    int raw;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &raw);

    int cali;
    adc_cali_raw_to_voltage(cali_handle, raw, &cali);

    return (uint16_t)cali;
}