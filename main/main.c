#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "main.h"
#include "string.h"
#include "driver/gpio.h"
#include "temp_sensor.h"

#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "oled_gfx.h"

#define HEATER_PIN GPIO_NUM_1

static const char *TAG = "MAIN";
bool zb_connected = false;
bool switch_state = true;

const float target_temp = 22.0;
const float temp_offset = 0.1;

float temps[128] = {-1.0f};
int temp_index = 0;

#define DEFINE_PSTRING(var, str)   \
    const struct                   \
    {                              \
        unsigned char len;         \
        char content[sizeof(str)]; \
    }(var) = {sizeof(str) - 1, (str)}


void report_output_binary_sensor(uint16_t value)
{
    esp_zb_zcl_set_attribute_val(
        HA_ESP_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT, 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
        ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, 
        &value, 
        false
    );
}

void report_temperature(float temp)
{
    uint16_t temp_zb = (uint16_t)(temp*100.);
    esp_zb_zcl_set_attribute_val(
        HA_ESP_ENDPOINT, 
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, 
        &temp_zb, 
        false
    );
}

void draw_graph()
{
    gfx_clear_area(0, 32, 128, 32);
    float max_temp = 0;
    float min_temp = 100;
    for(int i = 0; i < 128; i++){
        if(temps[i] > max_temp && temps[i] >= 0) max_temp = temps[i] + 0.5;
        if(temps[i] < min_temp && temps[i] >= 0) min_temp = temps[i] - 0.5;
    }

    float pxpt = (32.0) / (max_temp - min_temp);
    ESP_LOGI(TAG, "index: %d, max_temp: %f, min_temp: %f", temp_index, max_temp, min_temp);

    int prev_y = -1.0f;
    for(int i = 0; i < 128; i++){
        int index = (temp_index + i + 1) % 128;

        if(temps[index] < 0) continue;
        int y = pxpt * (temps[index] - min_temp);

        if(prev_y < 0){
            gfx_set_pixel(i, 64 - y);
        } else if(y > prev_y){
            gfx_fill_area(i, 64 - y, 1, y - prev_y);
        } else if(y < prev_y){
            gfx_fill_area(i, 64 - prev_y, 1, prev_y - y);
        } else {
            gfx_set_pixel(i, 64 - y);
        }

        prev_y = y;
    }
    
}

static void temp_task(void *pvParameters)
{
    for(;;){
        vTaskDelay(100 / portTICK_PERIOD_MS);

        float temp = get_temp();

        char temp_str[10];
        sprintf(temp_str, "%.2f C ", temp);
        gfx_draw_text(0, 10, temp_str);

        temps[temp_index] = temp;

        draw_graph();
        if(zb_connected)
            report_temperature(temp);

        // roughly keep temp at 25c by turning on/off the heater
        if(temp < target_temp - temp_offset && switch_state){
            gpio_set_level(HEATER_PIN, 1);
            if(zb_connected)
                report_output_binary_sensor(1);
            gfx_draw_text(0, 20, "heat on ");
        } else if(temp > target_temp + temp_offset || !switch_state){
            gpio_set_level(HEATER_PIN, 0);
            if(zb_connected)
                report_output_binary_sensor(0);
            gfx_draw_text(0, 20, "heat off");
        }

        gfx_flush();
        temp_index = (temp_index + 1) % 128;
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(0x%x), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster, message->attribute.id, message->attribute.data.size);

    // handle light on/off
    if (message->info.dst_endpoint == HA_ESP_ENDPOINT){
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL){
                switch_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : switch_state;
                gpio_set_level(GPIO_NUM_15, !switch_state);
                ESP_LOGI(TAG, "Light sets to %s", switch_state ? "On" : "Off");
            }
        }
    }
    
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device connected");
                zb_connected = true;
                gfx_draw_text(112, 0, "zb");
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            zb_connected = true;
            gfx_draw_text(112, 0, "zb");
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    // setup zigbee
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // setup basic cluster
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion = 0x0002;
    uint32_t HWVersion = 0x0002;
    DEFINE_PSTRING(ManufacturerName, "Seed Studio");
    DEFINE_PSTRING(ModelIdentifier, "Beer warmer");
    DEFINE_PSTRING(DateCode, "20250303");

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)&ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)&ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, (void *)&DateCode);

    // cluster identify
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

    // cluster light
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = 0,
    };
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    // cluster temperature measurement (0 to 30 degree celsius)
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value = 0,
        .max_value = 5000,
    };
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);

    // cluster binary sensor
    esp_zb_binary_input_cluster_cfg_t binary_input_cfg = {
        .out_of_service = false,
        .status_flags = 0,
        .present_value = 0,
    };
    esp_zb_attribute_list_t *esp_zb_binary_input_cluster = esp_zb_binary_input_cluster_create(&binary_input_cfg);

    // create cluster list
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_binary_input_cluster(esp_zb_cluster_list, esp_zb_binary_input_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // create endpoint list
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    // Register device
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 1,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&reporting_info);

    // setup automatic reporting for binary input
    esp_zb_zcl_reporting_info_t reporting_info_binary = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u8 = 1,
        .u.send_info.reported_value.u8 = 0,
        .attr_id = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&reporting_info_binary);

    // start zigbee
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    // setup onboard led
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_15, 0);

    // setup heater gpio
    gpio_set_direction(HEATER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(HEATER_PIN, 0);

    // setup temperature sensor
    for(int i = 0; i < 128; i++){
        temps[i] = -1.0f;
    }
    init_temp_sensor();

    // use internal antenna
    gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_3, 0);
    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_14, 0);

    // setup zigbee
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = TEST_I2C_SDA_GPIO,
        .scl_io_num = TEST_I2C_SCL_GPIO,
        .i2c_port = I2C_NUM_0,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&i2c_bus_conf, &bus_handle);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = TEST_I2C_DEV_ADDR,
        .scl_speed_hz = TEST_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1, // According to SSD1306 datasheet
        .dc_bit_offset = 6,       // According to SSD1306 datasheet
        .lcd_cmd_bits = 8,        // According to SSD1306 datasheet
        .lcd_param_bits = 8,      // According to SSD1306 datasheet
    };

    esp_lcd_new_panel_io_i2c(bus_handle, &io_config, &io_handle);

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
    };
    esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle);
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    // turn on display
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // mirror the display
    esp_lcd_panel_mirror(panel_handle, true, true);

    

    gfx_init(panel_handle, TEST_LCD_H_RES, TEST_LCD_V_RES);

    gfx_clear_area(0, 0, 128, 64);
    gfx_draw_text(0, 0, "Beer warmer");
    gfx_flush();

    // while(1){

    //     temps[temp_index] = temp_index > 0 ? temps[(temp_index - 1) % 128] + ((rand() % 2) - 0.5) : temps[119] + ((rand() % 2) - 0.5);

    //     char temp_str[10];
    //     sprintf(temp_str, "%.2f C ", temps[temp_index]);
    //     gfx_draw_text(0, 10, temp_str);

    //     draw_graph();
    //     gfx_flush();

    //     temp_index = (temp_index + 1) % 128;
    // }

    // task for keeping track of temperature
    xTaskCreate(temp_task, "temp_task", 4096, NULL, 5, NULL);

    // task for zigbee
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
