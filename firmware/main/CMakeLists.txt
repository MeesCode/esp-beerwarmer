idf_component_register(
    # PRIV_REQUIRES esp_lcd esp_driver_i2c
    SRCS
    "main.c"
    "temp_sensor.c"
    "oled_gfx.c"
    INCLUDE_DIRS "."
)
