set (SOURCES main.c)

idf_component_register(
	SRCS ${SOURCES}
    INCLUDE_DIRS . images
    REQUIRES driver esp_timer lvgl_esp32_drivers lvgl lis2mdl_STdC_Driver lsm6dso_STdC_Driver SW_ESP_NTP SW_I2c_Driver
    SW_IMU_Driver esp_wifi)

target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_CONF_INCLUDE_SIMPLE=1)
