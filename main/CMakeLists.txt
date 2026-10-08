idf_component_register(
    SRCS "main.c" "dc_motor_pwm.c"
	PRIV_REQUIRES esp_wifi nvs_flash esp_eth
    REQUIRES esp_driver_twai esp_driver_ledc esp_timer  esp_driver_gpio esp_event nvs_flash
    INCLUDE_DIRS "."
)
