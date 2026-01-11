set(app_src app_mqtt.c adc.c ota.c input_iot.c sim.c Soft_AP.c json_parser.c json_generator.c roll_pitch.c mpu6050.c quaternions.c gps.c wifi_sta.c http_server_app.c json_generator.c json_parser.c Soft_AP.c sim.c input_iot.c buzzer.c )
set(pri_req spi_flash driver nvs_flash esp_wifi esp_event esp_http_server esp_timer app_update esp_http_client esp_https_ota esp_adc mqtt)
idf_component_register(SRCS ${app_src} 
                       INCLUDE_DIRS "."
                       REQUIRES ${pri_req})