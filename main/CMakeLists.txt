idf_component_register(SRCS "roll_pitch.c" "mpu6050.c" "quaternions.c" "gps.c" "main.c" "wifi_sta.c" "http_server_app.c"

                    PRIV_REQUIRES spi_flash driver nvs_flash esp_wifi esp_event esp_http_server
                    INCLUDE_DIRS ".")

