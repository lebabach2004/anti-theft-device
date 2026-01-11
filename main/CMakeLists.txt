idf_component_register(SRCS  "main.c" 
                    # PRIV_REQUIRES spi_flash driver nvs_flash esp_wifi esp_event esp_http_server esp_timer app_update esp_http_client esp_https_ota esp_adc mqtt
                    INCLUDE_DIRS ".")

