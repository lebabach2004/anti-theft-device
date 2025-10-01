/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <driver/uart.h>
#include "sdkconfig.h"

#include "mpu6050.h"
#include "roll_pitch.h"
#include "quaternions.h"
#include "gps.h"

#define I2C_MASTER_SCL_IO    22 
#define I2C_MASTER_SDA_IO    21
#define PIN_CLK 18

#define PIN_MOSI 23
#define PIN_RESET 22
#define PIN_CS 5
#define BUF_SIZE 1024
GPS_t GPS;
static char gps_buffer[BUF_SIZE];
static char latest_nmea[BUF_SIZE];
int gps_index = 0;
static const char *TAG = "MAIN";

void gps_rx_task(void *arg){
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE - 1, 50 / portTICK_PERIOD_MS);
        for(uint8_t i=0; i<len; i++){
            if(data[i] != '\n' && gps_index < BUF_SIZE - 1){
                gps_buffer[gps_index++] = data[i];
            }
            else{
                gps_buffer[gps_index] = '\0'; 
                if(!strncmp(gps_buffer,"$GPGGA",6) || !strncmp(gps_buffer,"$GPRMC",6)){
                    strncpy(latest_nmea, gps_buffer, BUF_SIZE);
                }
                gps_index = 0;
            }
        }
    }
}
void gps_process_task(void *arg){
    while(1){
        vTaskDelay(1000/portTICK_PERIOD_MS);
        if(strlen(latest_nmea)>0 && GPS_validate(latest_nmea)){
            GPS_parse(latest_nmea);
            ESP_LOGI(TAG,"Lat=%.6f Lon=%.6f", GPS.dec_latitude, GPS.dec_longitude);
        }
    }
}
void app_main() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_14, GPIO_NUM_15, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // xTaskCreate(gps_rx_task, "gps_rx_task", 2048, NULL, 10, NULL);
    // xTaskCreate(gps_process_task,"gps_process_task",2048,NULL,11,NULL);


    esp_err_t ret;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float accel_bias[3] = {0.00f, 0.00f, 0.00f};
    float gyro_bias[3] = {0.00f, 0.00f, 0.00f};

        i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "I2C param config failed");
        return;
    }
    ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0 );
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "I2C driver install failed");
        return;
    }
    ret = mpu6050_init(I2C_NUM_0);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Initialization failed");
        return;
    }
    else{
        printf("Success\n");
    }
    mpu6050_calibrate(I2C_NUM_0, accel_bias, gyro_bias);
    roll_pitch_init();

    // Initialize Quaternion
    Quaternion q;
    quaternion_init(&q);
        while (1) {
        // Read raw data
        ret = mpu6050_read_raw_data(I2C_NUM_0, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        if (ret != ESP_OK) {
            ESP_LOGE("MPU6050", "Read failed");
            return;
        }

        // Convert raw data to physical values
        mpu6050_convert_accel(accel_x, accel_y, accel_z, &accel_x_g, &accel_y_g, &accel_z_g);
        mpu6050_convert_gyro(gyro_x, gyro_y, gyro_z, &gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

        // Update roll and pitch
        roll_pitch_update(accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);

        // Update Quaternion
        quaternion_update(&q, gyro_x_dps, gyro_y_dps, gyro_z_dps, accel_x_g, accel_y_g, accel_z_g, 0.01f); // 10 ms time step

        // Print results
        printf("Accel: X=%0.2f m/s^2, Y=%0.2f m/s^2, Z=%0.2f m/s^2\n", accel_x_g, accel_y_g, accel_z_g);
        printf("Gyro: X=%0.2f deg/s, Y=%0.2f deg/s, Z=%0.2f deg/s\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);
        // printf("Roll: %0.2f degrees\n", roll_get());
        // printf("Pitch: %0.2f degrees\n", pitch_get());

        // // Print Quaternion-based Roll, Pitch, Yaw
        // printf("Quaternion Roll: %0.2f degrees\n", quaternion_get_roll(&q));
        // printf("Quaternion Pitch: %0.2f degrees\n", quaternion_get_pitch(&q));
        // printf("Quaternion Yaw: %0.2f degrees\n", quaternion_get_yaw(&q));

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}