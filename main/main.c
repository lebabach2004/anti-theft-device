/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include "freertos/queue.h"
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <driver/uart.h>
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "mpu6050.h"
#include "roll_pitch.h"
#include "quaternions.h"
#include "gps.h"
#include "wifi_sta.h"
#include "http_server_app.h"
#include "Soft_AP.h"
#include "stdbool.h"
#include "sim.h"
#include <json_generator.h>
#include <json_parser.h>
#include "esp_task_wdt.h"
#include <input_iot.h>
#include "buzzer.h"
#define PIN_CLK 18
#define BUF_SIZE 1024
static const char *TAG = "MAIN";
GPS_t GPS;
QueueHandle_t eventQueue;
static char gps_buffer[BUF_SIZE];
static char latest_nmea[BUF_SIZE];
int gps_index = 0;
extern void mqtt_task(void *arg); 
extern void buzzer_task(void *arg);
extern char MAC_address[18];
// MPU6050 variables
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
float accel_x_g, accel_y_g, accel_z_g;
float gyro_x_dps, gyro_y_dps, gyro_z_dps;
float accel_bias[3] = {0.00f, 0.00f, 0.00f};
float gyro_bias[3] = {0.00f, 0.00f, 0.00f};
float accChange; // value of acceleration change

// mqtt_public_variables
char sim_public_mqtt_msg[BUF_SIZE];
extern char MAC_address[18];
uint8_t status=1;
extern bool antiTheft;
extern char *phone_list[10];
extern int phone_count;
uint16_t battery=60;
bool isCharging=false;

// variables from sim.c
extern bool updateLocation;
extern bool warning;
extern bool update_OTA;
// device state enum and variable
extern volatile alarm_state_t alarm_state;
typedef enum{
    NORMAL_STATE,
    ALERT_STATE,
    SOS_STATE,
    LOW_BATTERY_STATE
} device_state_t;
device_state_t device_state = NORMAL_STATE;

// json mqtt publish message
void sim_public_mqtt(){
    json_gen_str_t jstr;
    json_gen_str_start(&jstr, sim_public_mqtt_msg, BUF_SIZE, NULL, NULL);
    json_gen_start_object(&jstr);
    json_gen_obj_set_string(&jstr, "deviceId", MAC_address);
    json_gen_push_array(&jstr, "location");
    json_gen_arr_set_float(&jstr, GPS.dec_latitude);
    json_gen_arr_set_float(&jstr, GPS.dec_longitude);
    json_gen_pop_array(&jstr);
    json_gen_obj_set_int(&jstr, "status", status);
    json_gen_obj_set_bool(&jstr, "antiTheft", antiTheft);
    json_gen_obj_set_int(&jstr, "battery", battery);
    json_gen_obj_set_bool(&jstr, "isConnectBatteryCharge", isCharging);
    json_gen_end_object(&jstr);
    json_gen_str_end(&jstr);
    printf("Generated JSON: %s\n", sim_public_mqtt_msg);
}
void Task_Action_MqttMessage(void *arg) {
    for (;;) {
        if(updateLocation){
            static char url[128]; 
            snprintf(url, sizeof(url), "https://www.google.com/maps?q=%f,%f", GPS.dec_latitude, GPS.dec_longitude);
            for(int i=0;i<phone_count;i++){
                sim_send_sms(phone_list[i], url, 500);
            }
            updateLocation=false;
        }
        if(warning){
            alarm_state= STATE_IDLE;
            warning=false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
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
                if(!strncmp(gps_buffer,"$GPRMC",6)){
                    strncpy(latest_nmea, gps_buffer, BUF_SIZE);
                }
                gps_index = 0;
            }
        }
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
}
void gps_process_task(void *arg){
    while(1){
        vTaskDelay(5000/portTICK_PERIOD_MS);
        printf("Latest NMEA: %s\n", latest_nmea);
        if(strlen(latest_nmea)>0 && GPS_validate(latest_nmea)){
            GPS_parse(latest_nmea);
            printf("Lat= %f,Lon= %f\n", GPS.dec_latitude, GPS.dec_longitude);
            static char url[128]; 
            snprintf(url, sizeof(url), "https://www.google.com/maps?q=%f,%f", GPS.dec_latitude, GPS.dec_longitude);
            printf("Google Maps URL: %s\n", url);
        }
    }
}
void mpu6050_task(void *arg){
    float prev_accel_x_g = 0.0f;
    float prev_accel_y_g = 0.0f;
    float prev_accel_z_g = 0.0f;
    bool first_read = true;
    esp_err_t ret;
    while(1){
        ret = mpu6050_read_raw_data(I2C_NUM_0, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        if (ret != ESP_OK) {
            ESP_LOGE("MPU6050", "Read failed");
            return;
        }
        mpu6050_convert_accel(accel_x, accel_y, accel_z, &accel_x_g, &accel_y_g, &accel_z_g);
        mpu6050_convert_gyro(gyro_x, gyro_y, gyro_z, &gyro_x_dps, &gyro_y_dps, &gyro_z_dps);
        printf("Accel: X=%0.2f m/s^2, Y=%0.2f m/s^2, Z=%0.2f m/s^2\n", accel_x_g, accel_y_g, accel_z_g);
        printf("Gyro: X=%0.2f deg/s, Y=%0.2f deg/s, Z=%0.2f deg/s\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);
        if(!first_read){
            // Calculate the change in acceleration
            float dx = accel_x_g - prev_accel_x_g;
            float dy = accel_y_g - prev_accel_y_g;
            float dz = accel_z_g - prev_accel_z_g;
            // Calculate the magnitude of the acceleration change
            float delta_a = dx*dx + dy*dy + dz*dz;
            accChange = sqrtf(delta_a);
        }
        else {
            first_read = false;
        }
        prev_accel_x_g = accel_x_g;
        prev_accel_y_g = accel_y_g;
        prev_accel_z_g = accel_z_g;
        printf("Accel: X=%0.2f m/s^2, Y=%0.2f m/s^2, Z=%0.2f m/s^2\n", accel_x_g, accel_y_g, accel_z_g);
        printf("Gyro: X=%0.2f deg/s, Y=%0.2f deg/s, Z=%0.2f deg/s\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);
        printf("Acceleration Change: %0.2f m/s^2\n", accChange);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}
bool is_low_battery() {
    return false; 
}
bool accident_detected() {
    return false; 
}
bool is_theft_detected() {
    if(alarm_state == STATE_ALARM  && antiTheft ){
        return true;
    }
    return false; 
}
void Task_StateUpdate(void *pvParameters) {
    device_state_t new_state;
    for (;;) {
        if (is_low_battery()) 
            new_state = LOW_BATTERY_STATE;
        else if (accident_detected())
            new_state = SOS_STATE;
        else if (is_theft_detected()) 
            new_state = ALERT_STATE;
        else 
            new_state = NORMAL_STATE;
        device_state = new_state;
        if (xQueueSend(eventQueue, &new_state, 0) != pdPASS) {
            ESP_LOGW(TAG, "Queue full, state not sent!");
        }  
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
void Task_ActionHandler(void *pvParameters) {
    device_state_t state;
    for (;;) {
        if (xQueueReceive(eventQueue, &state, portMAX_DELAY)) {
            switch (state) {
                case ALERT_STATE:
                    // start_alarm(); 
                    buzzer_on_alarm();
                    printf("Device in ALERT_STATE\n");
                    break;
                case SOS_STATE: 
                    // send_sos_message(); 
                    break;
                case LOW_BATTERY_STATE: 
                    // send_low_battery_warning(); 
                    break;
                case NORMAL_STATE: 
                    buzzer_off();
                    printf("Device in NORMAL_STATE\n");
                    // stop_alarm(); 
                    break;
            }
        }
    }
}
void app_main() {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    eventQueue = xQueueCreate(10, sizeof(device_state_t));
    
    // Init SIM800C
    ret=SIM_init();
    if (ret != ESP_OK) {
        ESP_LOGE("SIM", "Initialization failed");
        return;
    }
    // Init GPS
    ret=GPS_init();
    if (ret != ESP_OK) {
        ESP_LOGE("GPS", "Initialization failed");
        return;
    }
    ret=buzzer_init(23);
    if (ret != ESP_OK) {
        ESP_LOGE("BUZZER", "Initialization failed");
        return;
    }
    input_io_create(GPIO_NUM_4, LO_TO_HI);
    // // Initialize MPU6050
    // ret = mpu6050_init(I2C_NUM_0);
    // if (ret != ESP_OK) {
    //     ESP_LOGE("MPU6050", "Initialization failed");
    //     return;
    // }
    // mpu6050_calibrate(I2C_NUM_0, accel_bias, gyro_bias);
    // wifi_start();
    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 4, NULL);
    esp_ap_start();
    start_webserver();
    // sim_public_mqtt();
    xTaskCreate(gps_rx_task, "gps_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(gps_process_task,"gps_process_task",2048,NULL,6,NULL);
    mqtt_connect("esp32_client", "broker.hivemq.com", 200);
    mqtt_subscribe("esp32/device", 0, 1000);
    mqtt_subscribe("esp32/updateOTA", 0, 1000);
    // xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 7, NULL);

    // Task to update device state based on conditions
    xTaskCreate(Task_StateUpdate, "StateUpdate", 2048, NULL, 7, NULL);
    xTaskCreate(Task_ActionHandler, "ActionHandler", 2048, NULL, 8, NULL);
    xTaskCreate(Task_Action_MqttMessage, "MqttMessage", 4096, NULL, 9, NULL);
    xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 10, NULL);
}