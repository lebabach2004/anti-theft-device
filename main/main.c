/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
/* -------------------- INCLUDES & DEFINES -------------------- */
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
// Các header ngoại vi của dự án
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
#include <input_iot.h>
#include "buzzer.h"
#include "adc.h"

/* -----------------------------------------------------------
 * 1. ĐỊNH NGHĨA & BIẾN TOÀN CỤC (GLOBAL VARIABLES)
 * ----------------------------------------------------------- */
#define BUF_SIZE 1024
static const char *TAG = "MAIN";
// GPS & Communication
GPS_t GPS;
QueueHandle_t eventQueue;
char gps_buffer[BUF_SIZE];
char latest_nmea[BUF_SIZE];
int gps_index = 0;
char sim_public_mqtt_msg[BUF_SIZE];

// Extern từ các module khác
extern char MAC_address[18];
extern bool antiTheft,updateLocation,warning,gps_active;
extern char *phone_list[10];
extern int phone_count;
extern bool update_OTA;
extern volatile alarm_state_t alarm_state;
extern void mqtt_task(void *arg); 
extern void buzzer_task(void *arg);
extern void gps_power_manager_task(void *arg);

// MPU6050 variables
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
float accel_x_g, accel_y_g, accel_z_g;
float gyro_x_dps, gyro_y_dps, gyro_z_dps;
float accel_bias[3] = {0.00f, 0.00f, 0.00f};
float gyro_bias[3] = {0.00f, 0.00f, 0.00f};
float accChange; 

// Device state variables
uint8_t battery_capacity=100;
uint8_t status=1;
uint16_t battery=60;
bool isCharging=false;

// State Machine Enum
typedef enum{
    NORMAL_STATE,
    ALERT_STATE,
    SOS_STATE,
    LOW_BATTERY_STATE
} device_state_t;
device_state_t device_state = NORMAL_STATE;

/* -----------------------------------------------------------
 * 2. CÁC HÀM TẠO TIN NHẮN JSON (MQTT MESSAGES)
 * ----------------------------------------------------------- */


/**
 * @brief Tạo JSON tổng hợp trạng thái (Vị trí, Pin, Chống trộm)
 */
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
/**
 * @brief Tạo JSON chỉ chứa thông tin dung lượng Pin
 */
void sim_public_battery_mqtt(uint8_t battery){
    json_gen_str_t jstr;
    json_gen_str_start(&jstr, sim_public_mqtt_msg, BUF_SIZE, NULL, NULL);
    json_gen_start_object(&jstr);
    json_gen_obj_set_int(&jstr, "battery", battery);
    json_gen_end_object(&jstr);
    json_gen_str_end(&jstr);
    printf("Generated JSON: %s\n", sim_public_mqtt_msg);
}

/* -----------------------------------------------------------
 * 3. HÀM KIỂM TRA ĐIỀU KIỆN (SENSOR & LOGIC CHECK)
 * ----------------------------------------------------------- */
bool is_low_battery() {
    if(battery_capacity < 20){
        return true;
    }
    return false; 
}
bool accident_detected() {
    return false; 
}
bool is_theft_detected() {
    if(alarm_state == STATE_ALARM   && antiTheft  ){
        printf("alarm_state=%d\n",alarm_state);
        return true;
    }
    return false; 
}

/* -----------------------------------------------------------
 * 4. CÁC LUỒNG XỬ LÝ (TASKS)
 * ----------------------------------------------------------- */

/**
 * @brief Đọc dữ liệu thô từ UART GPS và lọc chuỗi NMEA
 */
void gps_rx_task(void *arg){
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE - 1, 50 / portTICK_PERIOD_MS);
        // printf("GPS RX: %s\n", data);
        for(uint8_t i=0; i<len; i++){
            if(data[i] != '\n' && gps_index < BUF_SIZE - 1){
                gps_buffer[gps_index++] = data[i];
            }
            else{
                gps_buffer[gps_index] = '\0'; 
                if(!strncmp(gps_buffer,"$GPRMC",6) ){
                    // printf("Received NMEA: %s\n", gps_buffer);
                    strncpy(latest_nmea, gps_buffer, BUF_SIZE);
                }
                gps_index = 0;
            }
        }
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
}

/**
 * @brief Phân tích chuỗi NMEA và in tọa độ lên Serial
 */
void gps_process_task(void *arg){
    while(1){
        vTaskDelay(5000/portTICK_PERIOD_MS);
        if(!gps_active){
            latest_nmea[0]='\0';
            GPS.dec_latitude=0.0;
            GPS.dec_longitude=0.0;
        }
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

/**
 * @brief Đọc dữ liệu từ cảm biến MPU6050 (Gia tốc & Con quay hồi chuyển)
 */
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
        // printf("Accel: X=%0.2f m/s^2, Y=%0.2f m/s^2, Z=%0.2f m/s^2\n", accel_x_g, accel_y_g, accel_z_g);
        // printf("Gyro: X=%0.2f deg/s, Y=%0.2f deg/s, Z=%0.2f deg/s\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);
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
        // printf("Accel: X=%0.2f m/s^2, Y=%0.2f m/s^2, Z=%0.2f m/s^2\n", accel_x_g, accel_y_g, accel_z_g);
        // printf("Gyro: X=%0.2f deg/s, Y=%0.2f deg/s, Z=%0.2f deg/s\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);
        // printf("Acceleration Change: %0.2f m/s^2\n", accChange);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

/**
 * @brief Đọc giá trị Pin qua ADC và gửi dữ liệu lên MQTT định kỳ
 */
void read_battery_task(void *pb){
    while(1){
        static uint16_t read_pin_voltage_value;
        read_pin_voltage_value=get_value_adc(ADC_CHANNEL_4);
        float voltage = (read_pin_voltage_value / 4095.0) * 3.3 * 5;
        // Convert voltage to battery x3 18650 3.7V
        battery_capacity = (uint8_t) (( voltage / 11) * 100);
        if(battery_capacity > 100) battery_capacity = 100;
        printf("Battery Voltage: %.2f V, Battery Level: %d%%\n", voltage, battery_capacity);
        sim_public_battery_mqtt(battery_capacity);
        mqtt_publish("esp32/battery", sim_public_mqtt_msg, 1000);
        vTaskDelay(70000/portTICK_PERIOD_MS); 
    }
}
/**
 * @brief Cập nhật trạng thái thiết bị dựa trên các điều kiện cảm biến
 */
void Task_StateUpdate(void *pvParameters) {
    device_state_t new_state;
    for (;;) {
        if (accident_detected()) 
            new_state = SOS_STATE;
        else if (is_theft_detected())
            new_state = ALERT_STATE;
        else if (is_low_battery()) 
            new_state = LOW_BATTERY_STATE;
        else 
            new_state = NORMAL_STATE;
        device_state = new_state;
        if (xQueueSend(eventQueue, &new_state, 0) != pdPASS) {
            ESP_LOGW(TAG, "Queue full, state not sent!");
        }  
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
/**
 * @brief Thực thi hành động (Còi, SMS) dựa trên trạng thái nhận được từ Queue
 */
void Task_ActionHandler(void *pvParameters) {
    device_state_t state;
    static bool sos_sent = false, alert_sent = false,alert_low_pin=false;
    static TickType_t last_sms_time = 0;
    for (;;) {
        if (xQueueReceive(eventQueue, &state, portMAX_DELAY)) {
            switch (state) {
                case ALERT_STATE:
                    // start_alarm(); 
                    buzzer_on_alarm();
                    sim_send_alert_sms("Canh bao mat trom", &alert_sent, &last_sms_time);
                    sos_sent = false;
                    // printf("Device in ALERT_STATE\n");
                    break;
                case SOS_STATE: 
                    // send_sos_message(); 
                    sim_send_alert_sms("Canh bao tai nan", &sos_sent, &last_sms_time);
                    alert_sent = false;
                    break;
                case LOW_BATTERY_STATE: 
                    // send_low_battery_warning(); 
                    // printf("Device in LOW_BATTERY_STATE\n");
                    buzzer_off();
                    alert_sent = false;
                    sos_sent = false;
                    sim_send_battery_sms( battery_capacity, &alert_low_pin, &last_sms_time);
                    break;
                case NORMAL_STATE: 
                    buzzer_off();
                    alert_sent = false;
                    sos_sent = false;
                    // printf("Device in NORMAL_STATE\n");
                    break;
            }
        }
    }
}
/**
 * @brief Task xử lý các tin nhắn MQTT liên quan đến cập nhật vị trí và cảnh báo
 */
void Task_Action_MqttMessage(void *arg) {
    for (;;) {
        if(updateLocation){
            static char url[128]; 
            while(!GPS.dec_latitude && !GPS.dec_longitude){
                ESP_LOGI("GPS","Waiting for valid GPS data...");
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            snprintf(url, sizeof(url), "Vi tri hien tai: https://www.google.com/maps?q=%f,%f", GPS.dec_latitude, GPS.dec_longitude);
            for(int i=0;i<phone_count;i++){
                sim_send_sms(phone_list[i], url, 500);
            }
        }
        if(warning){
            alarm_state= STATE_IDLE;
            warning=false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/* -----------------------------------------------------------
 * 5. HÀM KHỞI TẠO CHÍNH (APP_MAIN)
 * ----------------------------------------------------------- */
void app_main() {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // Create event queue
    eventQueue = xQueueCreate(10, sizeof(device_state_t));
    
    // Init Module SIM
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
    // Init Buzzer
    ret=buzzer_init(23);
    if (ret != ESP_OK) {
        ESP_LOGE("BUZZER", "Initialization failed");
        return;
    }
    // Init ADC for battery reading
    ret=adc_init(ADC_CHANNEL_4);
    if (ret != ESP_OK) {
        ESP_LOGE("ADC", "Initialization failed");
        return;
    }
    input_io_create(GPIO_NUM_4, LO_TO_HI);

    // // Initialize MPU6050
    ret = mpu6050_init(I2C_NUM_0);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Initialization failed");
        return;
    }
    mpu6050_calibrate(I2C_NUM_0, accel_bias, gyro_bias);
    // C. Khởi tạo kết nối mạng (WiFi, AP, MQTT)

    // if (wifi_start()) {
    //     ESP_LOGI(TAG, "WiFi OK → normal run, wait OTA command");

    // } else {
    //     ESP_LOGI(TAG, "WiFi FAIL → start AP");
    //     esp_ap_start();
    // }
    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 4, NULL);
    esp_ap_start();
    start_webserver();
    // sim_public_mqtt();

    // D. Tạo các Task FreeRTOS
    xTaskCreate(gps_rx_task, "gps_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(gps_process_task,"gps_process_task",2048,NULL,6,NULL);
    mqtt_connect("esp32_client", "broker.hivemq.com", 1000);
    mqtt_subscribe("esp32/device", 0, 1000);
    mqtt_subscribe("esp32/updateOTA", 0, 1000);
    xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 7, NULL);
    
    xTaskCreate(Task_StateUpdate, "StateUpdate", 2048, NULL, 7, NULL);
    xTaskCreate(Task_ActionHandler, "ActionHandler", 4096, NULL, 8, NULL);
    xTaskCreate(Task_Action_MqttMessage, "MqttMessage", 4096, NULL, 9, NULL);
    xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 10, NULL);
    xTaskCreate(gps_power_manager_task, "gps_power_manager_task", 2048, NULL, 11, NULL);
    xTaskCreate(read_battery_task, "read_battery_task", 2048, NULL, 12, NULL);
}