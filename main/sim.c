#include "sim.h"
#define BUF_SIZE 1024
#define UART_SIM UART_NUM_2
#define MODULE_SIM_Tx 17
#define MODULE_SIM_Rx 16
static const char *TAG = "SIM";
char deviceId[32];
bool updateLocation = false;
bool antiTheft = false ;
bool warning = false;
bool update_OTA = false ;
esp_err_t SIM_init(void){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_SIM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_SIM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_SIM, MODULE_SIM_Tx, MODULE_SIM_Rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for module to stabilize
    return ESP_OK;
}
void handle_mqtt_rx(char *rx_data){
    char topic[64] = {0};
    char *topic_pos = strstr(rx_data, "+CMQTTRXTOPIC:");
    if(topic_pos){
        char *newline = strchr(topic_pos, '\n');
        if(newline){
            newline = strchr(newline + 1, '\n');
            if(newline){
                newline++; 
                sscanf(newline, "%63[^\r\n]", topic);
                printf("Received message on topic: %s\n", topic);
            }
            else{
                printf("No second newline found after +CMQTTRXTOPIC.\n");
                return;
            }
        }
        else{
            printf("No topic found in received data.\n");
            return;
        }
    }
    if(strcmp(topic, "esp32/device") == 0){
        char *start = strchr(rx_data, '{');
        char *end = strrchr(rx_data, '}');
        if (!(start && end && end > start)) {
            ESP_LOGE(TAG, "Invalid JSON format");
            return;
        }
        int json_len = end - start + 1;
        char json_str[256];
        strncpy(json_str, start, json_len);
        json_str[json_len] = '\0';
        printf("Extracted JSON: %s\n", json_str);
        jparse_ctx_t jctx;
        if (json_parse_start(&jctx, json_str, sizeof(json_str)) != OS_SUCCESS) {
            ESP_LOGE(TAG, "JSON parse start failed");
            return ; 
        }
        if (json_obj_get_string(&jctx, "deviceId", deviceId, sizeof(deviceId)) == OS_SUCCESS)
            ESP_LOGI(TAG, "deviceId: %s", deviceId);
        if (json_obj_get_bool(&jctx, "updateLocation", &updateLocation) == OS_SUCCESS)
            ESP_LOGI(TAG, "updateLocation: %s", updateLocation ? "true" : "false");
        if (json_obj_get_bool(&jctx, "antiTheft", &antiTheft) == OS_SUCCESS)
            ESP_LOGI(TAG, "antiTheft: %s", antiTheft ? "true" : "false");
        if (json_obj_get_bool(&jctx, "warning", &warning) == OS_SUCCESS)
            ESP_LOGI(TAG, "warning: %s", warning ? "true" : "false");
    }
    if(strcmp(topic, "esp32/updateOTA") == 0){
        char *start = strchr(rx_data, '{');
        char *end = strrchr(rx_data, '}');
        if (!(start && end && end > start)) {
            ESP_LOGE(TAG, "Invalid JSON format");
            return;
        }
        int json_len = end - start + 1;
        char json_str[256];
        strncpy(json_str, start, json_len);
        json_str[json_len] = '\0';
        printf("Extracted JSON: %s\n", json_str);
        jparse_ctx_t jctx;
        if (json_parse_start(&jctx, json_str, sizeof(json_str)) != OS_SUCCESS) {
            ESP_LOGE(TAG, "JSON parse start failed");
            return ; 
        }
        if(json_obj_get_bool(&jctx, "updateOTA", &update_OTA) == OS_SUCCESS)
            ESP_LOGI(TAG, "updateOTA: %s", update_OTA ? "true" : "false");
    }
    
}
static void send_at(const char *cmd, uint32_t timeout){
    uart_write_bytes(UART_SIM, cmd, strlen(cmd));
    uart_write_bytes(UART_SIM, "\r\n", 2);
    ESP_LOGI(TAG, ">> %s", cmd);
    vTaskDelay(pdMS_TO_TICKS(timeout));
}
void mqtt_task(void *arg) {
    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_SIM, data, BUF_SIZE - 1, 150 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = 0;
            ESP_LOGI(TAG, "<< %s", (char *)data);
            handle_mqtt_rx((char *)data);
        }
    }
}
void mqtt_connect(const char *client_id, const char *broker_url,uint32_t timeout) {
    send_at("AT+CMQTTSTART",timeout);
    
    char cmd[128];
    sprintf(cmd, "AT+CMQTTACCQ=0,\"%s\",0", client_id);
    send_at(cmd,timeout);

    sprintf(cmd, "AT+CMQTTCONNECT=0,\"tcp://%s:1883\",120,1", broker_url);
    send_at(cmd,timeout);

    ESP_LOGI(TAG, "MQTT connected to %s as %s", broker_url, client_id);
}
void mqtt_subscribe(const char *topic, uint8_t qos, uint32_t timeout) {
    char cmd[64];
    sprintf(cmd, "AT+CMQTTSUBTOPIC=0,%d,%d", strlen(topic), qos);
    send_at(cmd,timeout);
    send_at(topic,timeout);
    send_at("AT+CMQTTSUB=0,1",timeout);
    ESP_LOGI(TAG, "Subscribed to topic: %s", topic);
}
void mqtt_publish(const char *topic, const char *message, uint32_t timeout) {
    char cmd[64];
    sprintf(cmd, "AT+CMQTTTOPIC=0,%d", strlen(topic));
    send_at(cmd,timeout);
    send_at(topic,timeout);

    sprintf(cmd, "AT+CMQTTPAYLOAD=0,%d", strlen(message));
    send_at(cmd,timeout);
    send_at(message,timeout);

    send_at("AT+CMQTTPUB=0,1,60",timeout);
    ESP_LOGI(TAG, "Published message to topic %s: %s", topic, message);
}
void sim_send_sms(const char *phone_number, const char *message, uint32_t timeout) {
    send_at("AT+CMGF=1", timeout);
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", phone_number);
    send_at(cmd, 500);

    // 3. Gửi nội dung tin nhắn
    send_at(message, 500);

    // 4. Kết thúc bằng Ctrl+Z (ASCII 26)
    uint8_t ctrl_z = 26;
    uart_write_bytes(UART_SIM, (const char *)&ctrl_z, 1);
    ESP_LOGI(TAG, "SMS sent to %s: %s", phone_number, message);
}