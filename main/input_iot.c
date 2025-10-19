#include "stdio.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "input_iot.h"
#include "esp_timer.h"
#define CONFIRM_WINDOW_MS 500
#define ALARM_DURATION_MS 30000
#define TAG "Vibrate Sensor"
volatile alarm_state_t alarm_state = STATE_IDLE;
static volatile int pulse_count = 0;
input_callback_t input_callback=NULL;
static esp_timer_handle_t one_timer;
static void one_timer_cb(void* arg) {
    ESP_EARLY_LOGI("ISR", "Alarm State_1:%d",alarm_state);
    if (alarm_state == STATE_WAIT_CONFIRM) {
        printf("Pulse count: %d\n", pulse_count);
        if(pulse_count>=3){
            alarm_state = STATE_ALARM;
            ESP_LOGW(TAG, ">>> ALARM ON <<<");
            esp_timer_start_once(one_timer, ALARM_DURATION_MS * 1000);
        }
        else{
            alarm_state = STATE_IDLE;
        }
        pulse_count = 0;
    } else if (alarm_state == STATE_ALARM) {
        alarm_state = STATE_IDLE;
        pulse_count = 0;
    }

}
static void IRAM_ATTR gpio_input_handler(void* arg){
    // int gpio_num=(uint32_t) arg;
    // input_callback(gpio_num);
    static uint64_t now=0;
    uint64_t current=esp_timer_get_time();
    if(current-now< 20*1000 ){
        return;
    }
    now = current; 
    pulse_count++;
    ESP_EARLY_LOGI("ISR", "Alarm State_0:%d",alarm_state);

    if(alarm_state==STATE_IDLE){
        alarm_state = STATE_WAIT_CONFIRM;
        // Stop timer previous
        if (esp_timer_is_active(one_timer)) {
            esp_timer_stop(one_timer);
        }
        esp_timer_start_once(one_timer, CONFIRM_WINDOW_MS * 1000);
    }
}
void input_io_create(gpio_num_t gpio_num, interrupt_type_edge_t type){
    gpio_set_direction(gpio_num,GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio_num, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(gpio_num,type);
    esp_timer_create_args_t timer_args = {
        .callback = &one_timer_cb,
        .name = "one_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &one_timer));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio_num,gpio_input_handler,(void*)gpio_num);
}
int input_io_get_level(gpio_num_t gpio_num){
    return gpio_get_level(gpio_num);
}
void input_set_callback(void* cb){
    input_callback=cb;
}
