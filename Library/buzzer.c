#include "buzzer.h"
static bool buzzer_running = false;
#define PWM_FREQ_HZ     2500                // Tần số PWM: 2.5 kHz
#define PWM_DUTY_RES    LEDC_TIMER_13_BIT   
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_CHANNEL     LEDC_CHANNEL_0
#define PWM_MODE        LEDC_HIGH_SPEED_MODE
#define FREQ_MIN        500
#define FREQ_MAX        1500
#define STEP_FREQ       50
#define STEP_DELAY_MS   20
esp_err_t buzzer_init(uint8_t gpio_num){
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_DUTY_RES,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if(err != ESP_OK){
        return err;
    }
    ledc_channel_config_t ledc_channel = {
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = gpio_num,
        .duty       = 0,    
        .hpoint     = 0
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) return err;
    ESP_LOGI("BUZZER","Initialized Buzzer on GPIO %d",gpio_num);
    return ESP_OK;
}
void buzzer_off(void){
    buzzer_running=false;
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
    ledc_set_freq(PWM_MODE, PWM_CHANNEL, 20);
}
void buzzer_on_alarm(void){
    buzzer_running=true;
}
void buzzer_task(void *arg){
    while(1){
        if(buzzer_running){
            for(uint16_t freq=FREQ_MIN;freq<=FREQ_MAX;freq+=STEP_FREQ){
                ledc_set_freq(PWM_MODE, PWM_CHANNEL, freq);
                ledc_set_duty(PWM_MODE, PWM_CHANNEL, 1 << 12);
                ledc_update_duty(PWM_MODE, PWM_CHANNEL);
                if(!buzzer_running) break;
                vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
            }
            // for(uint16_t freq=FREQ_MAX;freq>=FREQ_MIN;freq-=STEP_FREQ){
            //     ledc_set_freq(PWM_MODE, PWM_CHANNEL, freq);
            //     ledc_set_duty(PWM_MODE, PWM_CHANNEL, 1 << 12);
            //     ledc_update_duty(PWM_MODE, PWM_CHANNEL);
            //     vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
            // }
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}