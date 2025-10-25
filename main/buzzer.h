#ifndef __BUZZER_H__
#define __BUZZER_H__
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
esp_err_t buzzer_init(uint8_t gpio_num);
void buzzer_off(void);
void buzzer_on_alarm(void);
void buzzer_task(void *arg);
#endif