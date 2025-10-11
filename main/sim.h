#ifndef __SIM_H__
#define __SIM_H__
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include <driver/uart.h>
#include <driver/gpio.h>
#include "json_parser.h"
esp_err_t SIM_init(void);
void mqtt_connect(const char *client_id, const char *broker_url,uint32_t timeout);
void mqtt_subscribe(const char *topic, uint8_t qos, uint32_t timeout);
void mqtt_publish(const char *topic, const char *message, uint32_t timeout);
void sim_send_sms(const char *phone_number, const char *message, uint32_t timeout);
#endif
