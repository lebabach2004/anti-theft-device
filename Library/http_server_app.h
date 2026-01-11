#ifndef __HTTP_SERVER_APP_H
#define __HTTP_SERVER_APP_H
#include "esp_err.h"
#include <esp_http_server.h>
void start_webserver(void);
void stop_webserver(void);
#endif