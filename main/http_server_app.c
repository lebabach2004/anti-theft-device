#include "http_server_app.h"
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
// #include "esp_tls_crypto.h"
#include "esp_event.h"
#include "esp_netif.h"
// #include "esp_tls.h"
#include "esp_check.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include <ctype.h>
#define EXAMPLE_HTTP_QUERY_KEY_MAX_LEN  (64)
static httpd_handle_t server = NULL;
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
static const char *TAG = "HTPP_SERVER_APP";

static char *phone_list[10];
static int phone_count = 0;

static int is_valid_phone(const char* number){
    int len = strlen(number);
    if(len < 10 || len > 11) return 0; 
    for(int i=0; i<len; i++){
        if(!isdigit((unsigned char)number[i])) return 0; 
    }
    return 1; 
}
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/interface", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } 
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

static esp_err_t get_server_interface_handler(httpd_req_t *req)
{
    // const char* resp_str = (const char*) "Hello world";
    // httpd_resp_send(req, resp_str, strlen(resp_str));
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req,(const char*)index_html_start,index_html_end-index_html_start);
    return ESP_OK;
}
static const httpd_uri_t get_server_interface = {
    .uri       = "/interface",
    .method    = HTTP_GET,
    .handler   = get_server_interface_handler,
    .user_ctx  = "NULL"
};
static char* url_decode_param(const char *body){
    const char *p = strstr(body, "number=");
    if(!p) return NULL;
    p += strlen("number=");
    return strdup(p);
}
static int add_number(const char* number){
    if(phone_count >= 10) return -1;
    phone_list[phone_count] = strdup(number);
    phone_count++;
    return 0;
}
static esp_err_t add_phonenumber_handler(httpd_req_t *req){
    int content_len = req->content_len;
    if(content_len <=0 || content_len>128){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad POST length");
        return ESP_FAIL;
    }
    char buf[129];
    int ret = httpd_req_recv(req, buf, content_len);
    if(ret <=0){
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
        return ESP_FAIL;
    }
    buf[ret]=0;
    printf("Received body: %s\n", buf);
    char *number = url_decode_param(buf);
    if(!number){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing number");
        return ESP_FAIL;
    }
    if(!is_valid_phone(number)){
        free(number);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid phone number format");
        return ESP_FAIL;
    }
    if(add_number(number)!=0){
        free(number);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Add failed");
        return ESP_FAIL;
    }
    free(number);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}
static const httpd_uri_t  add_phonenumber= {
    .uri="/add",
    .method=HTTP_POST,
    .handler=add_phonenumber_handler,
    .user_ctx=NULL
};
static char* get_list_csv(void) {
    static char buffer[1024];
    buffer[0] = 0;
    for(int i=0;i<phone_count;i++){
        strcat(buffer, phone_list[i]);
        if(i<phone_count-1) strcat(buffer,",");
    }
    return buffer;
}
static esp_err_t list_phonenumber_handler(httpd_req_t *req){
    char *list = get_list_csv();
    printf("List: %s\n", list);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, list);
    return ESP_OK;
}
static const httpd_uri_t  list_phonenumber= {
    .uri="/list",
    .method=HTTP_GET,
    .handler=list_phonenumber_handler,
    .user_ctx=NULL
};

static int remove_number(const char* number){
    for(int i=0;i<phone_count;i++){
        if(strcmp(phone_list[i], number)==0){
            free(phone_list[i]);
            for(int j=i;j<phone_count-1;j++)
                phone_list[j] = phone_list[j+1];
            phone_count--;
            return 0;
        }
    }
    return -1;
}
esp_err_t remove_phonenumber_handler(httpd_req_t *req){
    int content_len = req->content_len;
    if(content_len <=0 || content_len>128){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad POST length");
        return ESP_FAIL;
    }
    char buf[129];
    int ret = httpd_req_recv(req, buf, content_len);
    if(ret <=0){
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
        return ESP_FAIL;
    }
    buf[ret]=0;
    char *number = url_decode_param(buf);
    if(!number){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing number");
        return ESP_FAIL;
    }
    if(remove_number(number)!=0){
        free(number);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Remove failed");
        return ESP_FAIL;
    }
    free(number);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}
static const httpd_uri_t remove_phonenumber = {
    .uri="/remove",
    .method=HTTP_POST,
    .handler=remove_phonenumber_handler,
    .user_ctx=NULL
};
void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server,&get_server_interface);
        httpd_register_uri_handler(server, &add_phonenumber);
        httpd_register_uri_handler(server, &list_phonenumber);
        httpd_register_uri_handler(server, &remove_phonenumber);
        httpd_register_err_handler(server,HTTPD_404_NOT_FOUND,http_404_error_handler);
        //httpd_register_uri_handler(server, &echo);
    }
    else{
        ESP_LOGI(TAG, "Error starting server!");
    }
}
void stop_webserver(void)
{
    // Stop the httpd server
    httpd_stop(server);
}