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
#include "json_generator.h"
#include <json_parser.h>
#include "nvs_flash.h"

#define NVS_NAMESPACE "phonebook"
#define NVS_KEY "phones_json"

#define EXAMPLE_HTTP_QUERY_KEY_MAX_LEN  (64)
static httpd_handle_t server = NULL;
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
static const char *TAG = "HTPP_SERVER_APP";

char *phone_list[10];
int phone_count = 0;

static int is_valid_phone(const char* number){
    int len = strlen(number);
    if(len < 10 || len > 11) return 0; 
    for(int i=0; i<len; i++){
        if(!isdigit((unsigned char)number[i])) return 0; 
    }
    return 1; 
}
static int generate_phone_list_json(char *buffer, size_t buf_size) {
    json_gen_str_t jstr;
    json_gen_str_start(&jstr, buffer, buf_size, NULL, NULL);
    json_gen_start_object(&jstr);
    json_gen_push_array(&jstr, "phones");
    for (int i = 0; i < phone_count; i++) {
        json_gen_start_object(&jstr);
        json_gen_obj_set_int(&jstr, "id", i + 1);
        json_gen_obj_set_string(&jstr, "number", phone_list[i]);
        json_gen_end_object(&jstr);
    }
    json_gen_pop_array(&jstr);
    json_gen_end_object(&jstr);
    return json_gen_str_end(&jstr); 
}
static esp_err_t parse_number_from_req( char *buf, int ret, char *number_buf, int buf_size){
    jparse_ctx_t jctx;
    if (json_parse_start(&jctx, buf, ret) != OS_SUCCESS) {
        return ESP_FAIL; 
    }
     if (json_obj_get_string(&jctx, "number", number_buf, buf_size) != OS_SUCCESS) {
        json_parse_end(&jctx);
        return ESP_FAIL; // Missing number
    }
    json_parse_end(&jctx);
    return ESP_OK;
}
static esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
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
static esp_err_t load_phone_list_from_flash(void){
    nvs_handle_t handle;
    if(nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return ESP_FAIL;
    size_t sz = 0;
    if(nvs_get_str(handle, NVS_KEY, NULL, &sz) != ESP_OK || sz == 0){
        nvs_close(handle);
        return ESP_FAIL;
    }
    char *buf = malloc(sz);
    if(!buf){ 
        nvs_close(handle); 
        return ESP_ERR_NO_MEM; 
    }
    if(nvs_get_str(handle, NVS_KEY, buf, &sz) != ESP_OK){
        free(buf); 
        nvs_close(handle); 
        return ESP_FAIL;
    }
    nvs_close(handle);
    jparse_ctx_t jctx;
    if (json_parse_start(&jctx, buf, sz) != OS_SUCCESS) {
        free(buf);
        return ESP_FAIL; 
    }
    phone_count = 0;
    int array_len = 0;
    if (json_obj_get_array(&jctx, "phones", &array_len) == OS_SUCCESS) {
        for (int i = 0; i < array_len && i < 10; i++) {
            if (json_arr_get_object(&jctx, i) == OS_SUCCESS) {   
                char number[32];
                if (json_obj_get_string(&jctx, "number", number, sizeof(number)) == OS_SUCCESS) {
                    phone_list[phone_count++] = strdup(number);
                }
                json_arr_leave_object(&jctx); 
            }
        }
    }
    json_parse_end(&jctx);
    free(buf);
    return ESP_OK;
}
static esp_err_t save_phone_list_to_flash(void){
    char buffer[1024];
    int len = generate_phone_list_json(buffer, sizeof(buffer));
    if(len <=0) return ESP_FAIL;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if(err != ESP_OK) return err;
    err = nvs_set_str(handle, NVS_KEY, buffer);
    if(err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
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
    char number[32];
    if (parse_number_from_req(buf, ret, number, sizeof(number)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid number");
        return ESP_FAIL;
    }
    if(!is_valid_phone(number)){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid phone number format");
        return ESP_FAIL;
    }
    if(add_number(number)!=0){
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Add failed");
        return ESP_FAIL;
    }
    save_phone_list_to_flash();
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}
static const httpd_uri_t  add_phonenumber= {
    .uri="/add",
    .method=HTTP_POST,
    .handler=add_phonenumber_handler,
    .user_ctx=NULL
};
static esp_err_t list_phonenumber_handler(httpd_req_t *req){
    char buffer[1024];
    int len = generate_phone_list_json(buffer, sizeof(buffer));
    printf("Generated JSON: %s\n\n", buffer);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buffer, len-1); 
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
static esp_err_t remove_phonenumber_handler(httpd_req_t *req){
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
    char number[32];
    if (parse_number_from_req(buf, ret, number, sizeof(number)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid number");
        return ESP_FAIL;
    }
    if(remove_number(number)!=0){
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Remove failed");
        return ESP_FAIL;
    }
    save_phone_list_to_flash();
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
    load_phone_list_from_flash(); 
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