#include "adc.h"
adc_oneshot_unit_handle_t adc1_handle;
esp_err_t adc_init(adc_channel_t channel){
        //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));
    return ESP_OK;
}
uint16_t get_value_adc(adc_channel_t channel){
    uint16_t value;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel,(int*) &value));
    return value;
}