#include "power_control.h"
static const char *POWER_CONTROL_TAG = "power control";

esp_err_t SysRunningLedInit()
{
    esp_log_level_set(POWER_CONTROL_TAG, POWER_CONTROL_LOG);
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = SYSRUN_LED_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    esp_err_t err = gpio_config(&io_conf);

    ESP_LOGI(POWER_CONTROL_TAG, "%s", (err == ESP_OK) ? "system running instructor led initialized successfully" : "system running instructor led initialized fail");
    return err;
}

void SysRunningLedTask()
{
    for (int cnt = 0;; cnt++)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_IO_LED_SYSRUN, cnt % 2);
        ESP_LOGI(POWER_CONTROL_TAG, "System running indicator LED(G%d) status toggle", GPIO_OUTPUT_IO_LED_SYSRUN);
    } // end while

    // Never reach here
    vTaskDelete(NULL);
}