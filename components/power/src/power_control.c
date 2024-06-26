#include "power_control.h"
static const char *POWER_CONTROL_TAG = "power-control";

esp_err_t SysRunningLedInit()
{
    esp_log_level_set(POWER_CONTROL_TAG, POWER_CONTROL_LOG);

    gpio_config_t io_conf_led = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = SYSRUN_LED_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    esp_err_t err = gpio_config(&io_conf_led);

    gpio_config_t io_conf_gps_power = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPS_POWER_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    err = gpio_config(&io_conf_gps_power);
    gpio_set_level(GPIO_OUTPUT_IO_GPS_POWER, 0);

    // gpio_config_t io_conf_tof_power = {
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pin_bit_mask = TOF_POWER_PIN_SEL,
    //     .pull_down_en = 0,
    //     .pull_up_en = 0,
    // };
    // err = gpio_config(&io_conf_tof_power);
    // gpio_set_level(GPIO_OUTPUT_IO_TOF_POWER, 0);

    ESP_LOGI(POWER_CONTROL_TAG, "system running instructor led(G%d) initialized %s\r\n", GPIO_OUTPUT_IO_LED_SYSRUN, (err == ESP_OK) ? "successfully" : "fail");
    return err;
}

void SysRunningLedTask()
{
    for (uint8_t cnt = 0;; cnt++)
    {
        cnt = cnt % 2; /* 防止变量过大 */
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_IO_LED_SYSRUN, cnt);
    } // end while

    // Never reach here
    vTaskDelete(NULL);
}