#include "power_control.h"

void SysRunningLedInit()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = SYSRUN_LED_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

void SysRunningLedTask(void *arg)
{
    for (int cnt = 0;; cnt++)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_IO_LED_SYSRUN, cnt % 2);
    }
    vTaskDelete(NULL);
}