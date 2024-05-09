#ifndef __POWER_CONTROL_H_
#define __POWER_CONTROL_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define GPIO_OUTPUT_IO_LED_SYSRUN CONFIG_GPIO_LED_SYSRUN
#define SYSRUN_LED_PIN_SEL (1ULL << GPIO_OUTPUT_IO_LED_SYSRUN)
#define GPIO_OUTPUT_IO_GPS_POWER CONFIG_GPIO_GPS_POWER
#define GPS_POWER_PIN_SEL (1ULL << GPIO_OUTPUT_IO_GPS_POWER)

#define POWER_CONTROL_LOG ESP_LOG_INFO

esp_err_t SysRunningLedInit();
void SysRunningLedTask();

#endif