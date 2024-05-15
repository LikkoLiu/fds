#ifndef __MAIN_H_
#define	__MAIN_H_

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_pm.h"

#include "power_control.h"
#include "uart_gps.h"
#include "i2c_imu.h"
#include "i2c_tof.h"
#include "bt.h"
#include "algorithm.h"

#define MAIN_LOG ESP_LOG_INFO
#define MAIN_TAG "app_main"

#endif
