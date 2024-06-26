#ifndef __I2C_TOF_H_
#define	__I2C_TOF_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "i2cdev.h"
#include "vl53l1x.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "power_control.h"
#include "algorithm.h"

#define I2C_TOF_LOG ESP_LOG_INFO
#define RANGE_THRESHOLD 500
#define GET_RANGE_TIME 1000

extern TaskHandle_t xTofHandle;
extern eTaskState xAlgorithmTaskSt;

esp_err_t I2cTofInit(void);
void TofTask(void *pvParameters);

#endif
