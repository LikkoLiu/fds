#ifndef __AIGORITHM_H_
#define __AIGORITHM_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include <math.h>

#include "esp_dsp.h"

#define AIGORITHM_TAG "algorithm"
#define N_SAMPLES 2048 // Amount of real input samples

extern TaskHandle_t xAlgorithmHandle;

extern float fArrRoll[N_SAMPLES];
extern float fArrPitch[N_SAMPLES]; 
extern float fArrYaw[N_SAMPLES];
extern uint16_t usPtrArrImu;

void AlgorithmTask(void *pvParameters);

#endif