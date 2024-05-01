#ifndef __POWER_CONTROL_H_
#define	__POWER_CONTROL_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"


#define GPIO_OUTPUT_IO_LED_SYSRUN    CONFIG_GPIO_LED_SYSRUN
#define SYSRUN_LED_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_LED_SYSRUN)

void SysRunningLedInit();
void SysRunningLedTask(void *arg);

#endif