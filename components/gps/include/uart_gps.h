#ifndef __I2C_GPS_H_
#define	__I2C_GPS_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#define TXD_PIN (GPIO_NUM_11)
#define RXD_PIN (GPIO_NUM_10)

typedef struct {
    char lat[16];
    char lon[16];
    char utc[16];
    char day[8];
    char month[8];
    char year[8];
} gps_data_t;

void GpsTask(void *arg);
esp_err_t uartgpsdevInit(void);

#endif