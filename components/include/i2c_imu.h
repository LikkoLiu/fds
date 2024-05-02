#ifndef __I2C_IMU_H_
#define __I2C_IMU_H_

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_timer.h"

#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "bmi160.h"
#include "bmi160_defs.h"
#include "madgwickFilter.h"

#define I2C_NUM I2C_NUM_0
// #define __DEBUG__ 1

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD 0.0174533

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_IMU_SCL_IO CONFIG_I2C_IMU_SCL               /*!< gpio number for I2C IMU clock */
#define I2C_IMU_SDA_IO CONFIG_I2C_IMU_SDA               /*!< gpio number for I2C IMU data  */
#define I2C_IMU_NUM I2C_NUMBER(CONFIG_I2C_IMU_PORT_NUM) /*!< I2C port number for IMU dev */
#define I2C_IMU_FREQ_HZ CONFIG_I2C_IMU_FREQUENCY        /*!< I2C IMU clock frequency */
#define I2C_IMU_TX_BUF_DISABLE 0                        /*!< I2C IMU doesn't need buffer */
#define I2C_IMU_RX_BUF_DISABLE 0                        /*!< I2C IMU doesn't need buffer */

#define IMU_SENSOR_ADDR CONFIG_IMU_ADDR /*!< slave address for IMU sensor */

#define I2C_IMU_LOG ESP_LOG_INFO

esp_err_t I2cImuInit(void);
void ImuTask(void *pvParameters);

#endif
