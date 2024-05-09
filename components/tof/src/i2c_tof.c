#include "i2c_tof.h"

#define I2C_TOF_TAG "i2c-tof"

static const I2cDef I2CConfig = {
    .i2cPort = I2C_NUM_1,
    .i2cClockSpeed = 400000,
    .gpioSCLPin = 1,
    .gpioSDAPin = 3,
    .gpioPullup = GPIO_PULLUP_ENABLE,
};
static I2cDrv i2cBus = {
    .def = &I2CConfig,
};
static VL53L1_Dev_t dev;

/**
 * 设置VL53L0动态地址后8步的起始地址
 * 静态int nextI2CAddress = VL53L1X_DEFAULT_ADDRESS+8;
 */
static esp_err_t vl53l1xInit(VL53L1_Dev_t *pdev, I2C_Dev *I2cHandle)
{
    esp_err_t status = VL53L1_ERROR_NONE;

    pdev->I2cDevAddr = VL53L1X_DEFAULT_ADDRESS;
    pdev->I2Cx = I2cHandle;

    uint8_t byteData;
    uint16_t wordData;
    VL53L1_RdByte(pdev, 0x010F, &byteData);
    ESP_LOGI(I2C_TOF_TAG, "VL53L3CX Model_ID = 0x%02x", byteData);
    VL53L1_RdByte(pdev, 0x0110, &byteData);
    ESP_LOGI(I2C_TOF_TAG, "VL53L3CX Module_Type = 0x%02x", byteData);
    VL53L1_RdWord(pdev, 0x010F, &wordData);
    ESP_LOGI(I2C_TOF_TAG, "VL53L3CX = 0x%02x", wordData);

    // status = VL53L1_WaitDeviceBooted(pdev);
    if (status != VL53L1_ERROR_NONE)
    {
        ESP_LOGE(I2C_TOF_TAG, "VL53L3CX wait device booted fail");
        return status;
    }

    status = VL53L1_DataInit(pdev);
    if (status != VL53L1_ERROR_NONE)
    {
        ESP_LOGE(I2C_TOF_TAG, "VL53L3CX data init fail");
        return status;
    }

    status = VL53L1_StaticInit(pdev);
    if (status != VL53L1_ERROR_NONE)
    {
        ESP_LOGE(I2C_TOF_TAG, "VL53L3CX static init fail");
        return status;
    }

    return status;
}

esp_err_t I2cTofInit(void)
{
    esp_err_t err = ESP_OK;
    esp_log_level_set(I2C_TOF_TAG, I2C_TOF_LOG);

    i2ctofdevInit(&i2cBus);

    err = vl53l1xInit(&dev, &i2cBus);
    ESP_LOGI(I2C_TOF_TAG, "tof initialization %s \r\n", (err == ESP_OK) ? "successfully" : "fail");

    return err;
}

void TofTask(void *pvParameters)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    VL53L1_StopMeasurement(&dev);
    VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_LONG);
    VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);

    for (;;)
    {
        VL53L1_StartMeasurement(&dev);

        while (dataReady == 0)
        {
            status = VL53L1_GetMeasurementDataReady(&dev, &dataReady);
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
        range = rangingData.RangeMilliMeter;

        VL53L1_StopMeasurement(&dev);
        VL53L1_StartMeasurement(&dev);

        ESP_LOGI(I2C_TOF_TAG, "VL53L3CX Distance: %4dmm", range);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    } // end while

    // Never reach here
    vTaskDelete(NULL);
}