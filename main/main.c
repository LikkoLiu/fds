#include "main.h"

void app_main(void)
{
    esp_log_level_set(MAIN_TAG, MAIN_LOG);

    ESP_ERROR_CHECK(SysRunningLedInit()); /* 初始化 power 控制引脚状态 */
    ESP_ERROR_CHECK(I2cImuInit());
    ESP_ERROR_CHECK(I2cTofInit());

    ESP_ERROR_CHECK(vBtInit());
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    xTaskCreatePinnedToCore(&AlgorithmTask, "Algorithm", 1024 * 56, NULL, 5, &xAlgorithmHandle, 1); /* 堆栈大于 48 时，初始化失败 */
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    xTaskCreatePinnedToCore(&SysRunningLedTask, "SysRunningLedTask", 1024 * 4, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(&ImuTask, "IMU", 1024 * 15, NULL, 4, &xImuHandle, 1);
    xTaskCreatePinnedToCore(&TofTask, "TOF", 1024 * 12, NULL, 3, &xTofHandle, 1);

    do
    {
        xAlgorithmTaskSt = eTaskGetState(xTaskGetHandle("Algorithm"));
        ESP_LOGW(MAIN_TAG, "wait Algorithm task init...");
        vTaskDelay(200 / portTICK_PERIOD_MS);
    } while (eInvalid == eTaskGetState(xTaskGetHandle("Algorithm")));
}
