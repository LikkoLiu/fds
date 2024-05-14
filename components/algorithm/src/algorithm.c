#include "algorithm.h"

portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t xAlgorithmHandle = NULL;

__attribute__((aligned(16))) float fArrRoll[N_SAMPLES + 16] = {0};
__attribute__((aligned(16))) float fArrPitch[N_SAMPLES + 16] = {0};
__attribute__((aligned(16))) float fArrYaw[N_SAMPLES + 16] = {0};
uint16_t usPtrArrImu = 0;

int N = N_SAMPLES;
__attribute__((aligned(16))) float fArrRollFFT[N_SAMPLES + 16] = {0};
__attribute__((aligned(16))) float fArrPitchFFT[N_SAMPLES + 16] = {0};
__attribute__((aligned(16))) float fArrYawFFT[N_SAMPLES + 16] = {0};

__attribute__((aligned(16))) float wind[N_SAMPLES];

void vCopyImuData(const uint16_t usPtr, float *pfArrRoll, float *pfArrPitch, float *pfArrYaw)
{
    uint16_t usCurrentPtr = usPtr;
    for (uint16_t ucCnt = 0; ucCnt < N_SAMPLES; ucCnt++)
    {
        fArrRollFFT[ucCnt] = *(pfArrRoll + usCurrentPtr);
        fArrPitchFFT[ucCnt] = *(pfArrPitch + usCurrentPtr);
        fArrYawFFT[ucCnt] = *(pfArrYaw + usCurrentPtr);

        usCurrentPtr++;
        if (N_SAMPLES == usCurrentPtr)
        {
            usCurrentPtr = 0;
        }
    }
}

void AlgorithmTask(void *pvParameters)
{
    esp_err_t ret;
    esp_log_level_set(AIGORITHM_TAG, AIGORITHM_LOG);

    ESP_LOGD(AIGORITHM_TAG, "Algorithm task begin");
    ret = dsps_fft2r_init_fc32(NULL, N * 2);
    if (ret != ESP_OK)
    {
        ESP_LOGE(AIGORITHM_TAG, "Not possible to initialize FFT2R. Error = %i", ret);
        vTaskDelete(NULL);
    }
    ESP_LOGD(AIGORITHM_TAG, "Algorithm fft2r init ");
    /* 下面这个必须加！ */
    ret = dsps_fft4r_init_fc32(NULL, N * 4);
    if (ret != ESP_OK)
    {
        ESP_LOGE(AIGORITHM_TAG, "Not possible to initialize FFT4R. Error = %i", ret);
        vTaskDelete(NULL);
    }
    ESP_LOGD(AIGORITHM_TAG, "Algorithm fft4r init ");
    dsps_wind_hann_f32(wind, N >> 2);

    ESP_LOGI(AIGORITHM_TAG, "Algorithm task suspend !");
    vTaskSuspend(NULL);

    while (1)
    {
        ESP_LOGW(AIGORITHM_TAG, "Trigger algorithm detection!");
        ESP_LOGW(AIGORITHM_TAG, "Wait for 10s imu data collection to complete");
        vTaskDelay(10000 / portTICK_PERIOD_MS);

        // taskENTER_CRITICAL(&my_spinlock);
        vCopyImuData(usPtrArrImu, &fArrRoll[0], &fArrPitch[0], &fArrYaw[0]);
        // taskEXIT_CRITICAL(&my_spinlock);
        ESP_LOGW(AIGORITHM_TAG, "copy data successfully");

        vTaskSuspend(xTofHandle); /* 画图时挂起 TOF 打印 */
        ESP_LOGW(AIGORITHM_TAG, "Roll Raw");
        dsps_view(fArrRollFFT, N, 128, 30, -90, 90, '.');
        ESP_LOGW(AIGORITHM_TAG, "Pitch Raw");
        dsps_view(fArrPitchFFT, N, 128, 30, -90, 90, '.');
        ESP_LOGW(AIGORITHM_TAG, "Yaw Raw");
        dsps_view(fArrYawFFT, N, 128, 30, 0, 360, '.');
        vTaskResume(xTofHandle);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        for (uint16_t i = N - (N >> 2), usAssignmentCnt = 0; i < N; i++)
        {
            fArrRollFFT[i] = fArrRollFFT[i] * wind[usAssignmentCnt];
            fArrPitchFFT[i] = fArrPitchFFT[i] * wind[usAssignmentCnt];
            fArrYawFFT[i] = fArrYawFFT[i] * wind[usAssignmentCnt];
            usAssignmentCnt++;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);

        vTaskSuspend(xTofHandle); /* 画图时挂起 TOF 打印 */
        ESP_LOGW(AIGORITHM_TAG, "Roll Wind");
        dsps_view(&fArrRollFFT[N - (N >> 2)], (N >> 2), 128, 30, -90, 90, '.');
        ESP_LOGW(AIGORITHM_TAG, "Pitch Raw");
        dsps_view(&fArrPitchFFT[N - (N >> 2)], (N >> 2), 128, 30, -90, 90, '.');
        ESP_LOGW(AIGORITHM_TAG, "Yaw Raw");
        dsps_view(&fArrYawFFT[N - (N >> 2)], (N >> 2), 128, 30, 0, 360, '.');
        vTaskResume(xTofHandle);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        dsps_fft2r_fc32(&fArrRollFFT[N - (N >> 2)], (N >> 3)); /* 长度必须减半 */
        dsps_bit_rev2r_fc32(&fArrRollFFT[N - (N >> 2)], (N >> 3));
        dsps_cplx2real_fc32(&fArrRollFFT[N - (N >> 2)], (N >> 3));

        // // dsps_fft2r_fc32(fArrPitchFFT, N >> 1);
        // // dsps_bit_rev2r_fc32(fArrPitchFFT, N >> 1);
        // // dsps_cplx2real_fc32(fArrPitchFFT, N >> 1);

        // // dsps_fft2r_fc32(fArrYawFFT, N >> 1);
        // // dsps_bit_rev2r_fc32(fArrYawFFT, N >> 1);
        // // dsps_cplx2real_fc32(fArrYawFFT, N >> 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        for (uint16_t i = N - (N >> 2); i < N; i++)
        {
            fArrRollFFT[i] = 10 * log10f((fArrRollFFT[i * 2 + 0] * fArrRollFFT[i * 2 + 0] + fArrRollFFT[i * 2 + 1] * fArrRollFFT[i * 2 + 1] + 0.0000001) / N);
            //     // fArrPitchFFT[i] = 10 * log10f((fArrPitchFFT[i * 2 + 0] * fArrPitchFFT[i * 2 + 0] + fArrPitchFFT[i * 2 + 1] * fArrPitchFFT[i * 2 + 1] + 0.0000001) / N);
            //     // fArrYawFFT[i] = 10 * log10f((fArrYawFFT[i * 2 + 0] * fArrYawFFT[i * 2 + 0] + fArrYawFFT[i * 2 + 1] * fArrYawFFT[i * 2 + 1] + 0.0000001) / N);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);

        ESP_LOGW(AIGORITHM_TAG, "Roll FFT");
        dsps_view(&fArrRollFFT[N - (N >> 2)], (N >> 2), 128, 30, -100, 100, '.');
        // // ESP_LOGW(AIGORITHM_TAG, "Pitch FFT");
        // // dsps_view(fArrPitchFFT, (N >> 2), 128, 20, -50, 40, '.');
        // // ESP_LOGW(AIGORITHM_TAG, "Yaw FFT");
        // // dsps_view(fArrYawFFT, (N >> 2), 128, 20, -50, 40, '.');

        vTaskDelay(5000 / portTICK_PERIOD_MS);
        // vTaskResume(xImuHandle);
        ESP_LOGW(AIGORITHM_TAG, "Algorithm finish !");
        vTaskSuspend(NULL);
    } // end while

    // Never reach here
    vTaskDelete(NULL);
}