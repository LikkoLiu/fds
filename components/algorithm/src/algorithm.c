#include "algorithm.h"

static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t xAlgorithmHandle = NULL;

__attribute__((aligned(16))) float fArrRoll[N_SAMPLES] = {0};
__attribute__((aligned(16))) float fArrPitch[N_SAMPLES] = {0};
__attribute__((aligned(16))) float fArrYaw[N_SAMPLES] = {0};
uint16_t usPtrArrImu = 0;

int N = N_SAMPLES;
__attribute__((aligned(16))) float fArrRollFFT[N_SAMPLES];
__attribute__((aligned(16))) float fArrPitchFFT[N_SAMPLES];
__attribute__((aligned(16))) float fArrYawFFT[N_SAMPLES];

__attribute__((aligned(16))) float wind[N_SAMPLES];

void vCopyImuData(uint16_t usCurrentPtr, float *pfArrRoll, float *pfArrPitch, float *pfArrYaw)
{
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
    ESP_LOGW(AIGORITHM_TAG, "Algorithm task begin");
    ret = dsps_fft2r_init_fc32(NULL, N * 2);
    if (ret != ESP_OK)
    {
        ESP_LOGE(AIGORITHM_TAG, "Not possible to initialize FFT2R. Error = %i", ret);
        vTaskDelete(NULL);
    }
    ESP_LOGW(AIGORITHM_TAG, "Algorithm fft2r init ");
    /* 下面这个必须加！ */
    ret = dsps_fft4r_init_fc32(NULL, N * 4);
    if (ret != ESP_OK)
    {
        ESP_LOGE(AIGORITHM_TAG, "Not possible to initialize FFT4R. Error = %i", ret);
        vTaskDelete(NULL);
    }
    ESP_LOGW(AIGORITHM_TAG, "Algorithm fft4r init ");
    dsps_wind_hann_f32(wind, N >> 2);

    ESP_LOGW(AIGORITHM_TAG, "Algorithm task suspend !");
    vTaskSuspend(NULL);
    for (;;)
    {
        ESP_LOGI(AIGORITHM_TAG, "Trigger algorithm detection!");

        vTaskSuspend(xTofHandle);
        vTaskSuspend(xImuHandle);

        vCopyImuData(usPtrArrImu, &fArrRoll, &fArrPitch, &fArrYaw);

        ESP_LOGW(AIGORITHM_TAG, "Roll Raw");
        dsps_view(fArrRollFFT, N, 128, 30, -180, 180, '.');
        ESP_LOGW(AIGORITHM_TAG, "Pitch Raw");
        dsps_view(fArrPitchFFT, N, 128, 30, -180, 180, '.');
        ESP_LOGW(AIGORITHM_TAG, "Yaw Raw");
        dsps_view(fArrYawFFT, N, 128, 30, 0, 360, '.');

        for (int i = (N >> 2); i < N; i++)
        {
            fArrRollFFT[i] = fArrRollFFT[i] * wind[i];
            // fArrPitchFFT[i] = fArrPitchFFT[i] * wind[i];
            // fArrYawFFT[i] = fArrYawFFT[i] * wind[i];
        }

        ESP_LOGW(AIGORITHM_TAG, "Roll Wind");
        dsps_view(fArrRollFFT, N, 128, 30, -180, 180, '.');
        // ESP_LOGW(AIGORITHM_TAG, "Pitch Raw");
        // dsps_view(fArrPitchFFT, N, 128, 30, -180, 180, '.');
        // ESP_LOGW(AIGORITHM_TAG, "Yaw Raw");
        // dsps_view(fArrYawFFT, N, 128, 30, 0, 360, '.');

        taskENTER_CRITICAL(&my_spinlock);
        dsps_fft2r_fc32(&fArrRollFFT[N - (N >> 2)], N >> 2);
        dsps_bit_rev2r_fc32(&fArrRollFFT[N - (N >> 2)], N >> 2);
        dsps_cplx2real_fc32(&fArrRollFFT[N - (N >> 2)], N >> 2);
        esp_task_wdt_restart();

        // dsps_fft2r_fc32(fArrPitchFFT, N >> 1);
        // dsps_bit_rev2r_fc32(fArrPitchFFT, N >> 1);
        // dsps_cplx2real_fc32(fArrPitchFFT, N >> 1);

        // dsps_fft2r_fc32(fArrYawFFT, N >> 1);
        // dsps_bit_rev2r_fc32(fArrYawFFT, N >> 1);
        // dsps_cplx2real_fc32(fArrYawFFT, N >> 1);

        for (uint16_t i = N - (N >> 2); i < N; i++)
        {
            fArrRollFFT[i] = 10 * log10f((fArrRollFFT[i * 2 + 0] * fArrRollFFT[i * 2 + 0] + fArrRollFFT[i * 2 + 1] * fArrRollFFT[i * 2 + 1] + 0.0000001) / N);
            // fArrPitchFFT[i] = 10 * log10f((fArrPitchFFT[i * 2 + 0] * fArrPitchFFT[i * 2 + 0] + fArrPitchFFT[i * 2 + 1] * fArrPitchFFT[i * 2 + 1] + 0.0000001) / N);
            // fArrYawFFT[i] = 10 * log10f((fArrYawFFT[i * 2 + 0] * fArrYawFFT[i * 2 + 0] + fArrYawFFT[i * 2 + 1] * fArrYawFFT[i * 2 + 1] + 0.0000001) / N);
        }
        taskEXIT_CRITICAL(&my_spinlock);

        ESP_LOGW(AIGORITHM_TAG, "Roll FFT");
        dsps_view(&fArrRollFFT[N - (N >> 2)], (N >> 2), 128, 20, -50, 100, '.');
        // ESP_LOGW(AIGORITHM_TAG, "Pitch FFT");
        // dsps_view(fArrPitchFFT, (N >> 2), 128, 20, -50, 40, '.');
        // ESP_LOGW(AIGORITHM_TAG, "Yaw FFT");
        // dsps_view(fArrYawFFT, (N >> 2), 128, 20, -50, 40, '.');



        vTaskResume(xImuHandle);
        vTaskResume(xTofHandle);
        vTaskSuspend(NULL);
    } // end while

    // Never reach here
    vTaskDelete(NULL);
}