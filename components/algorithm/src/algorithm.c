#include "algorithm.h"

TaskHandle_t xAlgorithmHandle = NULL;

float fArrRoll[N_SAMPLES] = {0};
float fArrPitch[N_SAMPLES] = {0};
float fArrYaw[N_SAMPLES] = {0};
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
    ret = dsps_fft2r_init_fc32(NULL, N >> 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(AIGORITHM_TAG, "Not possible to initialize FFT2R. Error = %i", ret);
        return;
    }
    ret = dsps_fft4r_init_fc32(NULL, N >> 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(AIGORITHM_TAG, "Not possible to initialize FFT4R. Error = %i", ret);
        return;
    }
    dsps_wind_hann_f32(wind, N);

    vTaskSuspend(NULL);
    for (;;)
    {
        ESP_LOGI(AIGORITHM_TAG, "Trigger algorithm detection!");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        vCopyImuData(usPtrArrImu, &fArrRoll, &fArrPitch, &fArrYaw);

        for (int i = 0; i < N; i++)
        {
            fArrRollFFT[i] = fArrRollFFT[i] * wind[i];
            fArrPitchFFT[i] = fArrPitchFFT[i] * wind[i];
            fArrYawFFT[i] = fArrYawFFT[i] * wind[i];
        }

        dsps_fft2r_fc32(fArrRollFFT, N >> 1);
        dsps_bit_rev2r_fc32(fArrRollFFT, N >> 1);
        dsps_cplx2real_fc32(fArrRollFFT, N >> 1);

        dsps_fft2r_fc32(fArrPitchFFT, N >> 1);
        dsps_bit_rev2r_fc32(fArrPitchFFT, N >> 1);
        dsps_cplx2real_fc32(fArrPitchFFT, N >> 1);

        dsps_fft2r_fc32(fArrYawFFT, N >> 1);
        dsps_bit_rev2r_fc32(fArrYawFFT, N >> 1);
        dsps_cplx2real_fc32(fArrYawFFT, N >> 1);

        for (uint16_t i = 0; i < N / 2; i++)
        {
            fArrRollFFT[i] = 10 * log10f((fArrRollFFT[i * 2 + 0] * fArrRollFFT[i * 2 + 0] + fArrRollFFT[i * 2 + 1] * fArrRollFFT[i * 2 + 1] + 0.0000001) / N);
            fArrPitchFFT[i] = 10 * log10f((fArrPitchFFT[i * 2 + 0] * fArrPitchFFT[i * 2 + 0] + fArrPitchFFT[i * 2 + 1] * fArrPitchFFT[i * 2 + 1] + 0.0000001) / N);
            fArrYawFFT[i] = 10 * log10f((fArrYawFFT[i * 2 + 0] * fArrYawFFT[i * 2 + 0] + fArrYawFFT[i * 2 + 1] * fArrYawFFT[i * 2 + 1] + 0.0000001) / N);
        }

        ESP_LOGW(AIGORITHM_TAG, "Roll FFT");
        dsps_view(fArrRollFFT, N / 2, 64, 10, -60, 40, '.');
        ESP_LOGW(AIGORITHM_TAG, "Pitch FFT");
        dsps_view(fArrPitchFFT, N / 2, 64, 10, -60, 40, '.');
        ESP_LOGW(AIGORITHM_TAG, "Yaw FFT");
        dsps_view(fArrYawFFT, N / 2, 64, 10, -60, 40, '.');

        vTaskSuspend(NULL);
    } // end while

    // Never reach here
    vTaskDelete(NULL);
}