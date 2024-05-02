#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <math.h>

#define sampleFreqDef 512.0f // sample frequency in Hz
#define betaDef 0.1f         // 2 * proportional gain

void MadgwickInit();
void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void eulerAngles(float *roll, float *pitch, float *yaw);

#endif /* MADGWICK_FILTER_H */
