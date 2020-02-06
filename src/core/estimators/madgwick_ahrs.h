#ifndef Madgwick_estimator_H
#define Madgwick_estimator_H

#include <stdint.h>
#include "math.h"

typedef struct _madgwick_t {
	float beta;
	float q0, q1, q2, q3;
	float sampleRate;
} madgwick_t;

void madgwick_init(madgwick_t* madgwick, float sample_rate, float beta);
void madgwick_imu_ahrs(madgwick_t* Madgwick, float ax, float ay, float az, float gx, float gy, float gz);
void Madgwick_MARG_AHRS(madgwick_t* Madgwick, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

#endif

