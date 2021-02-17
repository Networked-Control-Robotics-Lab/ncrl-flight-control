#ifndef __madgwick_ahrs_h__
#define __madgwick_ahrs_h__

#include <stdint.h>
#include "math.h"

typedef struct _madgwick_t {
	float beta;
	float dt;
	float q[4];
} madgwick_t;

void madgwick_init(madgwick_t* madgwick, float sample_rate, float beta);
void ahrs_madgwick_filter_set_quat(madgwick_t* madgwick, float *q);
void madgwick_imu_ahrs(madgwick_t* madgwick, float *accel, float *gyro);
void madgwick_margs_ahrs(madgwick_t* madgwick, float *accel, float *gyro, float *mag);

#endif

