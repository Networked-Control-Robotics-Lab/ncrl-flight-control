#ifndef __AHRS__
#define __AHRS__

#include "se3_math.h"

typedef struct {
	euler_t attitude;
	float q[4];
} ahrs_t;

void ahrs_init(float *init_accel);
void ahrs_estimate(ahrs_t *ahrs, float *_accel, float *_gyro);

#endif
