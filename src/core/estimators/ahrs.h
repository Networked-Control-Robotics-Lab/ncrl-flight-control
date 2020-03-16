#ifndef __AHRS__
#define __AHRS__

#include "vector.h"

#define deg_to_rad(angle) (angle * 0.01745329252)
#define rad_to_deg(radian) (radian * 57.2957795056)

typedef struct {
	float roll;
	float pitch;
	float yaw;
} euler_t;

typedef struct {
	euler_t attitude;
	float q[4];
} ahrs_t;

void ahrs_init(float *init_accel);
void ahrs_estimate(ahrs_t *ahrs, float *_accel, float *_gyro);


#endif
