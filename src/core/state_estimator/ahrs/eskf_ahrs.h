#ifndef __ESKF_AHRS_H__
#define __ESKF_AHRS_H__

void eskf_ahrs_init(float dt);
void eskf_ahrs_predict(float *gyro);
void eskf_ahrs_accelerometer_correct(float *accel);
void eskf_ahrs_magnetometer_correct(float *mag);
void get_eskf_attitude_quaternion(float *q_out);

#endif
