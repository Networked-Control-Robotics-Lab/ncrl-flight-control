#ifndef __COMP_AHRS_H__
#define __COMP_AHRS_H__

void complementary_ahrs_init(float ahrs_dt);
void ahrs_complementary_filter_set_quat(float *_q);
void ahrs_imu_complementary_filter_estimate(float *q_out, float *accel, float *gyro);
void ahrs_marg_complementary_filter_estimate(float *q_out, float *accel, float *gyro, float *mag);

void convert_magnetic_field_to_quat(float *l, float *q);

#endif
