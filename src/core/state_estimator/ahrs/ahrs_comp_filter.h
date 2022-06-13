#ifndef __AHRS_COMP_FILTER_H__
#define __AHRS_COMP_FILTER_H__

void complementary_ahrs_init(float ahrs_dt);
void ahrs_imu_complementary_filter_estimate(float *q_out, float *accel, float *gyro);
void ahrs_marg_complementary_filter_estimate(float *q_out, float *accel, float *gyro, float *mag);

void convert_gravity_to_quat(float *a, float *q);
void convert_gravity_to_delta_quat(float *a, float *q);

void convert_magnetic_field_to_quat(float *l, float *q);
void convert_magnetic_field_to_delta_quat(float *l, float *q);

#endif
