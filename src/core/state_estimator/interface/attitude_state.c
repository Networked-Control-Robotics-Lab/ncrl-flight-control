#include "ins.h"

void get_attitude_euler_angles(float *roll, float *pitch, float *yaw)
{
	ins_ahrs_get_attitude_euler_angles(roll, pitch, yaw);
}

void get_attitude_quaternion(float *q)
{
	ins_ahrs_get_attitude_quaternion(q);
}

void get_rotation_matrix_b2i(float **R_b2i)
{
	ins_ahrs_get_rotation_matrix_b2i(R_b2i);
}

void get_rotation_matrix_i2b(float **R_i2b)
{
	get_rotation_matrix_i2b(R_i2b);
}
