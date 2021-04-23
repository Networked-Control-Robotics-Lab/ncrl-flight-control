#ifndef __ATTITUDE_STATE_H__
#define __ATTITUDE_STATE_H__

void get_attitude_euler_angles(float *roll, float *pitch, float *yaw);
void get_attitude_quaternion(float *q);
void get_rotation_matrix_b2i(float **R_b2i);
void get_rotation_matrix_i2b(float **R_i2b);

#endif
