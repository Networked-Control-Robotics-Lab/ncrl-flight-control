#ifndef __AHRS__
#define __AHRS__

#include "se3_math.h"

typedef struct {
	//eulers angle
	float roll;
	float pitch;
	float yaw;

	//quaternion
	float q[4];

	//direction cosine matrix
	float R_matrix[3 * 3];            //earth frame to body-fixed frame
	float R_transposed_matrix[3 * 3]; //body-fixed frame to earth frame
} attitude_t;

void ahrs_init(void);
void ahrs_estimate(void);

void get_attitude_euler_angles(float *roll, float *pitch, float *yaw);
void get_attitude_quaternion(float *q);
void get_attitude_direction_cosine_matrix(float **R_data);
void get_attitude_transposed_direction_cosine_matrix(float **R_transposed_data);

#endif
