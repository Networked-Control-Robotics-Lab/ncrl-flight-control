#include <math.h>
#include <stdio.h>
#include "arm_math.h"
#include "ahrs.h"
#include "ahrs_optitrack.h"
#include "matrix.h"
#include "quaternion.h"
#include "optitrack.h"

MAT_ALLOC(q, 4, 1);
MAT_ALLOC(R_gyro, 3, 3);

float optitrack_ahrs_dt = 0.0f;

void optitrack_ahrs_init(float ahrs_dt)
{
	MAT_INIT(q, 4, 1);
	MAT_INIT(R_gyro, 3, 3);

	optitrack_ahrs_dt = ahrs_dt;

	mat_data(q)[0] = 1.0f;
	mat_data(q)[1] = 0.0f;
	mat_data(q)[2] = 0.0f;
	mat_data(q)[3] = 0.0f;
}

void ahrs_optitrack_imu_fuse_estimate(float *q_out, float *gyro)
{
	/* quaternion integration (with gyroscope) */
	float w[4];
	w[0] = 0.0f;
	w[1] = gyro[0];
	w[2] = gyro[1];
	w[3] = gyro[2];

	float q_dot[4];
	quaternion_mult(w, mat_data(q), q_dot);

	float q_gyro[4];
	float half_dt = -0.5 * optitrack_ahrs_dt;
	q_gyro[0] = mat_data(q)[0] + (q_dot[0] * half_dt);
	q_gyro[1] = mat_data(q)[1] + (q_dot[1] * half_dt);
	q_gyro[2] = mat_data(q)[2] + (q_dot[2] * half_dt);
	q_gyro[3] = mat_data(q)[3] + (q_dot[3] * half_dt);
	quat_normalize(q_gyro); //XXX: currently gyroscope integration data serves nothing here

	/* get optitrack's attitude quaternion */
	float q_optitrack[4];
	optitrack_get_quaternion(q_optitrack);

	q_out[0] = q_optitrack[0];
	q_out[1] = q_optitrack[2];
	q_out[2] = q_optitrack[1];
	q_out[3] = q_optitrack[3];

	//TODO: use gyroscope integration result when new optirack data is not available yet
}
