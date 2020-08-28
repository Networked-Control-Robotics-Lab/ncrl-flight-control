#include <math.h>
#include <stdio.h>
#include "arm_math.h"
#include "ahrs.h"
#include "comp_ahrs.h"
#include "lpf.h"
#include "matrix.h"
#include "se3_math.h"
#include "quaternion.h"

/* reference paper: Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs */

MAT_ALLOC(q_gyro, 4, 1);
MAT_ALLOC(q, 4, 1);
MAT_ALLOC(dq, 4, 1);
MAT_ALLOC(w, 3, 1);
MAT_ALLOC(f, 4, 3);

float comp_ahrs_dt = 0.0f;

void complementary_ahrs_init(float ahrs_dt)
{
	MAT_INIT(q_gyro, 4, 1);
	MAT_INIT(q, 4, 1);
	MAT_INIT(dq, 4, 1);
	MAT_INIT(w, 3, 1);
	MAT_INIT(f, 4, 3);

	comp_ahrs_dt = ahrs_dt;

	_mat_(q)[0] = 1.0f;
	_mat_(q)[1] = 0.0f;
	_mat_(q)[2] = 0.0f;
	_mat_(q)[3] = 0.0f;
}

void convert_gravity_to_quat(float *a, float *q)
{
	float _sqrt;

	if(a[2] >= 0.0f) {
		//q0
		arm_sqrt_f32(0.5f * (a[2] + 1.0f), &_sqrt);
		q[0] = _sqrt;
		//q1
		arm_sqrt_f32(2.0f * (a[2] + 1.0f), &_sqrt);
		q[1] = +a[1] / _sqrt;
		//q2
		arm_sqrt_f32(2.0f * (a[2] + 1.0f), &_sqrt);
		q[2] = -a[0] / _sqrt;
		//q3
		q[3] = 0.0f;
	} else {
		//q0
		arm_sqrt_f32(2.0f * (1.0f - a[2]), &_sqrt);
		q[0] = -a[1] / _sqrt;
		//q1
		arm_sqrt_f32((1.0f - a[2]) * 0.5f, &_sqrt);
		q[1] = -_sqrt;
		//q2
		q[2] = 0.0f;
		//q3
		arm_sqrt_f32(2.0f * (1.0f - a[2]), &_sqrt);
		q[3] = -a[0] / _sqrt;
	}

	quat_normalize(q);
}

void convert_magnetic_field_to_quat(float *l, float *q)
{
	float gamma = l[0]*l[0] + l[1]*l[1];
	float sqrt_gamma, sqrt_2gamma, sqrt_2;
	arm_sqrt_f32(gamma, &sqrt_gamma);
	arm_sqrt_f32(2.0f * gamma, &sqrt_2gamma);
	arm_sqrt_f32(2.0f, &sqrt_2); //TODO: this is constant!

	float _sqrt;

	if(l[0] >= 0.0f) {
		arm_sqrt_f32(gamma + l[0]*sqrt_gamma, &_sqrt);
		q[0] = _sqrt / sqrt_2gamma;
		q[1] = 0.0f;
		q[2] = 0.0f;
		q[3] = -l[1] / (sqrt_2 * _sqrt);
	} else {
		arm_sqrt_f32(gamma - l[0]*sqrt_gamma, &_sqrt);
		q[0] = l[1] / (sqrt_2 * _sqrt);
		q[1] = 0.0f;
		q[2] = 0.0f;
		q[3] = -_sqrt / sqrt_2gamma;
	}

	quat_normalize(q);
}

/* directly assign new quaternion to complementary filter */
void ahrs_complementary_filter_set_quat(float *_q)
{
	quaternion_copy(_mat_(q), _q);
}

void ahrs_complementary_filter_estimate(float *q_out, float *accel, float *gyro)
{
	/* check the paper: Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MA
	 * by Roberto G. Valenti, Ivan Dryanovski and Jizhong Xiao */

	/* construct system transition function f */
	float half_q0_dt = 0.5f * _mat_(q)[0] * comp_ahrs_dt;
	float half_q1_dt = 0.5f * _mat_(q)[1] * comp_ahrs_dt;
	float half_q2_dt = 0.5f * _mat_(q)[2] * comp_ahrs_dt;
	float half_q3_dt = 0.5f * _mat_(q)[3] * comp_ahrs_dt;
	_mat_(f)[0] = -half_q1_dt;
	_mat_(f)[1] = -half_q2_dt;
	_mat_(f)[2] = -half_q3_dt;
	_mat_(f)[3] = +half_q0_dt;
	_mat_(f)[4] = -half_q3_dt;
	_mat_(f)[5] = +half_q2_dt;
	_mat_(f)[6] = +half_q3_dt;
	_mat_(f)[7] = +half_q0_dt;
	_mat_(f)[8] = -half_q1_dt;
	_mat_(f)[9] = -half_q2_dt;
	_mat_(f)[10] = +half_q1_dt;
	_mat_(f)[11] = +half_q0_dt;

	/* angular rate from rate gyro */
	_mat_(w)[0] = gyro[0];
	_mat_(w)[1] = gyro[1];
	_mat_(w)[2] = gyro[2];

	/* rate gyro integration */
	MAT_MULT(&f, &w, &dq); //calculate dq = f * w
	MAT_ADD(&q, &dq, &q_gyro);  //calculate x = x + dq
	quat_normalize(_mat_(q_gyro)); //renormalization

	/* convert gravity vector to quaternion */
	float q_gravity[4] = {0};
	normalize_3x1(accel); //normalize acceleromter
	convert_gravity_to_quat(accel, q_gravity);

	/* fuse gyroscope and acceleromter using LERP algorithm */
	float a = 0.995f;
	_mat_(q)[0] = (_mat_(q_gyro)[0] * a) + (q_gravity[0]* (1.0 - a));
	_mat_(q)[1] = (_mat_(q_gyro)[1] * a) + (q_gravity[1]* (1.0 - a));
	_mat_(q)[2] = (_mat_(q_gyro)[2] * a) + (q_gravity[2]* (1.0 - a));
	_mat_(q)[3] = (_mat_(q_gyro)[3] * a) + (q_gravity[3]* (1.0 - a));
	//it is crucial to renormalize the quaternion since LERP don't maintain
	//the unit length propertety of the quaternion
	quat_normalize(_mat_(q));

	q_out[0] = _mat_(q)[0];
	q_out[1] = _mat_(q)[1];
	q_out[2] = _mat_(q)[2];
	q_out[3] = _mat_(q)[3];
}

