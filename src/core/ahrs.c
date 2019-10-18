#include <math.h>

#include "arm_math.h"
#include "led.h"
#include "mpu6500.h"
#include "ahrs.h"
#include "vector.h"
#include "lpf.h"
#include "uart.h"
#include "matrix.h"

#define AHRS_SELECT AHRS_SELECT_CF

#define dt 0.0025 //0.0025s = 400Hz

imu_t imu;
ahrs_t ahrs;

vector3d_f_t accel_lpf_old, gyro_lpf_old;

MAT_ALLOC(x_priori, 4, 1);
MAT_ALLOC(x_posteriori, 4, 1);
MAT_ALLOC(dx, 4, 1);
MAT_ALLOC(w, 3, 1);
MAT_ALLOC(f, 4, 3);
MAT_ALLOC(y, 4, 1);
MAT_ALLOC(resid, 4, 1);
MAT_ALLOC(F, 4, 4);
MAT_ALLOC(P, 4, 4);
MAT_ALLOC(Ft, 4, 4);
MAT_ALLOC(FP, 4, 4);
MAT_ALLOC(PFt, 4, 4);
MAT_ALLOC(FP_PFt_Q, 4, 4);
MAT_ALLOC(dP, 4, 4);
MAT_ALLOC(R, 4, 4);
MAT_ALLOC(Q, 4, 4);
MAT_ALLOC(K, 4, 4);
MAT_ALLOC(dt_4x4, 4, 4) = {dt, 0, 0,0,
                           0, dt, 0, 0,
                           0, 0, dt, 0,
                           0, 0, 0, dt
                          };

void ahrs_ekf_init(void)
{
	//initialize matrices
	MAT_INIT(x_priori, 4, 1);
	MAT_INIT(x_posteriori, 4, 1);
	MAT_INIT(dx, 4, 1);
	MAT_INIT(w, 3, 1);
	MAT_INIT(f, 4, 3);
	MAT_INIT(y, 4, 1);
	MAT_INIT(resid, 4, 1);
	MAT_INIT(F, 4, 4);
	MAT_INIT(P, 4, 4);
	MAT_INIT(Ft, 4, 4);
	MAT_INIT(FP, 4, 4);
	MAT_INIT(PFt, 4, 4);
	MAT_INIT(FP_PFt_Q, 4, 4);
	MAT_INIT(dP, 4, 4);
	MAT_INIT(R, 4, 4);
	MAT_INIT(Q, 4, 4);
	MAT_INIT(dt_4x4, 4, 4);

	_mat_(P)[0] = _mat_(P)[5] = _mat_(P)[10] = _mat_(P)[15] =  100.0f;
	_mat_(Q)[0] = _mat_(Q)[5] = _mat_(Q)[10] = _mat_(Q)[15] = 0.1f;
	_mat_(R)[0] = _mat_(R)[5] = _mat_(R)[10] = _mat_(R)[15] = 0.001;

	//initialize lpf
	mpu6500_read_unscaled_data(&imu.unscaled_accel, &imu.unscaled_gyro, &imu.unscaled_temp);
	mpu6500_fix_bias(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6500_accel_convert_to_scale(&imu.unscaled_accel, &imu.raw_accel);
	mpu6500_gyro_convert_to_scale(&imu.unscaled_gyro, &imu.raw_gyro);
	accel_lpf_old = imu.raw_accel;
	gyro_lpf_old = imu.raw_gyro;

	calc_attitude_use_accel();

	attitude_t att_init;
	att_init.roll = ahrs.attitude.roll;
	att_init.pitch = ahrs.attitude.pitch;
	att_init.yaw = 0.0f;
	euler_to_quat(&att_init, &_mat_(x_priori)[0]);
}

//in: euler angle [radian], out: quaternion
void euler_to_quat(attitude_t *euler, float *q)
{
	float phi = euler->roll * 0.5f;
	float theta = euler->pitch * 0.5f;
	float psi = euler->yaw * 0.5f;

	q[0] = arm_cos_f32(phi)*arm_cos_f32(theta)*arm_cos_f32(psi) +
	       arm_sin_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi);
	q[1] = arm_sin_f32(phi)*arm_cos_f32(theta)*arm_cos_f32(psi) -
	       arm_cos_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi);
	q[2] = arm_cos_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi) +
	       arm_sin_f32(phi)*arm_cos_f32(theta)*arm_sin_f32(psi);
	q[3] = arm_cos_f32(phi)*arm_cos_f32(theta)*arm_sin_f32(psi) -
	       arm_sin_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi);
}

void quat_normalize(float *q)
{
	float sq_sum = (q[0])*(q[0]) + (q[1])*(q[1]) + (q[2])*(q[2]) + (q[3])*(q[3]);
	float norm;
	arm_sqrt_f32(sq_sum, &norm);
	q[0] /= norm;
	q[1] /= norm;
	q[2] /= norm;
	q[3] /= norm;
}

//in: quaterion, out: euler angle [radian]
void quat_to_euler(float *q, attitude_t *euler)
{
	euler->roll = atan2(2.0*(q[0]*q[1] + q[2]*q[3]), 1.0-2.0*(q[1]*q[1] + q[2]*q[2]));
	euler->pitch = asin(2.0*(q[0]*q[2] - q[3]*q[1]));
	euler->yaw = atan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0-2.0*(q[2]*q[2] + q[3]*q[3]));
}

void convert_gravity_to_quat(vector3d_f_t *a, float *q)
{
	float _sqrt;

	if(a->z >= 0.0f) {
		//q0
		arm_sqrt_f32(0.5f * (a->z + 1.0f), &_sqrt);
		q[0] = _sqrt;
		//q1
		arm_sqrt_f32(2.0f * (a->z + 1.0f), &_sqrt);
		q[1] = -a->y / _sqrt;
		//q2
		arm_sqrt_f32(2.0f * (a->z + 1.0f), &_sqrt);
		q[2] = a->x / _sqrt;
		//q3
		q[3] = 0.0f;
	} else {
		//q0
		arm_sqrt_f32(2.0f * (1.0f - a->z), &_sqrt);
		q[0] = -a->y / _sqrt;
		//q1
		arm_sqrt_f32((1.0f - a->z) * 0.5f, &_sqrt);
		q[1] = _sqrt;
		//q2
		q[2] = 0.0f;
		//q3
		arm_sqrt_f32(2.0f * (1.0f - a->z), &_sqrt);
		q[3] = a->x / _sqrt;
	}

	quat_normalize(q);
}

void calc_attitude_use_accel(void)
{
	ahrs.attitude.roll = rad_to_deg(atan2(imu.filtered_accel.x, imu.filtered_accel.z));
	ahrs.attitude.pitch = rad_to_deg(atan2(-imu.filtered_accel.y, imu.filtered_accel.z));
}

void ahr_ekf_state_predict(void)
{
	float half_q0_dt = 0.5f * _mat_(x_priori)[0] * dt;
	float half_q1_dt = 0.5f * _mat_(x_priori)[1] * dt;
	float half_q2_dt = 0.5f * _mat_(x_priori)[2] * dt;
	float half_q3_dt = 0.5f * _mat_(x_priori)[3] * dt;
	_mat_(f)[0]=-half_q1_dt;
	_mat_(f)[1]=-half_q2_dt;
	_mat_(f)[2]=-half_q3_dt;
	_mat_(f)[3]=+half_q0_dt;
	_mat_(f)[4]=-half_q3_dt;
	_mat_(f)[5]=+half_q2_dt;
	_mat_(f)[6]=+half_q3_dt;
	_mat_(f)[7]=+half_q0_dt;
	_mat_(f)[8]=-half_q1_dt;
	_mat_(f)[9]=-half_q2_dt;
	_mat_(f)[10]=+half_q1_dt;
	_mat_(f)[11]=+half_q0_dt;

	_mat_(w)[0] = deg_to_rad(imu.filtered_gyro.x);
	_mat_(w)[1] = deg_to_rad(imu.filtered_gyro.y);
	_mat_(w)[2] = deg_to_rad(imu.filtered_gyro.z);

	MAT_MULT(&f, &w, &dx); //calculate dx = f * w
	MAT_ADD(&x_priori, &dx, &x_priori);  //calculate x = x + dx

	quat_normalize(&_mat_(x_priori)[0]);

	//P = P + dt * (FP + PF' + Q)
	float wx = _mat_(w)[0];
	float wy = _mat_(w)[1];
	float wz = _mat_(w)[2];

	_mat_(F)[0]=0.0;
	_mat_(F)[1]=-0.5*wx;
	_mat_(F)[2]=-0.5*wy;
	_mat_(F)[3]=-0.5*wz;
	_mat_(F)[4]=0.5*wx;
	_mat_(F)[5]=0.0;
	_mat_(F)[6]=0.5*wz;
	_mat_(F)[7]=-0.5*wy;
	_mat_(F)[8]=0.5*wy;
	_mat_(F)[9]=-0.5*wz;
	_mat_(F)[10]=0.0;
	_mat_(F)[11]=0.5*wx;
	_mat_(F)[12]=0.5*wz;
	_mat_(F)[13]=0.5*wy;
	_mat_(F)[14]=-0.5*wx;
	_mat_(F)[15]=0.0;

	MAT_TRANS(&F, &Ft);                     //calculate F'
	MAT_MULT(&F, &P, &FP);                  //calculate F*P
	MAT_MULT(&P, &Ft, &PFt);                //calculate P*F'
	MAT_ADD(&FP, &PFt, &FP_PFt_Q);          //calculate F*P + P*F'
	MAT_ADD(&FP_PFt_Q, &Q, &FP_PFt_Q);      //calculate F*P + P*F + Q
	MAT_MULT(&dt_4x4, &FP_PFt_Q, &FP_PFt_Q) //calculate dt * (F*P + P*F + Q)
	MAT_ADD(&P, &FP_PFt_Q, &P);             //calculate P = P + dt * (F*P + P*F + Q)
}

void ahr_ekf_state_update(void)
{
	/* convert gravity vector to quaternion */
	vector3d_normalize(&imu.filtered_accel); //normalize acceleromter
	convert_gravity_to_quat(&imu.filtered_accel, &_mat_(y)[0]);

	/* calculate residual */
	_mat_(resid)[0] = _mat_(y)[0] - _mat_(x_priori)[0];
	_mat_(resid)[1] = _mat_(y)[1] - _mat_(x_priori)[1];
	_mat_(resid)[2] = _mat_(y)[2] - _mat_(x_priori)[2];
	_mat_(resid)[3] = _mat_(y)[3] - _mat_(x_priori)[3];

	/* calculate kalman gain */
	_mat_(K)[0] = _mat_(P)[0] / (_mat_(P)[0] + _mat_(R)[0]);
	_mat_(K)[5] = _mat_(P)[5] / (_mat_(P)[5] + _mat_(R)[5]);
	_mat_(K)[10] = _mat_(P)[10] / (_mat_(P)[10] + _mat_(R)[10]);
	_mat_(K)[15] = _mat_(P)[15] / (_mat_(P)[15] + _mat_(R)[15]);

	/* caluclate innovation */
	_mat_(x_posteriori)[0] += (_mat_(K)[0] * _mat_(resid)[0]);
	_mat_(x_posteriori)[1] += (_mat_(K)[5] * _mat_(resid)[1]);
	_mat_(x_posteriori)[2] += (_mat_(K)[10] * _mat_(resid)[2]);
	_mat_(x_posteriori)[3] += (_mat_(K)[15] * _mat_(resid)[3]);
	quat_normalize(&_mat_(x_posteriori)[0]); //renormalize quaternion

	/* update old state variable */
	_mat_(x_priori)[0] = _mat_(x_posteriori)[0];
	_mat_(x_priori)[1] = _mat_(x_posteriori)[1];
	_mat_(x_priori)[2] = _mat_(x_posteriori)[2];
	_mat_(x_priori)[3] = _mat_(x_posteriori)[3];

	/* update covariance matrix */
	_mat_(P)[0] *= (1.0f - _mat_(K)[0]);
	_mat_(P)[5] *= (1.0f - _mat_(K)[5]);
	_mat_(P)[10] *= (1.0f - _mat_(K)[10]);
	_mat_(P)[15] *= (1.0f - _mat_(K)[15]);

	/* convert fused attitude from quaternion to euler angle */
	quat_to_euler(&_mat_(x_posteriori)[0], &ahrs.attitude);
	ahrs.attitude.roll = rad_to_deg(ahrs.attitude.roll);
	ahrs.attitude.pitch = rad_to_deg(ahrs.attitude.pitch);
}

void ahrs_ekf_loop(void)
{
	ahr_ekf_state_predict();
	ahr_ekf_state_update();
}

void ahrs_complementary_filter_loop(void)
{
	/* construct system transition function f */
	float half_q0_dt = 0.5f * _mat_(x_priori)[0] * dt;
	float half_q1_dt = 0.5f * _mat_(x_priori)[1] * dt;
	float half_q2_dt = 0.5f * _mat_(x_priori)[2] * dt;
	float half_q3_dt = 0.5f * _mat_(x_priori)[3] * dt;
	_mat_(f)[0]=-half_q1_dt;
	_mat_(f)[1]=-half_q2_dt;
	_mat_(f)[2]=-half_q3_dt;
	_mat_(f)[3]=+half_q0_dt;
	_mat_(f)[4]=-half_q3_dt;
	_mat_(f)[5]=+half_q2_dt;
	_mat_(f)[6]=+half_q3_dt;
	_mat_(f)[7]=+half_q0_dt;
	_mat_(f)[8]=-half_q1_dt;
	_mat_(f)[9]=-half_q2_dt;
	_mat_(f)[10]=+half_q1_dt;
	_mat_(f)[11]=+half_q0_dt;

	/* angular rate from rate gyro */
	_mat_(w)[0] = deg_to_rad(imu.filtered_gyro.x);
	_mat_(w)[1] = deg_to_rad(imu.filtered_gyro.y);
	_mat_(w)[2] = deg_to_rad(imu.filtered_gyro.z);

	/* rate gyro integration */
	MAT_MULT(&f, &w, &dx); //calculate dx = f * w
	MAT_ADD(&x_priori, &dx, &x_priori);  //calculate x = x + dx
	quat_normalize(&_mat_(x_priori)[0]); //renormalization

	/* convert gravity vector to quaternion */
	float q_gravity[4] = {0};
	vector3d_normalize(&imu.filtered_accel); //normalize acceleromter
	convert_gravity_to_quat(&imu.filtered_accel, q_gravity);

	/* sensors fusion */
	float a = 0.0001f;
	_mat_(x_posteriori)[0] = (_mat_(x_priori)[0] * a) + (q_gravity[0]* (1.0 - a));
	_mat_(x_posteriori)[1] = (_mat_(x_priori)[1] * a) + (q_gravity[1]* (1.0 - a));
	_mat_(x_posteriori)[2] = (_mat_(x_priori)[2] * a) + (q_gravity[2]* (1.0 - a));
	_mat_(x_posteriori)[3] = (_mat_(x_priori)[3] * a) + (q_gravity[3]* (1.0 - a));
	quat_normalize(&_mat_(x_posteriori)[0]);

	/* update state variables for rate gyro */
	_mat_(x_priori)[0] = _mat_(x_posteriori)[0];
	_mat_(x_priori)[1] = _mat_(x_posteriori)[1];
	_mat_(x_priori)[2] = _mat_(x_posteriori)[2];
	_mat_(x_priori)[3] = _mat_(x_posteriori)[3];

	/* convert fused attitude from quaternion to euler angle */
	quat_to_euler(_mat_(x_posteriori), &ahrs.attitude);
	ahrs.attitude.roll = rad_to_deg(ahrs.attitude.roll);
	ahrs.attitude.pitch = rad_to_deg(ahrs.attitude.pitch);
}

void ahrs_init(void)
{
	ahrs_ekf_init();
}

void ahrs_estimate_euler(float *roll, float *pitch, float *yaw)
{
	/* read sensors */
	mpu6500_read_unscaled_data(&imu.unscaled_accel, &imu.unscaled_gyro, &imu.unscaled_temp);
	mpu6500_fix_bias(&imu.unscaled_accel, &imu.unscaled_gyro);
	mpu6500_accel_convert_to_scale(&imu.unscaled_accel, &imu.raw_accel);
	mpu6500_gyro_convert_to_scale(&imu.unscaled_gyro, &imu.raw_gyro);

	/* apply low pass filter */
	lpf(&imu.raw_accel, &accel_lpf_old, &imu.filtered_accel, 0.07);
	lpf(&imu.raw_gyro, &gyro_lpf_old, &imu.filtered_gyro, 0.07);

#if AHRS_SELECT == AHRS_SELECT_EKF
	ahrs_ekf_loop();
#endif

#if AHRS_SELECT == AHRS_SELECT_CF
	ahrs_complementary_filter_loop();
#endif

	attitude_t euler;
	quat_to_euler(&_mat_(x_posteriori)[0], &euler);

	*roll = rad_to_deg(euler.roll);
	*pitch = rad_to_deg(euler.pitch);
	*yaw = rad_to_deg(euler.yaw);
}
