#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "led.h"
#include "sbus_receiver.h"
#include "optitrack.h"
#include "ahrs.h"
#include "matrix.h"
#include "motor_thrust.h"
#include "motor.h"
#include "bound.h"
#include "lpf.h"
#include "imu.h"
#include "ahrs.h"

#define dt 0.0025 //[s]
#define MOTOR_TO_CG_LENGTH 16.25f //[cm]
#define MOTOR_TO_CG_LENGTH_M (MOTOR_TO_CG_LENGTH * 0.01) //[m]
#define COEFFICIENT_YAW 1.0f

extern optitrack_t optitrack;

MAT_ALLOC(J, 3, 3);
MAT_ALLOC(R, 3, 3);
MAT_ALLOC(Rd, 3, 3);
MAT_ALLOC(Rt, 3, 3);
MAT_ALLOC(Rtd, 3, 3);
MAT_ALLOC(RtdR, 3, 3);
MAT_ALLOC(RtRd, 3, 3);
MAT_ALLOC(RtRdWd, 3, 3);
MAT_ALLOC(W, 3, 1);
MAT_ALLOC(W_dot, 3, 1);
MAT_ALLOC(Wd, 3, 1);
MAT_ALLOC(W_hat, 3, 3);
MAT_ALLOC(Wd_dot, 3, 1);
MAT_ALLOC(JW, 3, 1);
MAT_ALLOC(WJW, 3, 1);
MAT_ALLOC(JWdot, 3, 1);
MAT_ALLOC(M, 3, 1);
MAT_ALLOC(eR_mat, 3, 3);
MAT_ALLOC(eR, 3, 1);
MAT_ALLOC(eW, 3, 1);
MAT_ALLOC(WRt, 3, 3);
MAT_ALLOC(WRtRd, 3, 3);
MAT_ALLOC(WRtRdWd, 3, 1);
MAT_ALLOC(RtRdWddot, 3, 1);
MAT_ALLOC(WRtRdWd_RtRdWddot, 3, 1);
MAT_ALLOC(J_WRtRdWd_RtRdWddot, 3, 1);
MAT_ALLOC(inertia_effect, 3, 1);

float krx, kry, krz;
float kwx, kwy, kwz;

float geometry_ctrl_forces[4];

float geometry_ctrl_feedback_moments[3];
float geometry_ctrl_feedfoward_moments[3];

void geometry_ctrl_init(void)
{
	MAT_INIT(J, 3, 3);
	MAT_INIT(R, 3, 3);
	MAT_INIT(Rd, 3, 3);
	MAT_INIT(Rt, 3, 3);
	MAT_INIT(Rtd, 3, 3);
	MAT_INIT(RtdR, 3, 3);
	MAT_INIT(RtRd, 3, 3);
	MAT_INIT(RtRdWd, 3, 3);
	MAT_INIT(W, 3, 1);
	MAT_INIT(W_dot, 3, 1);
	MAT_INIT(Wd, 3, 1);
	MAT_INIT(W_hat, 3, 3);
	MAT_INIT(Wd_dot, 3, 1);
	MAT_INIT(JW, 3, 1);
	MAT_INIT(WJW, 3, 1);
	MAT_INIT(JWdot, 3, 1);
	MAT_INIT(M, 3, 1);
	MAT_INIT(eR_mat, 3, 3);
	MAT_INIT(eR, 3, 1);
	MAT_INIT(eW, 3, 1);
	MAT_INIT(WRt, 3, 3);
	MAT_INIT(WRtRd, 3, 3);
	MAT_INIT(WRtRdWd, 3, 1);
	MAT_INIT(RtRdWddot, 3, 1);
	MAT_INIT(WRtRdWd_RtRdWddot, 3, 1);
	MAT_INIT(J_WRtRdWd_RtRdWddot, 3, 1);
	MAT_INIT(inertia_effect, 3, 1);

	MAT_INIT(J, 3, 3);
	_mat_(J)[0*3 + 0] = 0.01466f; //Ixx [kg*m^2]
	_mat_(J)[1*3 + 1] = 0.01466f; //Iyy [kg*m^2]
	_mat_(J)[2*3 + 2] = 0.02848f; //Izz [kg*m^2]

	/* attitude controller gains of geometry tracking controller */
	krx = 275.0f;
	kry = 275.0f;
	krz = 0.0f;
	kwx = 46.25f;
	kwy = 46.25f;
	kwz = 2750.0f;
}

void euler_to_rotation_matrix(euler_t *euler, float *r, float *r_transpose)
{
	/* check: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles */
	/* R = Rz(psi)Ry(theta)Rx(phi)*/
	float cos_phi = arm_cos_f32(euler->roll);
	float cos_theta = arm_cos_f32(euler->pitch);
	float cos_psi = arm_cos_f32(euler->yaw);
	float sin_phi = arm_sin_f32(euler->roll);
	float sin_theta = arm_sin_f32(euler->pitch);
	float sin_psi = arm_sin_f32(euler->yaw);

	//R
	r[0*3 + 0] = cos_theta * cos_psi;
	r[0*3 + 1] = (-cos_phi * sin_psi) + (sin_phi * sin_theta * cos_psi);
	r[0*3 + 2] = (sin_phi * sin_psi) + (cos_phi * sin_theta * cos_psi);

	r[1*3 + 0] = cos_theta * sin_psi;
	r[1*3 + 1] = (cos_phi * cos_psi) + (sin_phi * sin_theta * sin_psi);
	r[1*3 + 2] = (-sin_phi * cos_psi) + (cos_phi * sin_theta * sin_psi);

	r[2*3 + 0] = -sin_theta;
	r[2*3 + 1] = sin_phi * cos_theta;
	r[2*3 + 2] = cos_phi * cos_theta;

	//transpose(R)
	r_transpose[0*3 + 0] = r[0*3 + 0];
	r_transpose[1*3 + 0] = r[0*3 + 1];
	r_transpose[2*3 + 0] = r[0*3 + 2];

	r_transpose[0*3 + 1] = r[1*3 + 0];
	r_transpose[1*3 + 1] = r[1*3 + 0];
	r_transpose[2*3 + 1] = r[1*3 + 2];

	r_transpose[0*3 + 2] = r[2*3 + 0];
	r_transpose[1*3 + 2] = r[2*3 + 1];
	r_transpose[2*3 + 2] = r[2*3 + 2];
}

void quat_to_rotation_matrix(float *q, float *r, float *r_transpose)
{
	/* check: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles */
	float q1q1 = q[1] * q[1];
	float q2q2 = q[2] * q[2];
	float q3q3 = q[3] * q[3];
	float q1q2 = q[1] * q[2];
	float q0q2 = q[0] * q[2];
	float q0q3 = q[0] * q[3];
	float q1q3 = q[1] * q[3];
	float q2q3 = q[2] * q[3];
	float q0q1 = q[0] * q[1];

	//R
	r[0*3 + 0] = 1.0f - 2.0f * (q2q2 + q3q3);
	r[0*3 + 1] = 2.0f * (q1q2 - q0q3);
	r[0*3 + 2] = 2.0f * (q0q2 - q1q3);

	r[1*3 + 0] = 2.0f * (q1q2 + q0q3);
	r[1*3 + 1] = 1.0f - 2.0f * (q1q1 + q3q3);
	r[1*3 + 2] = 2.0f * (q2q3 - q0q1);

	r[2*3 + 0] = 2.0f * (q1q3 - q0q2);
	r[2*3 + 1] = 2.0f * (q0q1 + q2q3);
	r[2*3 + 2] = 1.0f - 2.0f * (q1q1 + q2q2);

	//transpose(R)
	r_transpose[0*3 + 0] = r[0*3 + 0];
	r_transpose[1*3 + 0] = r[0*3 + 1];
	r_transpose[2*3 + 0] = r[0*3 + 2];

	r_transpose[0*3 + 1] = r[1*3 + 0];
	r_transpose[1*3 + 1] = r[1*3 + 0];
	r_transpose[2*3 + 1] = r[1*3 + 2];

	r_transpose[0*3 + 2] = r[2*3 + 0];
	r_transpose[1*3 + 2] = r[2*3 + 1];
	r_transpose[2*3 + 2] = r[2*3 + 2];
}

void vee_map_3x3(float *mat, float *vec)
{
	vec[0] = mat[2*3 + 1];
	vec[1] = mat[0*3 + 2];
	vec[2] = mat[1*3 + 0];
}

void hat_map_3x3(float *vec, float *mat)
{
	mat[0*3 + 0] = 0.0f;
	mat[0*3 + 1] = -vec[2];
	mat[0*3 + 2] = +vec[1];
	mat[1*3 + 0] = +vec[2];
	mat[1*3 + 1] = 0.0f;
	mat[1*3 + 2] = -vec[0];
	mat[2*3 + 0] = -vec[1];
	mat[2*3 + 1] = +vec[0];
	mat[2*3 + 2] = 0.0f;
}

void cross_product_3x1(float *vec_a, float *vec_b, float *vec_result)
{
	vec_result[0] = vec_a[1]*vec_b[2] - vec_a[2]*vec_b[1];
	vec_result[1] = vec_a[2]*vec_b[0] - vec_a[0]*vec_b[2];
	vec_result[2] = vec_a[0]*vec_b[1] - vec_a[1]*vec_b[0];
}

void estimate_uav_dynamics(float *gyro, float *moments, float *m_rot_frame)
{
	static float angular_vel_last[3] = {0.0f};
	float angular_accel[3];

	angular_accel[0] = (gyro[0] - angular_vel_last[0]) / dt;
	angular_accel[1] = (gyro[1] - angular_vel_last[1]) / dt;
	angular_accel[2] = (gyro[2] - angular_vel_last[2]) / dt;
	angular_vel_last[0] = gyro[0];
	angular_vel_last[1] = gyro[1];
	angular_vel_last[2] = gyro[2];

	lpf(angular_accel[0], &_mat_(W_dot)[0], 0.01);
	lpf(angular_accel[1], &_mat_(W_dot)[1], 0.01);
	lpf(angular_accel[2], &_mat_(W_dot)[2], 0.01);

	//J* W_dot
	MAT_MULT(&J, &W_dot, &JWdot);
	//W x JW
	MAT_MULT(&J, &W, &JW);
	cross_product_3x1(_mat_(W), _mat_(JW), _mat_(WJW));
	//M = J * W_dot + W X (J * W)
	MAT_ADD(&JWdot, &WJW, &M);

	m_rot_frame[0] = _mat_(JWdot)[0];
	m_rot_frame[1] = _mat_(JWdot)[1];
	m_rot_frame[2] = _mat_(JWdot)[2];
	moments[0] = _mat_(M)[0];
	moments[1] = _mat_(M)[1];
	moments[2] = _mat_(M)[2];
}

void geometry_ctrl(euler_t *rc, float *attitude_q, float *gyro, float *output_forces, float *output_moments)
{
	/* XXX:refine this code! */
	/* yaw rate control  */
	float yaw_rate_d = rc->yaw;
	rc->yaw = 0.0f;
	_mat_(Wd)[2] = yaw_rate_d; //yaw rate control

	/* convert attitude (quaternion) to rotation matrix */
	quat_to_rotation_matrix(&attitude_q[0], _mat_(R), _mat_(Rt));

	/* convert radio command (euler angle) to rotation matrix */
	euler_to_rotation_matrix(rc, _mat_(Rd), _mat_(Rtd));

	/* W (angular velocity) */
	_mat_(W)[0] = gyro[0];
	_mat_(W)[1] = gyro[1];
	_mat_(W)[2] = gyro[2];

#if 1
	/* set desired angular velocity to 0 */
	_mat_(Wd)[0] = 0.0f;
	_mat_(Wd)[1] = 0.0f;
	//_mat_(Wd)[2] = 0.0f;
	_mat_(Wd_dot)[0] = 0.0f;
	_mat_(Wd_dot)[1] = 0.0f;
	_mat_(Wd_dot)[2] = 0.0f;
#endif

	/* calculate attitude error eR */
	MAT_MULT(&Rtd, &R, &RtdR);
	MAT_MULT(&Rt, &Rd, &RtRd);
	MAT_SUB(&RtdR, &RtRd, &eR_mat);
	vee_map_3x3(_mat_(eR_mat), _mat_(eR));
	_mat_(eR)[0] *= 0.5f;
	_mat_(eR)[1] *= 0.5f;
	_mat_(eR)[2] *= 0.5f;

	/* calculate attitude rate error eW */
	//MAT_MULT(&Rt, &Rd, &RtRd); //the term is duplicated
	MAT_MULT(&RtRd, &Wd, &RtRdWd);
	MAT_SUB(&W, &RtRdWd, &eW);

#if 1   //without trajectory planning
	/* calculate inertia effect */
	//W x JW
	MAT_MULT(&J, &W, &JW);
	cross_product_3x1(_mat_(W), _mat_(JW), _mat_(WJW));
	_mat_(inertia_effect)[0] = _mat_(WJW)[0] / 0.64;
	_mat_(inertia_effect)[1] = _mat_(WJW)[1] / 0.64;
	_mat_(inertia_effect)[2] = _mat_(WJW)[2] / 0.64;
#endif
#if 0   //with trajectory planning
	//W * R^T * Rd * Wd
	hat_map_3x3(_mat_(W), _mat_(W_hat));
	MAT_MULT(&W_hat, &Rt, &WRt);
	MAT_MULT(&WRt, &Rd, &WRtRd);
	MAT_MULT(&WRtRd, &Wd, &WRtRdWd);
	//R^T * Rd * Wd_dot
	//MAT_MULT(&Rt, &Rd, &RtRd); //the term is duplicated
	MAT_MULT(&RtRd, &Wd_dot, &RtRdWddot);
	//(W * R^T * Rd * Wd) - (R^T * Rd * Wd_dot)
	MAT_SUB(&WRtRdWd, &RtRdWddot, &WRtRdWd_RtRdWddot);
	//J*[(W * R^T * Rd * Wd) - (R^T * Rd * Wd_dot)]
	MAT_MULT(&J, &WRtRdWd_RtRdWddot, &J_WRtRdWd_RtRdWddot);
	//inertia effect = (W x JW) - J*[(W * R^T * Rd * Wd) - (R^T * Rd * Wd_dot)]
	MAT_SUB(&WJW, &J_WRtRdWd_RtRdWddot, &inertia_effect);

#endif
	/* control input M1, M2, M3 */
	output_moments[0] = -krx*_mat_(eR)[0] -kwx*_mat_(eW)[0] + _mat_(inertia_effect)[0];
	output_moments[1] = -kry*_mat_(eR)[1] -kwy*_mat_(eW)[1] + _mat_(inertia_effect)[1];
	output_moments[2] = -krz*_mat_(eR)[2] -kwz*_mat_(eW)[2] + _mat_(inertia_effect)[2];

	/* debug print */
	geometry_ctrl_feedback_moments[0] = (-krx*_mat_(eR)[0] -kwx*_mat_(eW)[0]) * 0.0098f; //gram force to newton
	geometry_ctrl_feedback_moments[1] = (-krx*_mat_(eR)[1] -kwx*_mat_(eW)[1]) * 0.0098f;
	geometry_ctrl_feedback_moments[2] = (-krx*_mat_(eR)[2] -kwx*_mat_(eW)[2]) * 0.0098f;
	geometry_ctrl_feedfoward_moments[0] = _mat_(inertia_effect)[0];
	geometry_ctrl_feedfoward_moments[1] = _mat_(inertia_effect)[1];
	geometry_ctrl_feedfoward_moments[2] = _mat_(inertia_effect)[2];
}

void thrust_allocate_quadrotor(float *motors, float *moments, float force_basis)
{
	static float l_div_4_pos = +0.25f * (1.0f / MOTOR_TO_CG_LENGTH_M);
	static float l_div_4_neg = -0.25f * (1.0f / MOTOR_TO_CG_LENGTH_M);
	static float b_div_4_pos = +0.25f * (1.0f / COEFFICIENT_YAW);
	static float b_div_4_neg = -0.25f * (1.0f / COEFFICIENT_YAW);

	geometry_ctrl_forces[0] = l_div_4_pos * moments[0] + l_div_4_pos * moments[1] + b_div_4_pos * moments[2] + force_basis;
	geometry_ctrl_forces[1] = l_div_4_neg * moments[0] + l_div_4_pos * moments[1] + b_div_4_neg * moments[2] + force_basis;
	geometry_ctrl_forces[2] = l_div_4_neg * moments[0] + l_div_4_neg * moments[1] + b_div_4_pos * moments[2] + force_basis;
	geometry_ctrl_forces[3] = l_div_4_pos * moments[0] + l_div_4_neg * moments[1] + b_div_4_neg * moments[2] + force_basis;

	/* assign motor pwm */
	float percentage_to_pwm = (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN);
	motors[0] = convert_motor_thrust_to_cmd(geometry_ctrl_forces[0]) * percentage_to_pwm + MOTOR_PULSE_MIN;
	motors[1] = convert_motor_thrust_to_cmd(geometry_ctrl_forces[1]) * percentage_to_pwm + MOTOR_PULSE_MIN;
	motors[2] = convert_motor_thrust_to_cmd(geometry_ctrl_forces[2]) * percentage_to_pwm + MOTOR_PULSE_MIN;
	motors[3] = convert_motor_thrust_to_cmd(geometry_ctrl_forces[3]) * percentage_to_pwm + MOTOR_PULSE_MIN;

	bound_float(&motors[0], MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_float(&motors[1], MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_float(&motors[2], MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_float(&motors[3], MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);

	set_motor_pwm_pulse(MOTOR1, (uint16_t)(motors[0]));
	set_motor_pwm_pulse(MOTOR2, (uint16_t)(motors[1]));
	set_motor_pwm_pulse(MOTOR3, (uint16_t)(motors[2]));
	set_motor_pwm_pulse(MOTOR4, (uint16_t)(motors[3]));
}

float uav_dynamics_m[3] = {0.0f};
float uav_dynamics_m_rot_frame[3] = {0.0f};
float motor_cmd[4];

void multirotor_geometry_control(imu_t *imu, ahrs_t *ahrs, radio_t *rc)
{
	//rc_mode_change_handler(rc);

	update_euler_heading_from_optitrack(&optitrack.q[0], &(ahrs->attitude.yaw));

	float control_forces[3], control_moments[3];
	euler_t desired_attitude;
	desired_attitude.roll = deg_to_rad(-rc->roll);
	desired_attitude.pitch = deg_to_rad(-rc->pitch);
	desired_attitude.yaw = deg_to_rad(-rc->yaw);
	float gyro[3];
	gyro[0] = deg_to_rad(imu->gyro_lpf.x);
	gyro[1] = deg_to_rad(imu->gyro_lpf.y);
	gyro[2] = deg_to_rad(imu->gyro_lpf.z);
	float throttle_force = convert_motor_cmd_to_thrust(rc->throttle / 100.0f); //FIXME
	estimate_uav_dynamics(gyro, uav_dynamics_m, uav_dynamics_m_rot_frame);
	geometry_ctrl(&desired_attitude, ahrs->q, gyro, control_forces, control_moments);

	if(rc->safety == false) {
		led_on(LED_R);
		led_off(LED_B);
		thrust_allocate_quadrotor(motor_cmd, control_moments, throttle_force);
	} else {
		led_on(LED_B);
		led_off(LED_R);
		motor_halt();
	}
}
