#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "gpio.h"
#include "sbus_receiver.h"
#include "optitrack.h"
#include "ahrs.h"
#include "matrix.h"
#include "motor_thrust.h"
#include "motor.h"
#include "bound.h"
#include "se3_math.h"
#include "lpf.h"
#include "imu.h"
#include "ahrs.h"
#include "autopilot.h"
#include "debug_link.h"

#define dt 0.0025 //[s]
#define MOTOR_TO_CG_LENGTH 16.25f //[cm]
#define MOTOR_TO_CG_LENGTH_M (MOTOR_TO_CG_LENGTH * 0.01) //[m]
#define COEFFICIENT_YAW 1.0f

#define newton_to_grams_force(n) (n * 101.97f) //[newton * m] to [gram force * m]

extern optitrack_t optitrack;

MAT_ALLOC(J, 3, 3);
MAT_ALLOC(R, 3, 3);
MAT_ALLOC(Rd, 3, 3);
MAT_ALLOC(Rt, 3, 3);
MAT_ALLOC(Rtd, 3, 3);
MAT_ALLOC(RtdR, 3, 3);
MAT_ALLOC(RtRd, 3, 3);
MAT_ALLOC(RtRdWd, 3, 3);
MAT_ALLOC(Re3, 3, 1);
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
MAT_ALLOC(kxex_kvev_mge3_mxd_dot_dot, 3, 1);
MAT_ALLOC(b1d, 3, 1);
MAT_ALLOC(b2d, 3, 1);
MAT_ALLOC(b3d, 3, 1);

float pos_error[3];
float vel_error[3];
float tracking_error_integral[3];

float krx, kry, krz;
float kwx, kwy, kwz;
float kpx, kpy, kpz;
float kvx, kvy, kvz;
float yaw_rate_ctrl_gain;
float k_tracking_i_gain[3];

float uav_mass;

float uav_dynamics_m[3] = {0.0f}; //M = (J * W_dot) + (W X JW)
float uav_dynamics_m_rot_frame[3] = {0.0f}; //M_rot = (J * W_dot)

float curr_pos[3];
float curr_vel[3], desired_vel[3];
float curr_accel[3], desired_accel[3];

autopilot_t autopilot;

bool attitude_manual_height_auto = false;

void geometry_ctrl_init(void)
{
	autopilot_init(&autopilot);

	float geo_fence_origin[3] = {0.0f, 0.0f, 0.0f};
	autopilot_set_enu_rectangular_fence(geo_fence_origin, 2.5f, 1.3f, 3.0f);

	MAT_INIT(J, 3, 3);
	MAT_INIT(R, 3, 3);
	MAT_INIT(Rd, 3, 3);
	MAT_INIT(Rt, 3, 3);
	MAT_INIT(Rtd, 3, 3);
	MAT_INIT(RtdR, 3, 3);
	MAT_INIT(RtRd, 3, 3);
	MAT_INIT(RtRdWd, 3, 3);
	MAT_INIT(Re3, 3, 1);
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
	MAT_INIT(kxex_kvev_mge3_mxd_dot_dot, 3, 1);
	MAT_INIT(b1d, 3, 1);
	MAT_INIT(b2d, 3, 1);
	MAT_INIT(b3d, 3, 1);

	/* uav inertia matrix */
	_mat_(J)[0*3 + 0] = 0.01466f; //Ixx [kg*m^2]
	_mat_(J)[1*3 + 1] = 0.01466f; //Iyy [kg*m^2]
	_mat_(J)[2*3 + 2] = 0.02848f; //Izz [kg*m^2]

	/* uav mass */
	uav_mass = 1150.0f; //[g]

	/* attitude controller */
	/* roll gains */
	krx = 300.0f;
	kwx = 40.25f;
	/* pitch gains */
	kry = 300.0f;
	kwy = 40.25f;
	/* yaw gains */
	krz = 2900.0f;
	kwz = 200.0;
	/* yaw rate gains
	 * if heading sensor is not presented then switch to yaw rate control mode */
	yaw_rate_ctrl_gain = 2750.0f;

	/* tracking controller */
	/* x-axis tracking gains  */
	kpx = 3.6f;
	kvx = 2.2f;
	/* y-axis tracking gains */
	kpy = 3.6f;
	kvy = 2.2f;
	/* z-axis tracking gains */
	kpz = 8.5f;
	kvz = 4.0f;

	k_tracking_i_gain[0] = 0.0f;
	k_tracking_i_gain[1] = 0.0f;
	k_tracking_i_gain[0] = 0.0f;
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

void reset_geometry_tracking_error_integral(void)
{
	tracking_error_integral[0] = 0.0f;
	tracking_error_integral[1] = 0.0f;
	tracking_error_integral[2] = 0.0f;
}

void geometry_manual_ctrl(euler_t *rc, float *attitude_q, float *gyro, float *output_moments, bool heading_present)
{
	/* convert radio command (euler angle) to rotation matrix */
	euler_to_rotation_matrix(rc, _mat_(Rd), _mat_(Rtd));

	/* W (angular velocity) */
	_mat_(W)[0] = gyro[0];
	_mat_(W)[1] = gyro[1];
	_mat_(W)[2] = gyro[2];

	/* set Wd and Wd_dot to 0 since there is no predefined trajectory */
	_mat_(Wd)[0] = 0.0f;
	_mat_(Wd)[1] = 0.0f;
	_mat_(Wd)[2] = 0.0f;
	_mat_(Wd_dot)[0] = 0.0f;
	_mat_(Wd_dot)[1] = 0.0f;
	_mat_(Wd_dot)[2] = 0.0f;

	float _krz, _kwz; //switch between full heading control and yaw rate control

	/* switch to yaw rate control mode if no heading information provided */
	if(heading_present == false) {
		/* yaw rate control only */
		_krz = 0.0f;
		_kwz = yaw_rate_ctrl_gain;
		_mat_(Wd)[2] = rc->yaw; //set yaw rate desired value
	} else {
		_krz = krz;
		_kwz = kwz;
	}

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

	/* calculate the inertia feedfoward term */
	//W x JW
	MAT_MULT(&J, &W, &JW);
	cross_product_3x1(_mat_(W), _mat_(JW), _mat_(WJW));
	_mat_(inertia_effect)[0] = _mat_(WJW)[0];
	_mat_(inertia_effect)[1] = _mat_(WJW)[1];
	_mat_(inertia_effect)[2] = _mat_(WJW)[2];

#if 0   /* inertia feedfoward term for motion planning (trajectory is known) */
	/* calculate inertia effect (trajectory is defined, Wd and Wd_dot are not zero) */
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
	output_moments[0] = -krx*_mat_(eR)[0] -kwx*_mat_(eW)[0] + newton_to_grams_force(_mat_(inertia_effect)[0]);
	output_moments[1] = -kry*_mat_(eR)[1] -kwy*_mat_(eW)[1] + newton_to_grams_force(_mat_(inertia_effect)[1]);
	output_moments[2] = -_krz*_mat_(eR)[2] -_kwz*_mat_(eW)[2] + newton_to_grams_force(_mat_(inertia_effect)[2]);
}

void geometry_tracking_ctrl(euler_t *rc, float *attitude_q, float *gyro, float *curr_pos,
                            float *curr_vel, float *desired_vel, float *curr_accel, float *desired_accel,
                            float *output_moments, float *output_force, bool manual_flight)
{
	/* ex = x - xd */
	float pos_des_ned[3];
	assign_vector_3x1_eun_to_ned(pos_des_ned, autopilot.wp_now.pos);
	pos_error[0] = curr_pos[0] - pos_des_ned[0];
	pos_error[1] = curr_pos[1] - pos_des_ned[1];
	pos_error[2] = curr_pos[2] - pos_des_ned[2];

	/* ev = v - vd */
	//float vel_des_ned[3];
	//assign_vector_3x1_eun_to_ned(vel_des_ned, autopilot.wp_now.vel);
	vel_error[0] = curr_vel[0] - desired_vel[0];
	vel_error[1] = curr_vel[1] - desired_vel[1];
	vel_error[2] = curr_vel[2] - desired_vel[2];

	float force_ff_ned[3];
	force_ff_ned[0] = 0.0f;
	force_ff_ned[1] = 0.0f;
	force_ff_ned[2] = 0.0f;
	//assign_vector_3x1_eun_to_ned(acc_des_ned, autopilot.wp_now.acc_feedforward);
	force_ff_ned[0] = newton_to_grams_force(uav_mass * force_ff_ned[0]);
	force_ff_ned[1] = newton_to_grams_force(uav_mass * force_ff_ned[1]);
	force_ff_ned[2] = newton_to_grams_force(uav_mass * force_ff_ned[2]);

	tracking_error_integral[0] += k_tracking_i_gain[0] * (pos_error[0]) * dt;
	tracking_error_integral[1] += k_tracking_i_gain[1] * (pos_error[1]) * dt;
	tracking_error_integral[2] += k_tracking_i_gain[2] * (pos_error[2]) * dt;

	bound_float(&tracking_error_integral[0], 150, -150);
	bound_float(&tracking_error_integral[1], 150, -150);
	bound_float(&tracking_error_integral[2], 50, -50);

	_mat_(kxex_kvev_mge3_mxd_dot_dot)[0] = -kpx*pos_error[0] - kvx*vel_error[0] +
	                                       force_ff_ned[0] - tracking_error_integral[0];
	_mat_(kxex_kvev_mge3_mxd_dot_dot)[1] = -kpy*pos_error[1] - kvy*vel_error[1] +
	                                       force_ff_ned[1] - tracking_error_integral[1];
	_mat_(kxex_kvev_mge3_mxd_dot_dot)[2] = -kpz*pos_error[2] - kvz*vel_error[2] +
	                                       force_ff_ned[2] - tracking_error_integral[2] - uav_mass;

	/* calculate the denominator of b3d */
	float b3d_denominator; //caution: this term should not be 0
	norm_3x1(_mat_(kxex_kvev_mge3_mxd_dot_dot), &b3d_denominator);
	b3d_denominator = -1.0f / b3d_denominator;

	if(manual_flight == true) {
		/* enable altitude control only, control roll and pitch manually */
		//convert radio command (euler angle) to rotation matrix
		euler_to_rotation_matrix(rc, _mat_(Rd), _mat_(Rtd));
	} else {
		/* enable tracking control for x and y axis */
		//b1d
		_mat_(b1d)[0] = arm_cos_f32(rc->yaw);
		_mat_(b1d)[1] = arm_sin_f32(rc->yaw);
		_mat_(b1d)[2] = 0.0f;
		//b3d = -kxex_kvev_mge3_mxd_dot_dot / ||kxex_kvev_mge3_mxd_dot_dot||
		_mat_(b3d)[0] = _mat_(kxex_kvev_mge3_mxd_dot_dot)[0] * b3d_denominator;
		_mat_(b3d)[1] = _mat_(kxex_kvev_mge3_mxd_dot_dot)[1] * b3d_denominator;
		_mat_(b3d)[2] = _mat_(kxex_kvev_mge3_mxd_dot_dot)[2] * b3d_denominator;
		//b2d = b3d X b1d / ||b3d X b1d||
		cross_product_3x1(_mat_(b3d), _mat_(b1d), _mat_(b2d));
		normalize_3x1(_mat_(b2d));
		/* proj[b1d] = b2d X b3d */
		cross_product_3x1(_mat_(b2d), _mat_(b3d), _mat_(b1d));

		//Rd = [b1d; b3d X b1d; b3d]
		_mat_(Rd)[0*3 + 0] = _mat_(b1d)[0];
		_mat_(Rd)[1*3 + 0] = _mat_(b1d)[1];
		_mat_(Rd)[2*3 + 0] = _mat_(b1d)[2];
		_mat_(Rd)[0*3 + 1] = _mat_(b2d)[0];
		_mat_(Rd)[1*3 + 1] = _mat_(b2d)[1];
		_mat_(Rd)[2*3 + 1] = _mat_(b2d)[2];
		_mat_(Rd)[0*3 + 2] = _mat_(b3d)[0];
		_mat_(Rd)[1*3 + 2] = _mat_(b3d)[1];
		_mat_(Rd)[2*3 + 2] = _mat_(b3d)[2];

		//transpose(Rd)
		_mat_(Rtd)[0*3 + 0] = _mat_(Rd)[0*3 + 0];
		_mat_(Rtd)[1*3 + 0] = _mat_(Rd)[0*3 + 1];
		_mat_(Rtd)[2*3 + 0] = _mat_(Rd)[0*3 + 2];
		_mat_(Rtd)[0*3 + 1] = _mat_(Rd)[1*3 + 0];
		_mat_(Rtd)[1*3 + 1] = _mat_(Rd)[1*3 + 0];
		_mat_(Rtd)[2*3 + 1] = _mat_(Rd)[1*3 + 2];
		_mat_(Rtd)[0*3 + 2] = _mat_(Rd)[2*3 + 0];
		_mat_(Rtd)[1*3 + 2] = _mat_(Rd)[2*3 + 1];
		_mat_(Rtd)[2*3 + 2] = _mat_(Rd)[2*3 + 2];
	}

	/* R * e3 */
	_mat_(Re3)[0] = _mat_(R)[0*3 + 2];
	_mat_(Re3)[1] = _mat_(R)[1*3 + 2];
	_mat_(Re3)[2] = _mat_(R)[2*3 + 2];
	/* f = -(-kx * ex - kv * ev - mge3 + m * x_d_dot_dot) . (R * e3) */
	float neg_kxex_kvev_mge3_mxd_dot_dot[3];
	neg_kxex_kvev_mge3_mxd_dot_dot[0] = -_mat_(kxex_kvev_mge3_mxd_dot_dot)[0];
	neg_kxex_kvev_mge3_mxd_dot_dot[1] = -_mat_(kxex_kvev_mge3_mxd_dot_dot)[1];
	neg_kxex_kvev_mge3_mxd_dot_dot[2] = -_mat_(kxex_kvev_mge3_mxd_dot_dot)[2];
	arm_dot_prod_f32(neg_kxex_kvev_mge3_mxd_dot_dot, _mat_(Re3), 3, output_force);

	/* W (angular velocity) */
	_mat_(W)[0] = gyro[0];
	_mat_(W)[1] = gyro[1];
	_mat_(W)[2] = gyro[2];

	/* set Wd and Wd_dot to 0 since there is no predefined trajectory */
	_mat_(Wd)[0] = 0.0f;
	_mat_(Wd)[1] = 0.0f;
	_mat_(Wd)[2] = 0.0f;
	_mat_(Wd_dot)[0] = 0.0f;
	_mat_(Wd_dot)[1] = 0.0f;
	_mat_(Wd_dot)[2] = 0.0f;

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

	/* calculate the inertia feedfoward term */
	//W x JW
	MAT_MULT(&J, &W, &JW);
	cross_product_3x1(_mat_(W), _mat_(JW), _mat_(WJW));
	_mat_(inertia_effect)[0] = _mat_(WJW)[0];
	_mat_(inertia_effect)[1] = _mat_(WJW)[1];
	_mat_(inertia_effect)[2] = _mat_(WJW)[2];

	/* control input M1, M2, M3 */
	output_moments[0] = -krx*_mat_(eR)[0] -kwx*_mat_(eW)[0] + newton_to_grams_force(_mat_(inertia_effect)[0]);
	output_moments[1] = -kry*_mat_(eR)[1] -kwy*_mat_(eW)[1] + newton_to_grams_force(_mat_(inertia_effect)[1]);
	output_moments[2] = -krz*_mat_(eR)[2] -kwz*_mat_(eW)[2] + newton_to_grams_force(_mat_(inertia_effect)[2]);
}

#define l_div_4_pos (+0.25f * (1.0f / MOTOR_TO_CG_LENGTH_M))
#define l_div_4_neg (-0.25f * (1.0f / MOTOR_TO_CG_LENGTH_M))
#define b_div_4_pos (+0.25f * (1.0f / COEFFICIENT_YAW))
#define b_div_4_neg (-0.25f * (1.0f / COEFFICIENT_YAW))
void thrust_force_allocate_quadrotor(float *moment, float total_force)
{
	float motor_pwm[4], motor_force[4];

	float distributed_force = total_force *= 0.25; //split force to 4 motors

	motor_force[0] = l_div_4_neg * moment[0] + l_div_4_pos * moment[1] + b_div_4_neg * moment[2] + distributed_force;
	motor_force[1] = l_div_4_pos * moment[0] + l_div_4_pos * moment[1] + b_div_4_pos * moment[2] + distributed_force;
	motor_force[2] = l_div_4_pos * moment[0] + l_div_4_neg * moment[1] + b_div_4_neg * moment[2] + distributed_force;
	motor_force[3] = l_div_4_neg * moment[0] + l_div_4_neg * moment[1] + b_div_4_pos * moment[2] + distributed_force;

	/* convert force to pwm */
	float percentage_to_pwm = (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN);
	motor_pwm[0] = convert_motor_thrust_to_cmd(motor_force[0]) * percentage_to_pwm + MOTOR_PULSE_MIN;
	motor_pwm[1] = convert_motor_thrust_to_cmd(motor_force[1]) * percentage_to_pwm + MOTOR_PULSE_MIN;
	motor_pwm[2] = convert_motor_thrust_to_cmd(motor_force[2]) * percentage_to_pwm + MOTOR_PULSE_MIN;
	motor_pwm[3] = convert_motor_thrust_to_cmd(motor_force[3]) * percentage_to_pwm + MOTOR_PULSE_MIN;

	bound_float(&motor_pwm[0], MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_float(&motor_pwm[1], MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_float(&motor_pwm[2], MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_float(&motor_pwm[3], MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);

	set_motor_pwm_pulse(MOTOR1, (uint16_t)(motor_pwm[0]));
	set_motor_pwm_pulse(MOTOR2, (uint16_t)(motor_pwm[1]));
	set_motor_pwm_pulse(MOTOR3, (uint16_t)(motor_pwm[2]));
	set_motor_pwm_pulse(MOTOR4, (uint16_t)(motor_pwm[3]));
}

void rc_mode_change_handler_geometry(radio_t *rc)
{
	static bool auto_flight_mode_last = false;

	//if mode switched to auto-flight
	if(rc->auto_flight == true && auto_flight_mode_last != true) {
		autopilot_set_mode(AUTOPILOT_HOVERING_MODE);
		/* set position setpoint to current position (enu) */
		autopilot.wp_now.pos[0] = optitrack.pos[0];
		autopilot.wp_now.pos[1] = optitrack.pos[1];
		autopilot.wp_now.pos[2] = optitrack.pos[2];
		desired_vel[0] = 0.0f;
		desired_vel[1] = 0.0f;
		desired_vel[2] = 0.0f;
		desired_accel[0] = 0.0f;
		desired_accel[1] = 0.0f;
		desired_accel[2] = 0.0f;
		reset_geometry_tracking_error_integral();
	}

	if(rc->auto_flight == false) {
		autopilot_set_mode(AUTOPILOT_MANUAL_FLIGHT_MODE);
		autopilot_mission_reset();
		autopilot.wp_now.pos[0] = 0.0f;
		autopilot.wp_now.pos[1] = 0.0f;
		autopilot.wp_now.pos[2] = 0.0f;
		desired_vel[0] = 0.0f;
		desired_vel[1] = 0.0f;
		desired_vel[2] = 0.0f;
		desired_accel[0] = 0.0f;
		desired_accel[1] = 0.0f;
		desired_accel[2] = 0.0f;
		reset_geometry_tracking_error_integral();
	}

	auto_flight_mode_last = rc->auto_flight;
}

void multirotor_geometry_control(imu_t *imu, ahrs_t *ahrs, radio_t *rc, float *desired_heading)
{
	rc_mode_change_handler_geometry(rc);

	bool optitrack_present = optitrack_available();

	euler_t desired_attitude;
	if(optitrack_present == true) {
		desired_attitude.roll = deg_to_rad(-rc->roll);
		desired_attitude.pitch = deg_to_rad(-rc->pitch);
		desired_attitude.yaw = deg_to_rad(*desired_heading); //yaw rate controller
	} else {
		desired_attitude.roll = deg_to_rad(-rc->roll);
		desired_attitude.pitch = deg_to_rad(-rc->pitch);
		desired_attitude.yaw = deg_to_rad(-rc->yaw); //heading controller
	}

	float gyro[3];
	gyro[0] = deg_to_rad(imu->gyro_lpf[0]);
	gyro[1] = deg_to_rad(imu->gyro_lpf[1]);
	gyro[2] = deg_to_rad(imu->gyro_lpf[2]);

#if 0
	estimate_uav_dynamics(gyro, uav_dynamics_m, uav_dynamics_m_rot_frame);
#endif

	/* convert attitude (quaternion) to rotation matrix */
	quat_to_rotation_matrix(ahrs->q, _mat_(R), _mat_(Rt));

	float control_moments[3] = {0.0f}, control_force = 0.0f;
	if(rc->auto_flight == true && optitrack_present == true) {
		autopilot_update_uav_state(optitrack.pos, optitrack.vel_filtered);
		autopilot_waypoint_handler();

		assign_vector_3x1_eun_to_ned(curr_pos, optitrack.pos);
		assign_vector_3x1_eun_to_ned(curr_vel, optitrack.vel_filtered);
		geometry_tracking_ctrl(&desired_attitude, ahrs->q, gyro, curr_pos,
		                       curr_vel, desired_vel, curr_accel, desired_accel, control_moments,
		                       &control_force, attitude_manual_height_auto);
	} else {
		geometry_manual_ctrl(&desired_attitude, ahrs->q, gyro, control_moments, optitrack_present);

		/* generate total thrust for quadrotor using rc in manual mode */
		control_force = 4.0f * convert_motor_cmd_to_thrust(rc->throttle / 100.0f); //FIXME
	}

	bool halt_motor;
	if((rc->throttle < 10.0f && autopilot_get_mode() == AUTOPILOT_MANUAL_FLIGHT_MODE) ||
	    (autopilot.wp_now.pos[2] < 15.0f && autopilot_get_mode() != AUTOPILOT_MANUAL_FLIGHT_MODE) ||
	    autopilot_get_mode() == AUTOPILOT_MOTOR_LOCKED_MODE) {
		halt_motor = true;
	} else {
		halt_motor = false;
	}

	if(rc->safety == false) {
		if(halt_motor == false) {
			led_on(LED_B);
			led_off(LED_R);
			thrust_force_allocate_quadrotor(control_moments, control_force);
		} else {
			led_on(LED_B);
			led_off(LED_R);
			*desired_heading = ahrs->attitude.yaw;
			motor_halt();
		}
	} else {
		if(rc->auto_flight == true) {
			autopilot_set_mode(AUTOPILOT_HOVERING_MODE);
		} else {
			autopilot_set_mode(AUTOPILOT_MANUAL_FLIGHT_MODE);
		}
		led_on(LED_B);
		led_off(LED_R);
		*desired_heading = ahrs->attitude.yaw;
		motor_halt();
	}
}

void send_geometry_moment_ctrl_debug(debug_msg_t *payload)
{
	float roll_error = rad_to_deg(_mat_(eR)[0]);
	float pitch_error = rad_to_deg(_mat_(eR)[1]);
	float yaw_error = rad_to_deg(_mat_(eR)[2]);

	float wx_error = rad_to_deg(_mat_(eW)[0]);
	float wy_error = rad_to_deg(_mat_(eW)[1]);
	float wz_error = rad_to_deg(_mat_(eW)[2]);

	float geometry_ctrl_feedback_moments[3];
	float geometry_ctrl_feedfoward_moments[3];

	/* calculate the feedback moment and convert the unit from [gram force * m] to [newton * m] */
	geometry_ctrl_feedback_moments[0] = (-krx*_mat_(eR)[0] -kwx*_mat_(eW)[0]) * 0.0098f;
	geometry_ctrl_feedback_moments[1] = (-krx*_mat_(eR)[1] -kwx*_mat_(eW)[1]) * 0.0098f;
	geometry_ctrl_feedback_moments[2] = (-krx*_mat_(eR)[2] -kwx*_mat_(eW)[2]) * 0.0098f;

	geometry_ctrl_feedfoward_moments[0] = _mat_(inertia_effect)[0];
	geometry_ctrl_feedfoward_moments[1] = _mat_(inertia_effect)[1];
	geometry_ctrl_feedfoward_moments[2] = _mat_(inertia_effect)[2];

	pack_debug_debug_message_header(payload, MESSAGE_ID_GEOMETRY_MOMENT_CTRL);
	pack_debug_debug_message_float(&roll_error, payload);
	pack_debug_debug_message_float(&pitch_error, payload);
	pack_debug_debug_message_float(&yaw_error, payload);
	pack_debug_debug_message_float(&wx_error, payload);
	pack_debug_debug_message_float(&wy_error, payload);
	pack_debug_debug_message_float(&wz_error, payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedback_moments[0], payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedback_moments[1], payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedback_moments[2], payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedfoward_moments[0], payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedfoward_moments[1], payload);
	pack_debug_debug_message_float(&geometry_ctrl_feedfoward_moments[2], payload);
}

void send_geometry_tracking_ctrl_debug(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_GEOMETRY_TRACKING_CTRL);
	pack_debug_debug_message_float(&pos_error[0], payload);
	pack_debug_debug_message_float(&pos_error[1], payload);
	pack_debug_debug_message_float(&pos_error[2], payload);
}

void send_uav_dynamics_debug(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_UAV_DYNAMICS_DEBUG);
	pack_debug_debug_message_float(&uav_dynamics_m[0], payload);
	pack_debug_debug_message_float(&uav_dynamics_m[1], payload);
	pack_debug_debug_message_float(&uav_dynamics_m[2], payload);
	pack_debug_debug_message_float(&uav_dynamics_m_rot_frame[0], payload);
	pack_debug_debug_message_float(&uav_dynamics_m_rot_frame[1], payload);
	pack_debug_debug_message_float(&uav_dynamics_m_rot_frame[2], payload);
}
