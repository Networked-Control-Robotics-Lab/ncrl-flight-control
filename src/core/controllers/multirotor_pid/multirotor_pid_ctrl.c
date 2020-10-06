#include <string.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "delay.h"
#include "bound.h"
#include "gpio.h"
#include "uart.h"
#include "pwm.h"
#include "sbus_radio.h"
#include "mpu6500.h"
#include "motor.h"
#include "optitrack.h"
#include "lpf.h"
#include "imu.h"
#include "ahrs.h"
#include "madgwick_ahrs.h"
#include "multirotor_pid_ctrl.h"
#include "multirotor_pid_param.h"
#include "motor_thrust_fitting.h"
#include "flight_ctrl_task.h"
#include "sys_time.h"
#include "proj_config.h"
#include "debug_link.h"
#include "autopilot.h"
#include "sys_param.h"
#include "position_sensor.h"

pid_control_t pid_roll;
pid_control_t pid_pitch;
pid_control_t pid_yaw_rate;
pid_control_t pid_yaw;
pid_control_t pid_pos_x;
pid_control_t pid_pos_y;
pid_control_t pid_alt;
pid_control_t pid_alt_vel;

autopilot_t autopilot;

//the output command from position_2d_control() have to be converted to body frame
float nav_ctl_roll_command; //body frame x direction control
float nav_ctl_pitch_command; //body frame y direction control

float motor1, motor2, motor3, motor4;

void multirotor_pid_controller_init(void)
{
	init_multirotor_pid_param_list();

	autopilot_init(&autopilot);

	float geo_fence_origin[3] = {0.0f, 0.0f, 0.0f};
	autopilot_set_enu_rectangular_fence(geo_fence_origin, 2.5f, 1.3f, 3.0f);

	pid_yaw_rate.output_min = -35.0f;
	pid_yaw_rate.output_max = +35.0f;

	pid_yaw.output_min = -35.0f;
	pid_yaw.output_max = +35.0f;

	pid_pos_x.output_min = -25.0f;
	pid_pos_x.output_max = +25.0f;

	pid_pos_y.output_min = -25.0f;
	pid_pos_y.output_max = +25.0f;

	pid_alt_vel.output_min = -100.0f;
	pid_alt_vel.output_max = +100.0f;

	/* modify local variables when user change them via ground station */
	set_sys_param_update_var_addr(MR_PID_GAIN_ROLL_P, &pid_roll.kp);
	set_sys_param_update_var_addr(MR_PID_GAIN_ROLL_D, &pid_roll.kd);
	set_sys_param_update_var_addr(MR_PID_GAIN_PITCH_P, &pid_pitch.kp);
	set_sys_param_update_var_addr(MR_PID_GAIN_PITCH_D, &pid_pitch.kd);
	set_sys_param_update_var_addr(MR_PID_GAIN_YAW_P, &pid_yaw.kp);
	set_sys_param_update_var_addr(MR_PID_GAIN_YAW_D, &pid_yaw.kd);
	set_sys_param_update_var_addr(MR_PID_GAIN_RATE_YAW, &pid_yaw_rate.kp);
	set_sys_param_update_var_addr(MR_PID_GAIN_POS_X_P, &pid_pos_x.kp);
	set_sys_param_update_var_addr(MR_PID_GAIN_POS_X_I, &pid_pos_x.ki);
	set_sys_param_update_var_addr(MR_PID_GAIN_POS_X_D, &pid_pos_x.kd);
	set_sys_param_update_var_addr(MR_PID_GAIN_POS_Y_P, &pid_pos_y.kp);
	set_sys_param_update_var_addr(MR_PID_GAIN_POS_Y_I, &pid_pos_y.ki);
	set_sys_param_update_var_addr(MR_PID_GAIN_POS_Y_D, &pid_pos_y.kd);
	set_sys_param_update_var_addr(MR_PID_GAIN_POS_Z_P, &pid_alt.kp);
	set_sys_param_update_var_addr(MR_PID_GAIN_POS_Z_I, &pid_alt.ki);
	set_sys_param_update_var_addr(MR_PID_GAIN_POS_Z_D, &pid_alt_vel.kp);
	set_sys_param_update_var_addr(MR_PID_HEIGHT_FEEDFOWARD_PWM, &pid_alt_vel.feedfoward);

	/* modify local variables when user change them via ground station */
	get_sys_param_float(MR_PID_GAIN_ROLL_P, &pid_roll.kp);
	get_sys_param_float(MR_PID_GAIN_ROLL_D, &pid_roll.kd);
	get_sys_param_float(MR_PID_GAIN_PITCH_P, &pid_pitch.kp);
	get_sys_param_float(MR_PID_GAIN_PITCH_D, &pid_pitch.kd);
	get_sys_param_float(MR_PID_GAIN_YAW_P, &pid_yaw.kp);
	get_sys_param_float(MR_PID_GAIN_YAW_D, &pid_yaw.kd);
	get_sys_param_float(MR_PID_GAIN_RATE_YAW, &pid_yaw_rate.kp);
	get_sys_param_float(MR_PID_GAIN_POS_X_P, &pid_pos_x.kp);
	get_sys_param_float(MR_PID_GAIN_POS_X_I, &pid_pos_x.ki);
	get_sys_param_float(MR_PID_GAIN_POS_X_D, &pid_pos_x.kd);
	get_sys_param_float(MR_PID_GAIN_POS_Y_P, &pid_pos_y.kp);
	get_sys_param_float(MR_PID_GAIN_POS_Y_I, &pid_pos_y.ki);
	get_sys_param_float(MR_PID_GAIN_POS_Y_D, &pid_pos_y.kd);
	get_sys_param_float(MR_PID_GAIN_POS_Z_P, &pid_alt.kp);
	get_sys_param_float(MR_PID_GAIN_POS_Z_I, &pid_alt.ki);
	get_sys_param_float(MR_PID_GAIN_POS_Z_D, &pid_alt_vel.kp);
	get_sys_param_float(MR_PID_HEIGHT_FEEDFOWARD_PWM, &pid_alt_vel.feedfoward);
}

void attitude_pid_control(pid_control_t *pid, float ahrs_attitude,
                          float setpoint_attitude, float angular_velocity)
{
	//error = reference (setpoint) - measurement
	pid->error_current = setpoint_attitude - ahrs_attitude;
	pid->error_integral += (pid->error_current * pid->ki * 0.0025);
	bound_float(&pid->error_integral, 10.0f, -10.0f);
	pid->error_derivative = -angular_velocity; //error_derivative = 0 (setpoint) - measurement_derivative
	pid->p_final = pid->kp * pid->error_current;
	pid->i_final = pid->error_integral;
	pid->d_final = pid->kd * pid->error_derivative;
	pid->output = pid->p_final + pid->i_final + pid->d_final;
}

void yaw_rate_p_control(pid_control_t *pid, float setpoint_yaw_rate, float angular_velocity)
{
	pid->error_current = setpoint_yaw_rate - angular_velocity;
	pid->p_final = pid->kp * pid->error_current;
	pid->output = pid->p_final;
	bound_float(&pid->output, pid->output_max, pid->output_min);
}

void set_yaw_pd_setpoint(pid_control_t *pid, float new_setpoint)
{
	pid->setpoint = new_setpoint;
}

void yaw_pd_control(pid_control_t *pid, float desired_heading, float ahrs_yaw, float yaw_rate, float loop_dt)
{
	pid->setpoint = desired_heading;
	pid->error_current = pid->setpoint - ahrs_yaw;

	/* find the shorter way to rotate */
	float compl_angle;
	if(pid->error_current < 0.0f) {
		compl_angle = pid->error_current + 360.0f;
		if(fabs(pid->error_current) > compl_angle) {
			pid->error_current = compl_angle;
		}
	} else if(pid->error_current > 0.0f) {
		compl_angle = pid->error_current - 360.0f;
		if(fabs(compl_angle) < pid->error_current) {
			pid->error_current = compl_angle;
		}
	}

	pid->error_derivative = -yaw_rate;
	pid->p_final = pid->kp * pid->error_current;
	pid->d_final = pid->kd * pid->error_derivative;
	pid->output = pid->p_final + pid->d_final;
	bound_float(&pid->output, pid->output_max, pid->output_min);
}

void reset_altitude_control_integral(pid_control_t *alt_pid)
{
	alt_pid->error_integral = 0.0f;
}

void altitude_control(float alt, float alt_vel, pid_control_t *alt_vel_pid, pid_control_t *alt_pid)
{
	if(alt_vel_pid->enable == false) {
		alt_vel_pid->output = 0.0f;
		return;
	}

	/* altitude control (control output becomes setpoint of velocity controller) */
	alt_pid->error_current = alt_pid->setpoint - alt;
	alt_pid->error_integral += (alt_pid->error_current * alt_pid->ki * 0.0025);
	alt_pid->p_final = alt_pid->kp * alt_pid->error_current;
	alt_pid->i_final = alt_pid->error_integral;
	alt_pid->output = alt_pid->p_final + alt_pid->i_final;

	/* altitude velocity control (control output effects throttle value) */
	float alt_vel_set = alt_pid->output;
	alt_vel_pid->error_current = alt_vel_set - alt_vel;
	alt_vel_pid->p_final = alt_vel_pid->kp * alt_vel_pid->error_current;
	alt_vel_pid->output = alt_vel_pid->p_final + alt_vel_pid->feedfoward;
	bound_float(&alt_vel_pid->output, alt_vel_pid->output_max, alt_vel_pid->output_min);
}

/* tramsform orientation control command from global (inertial) frame to body frame */
void angle_control_cmd_i2b_frame_tramsform(float yaw, float u_i_x, float u_i_y, float *u_b_x, float *u_b_y)
{
	float yaw_rad = deg_to_rad(yaw);
	float cos_neg_psi = arm_cos_f32(-yaw_rad);
	float sin_neg_psi = arm_sin_f32(-yaw_rad);

	*u_b_x = (cos_neg_psi * u_i_x) - (cos_neg_psi * u_i_y);
	*u_b_y = (sin_neg_psi * u_i_x) + (sin_neg_psi * u_i_y);
}

void reset_position_2d_control_integral(pid_control_t *pos_pid)
{
	pos_pid->error_integral = 0.0f;
}

void position_2d_control(float current_pos, float current_vel, pid_control_t *pos_pid)
{
	pos_pid->error_current = pos_pid->setpoint - current_pos;
	pos_pid->p_final = pos_pid->kp * pos_pid->error_current;
	pos_pid->error_integral += (pos_pid->error_current * pos_pid->ki * 0.0025);
	pos_pid->error_derivative = -current_vel;
	pos_pid->d_final = pos_pid->kd * pos_pid->error_derivative;
	bound_float(&pos_pid->error_integral, pos_pid->output_max, pos_pid->output_min);
	pos_pid->i_final = pos_pid->error_integral;
	pos_pid->output = pos_pid->p_final + pos_pid->i_final + pos_pid->d_final;
	bound_float(&pos_pid->output, pos_pid->output_max, pos_pid->output_min);
}

void thrust_pwm_allocate_quadrotor(float throttle_ctrl_precentage, float roll_ctrl_precentage,
                                   float pitch_ctrl_precentage, float yaw_ctrl_precentage)
{
	float percentage_to_pwm = 0.01 * (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN);

	float roll_pwm = roll_ctrl_precentage * (float)percentage_to_pwm;
	float pitch_pwm = pitch_ctrl_precentage * (float)percentage_to_pwm;
	float yaw_pwm = yaw_ctrl_precentage * (float)percentage_to_pwm;
	float throttle_ctrl_pwm = throttle_ctrl_precentage * (float)percentage_to_pwm + MOTOR_PULSE_MIN;

	float m1_pwm, m2_pwm, m3_pwm, m4_pwm;

	/* (ccw)    (cw)
	    m2       m1
	         x
	    m3       m4
	   (cw)    (ccw) */
	m1_pwm = throttle_ctrl_pwm - roll_pwm + pitch_pwm - yaw_pwm;
	m2_pwm = throttle_ctrl_pwm + roll_pwm + pitch_pwm + yaw_pwm;
	m3_pwm = throttle_ctrl_pwm + roll_pwm - pitch_pwm - yaw_pwm;
	m4_pwm = throttle_ctrl_pwm - roll_pwm - pitch_pwm + yaw_pwm;
	bound_float(&m1_pwm, MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_float(&m2_pwm, MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_float(&m3_pwm, MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);
	bound_float(&m4_pwm, MOTOR_PULSE_MAX, MOTOR_PULSE_MIN);

	//generate debug message
	motor1 = (m1_pwm - MOTOR_PULSE_MIN) / percentage_to_pwm;
	motor2 = (m2_pwm - MOTOR_PULSE_MIN) / percentage_to_pwm;
	motor3 = (m3_pwm - MOTOR_PULSE_MIN) / percentage_to_pwm;
	motor4 = (m4_pwm - MOTOR_PULSE_MIN) / percentage_to_pwm;
	bound_float(&motor1, 100.0f, 0.0f);
	bound_float(&motor2, 100.0f, 0.0f);
	bound_float(&motor3, 100.0f, 0.0f);
	bound_float(&motor4, 100.0f, 0.0f);

	set_motor_pwm_pulse(MOTOR1, (uint16_t)m1_pwm);
	set_motor_pwm_pulse(MOTOR2, (uint16_t)m2_pwm);
	set_motor_pwm_pulse(MOTOR3, (uint16_t)m3_pwm);
	set_motor_pwm_pulse(MOTOR4, (uint16_t)m4_pwm);
}

void rc_mode_change_handler_pid(radio_t *rc)
{
	static bool auto_flight_mode_last = false;

	//if mode switched to auto-flight
	if(rc->auto_flight == true && auto_flight_mode_last != true) {
		autopilot_set_mode(AUTOPILOT_HOVERING_MODE);
		/* set position setpoint to current position (enu) */
		float pos_enu[3];
		get_enu_position(pos_enu);
		autopilot.wp_now.pos[0] = pos_enu[0];
		autopilot.wp_now.pos[1] = pos_enu[1];
		autopilot.wp_now.pos[2] = pos_enu[2];

		pid_pos_x.enable = true;
		pid_pos_y.enable = true;
		pid_alt_vel.enable = true;
		pid_alt.enable = true;
		reset_position_2d_control_integral(&pid_pos_x);
		reset_position_2d_control_integral(&pid_pos_y);
	}

	if(rc->auto_flight == false) {
		autopilot_set_mode(AUTOPILOT_MANUAL_FLIGHT_MODE);
		autopilot_mission_reset();
		autopilot.wp_now.pos[0] = 0.0f;
		autopilot.wp_now.pos[1] = 0.0f;
		autopilot.wp_now.pos[2] = 0.0f;

		pid_pos_x.enable = false;
		pid_pos_y.enable = false;
		pid_alt.enable = false;
		pid_alt_vel.enable = false;
		reset_altitude_control_integral(&pid_alt);
		reset_position_2d_control_integral(&pid_pos_x);
		reset_position_2d_control_integral(&pid_pos_y);
	}

	auto_flight_mode_last = rc->auto_flight;
}

void multirotor_pid_control(radio_t *rc, float *desired_heading)
{
	rc_mode_change_handler_pid(rc);

	/* get imu data */
	float accel_lpf[3];
	float gyro_lpf[3];
	get_accel_lpf(accel_lpf);
	get_gyro_lpf(gyro_lpf);

	/* get enu position */
	float pos_enu[3];
	get_enu_position(pos_enu);

	/* get enu velocity */
	float vel_enu[3];
	get_enu_velocity(vel_enu);

	/* get current roll, pitch, yaw angle */
	float attitude_roll, attitude_pitch, attitude_yaw;
	get_attitude_euler_angles(&attitude_roll, &attitude_pitch, &attitude_yaw);

	/* altitude control */
	altitude_control(pos_enu[2], vel_enu[2], &pid_alt_vel, &pid_alt);

	autopilot_update_uav_state(pos_enu, vel_enu);
	autopilot_guidance_handler();
	/* feed position controller setpoint from autopilot (enu) */
	pid_pos_x.setpoint = autopilot.wp_now.pos[0];
	pid_pos_y.setpoint = autopilot.wp_now.pos[1];
	pid_alt.setpoint = autopilot.wp_now.pos[2];

	/* position control (in enu frame) */
	position_2d_control(pos_enu[0], vel_enu[0], &pid_pos_x);
	position_2d_control(pos_enu[1], vel_enu[1], &pid_pos_y);

	angle_control_cmd_i2b_frame_tramsform(attitude_yaw, pid_pos_x.output, pid_pos_y.output,
	                                      &nav_ctl_roll_command, &nav_ctl_pitch_command);

	float final_roll_cmd;
	float final_pitch_cmd;
	if(pid_pos_x.enable == true && pid_pos_y.enable == true && optitrack_available() == true) {
		/* auto-flight (sign difference is cause by enu frame) */
		final_roll_cmd = +nav_ctl_roll_command; //enu-y is contolled by roll
		final_pitch_cmd = -nav_ctl_pitch_command; //enu-x is controlled by pitch
	} else {
		/* manual flight */
		final_roll_cmd = -rc->roll;
		final_pitch_cmd = -rc->pitch;
	}

	float throttle_cmd;
	if(pid_alt_vel.enable == true) {
		/* auto-flight */
		throttle_cmd = pid_alt_vel.output;
	} else {
		/* manual flight */
		throttle_cmd = rc->throttle;
	}

	/* attitude control */
	attitude_pid_control(&pid_roll, attitude_roll, final_roll_cmd, gyro_lpf[0]);
	attitude_pid_control(&pid_pitch, attitude_pitch, final_pitch_cmd, gyro_lpf[1]);
	//used if magnetometer/optitrack not present
	yaw_rate_p_control(&pid_yaw_rate, -rc->yaw, gyro_lpf[2]);
	yaw_pd_control(&pid_yaw, *desired_heading, attitude_yaw, gyro_lpf[2], 0.0025);

	if(rc->safety == true) {
		led_on(LED_B);
		led_off(LED_R);
		set_yaw_pd_setpoint(&pid_yaw, attitude_yaw);
	} else {
		led_on(LED_R);
		led_off(LED_B);
	}

	/* disable control output if sensor not available */
	float yaw_ctrl_output = pid_yaw.output;
	if(optitrack_available() == false) {
		yaw_ctrl_output = pid_yaw_rate.output;
		pid_alt_vel.output = 0.0f;
	}

	bool lock_motor = false;

	//lock motor if throttle values is lower than 10% during manual flight
	lock_motor |= check_motor_lock_condition(rc->throttle < 10.0f &&
	                autopilot_is_manual_flight_mode());
	//lock motor if current height is lower than auto-landing threshold value
	lock_motor |= check_motor_lock_condition(autopilot.wp_now.pos[2] < 0.15f &&
	                autopilot_is_auto_flight_mode());
	//lock motor if motors are locked by autopilot
	lock_motor |= check_motor_lock_condition(autopilot_is_motor_locked_mode());
	//lock motor if radio safety botton is on
	lock_motor |= check_motor_lock_condition(rc->safety == true);
	//lock motor if autopilot locked it
	lock_motor |= check_motor_lock_condition(autopilot_motor_ls_lock() == true);

	if(lock_motor == false) {
		thrust_pwm_allocate_quadrotor(throttle_cmd, pid_roll.output,
		                              pid_pitch.output, yaw_ctrl_output);
	} else {
		motor_halt();
	}
}

void send_pid_debug_message(debug_msg_t *payload)
{
	pid_control_t pid = pid_roll;

	pack_debug_debug_message_header(payload, MESSAGE_ID_PID_DEBUG);
	pack_debug_debug_message_float(&pid.error_current, payload);
	pack_debug_debug_message_float(&pid.error_derivative, payload);
	pack_debug_debug_message_float(&pid.p_final, payload);
	pack_debug_debug_message_float(&pid.i_final, payload);
	pack_debug_debug_message_float(&pid.d_final, payload);
	pack_debug_debug_message_float(&pid.output, payload);
}

void send_motor_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_MOTOR);
	pack_debug_debug_message_float(&motor1, payload);
	pack_debug_debug_message_float(&motor2, payload);
	pack_debug_debug_message_float(&motor3, payload);
	pack_debug_debug_message_float(&motor4, payload);
}
