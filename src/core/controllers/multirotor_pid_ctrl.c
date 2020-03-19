#include <string.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "delay.h"
#include "bound.h"
#include "led.h"
#include "uart.h"
#include "pwm.h"
#include "sbus_receiver.h"
#include "mpu6500.h"
#include "motor.h"
#include "optitrack.h"
#include "lpf.h"
#include "imu.h"
#include "ahrs.h"
#include "madgwick_ahrs.h"
#include "multirotor_pid_ctrl.h"
#include "multirotor_geometry_ctrl.h"
#include "motor_thrust.h"
#include "fc_task.h"
#include "sys_time.h"
#include "proj_config.h"
#include "debug_link.h"
#include "autopilot.h"

extern optitrack_t optitrack;

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
	autopilot_init(&autopilot);

	float geo_fence_origin[3] = {0.0f, 0.0f, 0.0f};
	autopilot_set_enu_rectangular_fence(geo_fence_origin, 2.5f, 1.3f, 3.0f);

	/* attitude controllers */
	pid_roll.kp = 0.41f;
	pid_roll.ki = 0.0f;
	pid_roll.kd = 0.05f;

	pid_pitch.kp = 0.41f;
	pid_pitch.ki = 0.0f;
	pid_pitch.kd = 0.05f;

	pid_yaw_rate.kp = 0.3f;
	pid_yaw_rate.ki = 0.0f;
	pid_yaw_rate.kd = 0.0f;
	pid_yaw_rate.output_min = -35.0f;
	pid_yaw_rate.output_max = 35.0f;

	pid_yaw.kp = 0.3f;
	pid_yaw.ki = 0.0f;
	pid_yaw.kd = -0.15f;
	pid_yaw.setpoint = 0.0f;
	pid_yaw.output_min = -35.0f;
	pid_yaw.output_max = 35.0f;

	/* positon and velocity controllers */
	pid_pos_x.kp = 0.15f;
	pid_pos_x.ki = 0.006f;
	pid_pos_x.kd = 0.067f;
	pid_pos_x.output_min = -25.0f;
	pid_pos_x.output_max = +25.0f;

	pid_pos_y.kp = 0.15f;
	pid_pos_y.ki = 0.006f;
	pid_pos_y.kd = 0.067f;
	pid_pos_y.output_min = -25.0f;
	pid_pos_y.output_max = +25.0f;

	pid_alt.kp = 3.5f;
	pid_alt.ki = 0.0f;
	pid_alt.kd = 0.0f;

	pid_alt_vel.kp = 0.1f;
	pid_alt_vel.ki = 0.0f;
	pid_alt_vel.kd = 0.0f;
	pid_alt_vel.feedfoward = 45.0f; //% of basis throttle power against the gravity
	pid_alt_vel.output_min = -100.0f;
	pid_alt_vel.output_max = +100.0f;
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

	pid->error_derivative = yaw_rate;
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
	*u_b_x = (arm_cos_f32(-yaw_rad) * u_i_x) - (arm_sin_f32(-yaw_rad) * u_i_y);
	*u_b_y = (arm_sin_f32(-yaw_rad) * u_i_x) + (arm_cos_f32(-yaw_rad) * u_i_y);
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
		/* set position setpoint to current position (enu) */
		autopilot.wp_now.pos[0] = optitrack.pos[0];
		autopilot.wp_now.pos[1] = optitrack.pos[1];
		autopilot.wp_now.pos[2] = optitrack.pos[2];

		pid_pos_x.enable = true;
		pid_pos_y.enable = true;
		pid_alt_vel.enable = true;
		pid_alt.enable = true;
		reset_position_2d_control_integral(&pid_pos_x);
		reset_position_2d_control_integral(&pid_pos_y);
	}

	if(rc->auto_flight == false) {
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

void multirotor_pid_control(imu_t *imu, ahrs_t *ahrs, radio_t *rc, float *desired_heading)
{
	rc_mode_change_handler_pid(rc);

	/* altitude control */
	altitude_control(optitrack.pos[2], optitrack.vel_filtered[2], &pid_alt_vel, &pid_alt);

	autopilot_update_uav_info(optitrack.pos, optitrack.vel_filtered);
	autopilot_waypoint_handler();
	/* feed position controller setpoint from autopilot (enu) */
	pid_pos_x.setpoint = autopilot.wp_now.pos[0];
	pid_pos_y.setpoint = autopilot.wp_now.pos[1];
	pid_alt.setpoint = autopilot.wp_now.pos[2];

	/* position control (in enu frame) */
	position_2d_control(optitrack.pos[0], optitrack.vel_filtered[0], &pid_pos_x);
	position_2d_control(optitrack.pos[1], optitrack.vel_filtered[1], &pid_pos_y);
	angle_control_cmd_i2b_frame_tramsform(ahrs->attitude.yaw, pid_pos_x.output, pid_pos_y.output,
	                                      &nav_ctl_pitch_command, &nav_ctl_roll_command);

	float final_roll_cmd;
	float final_pitch_cmd;
	if(pid_pos_x.enable == true && pid_pos_y.enable == true && optitrack_available() == true) {
		/* auto-flight */
		final_roll_cmd = nav_ctl_pitch_command; //enu-y direction (north) control
		final_pitch_cmd = -nav_ctl_roll_command; //enu-x direction (east) controll  XXX:fix sign!
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
	attitude_pid_control(&pid_roll, ahrs->attitude.roll, final_roll_cmd, imu->gyro_lpf[0]);
	attitude_pid_control(&pid_pitch, ahrs->attitude.pitch, final_pitch_cmd, imu->gyro_lpf[1]);
	yaw_rate_p_control(&pid_yaw_rate, -rc->yaw, imu->gyro_lpf[2]); //used if magnetometer/optitrack not performed
	yaw_pd_control(&pid_yaw, *desired_heading, ahrs->attitude.yaw, imu->gyro_lpf[2], 0.0025);

	/* disable control output if sensor not available */
	float yaw_ctrl_output = pid_yaw.output;
	if(optitrack_available() == false) {
		yaw_ctrl_output = pid_yaw_rate.output;
		pid_alt_vel.output = 0.0f;
	}

	bool halt_motor;
	if((rc->throttle < 10.0f && autopilot.mode == AUTOPILOT_MANUAL_FLIGHT_MODE) ||
	    (autopilot.wp_now.pos[2] < 15.0f && autopilot.mode == AUTOPILOT_TAKEOFF_MODE) ||
	    autopilot.mode == AUTOPILOT_MOTOR_LOCKED_MODE) {
		halt_motor = true;
	} else {
		halt_motor = false;
	}

	if(rc->safety == false) {
		if(halt_motor == false) {
			led_on(LED_R);
			led_off(LED_B);
			thrust_pwm_allocate_quadrotor(throttle_cmd, pid_roll.output, pid_pitch.output, yaw_ctrl_output);
		} else {
			led_on(LED_B);
			led_off(LED_R);
			set_yaw_pd_setpoint(&pid_yaw, ahrs->attitude.yaw);
			motor_halt();
		}
	} else {
		autopilot.mode = AUTOPILOT_MANUAL_FLIGHT_MODE;
		led_on(LED_B);
		led_off(LED_R);
		set_yaw_pd_setpoint(&pid_yaw, ahrs->attitude.yaw);
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
