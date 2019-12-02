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
#include "flight_ctl.h"
#include "sys_time.h"

#define FLIGHT_CTL_PRESCALER_RELOAD 10

void rc_safety_protection(void);

extern optitrack_t optitrack;

SemaphoreHandle_t flight_ctl_semphr;

imu_t imu;
ahrs_t ahrs;

pid_control_t pid_roll;
pid_control_t pid_pitch;
pid_control_t pid_yaw_rate;
pid_control_t pid_yaw;
pid_control_t pid_pos_x;
pid_control_t pid_pos_y;
pid_control_t pid_vel_x;
pid_control_t pid_vel_y;
pid_control_t pid_alt;
pid_control_t pid_alt_vel;

//the output command from position_2d_control() have to be converted to body frame
float nav_ctl_roll_command; //body frame x direction control
float nav_ctl_pitch_command; //body frame y direction control

float motor1, motor2, motor3, motor4;
radio_t rc;

void pid_controller_init(void)
{
	/* attitude controllers */
	pid_roll.kp = 0.3f;
	pid_roll.ki = 0.0f;
	pid_roll.kd = 0.09f;

	pid_pitch.kp = 0.3f;
	pid_pitch.ki = 0.0f;
	pid_pitch.kd = 0.09f;

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
	pid_pos_x.kp = 0.35f;
	pid_pos_x.ki = 0.2f;
	pid_pos_x.kd = 0.00f;

	pid_pos_y.kp = 0.35f;
	pid_pos_y.ki = 0.2f;
	pid_pos_y.kd = 0.00f;

	pid_vel_x.kp = 0.035f;
	pid_vel_x.ki = 0.0f;
	pid_vel_x.kd = 0.0f;
	pid_vel_x.output_min = -15.0f;
	pid_vel_x.output_max = +15.0f;

	pid_vel_y.kp = 0.035f;
	pid_vel_y.ki = 0.0f;
	pid_vel_y.kd = 0.0f;
	pid_vel_y.output_min = -15.0f;
	pid_vel_y.output_max = +15.0f;

	pid_alt.kp = 0.25f;
	pid_alt.ki = 0.01f;
	pid_alt.kd = 0.0f;

	pid_alt_vel.kp = 0.1f;
	pid_alt_vel.ki = 0.0f;
	pid_alt_vel.kd = 0.0f;
	pid_alt_vel.output_min = -100.0f;
	pid_alt_vel.output_max = +100.0f;
}

void flight_ctl_semaphore_handler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(flight_ctl_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void rc_mode_change_handler(radio_t *rc)
{
	static int flight_mode_last = FLIGHT_MODE_MANUAL;

	//if switching mode from manual mode to halting
	if(rc->flight_mode == FLIGHT_MODE_HALTING && flight_mode_last == FLIGHT_MODE_MANUAL) {
		pid_alt.enable =
		        pid_alt_vel.enable = true;
		pid_alt.setpoint = optitrack.pos_z;
	}

	//if switching mode from manual mode to navigation
	if(rc->flight_mode == FLIGHT_MODE_HALTING && flight_mode_last == FLIGHT_MODE_MANUAL) {
		pid_alt.enable =true;
		pid_alt_vel.enable = true;
		pid_alt.setpoint = optitrack.pos_z;
		pid_vel_x.enable = true;
		pid_vel_y.enable = true;
	}

	//if switching mode from halting to naviation
	if(rc->flight_mode == FLIGHT_MODE_HALTING && flight_mode_last == FLIGHT_MODE_MANUAL) {
		pid_alt.enable =true;
		pid_alt_vel.enable = true;
		pid_alt.setpoint = optitrack.pos_z;
		pid_vel_x.enable = true;
		pid_vel_y.enable = true;
	}

	//if current mode if maunal
	if(rc->flight_mode == FLIGHT_MODE_MANUAL) {
		pid_alt_vel.enable = false;
		pid_vel_x.enable = false;
		pid_vel_y.enable = false;
		reset_altitude_control_integral(&pid_alt);
		reset_position_2d_control_integral(&pid_pos_x);
		reset_position_2d_control_integral(&pid_pos_y);
	}

	flight_mode_last = rc->flight_mode;
}

void task_flight_ctl(void *param)
{
	euler_t att_euler_est;

	mpu6500_init(&imu);
	motor_init();

	ahrs_init(imu.accel_raw);
	madgwick_t madgwick_ahrs_info;
	madgwick_init(&madgwick_ahrs_info, 400, 0.95);

	pid_controller_init();

	led_off(LED_R);
	led_off(LED_G);
	led_on(LED_B);

	rc_safety_protection();

	while(1) {
		while(xSemaphoreTake(flight_ctl_semphr, 9) == pdFALSE);

		//gpio_toggle(MOTOR7_FREQ_TEST);

		read_rc_info(&rc);
		rc_mode_change_handler(&rc);
#if 1
		ahrs_estimate(&att_euler_est, ahrs.q, imu.accel_lpf, imu.gyro_lpf);
		ahrs.attitude.roll = att_euler_est.roll;
		ahrs.attitude.pitch = att_euler_est.pitch;
#endif

#if 0
		madgwick_imu_ahrs(&madgwick_ahrs_info,
		                  imu.accel_lpf.x,
		                  imu.accel_lpf.y,
		                  imu.accel_lpf.z,
		                  deg_to_rad(imu.gyro_lpf.x),
		                  deg_to_rad(imu.gyro_lpf.y),
		                  deg_to_rad(imu.gyro_lpf.z));

		ahrs.attitude.roll = att_euler_est.roll = madgwick_ahrs_info.Roll;
		ahrs.attitude.pitch = att_euler_est.pitch = madgwick_ahrs_info.Pitch;

		ahrs.q[0] = madgwick_ahrs_info.q0;
		ahrs.q[1] = madgwick_ahrs_info.q1;
		ahrs.q[2] = madgwick_ahrs_info.q2;
		ahrs.q[3] = madgwick_ahrs_info.q3;
#endif

		update_euler_heading_from_optitrack(&optitrack.q[0], &(ahrs.attitude.yaw));

		/* altitude control */
		altitude_control(optitrack.pos_z, optitrack.vel_lpf_z, &pid_alt_vel, &pid_alt);

		/* position control (in ned configuration) */
		float pos_x_set = 0.0f, pos_y_set = 0.0f;
		position_2d_control(optitrack.pos_x, optitrack.vel_lpf_x, pos_x_set, &pid_vel_x, &pid_pos_x);
		position_2d_control(optitrack.pos_y, optitrack.vel_lpf_y, pos_y_set, &pid_vel_y, &pid_pos_y);
		angle_control_cmd_i2b_frame_tramsform(ahrs.attitude.yaw, pid_vel_x.output, pid_vel_y.output,
		                                      &nav_ctl_pitch_command, &nav_ctl_roll_command);

		float final_roll_cmd = -rc.roll;
		float final_pitch_cmd = -rc.pitch;
		if(pid_vel_x.enable == true && pid_vel_y.enable == true && optitrack_available() == true) {
			final_roll_cmd -= nav_ctl_roll_command; //y directional control
			final_pitch_cmd -= nav_ctl_pitch_command; //x directional control
		}

		/* attitude control */
		attitude_pid_control(&pid_roll, att_euler_est.roll, final_roll_cmd, imu.gyro_lpf.x);
		attitude_pid_control(&pid_pitch, att_euler_est.pitch, final_pitch_cmd, imu.gyro_lpf.y);
		yaw_rate_p_control(&pid_yaw_rate, -rc.yaw, imu.gyro_lpf.z); //used if magnetometer/optitrack not performed
		yaw_pd_control(&pid_yaw, rc.yaw, ahrs.attitude.yaw, imu.gyro_lpf.z, 0.0025);

		/* disable control output if sensor not available */
		float yaw_ctrl_output = pid_yaw.output;
		if(optitrack_available() == false) {
			yaw_ctrl_output = pid_yaw_rate.output;
			pid_alt_vel.output = 0.0f;
		}

		if(rc.safety == false) {
			led_on(LED_R);
			led_off(LED_B);
			motor_control(rc.throttle, pid_alt_vel.output, pid_roll.output, pid_pitch.output, yaw_ctrl_output);
		} else {
			led_on(LED_B);
			led_off(LED_R);
			set_yaw_pd_setpoint(&pid_yaw, ahrs.attitude.yaw);
			motor_control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		}

		taskYIELD();
	}
}

void rc_safety_protection(void)
{
	radio_t rc;

	float time_last = 0.0f;
	float time_current = 0.0f;

	led_off(LED_R);
	led_off(LED_G);
	led_off(LED_B);

	do {
		time_current = get_sys_time_ms();
		if(time_current - time_last > 100.0f) {
			led_toggle(LED_R);
			time_last = time_current;
		}
		read_rc_info(&rc);
	} while(rc_safety_check(&rc) == 1);
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
	pid->p_final = pid->kp * -pid->error_current;
	pid->output = pid->p_final;
	bound_float(&pid->output, pid->output_max, pid->output_min);
}

void set_yaw_pd_setpoint(pid_control_t *pid, float new_setpoint)
{
	pid->setpoint = new_setpoint;
}

void yaw_pd_control(pid_control_t *pid, float rc_yaw, float ahrs_yaw, float yaw_rate, float loop_dt)
{
	/* changing setpoint if yaw joystick exceed the +-5 degree zone */
	if(rc_yaw > +5.0f || rc_yaw < -5.0f) {
		pid->setpoint += rc_yaw * loop_dt;
		/* signal bounding */
		if(pid->setpoint > +180.0f) {
			pid->setpoint -= 360.0f;
		} else if(pid->setpoint < -180.0f) {
			pid->setpoint += 360.0f;
		}
	}

	pid->error_current = pid->setpoint - ahrs_yaw;
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
	alt_vel_pid->output = alt_vel_pid->p_final;
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

void position_2d_control(float pos, float vel, float pos_set,
                         pid_control_t *vel_pid, pid_control_t *pos_pid)
{
	pos_pid->error_current = pos_set - pos;
	pos_pid->p_final = pos_pid->kp * pos_pid->error_current;
	pos_pid->error_integral += (pos_pid->error_current * pos_pid->ki * 0.0025);
	bound_float(&pos_pid->error_integral, 50.0f, -50.0f);
	pos_pid->i_final = pos_pid->error_integral;
	pos_pid->output = pos_pid->p_final + pos_pid->i_final;

	float vel_set = pos_pid->output;
	vel_pid->error_current = vel_set - vel;
	vel_pid->p_final = vel_pid->kp * vel_pid->error_current;
	vel_pid->output = vel_pid->p_final;
	bound_float(&vel_pid->output, vel_pid->output_max, vel_pid->output_min);
}

void motor_control(volatile float throttle_percentage, float throttle_ctrl_precentage, float roll_ctrl_precentage,
                   float pitch_ctrl_precentage, float yaw_ctrl_precentage)
{
	float percentage_to_pwm = 0.01 * (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN);

	float power_basis = throttle_percentage * percentage_to_pwm + MOTOR_PULSE_MIN;

	float roll_pwm = roll_ctrl_precentage * (float)percentage_to_pwm;
	float pitch_pwm = pitch_ctrl_precentage * (float)percentage_to_pwm;
	float yaw_pwm = yaw_ctrl_precentage * (float)percentage_to_pwm;
	float throttle_ctrl_pwm = throttle_ctrl_precentage * (float)percentage_to_pwm;

	float m1_pwm, m2_pwm, m3_pwm, m4_pwm;

	/* (ccw)    (cw)
	    m2       m1
	         x
	    m3       m4
	   (cw)    (ccw) */
	m1_pwm = power_basis + throttle_ctrl_pwm + roll_pwm + pitch_pwm - yaw_pwm;
	m2_pwm = power_basis + throttle_ctrl_pwm - roll_pwm + pitch_pwm + yaw_pwm;
	m3_pwm = power_basis + throttle_ctrl_pwm - roll_pwm - pitch_pwm - yaw_pwm;
	m4_pwm = power_basis + throttle_ctrl_pwm + roll_pwm - pitch_pwm + yaw_pwm;
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
