#include "bound.h"
#include "controller.h"
#include "motor.h"
#include "uart.h"
#include "delay.h"

float motor1, motor2, motor3, motor4;

void attitude_pd_control(pid_control_t *pid, float ahrs_attitude,
                         float setpoint_attitude, float angular_velocity)
{
	//error = reference (setpoint) - measurement
	pid->error_current = setpoint_attitude - ahrs_attitude;

	pid->error_derivative = -angular_velocity; //error_derivative = 0 (setpoint) - measurement_derivative

	pid->p_final = pid->kp * pid->error_current;
	pid->d_final = pid->kd * pid->error_derivative;

	pid->output = pid->p_final + pid->d_final;

	if(pid->output > pid->output_max) pid->output = pid->output_max;
	if(pid->output < pid->output_min) pid->output = pid->output_min;
}

void yaw_rate_p_control(pid_control_t *pid, float setpoint_yaw_rate, float angular_velocity)
{
	pid->error_current = setpoint_yaw_rate - angular_velocity;

	pid->p_final = pid->kp * pid->error_current;
	pid->output = pid->p_final;

	bound_float(&pid->output, pid->output_max, pid->output_min);
}

void motor_control(volatile float throttle_percentage, float roll_ctrl_precentage,
                   float pitch_ctrl_precentage, float yaw_ctrl_precentage)
{
	float percentage_to_pwm = 0.01 * (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN);

	float power_basis = throttle_percentage * percentage_to_pwm + MOTOR_PULSE_MIN;

	float roll_pwm = roll_ctrl_precentage * (float)percentage_to_pwm;
	float pitch_pwm = pitch_ctrl_precentage * (float)percentage_to_pwm;
	float yaw_pwm = yaw_ctrl_precentage * (float)percentage_to_pwm;

	float m1_pwm, m2_pwm, m3_pwm, m4_pwm;

	/* (ccw)    (cw)
	    m2       m1
	         x
	    m3       m4
	   (cw)    (ccw) */
	m1_pwm = power_basis + roll_pwm + pitch_pwm - yaw_pwm;
	m2_pwm = power_basis - roll_pwm + pitch_pwm + yaw_pwm;
	m3_pwm = power_basis - roll_pwm - pitch_pwm - yaw_pwm;
	m4_pwm = power_basis + roll_pwm - pitch_pwm + yaw_pwm;
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
