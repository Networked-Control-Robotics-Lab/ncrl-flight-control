#include <string.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "delay.h"
#include "bound.h"
#include "motor.h"
#include "ahrs.h"
#include "pid.h"
#include "tracking_ctrl.h"

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
