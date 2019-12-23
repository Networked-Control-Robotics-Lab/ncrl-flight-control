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
#include "attitude_pd_ctrl.h"

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
