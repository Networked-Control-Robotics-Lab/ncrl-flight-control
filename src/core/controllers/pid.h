#ifndef __PID_H__
#define __PID_H__

#include <stdbool.h>

typedef struct {
	float kp;
	float ki;
	float kd;
	float p_final;
	float i_final;
	float d_final;
	float setpoint;
	float error_current;
	float error_last;
	float error_integral;
	float error_derivative;
	float feedfoward;
	float output;
	float output_max;
	float output_min;
	bool enable;
} pid_control_t;

#endif
