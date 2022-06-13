#ifndef __MULTIROTOR_PID_PARAM_H__
#define __MULTIROTOR_PID_PARAM_H__

#include "common_list.h"

enum {
	//... reserved for common parameters
	/*----------------------------------*/
	MR_PID_GAIN_ROLL_P = COMMON_PARAM_CNT,
	MR_PID_GAIN_ROLL_D,
	MR_PID_GAIN_PITCH_P,
	MR_PID_GAIN_PITCH_D,
	MR_PID_GAIN_YAW_P,
	MR_PID_GAIN_YAW_D,
	MR_PID_GAIN_RATE_YAW,
	MR_PID_GAIN_POS_X_P,
	MR_PID_GAIN_POS_X_I,
	MR_PID_GAIN_POS_X_D,
	MR_PID_GAIN_POS_Y_P,
	MR_PID_GAIN_POS_Y_I,
	MR_PID_GAIN_POS_Y_D,
	MR_PID_GAIN_POS_Z_P,
	MR_PID_GAIN_POS_Z_I,
	MR_PID_GAIN_POS_Z_D,
	MR_PID_HEIGHT_FEEDFOWARD_PWM,
	/*----------------------------------*/
	MR_PID_PARAM_LIST_SIZE
};

void init_multirotor_pid_param_list(void);

#endif
