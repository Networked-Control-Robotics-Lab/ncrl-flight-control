#include "sys_param.h"
#include "common_list.h"
#include "multirotor_pid_list.h"

sys_param_data multirotor_pid_param_list[MR_PID_PARAM_LIST_SIZE];

void init_multirotor_pid_param_list(void)
{
	init_sys_param_list(multirotor_pid_param_list, MR_PID_PARAM_LIST_SIZE);

	init_common_params();

	init_sys_param_float(MR_PID_GAIN_ROLL_P, "GAIN_ROLL_P", 0.41f);
	init_sys_param_float(MR_PID_GAIN_ROLL_D, "GAIN_ROLL_D", 0.05f);
	init_sys_param_float(MR_PID_GAIN_PITCH_P, "GAIN_PITCH_P", 0.41f);
	init_sys_param_float(MR_PID_GAIN_PITCH_D, "GAIN_PITCH_D", 0.05f);
	init_sys_param_float(MR_PID_GAIN_YAW_P, "GAIN_YAW_P", 0.3f);
	init_sys_param_float(MR_PID_GAIN_YAW_D, "GAIN_YAW_D", -0.15f);
	init_sys_param_float(MR_PID_GAIN_RATE_ONLY_YAW_D, "GAIN_RATE_ONLY_YAW_D", 0.3f);
	init_sys_param_float(MR_PID_GAIN_POS_X_P, "GAIN_POS_X_P", 0.15f);
	init_sys_param_float(MR_PID_GAIN_POS_X_I, "GAIN_POS_X_I", 0.006f);
	init_sys_param_float(MR_PID_GAIN_POS_X_D, "GAIN_POS_X_D", 0.067f);
	init_sys_param_float(MR_PID_GAIN_POS_Y_P, "GAIN_POS_Y_P", 0.15f);
	init_sys_param_float(MR_PID_GAIN_POS_Y_I, "GAIN_POS_Y_I", 0.006f);
	init_sys_param_float(MR_PID_GAIN_POS_Y_D, "GAIN_POS_Y_D", 0.067f);
	init_sys_param_float(MR_PID_GAIN_POS_Z_P, "GAIN_POS_Z_P", 3.5f);
	init_sys_param_float(MR_PID_GAIN_POS_Z_I, "GAIN_POS_Z_I", 0.0f);
	init_sys_param_float(MR_PID_GAIN_POS_Z_D, "GAIN_POS_Z_D", 0.1f);
	init_sys_param_float(MR_PID_HEIGHT_FEEDFOWARD_PWM, "HEIGHT_FEEDFOWARD_PWM", 45.0f);
}
