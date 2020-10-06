#include "sys_param.h"
#include "common_list.h"
#include "multirotor_pid_param.h"

sys_param_data multirotor_pid_param_list[MR_PID_PARAM_LIST_SIZE];

void init_multirotor_pid_param_list(void)
{
	init_sys_param_list(multirotor_pid_param_list, MR_PID_PARAM_LIST_SIZE);

	init_common_params();

	init_sys_param_float(MR_PID_GAIN_ROLL_P, "ROLL_KP", 0.41f);
	init_sys_param_float(MR_PID_GAIN_ROLL_D, "ROLL_KD", 0.05f);
	init_sys_param_float(MR_PID_GAIN_PITCH_P, "PITCH_KP", 0.41f);
	init_sys_param_float(MR_PID_GAIN_PITCH_D, "PITCH_KD", 0.05f);
	init_sys_param_float(MR_PID_GAIN_YAW_P, "YAW_KP", 0.3f);
	init_sys_param_float(MR_PID_GAIN_YAW_D, "YAW_KD", 0.15f);
	init_sys_param_float(MR_PID_GAIN_RATE_YAW, "YAW_RATE_GAIN", 0.3f);
	init_sys_param_float(MR_PID_GAIN_POS_X_P, "POS_X_KP", 15.0f);
	init_sys_param_float(MR_PID_GAIN_POS_X_I, "POS_X_KI", 0.0f);
	init_sys_param_float(MR_PID_GAIN_POS_X_D, "POS_X_KD", 13.0f);
	init_sys_param_float(MR_PID_GAIN_POS_Y_P, "POS_Y_KP", 15.0f);
	init_sys_param_float(MR_PID_GAIN_POS_Y_I, "POS_Y_KI", 0.0f);
	init_sys_param_float(MR_PID_GAIN_POS_Y_D, "POS_Y_KD", 13.0f);
	init_sys_param_float(MR_PID_GAIN_POS_Z_P, "POS_Z_KP", 3.5f);
	init_sys_param_float(MR_PID_GAIN_POS_Z_I, "POS_Z_KI", 0.0f);
	init_sys_param_float(MR_PID_GAIN_POS_Z_D, "POS_Z_KD", 10.0f);
	init_sys_param_float(MR_PID_HEIGHT_FEEDFOWARD_PWM, "HEIGHT_PWM_FF", 45.0f);

	load_param_list_from_flash();
}
