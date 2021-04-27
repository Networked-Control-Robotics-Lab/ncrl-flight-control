#include "sys_param.h"
#include "common_list.h"
#include "multirotor_geometry_param.h"

sys_param_data multirotor_geometry_param_list[MR_GEO_PARAM_LIST_SIZE];

void init_multirotor_geometry_param_list(void)
{
	init_sys_param_list(multirotor_geometry_param_list, MR_GEO_PARAM_LIST_SIZE);

	init_common_params();

	init_sys_param_float(MR_GEO_GAIN_ROLL_P, "ROLL_KP", 2.95f);
	init_sys_param_float(MR_GEO_GAIN_ROLL_D, "ROLL_KD", 0.36f);
	init_sys_param_float(MR_GEO_GAIN_PITCH_P, "PITCH_KP", 2.95f);
	init_sys_param_float(MR_GEO_GAIN_PITCH_D, "PITCH_KD", 0.36f);
	init_sys_param_float(MR_GEO_GAIN_YAW_P, "YAW_KP", 28.4f);
	init_sys_param_float(MR_GEO_GAIN_YAW_D, "YAW_KD", 1.96f);
	init_sys_param_float(MR_GEO_GAIN_RATE_YAW, "YAW_RATE_GAIN", 26.9f);
	init_sys_param_float(MR_GEO_GAIN_POS_X, "POS_X_KP", 9.31f);
	init_sys_param_float(MR_GEO_GAIN_VEL_X, "POS_X_KD", 6.86f);
	init_sys_param_float(MR_GEO_GAIN_POS_Y, "POS_Y_KP", 11.27f);
	init_sys_param_float(MR_GEO_GAIN_VEL_Y, "POS_Y_KD", 7.84f);
	init_sys_param_float(MR_GEO_GAIN_POS_Z, "POS_Z_KP", 8.53f);
	init_sys_param_float(MR_GEO_GAIN_VEL_Z, "POS_Z_KD", 3.92f);
	init_sys_param_float(MR_GEO_GAIN_POS_X_I, "POS_I_GAIN_X", 0.0f);
	init_sys_param_float(MR_GEO_GAIN_POS_Y_I, "POS_I_GAIN_Y", 0.0f);
	init_sys_param_float(MR_GEO_GAIN_POS_Z_I, "POS_I_GAIN_Z", 0.0f);
	init_sys_param_float(MR_GEO_UAV_MASS, "UAV_MASS", 1.15f);
	init_sys_param_float(MR_GEO_INERTIA_JXX, "INERTIA_JXX", 0.01466f);
	init_sys_param_float(MR_GEO_INERTIA_JYY, "INERTIA_JYY", 0.01466f);
	init_sys_param_float(MR_GEO_INERTIA_JZZ, "INERTIA_JZZ", 0.02848f);
	init_sys_param_float(PWM_TO_THRUST_C1, "PWM_TO_THRUST_C1", -2842.8f);
	init_sys_param_float(PWM_TO_THRUST_C2, "PWM_TO_THRUST_C2", 3951.7f);
	init_sys_param_float(PWM_TO_THRUST_C3, "PWM_TO_THRUST_C3", -1925.4f);
	init_sys_param_float(PWM_TO_THRUST_C4, "PWM_TO_THRUST_C4", 1381.3f);
	init_sys_param_float(PWM_TO_THRUST_C5, "PWM_TO_THRUST_C5", 257.37f);
	init_sys_param_float(PWM_TO_THRUST_C6, "PWM_TO_THRUST_C6", -7.0118f);
	init_sys_param_float(THRUST_TO_PWM_C1, "THRUST_TO_PWM_C1", 1.169e-14);
	init_sys_param_float(THRUST_TO_PWM_C2, "THRUST_TO_PWM_C2", -2.264e-11);
	init_sys_param_float(THRUST_TO_PWM_C3, "THRUST_TO_PWM_C3", 1.697e-08);
	init_sys_param_float(THRUST_TO_PWM_C4, "THRUST_TO_PWM_C4", -6.715e-06);
	init_sys_param_float(THRUST_TO_PWM_C5, "THRUST_TO_PWM_C5", 2.336e-03);
	init_sys_param_float(THRUST_TO_PWM_C6, "THRUST_TO_PWM_C6", 3.082e-02);
	init_sys_param_float(THRUST_MAX, "THRUST_MAX", 845.0f);

	load_param_list_from_flash();
}
