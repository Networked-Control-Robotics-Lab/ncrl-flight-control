#include "sys_param.h"
#include "common_list.h"
#include "multirotor_geometry_list.h"

sys_param_data multirotor_geometry_param_list[MR_GEO_PARAM_LIST_SIZE];

void init_multirotor_geometry_param_list(void)
{
	init_sys_param_list(multirotor_geometry_param_list, MR_GEO_PARAM_LIST_SIZE);

	init_common_params();

	init_sys_param_float(MR_GEO_GAIN_ROLL_P, "GAIN_ROLL_P", 300.0f);
	init_sys_param_float(MR_GEO_GAIN_ROLL_D, "GAIN_ROLL_D", 40.25f);
	init_sys_param_float(MR_GEO_GAIN_PITCH_P, "GAIN_PITCH_P", 300.0f);
	init_sys_param_float(MR_GEO_GAIN_PITCH_D, "GAIN_PITCH_D", 40.25f);
	init_sys_param_float(MR_GEO_GAIN_YAW_P, "GAIN_YAW_P", 2900.0f);
	init_sys_param_float(MR_GEO_GAIN_YAW_D, "GAIN_YAW_D", 200.0);
	init_sys_param_float(MR_GEO_GAIN_RATE_YAW, "GAIN_RATE_YAW", 2750.0f);
	init_sys_param_float(MR_GEO_GAIN_POS_X, "GAIN_POS_X", 3.6f);
	init_sys_param_float(MR_GEO_GAIN_VEL_X, "GAIN_VEL_X", 2.2f);
	init_sys_param_float(MR_GEO_GAIN_POS_Y, "GAIN_POS_Y", 3.6f);
	init_sys_param_float(MR_GEO_GAIN_VEL_Y, "GAIN_VEL_Y", 2.2f);
	init_sys_param_float(MR_GEO_GAIN_POS_Z, "GAIN_POS_Z", 8.5f);
	init_sys_param_float(MR_GEO_GAIN_VEL_Z, "GAIN_VEL_Z", 4.0f);
	init_sys_param_float(MR_GEO_GAIN_POS_X_I, "GAIN_POS_X_I", 0.0f);
	init_sys_param_float(MR_GEO_GAIN_POS_Y_I, "GAIN_POS_Y_I", 0.0f);
	init_sys_param_float(MR_GEO_GAIN_POS_Z_I, "GAIN_POS_Z_I", 0.0f);
	init_sys_param_float(MR_GEO_UAV_MASS, "UAV_MASS", 1150.0f);
	init_sys_param_float(MR_GEO_INERTIA_JXX, "INERTIA_JXX", 0.01466f);
	init_sys_param_float(MR_GEO_INERTIA_JYY, "INERTIA_JYY", 0.01466f);
	init_sys_param_float(MR_GEO_INERTIA_JZZ, "INERTIA_JZZ", 0.02848f);

	load_param_list_from_flash();
}
