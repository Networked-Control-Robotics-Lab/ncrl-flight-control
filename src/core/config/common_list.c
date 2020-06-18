#include "sys_param.h"
#include "common_list.h"

void init_common_params(void)
{
	init_sys_param_s16(COMMON_RC_THROTTLE_MAX, "RC_THROTTLE_MAX", 1680);
	init_sys_param_s16(COMMON_RC_THROTTLE_MIN, "RC_THROTTLE_MIN", 368);
	init_sys_param_s16(COMMON_RC_ROLL_MAX, "RC_ROLL_MAX", 1680);
	init_sys_param_s16(COMMON_RC_ROLL_MIN, "RC_ROLL_MIN", 366);
	init_sys_param_s16(COMMON_RC_PITCH_MAX, "RC_PITCH_MAX", 1680);
	init_sys_param_s16(COMMON_RC_PITCH_MIN, "RC_PITCH_MIN", 366);
	init_sys_param_s16(COMMON_RC_YAW_MAX, "RC_YAW_MAX", 1680);
	init_sys_param_s16(COMMON_RC_YAW_MIN, "RC_YAW_MIN", 371);
	init_sys_param_s16(COMMON_RC_SAFETY_MAX, "RC_SAFETY_MAX", 1904);
	init_sys_param_s16(COMMON_RC_SAFETY_MIN, "RC_SAFETY_MIN", 144);
	init_sys_param_s16(COMMON_RC_AUTO_FLIGHT_MAX, "RC_AP_MAX", 1904);
	init_sys_param_s16(COMMON_RC_AUTO_FLIGHT_MIN, "RC_AP_MIN", 144);
	init_sys_param_s16(COMMON_RC_MODE_MAX, "RC_MODE_MAX", 1904);
	init_sys_param_s16(COMMON_RC_MODE_MID, "RC_MODE_MID", 1024);
	init_sys_param_s16(COMMON_RC_MODE_MIN, "RC_MODE_MIN", 144);
}
