#ifndef __MULTIROTOR_GEOMETRY_H__
#define __MULTIROTOR_GEOMETRY_H__

#include "common_list.h"

enum {
	//... reserved for common parameters
	/*----------------------------------*/
	MR_GEO_GAIN_ROLL_P = COMMON_PARAM_CNT,
	MR_GEO_GAIN_ROLL_D,
	MR_GEO_GAIN_PITCH_P,
	MR_GEO_GAIN_PITCH_D,
	MR_GEO_GAIN_YAW_P,
	MR_GEO_GAIN_YAW_D,
	MR_GEO_GAIN_RATE_ONLY_YAW_D,
	MR_GEO_GAIN_POS_X,
	MR_GEO_GAIN_VEL_X,
	MR_GEO_GAIN_POS_Y,
	MR_GEO_GAIN_VEL_Y,
	MR_GEO_GAIN_POS_Z,
	MR_GEO_GAIN_VEL_Z,
	MR_GEO_GAIN_POS_X_I,
	MR_GEO_GAIN_POS_Y_I,
	MR_GEO_GAIN_POS_Z_I,
	MR_GEO_UAV_MASS,
	MR_GEO_INERTIA_JXX,
	MR_GEO_INERTIA_JYY,
	MR_GEO_INERTIA_JZZ,
	/*----------------------------------*/
	MR_GEO_PARAM_LIST_SIZE
} MULTIROTOR_GEOMETRY_PARAM_ID;

void init_multirotor_geometry_param_list(void);

#endif
