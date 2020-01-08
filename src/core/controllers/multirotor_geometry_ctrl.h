#ifndef __MULTIROTOR_GEOMETRY_CTRL_H__
#define __MULTIROTOR_GEOMETRY_CTRL_H__

#include "imu.h"
#include "ahrs.h"

void geometry_ctrl_init(void);
void multirotor_geometry_control(imu_t *imu, ahrs_t *ahrs, radio_t *rc);

#endif
