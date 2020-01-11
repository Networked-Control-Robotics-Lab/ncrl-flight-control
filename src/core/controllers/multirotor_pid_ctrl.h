#ifndef __MULTIROTOR_PID_CTRL_H__
#define __MULTIROTOR_PID_CTRL_H__

#include <stdbool.h>
#include "pid.h"
#include "ahrs.h"

void multirotor_pid_controller_init(void);
void multirotor_pid_control(imu_t *imu, ahrs_t *ahrs, radio_t *rc, float desired_heading);

void motor_control(volatile float throttle_percentage, float throttle_ctrl_precentage, float roll_ctrl_precentage,
		   float pitch_ctrl_precentage, float yaw_ctrl_precentage);

#endif
