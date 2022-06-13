#ifndef __MULTIROTOR_PID_CTRL_H__
#define __MULTIROTOR_PID_CTRL_H__

#include <stdbool.h>
#include "pid.h"
#include "ahrs.h"
#include "debug_link.h"
#include "imu.h"
#include "sbus_radio.h"

void multirotor_pid_controller_init(void);
void multirotor_pid_control(radio_t *rc);

void motor_control(volatile float throttle_percentage, float throttle_ctrl_precentage, float roll_ctrl_precentage,
                   float pitch_ctrl_precentage, float yaw_ctrl_precentage);

void send_pid_debug_message(debug_msg_t *payload);
void send_motor_debug_message(debug_msg_t *payload);

#endif
