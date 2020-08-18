#ifndef __FLIGHT_CTRL_TASK_H__
#define __FLIGHT_CTRL_TASK_H__

#include "debug_link.h"

void task_flight_ctrl(void *param);
void flight_ctl_semaphore_handler(void);

void send_imu_debug_message(debug_msg_t *payload);
void send_attitude_euler_debug_message(debug_msg_t *payload);
void send_attitude_quaternion_debug_message(debug_msg_t *payload);
void send_attitude_imu_debug_message(debug_msg_t *payload);

#endif
