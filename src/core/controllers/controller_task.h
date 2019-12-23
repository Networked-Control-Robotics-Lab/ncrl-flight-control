#ifndef __CONTROLLER_TASK_H__
#define __CONTROLLER_TASK_H__

void motor_control(volatile float throttle_percentage, float throttle_ctrl_precentage, float roll_ctrl_precentage,
                   float pitch_ctrl_precentage, float yaw_ctrl_precentage);

void task_flight_ctl(void *param);
void flight_ctl_semaphore_handler(void);

#endif
