#ifndef __ATTITUDE_PD_CTRL_H__
#define __ATTITUDE_PD_CTRL_H__

#include <stdbool.h>
#include "pid.h"

void attitude_pid_control(pid_control_t *pid, float ahrs_attitude,
                          float setpoint_attitude, float angular_velocity);
void yaw_rate_p_control(pid_control_t *pid, float setpoint_yaw_rate,
                        float angular_velocity);
void set_yaw_pd_setpoint(pid_control_t *pid, float new_setpoint);
void yaw_pd_control(pid_control_t *pid, float rc_yaw, float ahrs_yaw, float yaw_rate, float loop_dt);
void reset_altitude_control_integral(pid_control_t *alt_pid);
void altitude_control(float alt, float alt_vel, pid_control_t *alt_vel_pid, pid_control_t *alt_pid);
void angle_control_cmd_i2b_frame_tramsform(float yaw, float u_i_x, float u_i_y, float *u_b_x, float *u_b_y);
void reset_position_2d_control_integral(pid_control_t *pos_pid);
void position_2d_control(float current_pos, float current_vel, pid_control_t *pos_pid);

#endif
