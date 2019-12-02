#ifndef __FLIGHT_CTL_H__
#define __FLIGHT_CTL_H__

#include <stdbool.h>

typedef struct {
	float kp;
	float ki;
	float kd;
	float p_final;
	float i_final;
	float d_final;
	float setpoint;
	float error_current;
	float error_last;
	float error_integral;
	float error_derivative;
	float output;
	float output_max;
	float output_min;
	bool enable;
} pid_control_t;

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
void position_2d_control(float pos, float vel, float pos_set,
                         pid_control_t *vel_pid, pid_control_t *pos_pid);

void motor_control(volatile float throttle_percentage, float throttle_ctrl_precentage, float roll_ctrl_precentage,
                   float pitch_ctrl_precentage, float yaw_ctrl_precentage);

void task_flight_ctl(void *param);
void flight_ctl_semaphore_handler(void);

#endif
