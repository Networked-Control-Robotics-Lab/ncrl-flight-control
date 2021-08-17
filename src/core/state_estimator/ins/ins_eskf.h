#ifndef __INS_ESKF_H__
#define __INS_ESKF_H__

#include "ahrs.h"

typedef struct {
	float mag_correct_freq;
	float mag_time_last;

	float baro_correct_freq;
	float baro_time_last;

	float rangefinder_correct_freq;
	float rangefinder_time_last;

	float gps_correct_freq;
	float gps_time_last;
} ins_eskf_t;

void ins_eskf_reset_process_covariance_matrix(void);
void ins_eskf_init(float dt);
float ins_eskf_get_covariance_matrix_norm(void);
bool ins_eskf_is_stable(void);
void ins_eskf_estimate(attitude_t *attitude,
                       float *pos_ned_raw, float *vel_ned_raw,
                       float *pos_ned_fused, float *vel_ned_fused);

void ins_eskf_get_attitude_quaternion(float *q);
void ins_eskf_get_position_ned(float *pos);
void ins_eskf_get_velocity_ned(float *vel);

void send_ins_eskf1_covariance_matrix_debug_message(debug_msg_t *payload);
void send_ins_eskf_correct_freq_debug_message(debug_msg_t *payload);

#endif
