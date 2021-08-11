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

void ins_eskf_init(float dt);
bool ins_eskf_is_stable(void);
bool ins_eskf_estimate(attitude_t *attitude,
                       float *pos_enu_raw, float *vel_enu_raw,
                       float *pos_enu_fused, float *vel_enu_fused);

void get_ins_eskf_attitude_quaternion(float *q_out);

void send_ins_eskf1_covariance_matrix_debug_message(debug_msg_t *payload);
void send_ins_eskf_correct_freq_debug_message(debug_msg_t *payload);

#endif
