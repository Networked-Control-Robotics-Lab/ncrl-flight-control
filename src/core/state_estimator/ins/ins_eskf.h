#ifndef __INS_ESKF_H__
#define __INS_ESKF_H__

void eskf_ins_init(float dt);
bool ins_eskf_estimate(attitude_t *attitude,
                       float *pos_enu_raw, float *vel_enu_raw,
                       float *pos_enu_fused, float *vel_enu_fused);

void get_eskf_ins_attitude_quaternion(float *q_out);

void send_ins_eskf1_covariance_matrix_debug_message(debug_msg_t *payload);

#endif
