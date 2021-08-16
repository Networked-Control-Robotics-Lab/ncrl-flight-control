#ifndef __DEBUG_MSG_H__
#define __DEBUG_MSG_H__

void send_alt_est_debug_message(debug_msg_t *payload);
void send_ins_sensor_debug_message(debug_msg_t *payload);
void send_ins_raw_position_debug_message(debug_msg_t *payload);
void send_ins_fusion_debug_message(debug_msg_t *payload);
void send_gps_accuracy_debug_message(debug_msg_t *payload);
void send_optitrack_vio_debug_message(debug_msg_t *payload);
void send_gnss_ins_cov_norm_debug_message(debug_msg_t *payload);

#endif
