#ifndef __MAVLINK_TASK_H__
#define __MAVLINK_TASK_H__

#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"

typedef struct {
	mavlink_message_t mav_msg;
} mavlink_queue_item_t;

typedef struct {
	char status_text[50];
} mavlink_calib_status_text_item_t;

typedef struct {
	int recvd_msg_id;
	float recvd_time;
} mavlink_recpt_record_item_t;

void mavlink_tx_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                              UBaseType_t priority);
void mavlink_rx_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                              UBaseType_t priority);

void send_mavlink_calibration_status_text(char *status_text);

void mavlink_rx_debug_enable(void);
void mavlink_rx_debug_disable(void);
bool get_mavlink_reception_record(int *msg_id, float *recvd_time);

#endif

