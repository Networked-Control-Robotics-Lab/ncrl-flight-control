#ifndef __MAV_COMMAND_H__
#define __MAV_COMMAND_H__

#include <stdbool.h>

typedef struct {
	bool send_ack_msg;

	uint8_t ack_target_sys;
	uint16_t ack_cmd;
	uint8_t ack_result;
} cmd_long_msg_manager_t;

void mav_command_long(mavlink_message_t *received_msg);
void command_long_microservice_handler(void);

#endif
