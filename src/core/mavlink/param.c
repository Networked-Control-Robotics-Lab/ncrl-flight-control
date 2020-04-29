#include <stdint.h>
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "./publisher.h"

void mav_param_request_list(mavlink_message_t *received_msg)
{
	mavlink_message_t msg;

	char param_name[16] = "test";
	float param_val = 0.0f;
	uint8_t param_type = MAV_PARAM_TYPE_REAL32;
	uint16_t param_cnt = 1;
	uint16_t param_index = 0;

	mavlink_msg_param_value_pack_chan(1, 1, MAVLINK_COMM_1, &msg, param_name, param_val, param_type,
	                                  param_cnt, param_index);
	send_mavlink_msg_to_uart(&msg);
}

void parameter_microservice_handler(void)
{
}
