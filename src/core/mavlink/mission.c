#include <stdint.h>
#include "mavlink.h"
#include "../mavlink/publisher.h"

static void mavlink_send_capability(void)
{
	mavlink_message_t msg;

	uint64_t cap = MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
	               MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
	uint32_t flight_sw_version = 1;
	uint32_t middleware_sw_ver = 2;
	uint32_t os_sw_version = 10; //freertos ver. = 10.2.1
	uint32_t board_version = 1;
	uint8_t *flight_custom_ver = NULL;
	uint8_t *middleware_custom_ver = NULL;
	uint8_t *os_custom_ver = NULL;
	uint16_t vendor_id = 10000;
	uint16_t product_id = 10001;
	uint64_t uid = 100;

	mavlink_msg_autopilot_version_pack_chan(1, 1, MAVLINK_COMM_1, &msg, cap, flight_sw_version, middleware_sw_ver,
	                                        os_sw_version, board_version, flight_custom_ver, middleware_custom_ver,
	                                        os_custom_ver, vendor_id, product_id, uid, NULL);
	send_mavlink_msg_to_uart(&msg);
}

void mav_command_long(mavlink_message_t *received_msg)
{
	mavlink_command_long_t mav_command_long;
	mavlink_msg_command_long_decode(received_msg, &mav_command_long);

	switch(mav_command_long.command) {
	case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
		mavlink_send_capability();
		break;
	case MAV_CMD_COMPONENT_ARM_DISARM:
		break;
	}
}

void mav_mission_request_list(mavlink_message_t *received_msg)
{
	mavlink_message_t msg;

	uint16_t waypoint_cnt = 0;
	mavlink_msg_mission_count_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0, waypoint_cnt, MAV_MISSION_TYPE_MISSION);
	send_mavlink_msg_to_uart(&msg);
}
