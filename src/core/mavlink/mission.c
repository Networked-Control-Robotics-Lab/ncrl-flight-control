#include <stdint.h>
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"
#include "autopilot.h"

uint16_t waypoint_cnt = 0;
uint16_t waypoint_send_index = 0;
bool send_waypoint_flag = false;

int mission_count_to_receive = 0;
int mission_recept_index = 0;
bool receive_mission_flag = false;

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

static void mav_cmd_long_takeoff(void)
{
	autopilot_trigger_auto_takeoff();
}

static void mav_cmd_long_land(void)
{
	autopilot_trigger_auto_landing();
}

static void mav_cmd_long_override_goto(mavlink_command_long_t *cmd_long)
{
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
	case MAV_CMD_NAV_TAKEOFF:
		mav_cmd_long_takeoff();
		break;
	case MAV_CMD_NAV_LAND:
		mav_cmd_long_land();
		break;
	case MAV_CMD_OVERRIDE_GOTO:
		mav_cmd_long_override_goto(&mav_command_long);
		break;
	}
}

void mav_mission_request_list(mavlink_message_t *received_msg)
{
	mavlink_message_t msg;

	waypoint_cnt = 0; //XXX: read waypoint
	mavlink_msg_mission_count_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
	                                    waypoint_cnt, MAV_MISSION_TYPE_MISSION);
	send_mavlink_msg_to_uart(&msg);

	if(waypoint_cnt > 0) {
		//trigger microservice handler
		send_waypoint_flag = true;
	}
}

void mav_mission_count(mavlink_message_t *received_msg)
{
	mission_count_to_receive = mavlink_msg_mission_count_get_count(received_msg);
	if(mission_count_to_receive <= 0) {
		return;
	}

	receive_mission_flag = true;
	mission_recept_index = 0;

	mavlink_message_t msg;
	mavlink_msg_mission_request_int_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
	                mission_recept_index, MAV_MISSION_TYPE_MISSION);
	send_mavlink_msg_to_uart(&msg);

	//XXX: enable microservice handler to handle timeout
}

void mav_mission_item_int(mavlink_message_t *received_msg)
{
	if(receive_mission_flag == false) return;

	//XXX: add new waypoint to the manager

	mavlink_message_t msg;

	mission_recept_index++;
	if(mission_recept_index == mission_count_to_receive) {
		/* disable microservice handler and reset variables */
		receive_mission_flag = false;
		mission_recept_index = 0;

		/* do ack */
		mavlink_msg_mission_ack_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
		                                  MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
		send_mavlink_msg_to_uart(&msg);
	}

	/* request for next mission item */
	mavlink_msg_mission_request_int_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
	                mission_recept_index, MAV_MISSION_TYPE_MISSION);
	send_mavlink_msg_to_uart(&msg);
}

void mission_waypoint_microservice_handler(void)
{
	if(send_waypoint_flag == true) {
		uint8_t frame = MAV_FRAME_GLOBAL;
		uint16_t command = MAV_GOTO_DO_CONTINUE;
		uint8_t current = 0;
		uint8_t autocontinue = 1;
		float param1 = 0.0f;
		float param2 = 0.0f;
		float param3 = 0.0f;
		float param4 = 0.0f;
		float pos[3] = {0.0f};
		uint8_t mission_type = 	MAV_MISSION_TYPE_MISSION;

		mavlink_message_t msg;
		mavlink_msg_mission_item_int_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
		                                       waypoint_send_index, frame, command, current, autocontinue, param1,
		                                       param2, param3, param4, pos[0], pos[1], pos[2], mission_type);
		send_mavlink_msg_to_uart(&msg);

		//XXX: add non-blocked delay

		if(waypoint_send_index == (waypoint_cnt - 1)) {
			waypoint_cnt = 0;
			waypoint_send_index = 0;
			send_waypoint_flag = false;
		} else {
			waypoint_send_index++;
		}
	}
}
