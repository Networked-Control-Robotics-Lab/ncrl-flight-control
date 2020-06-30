#include <stdint.h>
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"
#include "autopilot.h"
#include "sys_time.h"

#define MISSION_TIMEOUT_TIME 1.0f //[s]
#define MISSION_RETRY_TIMES 5

uint16_t waypoint_cnt = 0;
uint16_t waypoint_send_index = 0;
bool send_waypoint_flag = false;

int mission_count_to_receive = 0;
int mission_recept_index = 0;
int received_mission_type = 0;
bool receive_mission_flag = false;
float receive_timeout_start_time = 0.0f;
int receive_retry_times = 0.0f;

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
	                                    waypoint_cnt, received_mission_type);
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

	mavlink_message_t msg;

	/* decode mission count message */
	mavlink_mission_count_t mission_count;
	mavlink_msg_mission_count_decode(received_msg, &mission_count);

	received_mission_type = mission_count.mission_type;

	/* reject mission if waypoint number exceeded maximum acceptable size */
	if(mission_count_to_receive > WAYPOINT_NUM_MAX) {
		/* do ack */
		mavlink_msg_mission_ack_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
		                                  MAV_MISSION_NO_SPACE, received_mission_type);
		send_mavlink_msg_to_uart(&msg);
		return;
	}

	/* clear autopilot waypoint list */
	autopilot_clear_waypoint_list();

	receive_mission_flag = true;
	mission_recept_index = 0;

	/* request for first mission item */
	mavlink_msg_mission_request_int_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
	                mission_recept_index, received_mission_type);
	send_mavlink_msg_to_uart(&msg);

	/* start timeout timer and reset retry counter */
	receive_timeout_start_time = get_sys_time_s();
	receive_retry_times = 0;
}

void mav_mission_item_int(mavlink_message_t *received_msg)
{
	if(receive_mission_flag == false) return;

	int autopilot_retval;
	mavlink_message_t msg;

	/* decode received message */
	mavlink_mission_item_int_t mission_item;
	mavlink_msg_mission_item_int_decode(received_msg, &mission_item);

	if(mission_item.seq == mission_recept_index) {
		/* sequence number is correct, save received mission to the list */
		autopilot_retval =  autopilot_add_new_waypoint_wgs84(
		                            mission_item.x, mission_item.y, mission_item.z);

		/* autopilot rejected incomed mission, closed the protocol */
		if(autopilot_retval != AUTOPILOT_SET_SUCCEED) {
			receive_mission_flag = false;

			/* do ack */
			mavlink_msg_mission_ack_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
			                                  MAV_MISSION_ERROR, received_mission_type);
			send_mavlink_msg_to_uart(&msg);

			return;
		}
	} else {
		/* inconsistent sequence number, re-send the request message */
		mavlink_msg_mission_request_int_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
		                mission_recept_index, received_mission_type);
		send_mavlink_msg_to_uart(&msg);

		/* start timeout timer and reset retry counter */
		receive_timeout_start_time = get_sys_time_s();
		receive_retry_times = 0;
	}

	mission_recept_index++;

	if(mission_recept_index == mission_count_to_receive) {
		/* disable microservice handler and reset variables */
		receive_mission_flag = false;
		mission_count_to_receive = 0;
		mission_recept_index = 0;

		/* do ack */
		mavlink_msg_mission_ack_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
		                                  MAV_MISSION_ACCEPTED, received_mission_type);
		send_mavlink_msg_to_uart(&msg);
	} else {
		/* request for next mission item */
		mavlink_msg_mission_request_int_pack_chan(1, 1, MAVLINK_COMM_1, &msg, 255, 0,
		                mission_recept_index, received_mission_type);
		send_mavlink_msg_to_uart(&msg);

		/* start timeout timer and reset retry counter */
		receive_timeout_start_time = get_sys_time_s();
		receive_retry_times = 0;
	}
}

void mission_waypoint_microservice_handler_out(void)
{
	if(send_waypoint_flag == false) return;

	//float curr_time = get_sys_time_s();

	uint8_t frame = MAV_FRAME_GLOBAL;
	uint16_t command = MAV_GOTO_DO_CONTINUE;
	uint8_t current = 0;
	uint8_t autocontinue = 1;
	float param1 = 0.0f;
	float param2 = 0.0f;
	float param3 = 0.0f;
	float param4 = 0.0f;
	float pos[3] = {0.0f};
	uint8_t mission_type = 	received_mission_type;

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

void mission_waypoint_microservice_handler_in(void)
{
	if(receive_mission_flag == false) return;

	float curr_time = get_sys_time_s();
	if((curr_time - receive_timeout_start_time) > MISSION_TIMEOUT_TIME) {
		/* timeout, send request message */
		if(receive_retry_times <= MISSION_RETRY_TIMES) {
			mavlink_message_t msg;
			mavlink_msg_mission_request_int_pack_chan(
			        1, 1, MAVLINK_COMM_1, &msg, 255, 0,
			        mission_recept_index, received_mission_type);
			send_mavlink_msg_to_uart(&msg);
			receive_retry_times++;
		} else {
			/* exceeded maximum retry time, close the prototcol! */
			receive_mission_flag = false;
		}
	}
}

void mission_waypoint_microservice_handler(void)
{
	mission_waypoint_microservice_handler_out();
	mission_waypoint_microservice_handler_in();
}
