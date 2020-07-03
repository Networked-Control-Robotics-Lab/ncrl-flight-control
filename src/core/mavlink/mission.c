#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"
#include "autopilot.h"
#include "sys_time.h"
#include "mission.h"
#include "delay.h"

#define MISSION_TIMEOUT_TIME 2.0f //[s]
#define MISSION_RETRY_TIMES 5

mavlink_mission_manager mission_manager;

/****************************
 * mavlink message handlers *
 ****************************/
static void mavlink_send_capability(void)
{
	mavlink_message_t msg;

	uint64_t cap = MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
	               MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
	uint32_t flight_sw_version = 18000000;
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

static void mav_cmd_preflight_calibration(mavlink_message_t *received_msg,
                mavlink_command_long_t *cmd_long)
{
	uint16_t command = MAV_CMD_PREFLIGHT_CALIBRATION;
	uint8_t result = MAV_RESULT_ACCEPTED;
	uint8_t param1 = 0;
	int32_t param2 = 0;

	mavlink_message_t msg;
	mavlink_msg_command_ack_pack_chan(1, 0, MAVLINK_COMM_1, &msg,
	                                  command, result, param1, param2,
	                                  255, MAV_COMP_ID_MISSIONPLANNER);
	send_mavlink_msg_to_uart(&msg);

	freertos_task_delay(1000);

	/* detected stage */
	send_mavlink_status_text("[cal] calibration started: 2 accel", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] back orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] front orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] left orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] right orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] up orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] down orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	/* measured stage */
	send_mavlink_status_text("[cal] back side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] front side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] left side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] right side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] up side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] down side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);
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
	case MAV_CMD_PREFLIGHT_CALIBRATION:
		mav_cmd_preflight_calibration(received_msg, &mav_command_long);
		break;
	}
}

void mav_mission_request_list(mavlink_message_t *received_msg)
{
	mavlink_message_t msg;

	int waypoint_cnt = autopilot_get_waypoint_count();
	mavlink_msg_mission_count_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
	                                    received_msg->sysid, received_msg->compid,
	                                    waypoint_cnt, MAV_MISSION_TYPE_MISSION);
	send_mavlink_msg_to_uart(&msg);

	if(waypoint_cnt > 0) {
		//trigger microservice handler
		mission_manager.send_mission = true;

		/* reset timeout timer */
		mission_manager.sender_timout_timer = get_sys_time_s();
	}
}

void mav_mission_count(mavlink_message_t *received_msg)
{
	mission_manager.recept_cnt = mavlink_msg_mission_count_get_count(received_msg);

	if(mission_manager.recept_cnt <= 0) {
		return;
	}

	mavlink_message_t msg;

	/* decode mission count message */
	mavlink_mission_count_t mission_count;
	mavlink_msg_mission_count_decode(received_msg, &mission_count);

	mission_manager.recvd_mission_type = mission_count.mission_type;

	/* reject mission if waypoint number exceeded maximum acceptable size */
	if(mission_manager.recept_cnt > WAYPOINT_NUM_MAX) {
		/* do ack */
		mavlink_msg_mission_ack_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
		                                  received_msg->sysid, received_msg->compid,
		                                  MAV_MISSION_NO_SPACE, mission_manager.recvd_mission_type);
		send_mavlink_msg_to_uart(&msg);
		return;
	}

	/* clear autopilot waypoint list */
	autopilot_clear_waypoint_list();

	mission_manager.receive_mission = true;
	mission_manager.recept_index = 0;

	/* request for first mission item */
	mavlink_msg_mission_request_int_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
	                received_msg->sysid, received_msg->compid,
	                mission_manager.recept_index, mission_manager.recvd_mission_type);
	send_mavlink_msg_to_uart(&msg);

	/* start timeout timer and reset retry counter */
	mission_manager.recept_timout_timer = get_sys_time_s();
	mission_manager.recept_retry = 0;
}

void mav_mission_item_int(mavlink_message_t *received_msg)
{
	if(mission_manager.receive_mission == false) return;

	int autopilot_retval;
	mavlink_message_t msg;

	/* decode received message */
	mavlink_mission_item_int_t mission_item;
	mavlink_msg_mission_item_int_decode(received_msg, &mission_item);

	if(mission_item.seq == mission_manager.recept_index) {
		/* sequence number is correct, save received mission to the list */
		autopilot_retval =  autopilot_add_new_waypoint_gps_mavlink(
		                            mission_item.x, mission_item.y, mission_item.z, mission_item.command);

		/* autopilot rejected incomed mission, closed the protocol */
		if(autopilot_retval != AUTOPILOT_SET_SUCCEED) {
			mission_manager.receive_mission = false;

			/* do ack */
			mavlink_msg_mission_ack_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
			                                  received_msg->sysid, received_msg->compid,
			                                  MAV_MISSION_ERROR, mission_manager.recvd_mission_type);
			send_mavlink_msg_to_uart(&msg);

			return;
		}
	} else {
		/* inconsistent sequence number, re-send the request message */
		mavlink_msg_mission_request_int_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
		                received_msg->sysid, received_msg->compid,
		                mission_manager.recept_index, mission_manager.recvd_mission_type);
		send_mavlink_msg_to_uart(&msg);

		/* start timeout timer and reset retry counter */
		mission_manager.recept_timout_timer = get_sys_time_s();
		mission_manager.recept_retry = 0;
	}

	mission_manager.recept_index++;

	if(mission_manager.recept_index == mission_manager.recept_cnt) {
		/* disable microservice handler and reset variables */
		mission_manager.receive_mission = false;
		mission_manager.recept_cnt = 0;
		mission_manager.recept_index = 0;

		/* do ack */
		mavlink_msg_mission_ack_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
		                                  received_msg->sysid, received_msg->compid,
		                                  MAV_MISSION_ACCEPTED, mission_manager.recvd_mission_type);
		send_mavlink_msg_to_uart(&msg);
	} else {
		/* request for next mission item */
		mavlink_msg_mission_request_int_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
		                received_msg->sysid, received_msg->compid,
		                mission_manager.recept_index, mission_manager.recvd_mission_type);
		send_mavlink_msg_to_uart(&msg);

		/* start timeout timer and reset retry counter */
		mission_manager.recept_timout_timer = get_sys_time_s();
		mission_manager.recept_retry = 0;
	}
}

void mav_mission_request_int(mavlink_message_t *received_msg)
{
	if(mission_manager.send_mission == false) return;

	mavlink_message_t msg;

	/* decode request int message */
	mavlink_mission_request_int_t mission_request_int;
	mavlink_msg_mission_request_int_decode(received_msg, &mission_request_int);

	/* read latitude, longitude, height and command from autopilot waypoint list */
	int32_t latitude, longitude;
	float height;
	uint16_t command;

	bool retval = autopilot_get_waypoint_gps_mavlink(mission_request_int.seq,
	                &latitude, &longitude, &height, &command);

	/* if ground station inquired an invalid mission sequence number */
	if(retval == false) {
		/* end the protocol */
		mission_manager.receive_mission = false;

		/* do ack */
		int mission_type = MAV_MISSION_TYPE_MISSION;
		mavlink_msg_mission_ack_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
		                                  received_msg->sysid, received_msg->compid,
		                                  MAV_MISSION_INVALID_SEQUENCE,
		                                  mission_type);
		send_mavlink_msg_to_uart(&msg);

		return;
	}

	/* send waypoint to ground station */
	uint8_t frame = MAV_FRAME_GLOBAL;
	uint8_t current = 0;
	uint8_t autocontinue = 1;
	float params[4] = {0.0f};
	uint8_t mission_type = 	mission_manager.recvd_mission_type;

	mavlink_msg_mission_item_int_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
	                                       received_msg->sysid, received_msg->compid,
	                                       mission_request_int.seq,
	                                       frame, command, current, autocontinue,
	                                       params[0], params[1], params[2], params[3],
	                                       latitude, longitude, height, mission_type);
	send_mavlink_msg_to_uart(&msg);

	/* reset timeout timer */
	mission_manager.sender_timout_timer = get_sys_time_s();
}

void mav_mission_ack(mavlink_message_t *received_msg)
{
	if(mission_manager.send_mission == false) return;

	/* ground station received all missions, ready to close the protocol */
	mission_manager.send_mission = false;
}

void mav_mission_clear_all(mavlink_message_t *received_msg)
{
	//XXX: not supported by old version qgroundcontrol, need to test on new version
	//     later

	mavlink_message_t msg;

	/* not supposed to receive this message while receiving or sending waypoing list */
	if(mission_manager.send_mission == true || mission_manager.receive_mission == true) {
		/* do ack */
		mavlink_msg_mission_ack_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
		                                  received_msg->sysid, received_msg->compid,
		                                  MAV_MISSION_ERROR, MAV_MISSION_TYPE_ALL);
		send_mavlink_msg_to_uart(&msg);
		return;
	}

	/* erase the whole waypoint list */
	autopilot_clear_waypoint_list();

	/* do ack */
	mavlink_msg_mission_ack_pack_chan(1, 1, MAVLINK_COMM_1, &msg,
	                                  received_msg->sysid, received_msg->compid,
	                                  MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_ALL);
	send_mavlink_msg_to_uart(&msg);
}

/*****************************************
 * mavlink mission microservice handlers *
 *****************************************/
void mission_waypoint_microservice_handler_out(void)
{
	if(mission_manager.send_mission == false) return;

	float curr_time = get_sys_time_s();
	if((curr_time - mission_manager.sender_timout_timer) > MISSION_TIMEOUT_TIME) {
		mission_manager.send_mission = false;
	}
}

void mission_waypoint_microservice_handler_in(void)
{
	if(mission_manager.receive_mission == false) return;

	float curr_time = get_sys_time_s();
	if((curr_time - mission_manager.recept_timout_timer) > MISSION_TIMEOUT_TIME) {
		/* timeout, send request message */
		if(mission_manager.recept_retry <= MISSION_RETRY_TIMES) {
			mavlink_message_t msg;
			mavlink_msg_mission_request_int_pack_chan(
			        1, 1, MAVLINK_COMM_1, &msg, 255, 0,
			        mission_manager.recept_index, mission_manager.recvd_mission_type);
			send_mavlink_msg_to_uart(&msg);
			mission_manager.recept_retry++;
		} else {
			/* exceeded maximum retry time, close the prototcol! */
			mission_manager.receive_mission = false;
		}
	}
}

void mission_waypoint_microservice_handler(void)
{
	mission_waypoint_microservice_handler_out();
	mission_waypoint_microservice_handler_in();
}
