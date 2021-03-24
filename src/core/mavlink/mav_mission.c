#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "autopilot.h"
#include "sys_time.h"
#include "delay.h"
#include "../mavlink/mav_mission.h"
#include "../mavlink/mav_publisher.h"
#include "common_list.h"
#include "sys_param.h"

#define MISSION_TIMEOUT_TIME 2.0f //[s]
#define MISSION_RETRY_TIMES 5

mavlink_mission_manager mission_manager;

/****************************
 * mavlink message handlers *
 ****************************/
void mav_mission_request_list(mavlink_message_t *received_msg)
{
	mavlink_message_t msg;

	int waypoint_cnt = autopilot_get_waypoint_count();

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_msg_mission_count_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
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

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* reject mission if waypoint number exceeded maximum acceptable size */
	if(mission_manager.recept_cnt > TRAJ_WP_MAX_NUM) {
		/* do ack */
		mavlink_msg_mission_ack_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
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
	mavlink_msg_mission_request_int_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
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

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	if(mission_item.seq == mission_manager.recept_index) {
		/* sequence number is correct, save received mission to the list */
		autopilot_retval =  autopilot_add_new_waypoint_gps_mavlink(
		                            mission_item.x, mission_item.y, mission_item.z, mission_item.command);

		/* autopilot rejected incomed mission, closed the protocol */
		if(autopilot_retval != AUTOPILOT_SET_SUCCEED) {
			mission_manager.receive_mission = false;

			/* do ack */
			mavlink_msg_mission_ack_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
			                                  received_msg->sysid, received_msg->compid,
			                                  MAV_MISSION_ERROR, mission_manager.recvd_mission_type);
			send_mavlink_msg_to_uart(&msg);

			return;
		}
	} else {
		/* inconsistent sequence number, re-send the request message */
		mavlink_msg_mission_request_int_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
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
		mavlink_msg_mission_ack_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
		                                  received_msg->sysid, received_msg->compid,
		                                  MAV_MISSION_ACCEPTED, mission_manager.recvd_mission_type);
		send_mavlink_msg_to_uart(&msg);
	} else {
		/* request for next mission item */
		mavlink_msg_mission_request_int_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
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

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* if ground station inquired an invalid mission sequence number */
	if(retval == false) {
		/* end the protocol */
		mission_manager.receive_mission = false;

		/* do ack */
		int mission_type = MAV_MISSION_TYPE_MISSION;
		mavlink_msg_mission_ack_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
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

	mavlink_msg_mission_item_int_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
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

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* not supposed to receive this message while receiving or sending waypoing list */
	if(mission_manager.send_mission == true || mission_manager.receive_mission == true) {
		/* do ack */
		mavlink_msg_mission_ack_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
		                                  received_msg->sysid, received_msg->compid,
		                                  MAV_MISSION_ERROR, MAV_MISSION_TYPE_ALL);
		send_mavlink_msg_to_uart(&msg);
		return;
	}

	/* erase the whole waypoint list */
	autopilot_clear_waypoint_list();

	/* do ack */
	mavlink_msg_mission_ack_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg,
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
			float sys_id;
			get_sys_param_float(MAV_SYS_ID, &sys_id);
			mavlink_msg_mission_request_int_pack_chan(
			        (uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg, 255, 0,
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
