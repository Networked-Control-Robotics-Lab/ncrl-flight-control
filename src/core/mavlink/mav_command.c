#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "autopilot.h"
#include "sys_time.h"
#include "accel_calibration.h"
#include "compass_calibration.h"
#include "../mavlink/mav_mission.h"
#include "../mavlink/mav_publisher.h"
#include "calibration_task.h"
#include "sys_param.h"
#include "esc_calibration.h"
#include "common_list.h"
#include "takeoff_landing.h"
#include "waypoint_following.h"
#include "mav_command.h"

void command_long_trigger_ack_sending(uint8_t target_system, uint16_t ack_cmd, uint8_t ack_result);

cmd_long_msg_manager_t cmd_long_msg_manager;

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

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_msg_autopilot_version_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg, cap, flight_sw_version, middleware_sw_ver,
	                                        os_sw_version, board_version, flight_custom_ver, middleware_custom_ver,
	                                        os_custom_ver, vendor_id, product_id, uid, NULL);
	send_mavlink_msg_to_uart(&msg);
}

static void mav_cmd_long_takeoff(uint8_t sender_id, mavlink_command_long_t *cmd_long)
{
	uint8_t ack_result;

	/* change autopilot state */
	int retval = autopilot_trigger_auto_takeoff();
	switch(retval) {
	case AUTOPILOT_SET_SUCCEED:
		ack_result = MAV_RESULT_ACCEPTED;
		break;
	case AUTOPILOT_ALREADY_TAKEOFF:
	default:
		ack_result = MAV_RESULT_DENIED;
	}

	/* send ack message if the sender ask to confirm */
	if(cmd_long->confirmation == 1) {
		command_long_trigger_ack_sending(sender_id, MAV_CMD_NAV_TAKEOFF, ack_result);
	}
}

static void mav_cmd_long_land(uint8_t sender_id, mavlink_command_long_t *cmd_long)
{
	uint8_t ack_result;

	/* change autopilot state */
	int retval = autopilot_trigger_auto_landing();
	switch(retval) {
	case AUTOPILOT_SET_SUCCEED:
		ack_result = MAV_RESULT_ACCEPTED;
		break;
	case AUTOPILOT_NOT_IN_HOVERING_MODE:
	default:
		ack_result = MAV_RESULT_DENIED;
	}

	/* send ack message if the sender ask to confirm */
	if(cmd_long->confirmation == 1) {
		command_long_trigger_ack_sending(sender_id, MAV_CMD_NAV_LAND, ack_result);
	}
}

static void mav_cmd_override_goto_handler(uint8_t sender_id, mavlink_command_long_t *cmd_long)
{
	float yaw = 0;
	float pos[3] = {0};

	uint8_t ack_result;

	switch((int)cmd_long->param3) {
	case MAV_FRAME_GLOBAL: /* format: latitude, longitude, altitude */
		//TODO
		ack_result = MAV_RESULT_DENIED;
		break;
	case MAV_FRAME_LOCAL_NED: /* format: x, y, z in local ned frame */
		yaw = cmd_long->param4;
		pos[1] = cmd_long->param5;
		pos[0] = cmd_long->param6;
		pos[2] = -cmd_long->param7;
		ack_result = MAV_RESULT_ACCEPTED;
		break;
	case MAV_FRAME_LOCAL_ENU: /* format: x, y, z in local enu frame */
		yaw = cmd_long->param4;
		pos[0] = cmd_long->param5;
		pos[1] = cmd_long->param6;
		pos[2] = cmd_long->param7;

		/* change autopilot state */
		int retval = autopilot_goto_waypoint_now(yaw, pos, true);
		if(retval == AUTOPILOT_SET_SUCCEED) {
			ack_result = MAV_RESULT_ACCEPTED;
		} else if(retval == AUTOPILOT_WAYPOINT_OUT_OF_FENCE) {
			ack_result = MAV_RESULT_DENIED;
		} else {
			ack_result = MAV_RESULT_DENIED;
		}

		break;
	default:
		ack_result = MAV_RESULT_DENIED;
	}

	/* send ack message if the sender ask to confirm */
	if(cmd_long->confirmation == 1) {
		command_long_trigger_ack_sending(sender_id, MAV_CMD_OVERRIDE_GOTO, ack_result);
	}
}

static void mav_cmd_override_mission_resume_handler(uint8_t sender_id, mavlink_command_long_t *cmd_long)
{
	uint8_t ack_result;

	/* change autopilot state */
	int retval = autopilot_resume_waypoint_mission();
	switch(retval) {
	case AUTOPILOT_SET_SUCCEED:
		ack_result = MAV_RESULT_ACCEPTED;
		break;
	case AUTOPILOT_NO_HALTED_WAYPOINT_MISSION:
	default:
		ack_result = MAV_RESULT_DENIED;
	}

	/* send ack message if the sender ask to confirm */
	if(cmd_long->confirmation == 1) {
		command_long_trigger_ack_sending(sender_id, MAV_CMD_OVERRIDE_GOTO, ack_result);
	}
}

static void mav_cmd_override_mission_halt_handler(uint8_t sender_id, mavlink_command_long_t *cmd_long)
{
	uint8_t ack_result;

	int retval = autopilot_halt_waypoint_mission();
	switch(retval) {
	case AUTOPILOT_SET_SUCCEED:
		ack_result = MAV_RESULT_ACCEPTED;
		break;
	case AUTOPILOT_NOT_IN_WAYPOINT_MODE:
	default:
		ack_result = MAV_RESULT_DENIED;
	}

	/* send ack message if the sender ask to confirm */
	if(cmd_long->confirmation == 1) {
		command_long_trigger_ack_sending(sender_id, MAV_CMD_OVERRIDE_GOTO, ack_result);
	}
}

static void mav_cmd_long_override_goto(uint8_t sender_id, mavlink_command_long_t *cmd_long)
{
	if((int)cmd_long->param1 == MAV_GOTO_DO_HOLD) {
		if((int)cmd_long->param2 == MAV_GOTO_HOLD_AT_SPECIFIED_POSITION) {
			/* goto specified position */
			mav_cmd_override_goto_handler(sender_id, cmd_long);
		} else if((int)cmd_long->param2 == MAV_GOTO_HOLD_AT_CURRENT_POSITION) {
			/* mission halt (hovering) */
			mav_cmd_override_mission_halt_handler(sender_id, cmd_long);
		}
	} else if((int)cmd_long->param1 == MAV_GOTO_DO_CONTINUE) {
		/* mission resume */
		mav_cmd_override_mission_resume_handler(sender_id, cmd_long);
	}
}

static void mav_cmd_preflight_calibration(mavlink_message_t *received_msg,
                mavlink_command_long_t *cmd_long)
{
	uint16_t command = MAV_CMD_PREFLIGHT_CALIBRATION;
	uint8_t result = MAV_RESULT_ACCEPTED;
	uint8_t param1 = 0;
	int32_t param2 = 0;

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_command_ack_pack_chan((uint8_t)sys_id, 0, MAVLINK_COMM_1, &msg,
	                                  command, result, param1, param2,
	                                  255, MAV_COMP_ID_MISSIONPLANNER);
	send_mavlink_msg_to_uart(&msg);

	if((int)cmd_long->param1 == 1) {
		/* gyro calibration is not required in this firmware,
		 * we do this during every boot time */
		send_mavlink_status_text("[cal] calibration cancelled", 6, 0, 0);
	} else if((int)cmd_long->param5 == 1) {
		/* accelerometer scale calibration */
		wakeup_calibration_task(ACCEL_SCALE_CALIBRATION);
	} else if((int)cmd_long->param5 == 2) {
		/* accelerometer offset calibration */
		wakeup_calibration_task(ACCEL_OFFSET_CALIBRATION);
	} else if((int)cmd_long->param2 == 1) {
		/* compass calibration */
		wakeup_calibration_task(COMPASS_CALIBRATION);
	} else if((int)cmd_long->param7 == 1) {
		send_mavlink_status_text("esc calibration start", MAV_SEVERITY_WARNING, 0, 0);
		trigger_esc_range_calibration();
	} else {
		/* not supported type calibration */
		send_mavlink_status_text("[cal] calibration cancelled", 6, 0, 0);
	}

	/* if user cancelled the calibration*/
	if((int)cmd_long->param1 == 0 && (int)cmd_long->param2 == 0 &&
	    (int)cmd_long->param3 == 0 && (int)cmd_long->param4 == 0 &&
	    (int)cmd_long->param5 == 0 && (int)cmd_long->param6 == 0 &&
	    (int)cmd_long->param7 == 0) {
		cancel_device_calibration();
	}
}

static void mav_cmd_preflight_storage(mavlink_message_t *received_msg,
                                      mavlink_command_long_t *cmd_long)
{
	/* reset parmater list to default */
	if((int)cmd_long->param1 == 2) {
		//TODO: should be allowed only in preflight mode!
		reset_sys_param_list_to_default();
		save_param_list_to_flash();
	}
}

static void mav_cmd_long_mission_start(uint8_t sender_id, mavlink_command_long_t *cmd_long)
{
	//TODO: support start and end waypoint assignment
	//(int)cmd_long->param1; //start waypoint
	//(int)cmd_long->param2; //end waypoint

	uint8_t ack_result;

	int retval = autopilot_waypoint_mission_start(false);
	switch(retval) {
	case AUTOPILOT_SET_SUCCEED:
		ack_result = MAV_RESULT_ACCEPTED;
		break;
	case AUTOPILOT_WAYPOINT_LIST_EMPYT:
	default:
		ack_result = MAV_RESULT_DENIED;
	}

	/* send ack message if the sender ask to confirm */
	if(cmd_long->confirmation == 1) {
		command_long_trigger_ack_sending(sender_id, MAV_CMD_OVERRIDE_GOTO, ack_result);
	}
}

void mav_command_long(mavlink_message_t *received_msg)
{
	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* decode command_long message */
	mavlink_command_long_t mav_command_long;
	mavlink_msg_command_long_decode(received_msg, &mav_command_long);

	/* ignore the message if the target id not matched to the system id */
	if((uint8_t)sys_id != mav_command_long.target_system) {
		return;
	}

	uint8_t sender_id = received_msg->sysid;

	switch(mav_command_long.command) {
	case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
		mavlink_send_capability();
		break;
	case MAV_CMD_COMPONENT_ARM_DISARM:
		break;
	case MAV_CMD_NAV_TAKEOFF:
		mav_cmd_long_takeoff(sender_id, &mav_command_long);
		break;
	case MAV_CMD_NAV_LAND:
		mav_cmd_long_land(sender_id, &mav_command_long);
		break;
	case MAV_CMD_OVERRIDE_GOTO:
		mav_cmd_long_override_goto(sender_id, &mav_command_long);
		break;
	case MAV_CMD_PREFLIGHT_CALIBRATION:
		mav_cmd_preflight_calibration(received_msg, &mav_command_long);
		break;
	case MAV_CMD_PREFLIGHT_STORAGE:
		mav_cmd_preflight_storage(received_msg, &mav_command_long);
		break;
	case MAV_CMD_MISSION_START:
		mav_cmd_long_mission_start(sender_id, &mav_command_long);
		break;
	}
}

void command_long_trigger_ack_sending(uint8_t target_system, uint16_t ack_cmd, uint8_t ack_result)
{
	cmd_long_msg_manager.send_ack_msg = true;
	cmd_long_msg_manager.ack_target_sys = target_system;
	cmd_long_msg_manager.ack_cmd = ack_cmd;
	cmd_long_msg_manager.ack_result = ack_result;
}

void command_long_microservice_handler(void)
{
	/* XXX: replace with freertos queue? */
	if(cmd_long_msg_manager.send_ack_msg == true) {
		cmd_long_msg_manager.send_ack_msg = false;

		float sys_id;
		get_sys_param_float(MAV_SYS_ID, &sys_id);

		uint16_t cmd = cmd_long_msg_manager.ack_cmd;
		uint8_t target_system = cmd_long_msg_manager.ack_target_sys;
		uint8_t target_component = 1;
		uint8_t progress = UINT8_MAX;
		uint8_t result = cmd_long_msg_manager.ack_result;
		uint8_t result_param2 = 0;

		mavlink_message_t msg;
		mavlink_msg_command_ack_pack((uint8_t)sys_id, 1, &msg, cmd, result, progress,
		                             result_param2, target_system, target_component);
		send_mavlink_msg_to_uart(&msg);
	}
}
