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
	case MAV_CMD_PREFLIGHT_STORAGE:
		mav_cmd_preflight_storage(received_msg, &mav_command_long);
		break;
	}
}
