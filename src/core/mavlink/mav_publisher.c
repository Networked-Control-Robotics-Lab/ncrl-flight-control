#include "stm32f4xx.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/mav_publisher.h"
#include "uart.h"
#include "ahrs.h"
#include "sys_time.h"
#include "optitrack.h"
#include "sbus_radio.h"
#include "system_state.h"
#include "common_list.h"
#include "sys_param.h"
#include "gps.h"

extern attitude_t attitude;

void send_mavlink_msg_to_uart(mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	uart3_puts((char *)buf, len);
}

void send_mavlink_heartbeat(void)
{
	uint32_t custom_mode = 65536;
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	base_mode |= MAV_MODE_STABILIZE_DISARMED;
	//base_mode |= MAV_MODE_STABILIZE_ARMED;

	uint8_t sys_status = MAV_STATE_STANDBY;
	//uint8_t sys_status = MAV_STATE_ACTIVE;
	//uint8_t sys_status = MAV_STATE_CALIBRATING;

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* send heartbeat with PX4 id to exploit full functionality of qgroundcontrol */
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack((uint8_t)sys_id, 1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
	                           base_mode, custom_mode, sys_status);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_status_text(char *s, uint8_t severity, uint16_t id, uint8_t seq)
{
	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_statustext_pack((uint8_t)sys_id, 1, &msg, severity, s, id, seq);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_system_status(void)
{
	float battery_voltage = 12.5 * 1000;
	float battery_remain_percentage = 100;
	mavlink_message_t msg;

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_msg_sys_status_pack((uint8_t)sys_id, 1, &msg, 0, 0, 0, 0, battery_voltage, -1,
	                            battery_remain_percentage, 0, 0, 0, 0, 0, 0);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_rc_channels(void)
{
	uint16_t rc_val[18];
	uint8_t rssi = 0;
	sbus_get_unscaled(rc_val);

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	float boot_time_ms = get_sys_time_ms();

	mavlink_msg_rc_channels_pack((uint8_t)sys_id, 1, &msg, boot_time_ms, 8, rc_val[0], rc_val[1],
	                             rc_val[2], rc_val[3], rc_val[4], rc_val[5], rc_val[6],
	                             rc_val[7], rc_val[8], rc_val[9], rc_val[10], rc_val[11],
	                             rc_val[12], rc_val[13], rc_val[14], rc_val[15], rc_val[16],
	                             rc_val[17], rssi);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_attitude(void)
{
	float roll = deg_to_rad(attitude.roll);
	float pitch = deg_to_rad(attitude.pitch);
	float yaw = deg_to_rad(attitude.yaw);
	uint32_t curr_time_ms = (uint32_t)get_sys_time_ms();

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_attitude_pack((uint8_t)sys_id, 1, &msg, curr_time_ms, roll, pitch, yaw, 0.0, 0.0, 0.0);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_attitude_quaternion(void)
{
	float roll_speed = 0.0f;
	float pitch_speed = 0.0f;
	float yaw_speed = 0.0f;
	float *repr_offset_q = 0;

	uint32_t curr_time_ms = (uint32_t)get_sys_time_ms();

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_attitude_quaternion_pack((uint8_t)sys_id, 1, &msg, curr_time_ms,
	                                     attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3],
	                                     roll_speed, pitch_speed, yaw_speed, repr_offset_q);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_gps(void)
{
	uint32_t curr_time_ms = (uint32_t)get_sys_time_ms();

	int32_t longitude = 0, latitude = 0, height_msl = 0;
	uint8_t sv_num = 0;
	uint8_t fix_type = 0;
	float pdop = 0.0f;
	float hdop = 0.0f;
	float vdop = 0.0f;
	float ground_speed = 0;
	uint16_t cog = 0;
	uint32_t altitude_msl = 0;
	float h_acc = 0.0f;
	float v_acc = 0.0f;
	uint32_t vel_acc = 0;
	uint32_t heading_acc = 0;
	float gps_yaw = 0;

	get_gps_longitude_latitude_height_s32(&longitude, &latitude, &height_msl);
	get_gps_dilution_of_precision(&pdop, &hdop, &vdop);
	sv_num = get_gps_satellite_numbers();
	fix_type = get_gps_fix_type();
	get_gps_position_uncertainty(&h_acc, &v_acc);
	ground_speed = get_gps_ground_speed();
	gps_yaw = get_gps_heading() * 1e2;

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_gps_raw_int_pack((uint8_t)sys_id, 1, &msg, curr_time_ms,
	                             fix_type, latitude, longitude, height_msl,
	                             (uint16_t)hdop, (uint16_t)vdop, (uint16_t)ground_speed, cog, sv_num,
	                             altitude_msl, (uint32_t)h_acc, (uint32_t)v_acc,
	                             vel_acc, heading_acc, (uint16_t)gps_yaw);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_local_position_ned(void)
{
	float pos[3];
	float vel[3];
	get_enu_position(pos);
	get_enu_velocity(vel);

	uint32_t curr_time_ms = (uint32_t)get_sys_time_ms();

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_local_position_ned_pack((uint8_t)sys_id, 1, &msg, curr_time_ms,
	                                    pos[0], pos[1], pos[2],
	                                    vel[0], vel[1], vel[2]);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_current_waypoint(void)
{
	int curr_waypoint = 0;

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_mission_current_pack((uint8_t)sys_id, 1, &msg, curr_waypoint);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_reached_waypoint(void)
{
	int curr_waypoint = 0;

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_mission_item_reached_pack((uint8_t)sys_id, 1, &msg, curr_waypoint);
	send_mavlink_msg_to_uart(&msg);
}
