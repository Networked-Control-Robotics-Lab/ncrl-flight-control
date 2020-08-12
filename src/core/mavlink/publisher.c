#include "stm32f4xx.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"
#include "uart.h"
#include "ahrs.h"
#include "sys_time.h"
#include "optitrack.h"
#include "sbus_radio.h"

extern ahrs_t ahrs;

void send_mavlink_msg_to_uart(mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	uart3_puts((char *)buf, len);
}

void send_mavlink_heartbeat(void)
{
	uint32_t custom_mode = 65566;
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	base_mode |= MAV_MODE_STABILIZE_DISARMED;
	//base_mode |= MAV_MODE_STABILIZE_ARMED;

	uint8_t sys_status = MAV_STATE_STANDBY;
	//uint8_t sys_status = MAV_STATE_ACTIVE;
	//uint8_t sys_status = MAV_STATE_CALIBRATING;

	/* send heartbeat with PX4 id to exploit full functionality of qgroundcontrol */
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
	                           base_mode, custom_mode, sys_status);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_status_text(char *s, uint8_t severity, uint16_t id, uint8_t seq)
{
	mavlink_message_t msg;
	mavlink_msg_statustext_pack(1, 1, &msg, severity, s, id, seq);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_system_status(void)
{
	float battery_voltage = 12.5 * 1000;
	float battery_remain_percentage = 100;
	mavlink_message_t msg;

	mavlink_msg_sys_status_pack(1, 1, &msg, 0, 0, 0, 0, battery_voltage, -1,
	                            battery_remain_percentage, 0, 0, 0, 0, 0, 0);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_rc_channels(void)
{
	uint16_t rc_val[18];
	uint8_t rssi = 0;
	sbus_get_unscaled(rc_val);

	mavlink_message_t msg;
	float boot_time_ms = get_sys_time_ms();
	mavlink_msg_rc_channels_pack(1, 1, &msg, boot_time_ms, 8, rc_val[0], rc_val[1],
	                             rc_val[2], rc_val[3], rc_val[4], rc_val[5], rc_val[6],
	                             rc_val[7], rc_val[8], rc_val[9], rc_val[10], rc_val[11],
	                             rc_val[12], rc_val[13], rc_val[14], rc_val[15], rc_val[16],
	                             rc_val[17], rssi);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_attitude(void)
{
	float roll = deg_to_rad(ahrs.attitude.roll);
	float pitch = deg_to_rad(ahrs.attitude.pitch);
	float yaw = deg_to_rad(ahrs.attitude.yaw);
	uint32_t curr_time_ms = (uint32_t)get_sys_time_ms();

	mavlink_message_t msg;
	mavlink_msg_attitude_pack(1, 1, &msg, curr_time_ms, roll, pitch, yaw, 0.0, 0.0, 0.0);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_attitude_quaternion(void)
{
	float roll_speed = 0.0f;
	float pitch_speed = 0.0f;
	float yaw_speed = 0.0f;
	float *repr_offset_q = 0;

	uint32_t curr_time_ms = (uint32_t)get_sys_time_ms();

	mavlink_message_t msg;
	mavlink_msg_attitude_quaternion_pack(1, 1, &msg, curr_time_ms,
	                                     ahrs.q[0], ahrs.q[1], ahrs.q[2], ahrs.q[3],
	                                     roll_speed, pitch_speed, yaw_speed, repr_offset_q);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_gps(void)
{
	float latitude = 0.0f, longitude = 0.0f, altitude = 0.0f;
	float gps_vel_x = 0.0f, gps_vel_y = 0.0f;
	float heading = 0.0f;
	uint32_t curr_time_ms = (uint32_t)get_sys_time_ms();

	mavlink_message_t msg;
	mavlink_msg_global_position_int_pack(1, 1, &msg, curr_time_ms, latitude, longitude, altitude, 0,
	                                     gps_vel_x, gps_vel_y, altitude, heading);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_local_position_ned(void)
{
	float pos[3];
	float vel[3];
	optitrack_read_pos(pos); //XXX: add abstraction layer for this
	optitrack_read_vel(vel);

	pos[0] *= 0.01f;
	pos[1] *= 0.01f;
	pos[2] *= 0.01f;
	vel[0] *= 0.01f;
	vel[1] *= 0.01f;
	vel[2] *= 0.01f;

	uint32_t curr_time_ms = (uint32_t)get_sys_time_ms();

	mavlink_message_t msg;
	mavlink_msg_local_position_ned_pack(1, 1, &msg, curr_time_ms,
	                                    pos[0], pos[1], pos[2],
	                                    vel[0], vel[1], vel[2]);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_current_waypoint(void)
{
	int curr_waypoint = 0;

	mavlink_message_t msg;
	mavlink_msg_mission_current_pack(1, 1, &msg, curr_waypoint);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_reached_waypoint(void)
{
	int curr_waypoint = 0;

	mavlink_message_t msg;
	mavlink_msg_mission_item_reached_pack(1, 1, &msg, curr_waypoint);
	send_mavlink_msg_to_uart(&msg);
}
