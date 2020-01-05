#include "stm32f4xx.h"
#include "mavlink.h"
#include "uart.h"

void send_mavlink_msg_to_uart(mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	uart3_puts((char *)buf, len);
}

void send_mavlink_heartbeat(void)
{
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_QUADROTOR,
				   MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_system_status(void)
{
	float battery_voltage = 12.5 * 1000;
	float battery_remain_percentage = 100;
		mavlink_message_t msg;

	mavlink_msg_sys_status_pack(1, 0, &msg, 0, 0, 0, 0, battery_voltage, -1,
				    battery_remain_percentage, 0, 0, 0, 0, 0, 0);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_attitude(void)
{
	float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; //[radian]
	uint32_t curr_time_ms = 0;

	mavlink_message_t msg;
	mavlink_msg_attitude_pack(1, 200, &msg, curr_time_ms, roll, pitch, yaw, 0.0, 0.0, 0.0);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_gps(void)
{
	float latitude = 0.0f, longitude = 0.0f, altitude = 0.0f;
	float gps_vel_x = 0.0f, gps_vel_y = 0.0f;
	float heading = 0.0f;
	uint32_t curr_time_ms = 0;
	
	mavlink_message_t msg;
	mavlink_msg_global_position_int_pack(1, 220, &msg, curr_time_ms, latitude, longitude, altitude, 0,
					     gps_vel_x, gps_vel_y, altitude, heading);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_current_waypoint(void)
{
	int curr_waypoint = 0;

	mavlink_message_t msg;		
	mavlink_msg_mission_current_pack(1, 0, &msg, curr_waypoint);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_reached_waypoint(void)
{
		int curr_waypoint = 0;

		mavlink_message_t msg;		
		mavlink_msg_mission_item_reached_pack(1, 0, &msg, curr_waypoint);
		send_mavlink_msg_to_uart(&msg);
}
