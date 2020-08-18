#ifndef __MAV_PUBLISHER_H__
#define __MAV_PUBLISHER_H__

void send_mavlink_msg_to_uart(mavlink_message_t *msg);

void send_mavlink_heartbeat(void);
void send_mavlink_rc_channels(void);
void send_mavlink_status_text(char *s, uint8_t severity, uint16_t id, uint8_t seq);
void send_mavlink_system_status(void);
void send_mavlink_attitude(void);
void send_mavlink_attitude_quaternion(void);
void send_mavlink_gps(void);
void send_mavlink_local_position_ned(void);
void send_mavlink_current_waypoint(void);
void send_mavlink_reached_waypoint(void);

#endif
