#ifndef __MAVLINK_PUBLISHER_H__
#define __MAVLINK_PUBLISHER_H__

void send_mavlink_msg_to_uart(mavlink_message_t *msg);

void send_mavlink_heartbeat(void);
void send_mavlink_system_status(void);
void send_mavlink_attitude(void);
void send_mavlink_gps(void);
void send_mavlink_current_waypoint(void);
void send_mavlink_reached_waypoint(void);

#endif
