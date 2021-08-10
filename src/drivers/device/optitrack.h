#ifndef __OPTITRACK_H__
#define __OPTITRACK_H__

#include <stdint.h>
#include <stdbool.h>
#include "debug_link.h"

#define OPTITRACK_SERIAL_MSG_SIZE 32

typedef struct {
	uint8_t id;

	/* position [m] */
	float pos_enu[3];
	float pos_last_enu[3];

	/* velocity (numerical differentiation) [m/s] */
	float vel_enu[3];
	float vel_filtered[3];

	/* orientation (quaternion) */
	float q[4];

	float time_now;
	float time_last;
	float update_rate;

	volatile int buf_pos;
	uint8_t buf[OPTITRACK_SERIAL_MSG_SIZE];
	bool vel_ready;
} optitrack_t ;

void optitrack_init(int id);
int optitrack_serial_decoder(uint8_t *buf);
void optitrack_isr_handler(uint8_t c);

void optitrack_update(void);
bool optitrack_available(void);

void optitrack_get_position_enu(float *pos);
float optitrack_get_position_enu_x(void);
float optitrack_get_position_enu_y(void);
float optitrack_get_position_enu_z(void);

void optitrack_get_velocity_enu(float *vel);
float optitrack_get_velocity_enu_x(void);
float optitrack_get_velocity_enu_y(void);
float optitrack_get_velocity_enu_z(void);

void optitrack_get_position_ned(float *pos);
float optitrack_get_position_ned_x(void);
float optitrack_get_position_ned_y(void);
float optitrack_get_position_ned_z(void);

void optitrack_get_velocity_ned(float *vel);
float optitrack_get_velocity_ned_x(void);
float optitrack_get_velocity_ned_y(void);
float optitrack_get_velocity_ned_z(void);

void optitrack_get_quaternion(float *q);

void send_optitrack_position_debug_message(debug_msg_t *payload);
void send_optitrack_quaternion_debug_message(debug_msg_t *payload);
void send_optitrack_velocity_debug_message(debug_msg_t *payload);

#endif
