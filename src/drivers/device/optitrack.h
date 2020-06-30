#ifndef __OPTITRACK_H__
#define __OPTITRACK_H__

#include <stdint.h>
#include <stdbool.h>
#include "debug_link.h"

#define OPTITRACK_SERIAL_MSG_SIZE 32

typedef struct {
	uint8_t id;

	/* position [cm] */
	float pos[3];

	/* velocity (numerical onboard calcuation) */
	float vel_raw[3];
	float vel_filtered[3];

	/* orientation (quaternion) */
	float q[4];

	float time_now;
	float time_last;
	float recv_freq;

	volatile int buf_pos;
	uint8_t buf[OPTITRACK_SERIAL_MSG_SIZE];
	float pos_last[3];
	bool vel_ready;
} optitrack_t ;

void optitrack_init(int id);
int optitrack_serial_decoder(uint8_t *buf);
void optitrack_isr_handler(uint8_t c);

void optitrack_update(void);
bool optitrack_available(void);
void optitrack_read_pos(float *pos_enu);
float optitrack_read_height(void);
void optitrack_read_vel(float *vel_enu);

void send_optitrack_position_debug_message(debug_msg_t *payload);
void send_optitrack_quaternion_debug_message(debug_msg_t *payload);
void send_optitrack_velocity_debug_message(debug_msg_t *payload);

#endif
