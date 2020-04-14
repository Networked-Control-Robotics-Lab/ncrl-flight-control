#ifndef __OPTITRACK_H__
#define __OPTITRACK_H__

#include <stdint.h>
#include <stdbool.h>
#include "debug_link.h"

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
} optitrack_t ;

int optitrack_serial_decoder(uint8_t *buf);
void optitrack_handler(uint8_t c);
void optitrack_init(int id);
bool optitrack_available(void);
void optitrack_read(float *pos_ned, float *vel_ned);

void send_optitrack_position_debug_message(debug_msg_t *payload);
void send_optitrack_quaternion_debug_message(debug_msg_t *payload);
void send_optitrack_velocity_debug_message(debug_msg_t *payload);

#endif
