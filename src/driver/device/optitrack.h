#ifndef __OPTITRACK_H__
#define __OPTITRACK_H__

#include <stdint.h>

void optitrack_handler(uint8_t c);

typedef struct {
	/* position */
	float pos_x; //[cm]
	float pos_y;
	float pos_z;

	/* orientation (quaternion) */
	float quat_x;
	float quat_y;
	float quat_z;
	float quat_w;
} optitrack_t ;

int optitrack_serial_decoder(uint8_t *buf);

#endif
