#ifndef __SBUS_RECEIVER_H__
#define __SBUS_RECEIVER_H__

#include <stdint.h>

typedef struct {
	float throttle;
	float roll;
	float pitch;
	float yaw;
	float safety;
} radio_t;

void sbus_rc_handler(uint8_t byte);

#endif
