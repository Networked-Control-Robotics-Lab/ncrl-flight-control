#ifndef __LINK__
#define __LINK__

#include <stdint.h>

enum {
	MESSAGE_ID_IMU = 0,
	MESSAGE_ID_ATTITUDE_EULER = 1,
	MESSAGE_ID_ATTITUDE_IMU = 2,
	MESSAGE_ID_EKF = 3,
	MESSAGE_ID_ATTITUDE_QUAT = 4,
	MESSAGE_ID_PID_DEBUG = 5,
	MESSAGE_ID_MOTOR = 6
} MESSAGE_ID;

typedef struct {
	uint8_t *payload;
	int payload_count;
} package_t;

void telemetry_loop();

#endif
