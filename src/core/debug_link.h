#ifndef __DEBUG_LINK__
#define __DEBUG_LINK__

#include <stdint.h>

typedef struct {
	uint8_t s[200];
	int len;
} debug_msg_t;

enum {
	MESSAGE_ID_IMU = 0,
	MESSAGE_ID_ATTITUDE_EULER = 1,
	MESSAGE_ID_ATTITUDE_IMU = 2,
	MESSAGE_ID_EKF = 3,
	MESSAGE_ID_ATTITUDE_QUAT = 4,
	MESSAGE_ID_PID_DEBUG = 5,
	MESSAGE_ID_MOTOR = 6,
	MESSAGE_ID_OPTITRACK_POSITION = 7,
	MESSAGE_ID_OPTITRACK_QUATERNION = 8
} MESSAGE_ID;

typedef struct {
	uint8_t *payload;
	int payload_count;
} package_t;

void task_debug_link(void *param);

#endif
