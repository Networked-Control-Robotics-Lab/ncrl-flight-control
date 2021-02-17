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
	MESSAGE_ID_OPTITRACK_QUATERNION = 8,
	MESSAGE_ID_OPTITRACK_VELOCITY = 9,
	MESSAGE_ID_GENERAL_FLOAT = 10,
	MESSAGE_ID_GEOMETRY_MOMENT_CTRL = 11,
	MESSAGE_ID_UAV_DYNAMICS_DEBUG = 12,
	MESSAGE_ID_FREE_FALL = 13,
	MESSAGE_ID_GEOMETRY_TRACKING_CTRL = 14,
	MESSAGE_ID_COMPASS = 15,
	MESSAGE_ID_BAROMETER = 16,
	MESSAGE_ID_ALT_EST = 17,
	MESSAGE_ID_INS_SENSOR = 18,
	MESSAGE_ID_INS_RAW_POSITION = 19,
	MESSAGE_ID_INS_FUSION = 20,
	MESSAGE_ID_AHRS_COMPASS_QUALITY_CHECK = 21,
	MESSAGE_ID_ICL_MASS_ESTIMATION = 31,
	MESSAGE_ID_ICL_INERTIA_ESTIMATION = 32
} MESSAGE_ID;

typedef struct {
	uint8_t *payload;
	int payload_count;
} package_t;

void pack_debug_debug_message_header(debug_msg_t *payload, int message_id);
void pack_debug_debug_message_float(float *data_float, debug_msg_t *payload);

void send_onboard_data(uint8_t *payload, int payload_count);

#endif
