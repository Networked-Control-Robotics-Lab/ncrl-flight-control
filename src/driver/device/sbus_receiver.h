#ifndef __SBUS_RECEIVER_H__
#define __SBUS_RECEIVER_H__

#include <stdint.h>
#include <stdbool.h>

/* calibrated rc signal */
#define RC_THROTTLE_MAX 1680
#define RC_THROTTLE_MIN 368
#define RC_ROLL_MAX 1680
#define RC_ROLL_MIN 366
#define RC_PITCH_MAX 1680
#define RC_PITCH_MIN 368
#define RC_YAW_MAX 1680
#define RC_YAW_MIN 371
#define RC_SAFETY_MAX 1904
#define RC_SAFETY_MIN 144
#define RC_FLIGHT_MODE_MAX 1904
#define RC_FLIGHT_MODE_MID 1024
#define RC_FLIGHT_MODE_MIN 144

/* define radio control range */
#define RC_THROTTLE_RANGE_MAX 100.0f
#define RC_THROTTLE_RANGE_MIN 0.0f
#define RC_ROLL_RANGE_MAX +35.0f
#define RC_ROLL_RANGE_MIN -35.0f
#define RC_PITCH_RANGE_MAX +35.0f
#define RC_PITCH_RANGE_MIN -35.0f
#define RC_YAW_RANGE_MAX +35.0f
#define RC_YAW_RANGE_MIN -35.0f
#define RC_SAFETY_THRESH ((float)(RC_SAFETY_MAX - RC_SAFETY_MIN) / 2.0)

enum {
	FLIGHT_MODE_MANUAL = 0,
	FLIGHT_MODE_HOVERING = 1,
	FLIGHT_MODE_NAVIGATION = 2
} FLIGHT_MODE;

typedef struct {
	float throttle;
	float roll;
	float pitch;
	float yaw;
	bool safety;
	int flight_mode;
} radio_t;

void sbus_rc_handler(uint8_t byte);
void read_rc_info(radio_t *rc);
int rc_safety_check(radio_t *rc);
void debug_print_rc_info(radio_t *rc);
void debug_print_rc_val(void);

#endif
