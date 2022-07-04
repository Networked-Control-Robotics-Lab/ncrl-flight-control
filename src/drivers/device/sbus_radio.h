#ifndef __SBUS_RADIO_H__
#define __SBUS_RADIO_H__

#include <stdint.h>
#include <stdbool.h>

/* calibrated rc signal */
#define RC_THROTTLE_MAX 1686
#define RC_THROTTLE_MIN 304

#define RC_ROLL_MAX 1707
#define RC_ROLL_MIN 330

#define RC_PITCH_MAX 1716
#define RC_PITCH_MIN 356

#define RC_YAW_MAX 1689
#define RC_YAW_MIN 306

#define RC_SAFETY_MAX 1904
#define RC_SAFETY_MIN 144

#define RC_SAFETY_MAX 1904
#define RC_SAFETY_MIN 144

#define RC_AUTO_FLIGHT_MAX 1904
#define RC_AUTO_FLIGHT_MIN 144

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
#define RC_AUTO_FLIGHT_THRESH ((float)(RC_AUTO_FLIGHT_MAX - RC_AUTO_FLIGHT_MIN) / 2.0)

enum {
	RC_AUX_MODE1 = 0, /* first position of the switch from top */
	RC_AUX_MODE2 = 1, /* second position of the switch from top*/
	RC_AUX_MODE3 = 2, /* third position of the switch from top*/
} AUX_CHAN_MODE;

typedef struct {
	float throttle;
	float roll;
	float pitch;
	float yaw;
	bool safety;
	bool auto_flight;
	int aux1_mode;
} radio_t;

typedef struct {
	uint8_t buf[25];
	int buf_recept_cnt;
	uint16_t rc_val[15];
} sbus_t;

void sbus_rc_isr_handler(uint8_t byte);
void sbus_rc_read(radio_t *rc);
void sbus_get_unscaled(uint16_t *rc_val);
int rc_safety_check(radio_t *rc);
void debug_print_rc_info(void);
void debug_print_rc_val(void);

#endif
