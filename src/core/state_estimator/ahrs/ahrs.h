#ifndef __AHRS_H__
#define __AHRS_H__

#include <stdbool.h>
#include "se3_math.h"
#include "debug_link.h"

typedef struct {
	//eulers angle
	float roll;
	float pitch;
	float yaw;

	//quaternion
	float q[4];

	//direction cosine matrix (rotation matrix)
	float R_i2b[3 * 3]; //earth frame to body-fixed frame
	float R_b2i[3 * 3]; //body-fixed frame to earth frame
} attitude_t;

void ahrs_init(void);
void init_ahrs_quaternion_with_accel_and_compass(float *q_ahrs);
void ahrs_estimate(attitude_t *attitude);
bool ahrs_compass_quality_test(float *mag_new);

void send_ahrs_compass_quality_check_debug_message(debug_msg_t *payload);

#endif
