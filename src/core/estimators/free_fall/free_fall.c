#include <stdbool.h>
#include "arm_math.h"
#include "debug_link.h"
#include "imu.h"

bool free_fall_detect(float *g, float *g_norm)
{
	float accel_norm;
	arm_sqrt_f32(g[0]*g[0] + g[1]*g[1] + g[2]*g[2], &accel_norm);

	*g_norm = accel_norm;

	float threshold = 9.8 * 0.5; //0.1g acceleration
	if(accel_norm <= threshold) return true;

	return false;
}

void send_free_fall_debug_message(debug_msg_t *payload)
{
	float accel[3], g_norm;
	get_accel_lpf(accel);

	float free_fall_state;
	if(free_fall_detect(accel, &g_norm) == true) {
		free_fall_state = 1.0f;
	} else {
		free_fall_state = 0.0f;
	}

	pack_debug_debug_message_header(payload, MESSAGE_ID_FREE_FALL);
	pack_debug_debug_message_float(&g_norm, payload);
	pack_debug_debug_message_float(&free_fall_state, payload);
}
