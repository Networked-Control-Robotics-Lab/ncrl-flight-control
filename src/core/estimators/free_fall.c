#include <stdbool.h>
#include "arm_math.h"

bool free_fall_detect(float *accel)
{
	float accel_norm;
	arm_sqrt_f32(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2], &accel_norm);

	float threshold = 9.8 * 0.1; //0.1g acceleration
	if(accel_norm <= threshold) return true;

	return false;
}
