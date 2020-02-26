#include <stdint.h>
#include <bound.h>

/* using polynomial functions for thrust curve line fitting */

#define THRUST_MAX 845.0 //[g]

//FIXME: Current range is 0~0.1 rather than 0%~100%
float convert_motor_cmd_to_thrust(float percentage)
{
	bound_float(&percentage, 100.0f, 0.0f);

	float x = percentage;
	float x_squared = x * x;
	float x_cubed = x_squared * x;
	float thrust = (-1831.0f) * x_cubed + (2836.0f) * x_squared + (-147.0f) * x;

	bound_float(&thrust, THRUST_MAX, 0);

	return thrust;
}

//FIXME: Current range is 0~0.1 rather than 0%~100%
float convert_motor_thrust_to_cmd(float thrust)
{
	bound_float(&thrust, THRUST_MAX, 0.0f);

	float x = thrust;
	float x_squared = x * x;
	float x_cubed = x_squared * x;

	float percentage = (0.0000000027) * x_cubed + (-0.0000037421) * x_squared + (0.0023565529) * x;
	bound_float(&percentage, 1.0f, 0.0f);

	return percentage;
}
