#include <stdint.h>

/* using polynomial functions for thrust curve line fitting */

float convert_motor_cmd_to_thrust(float percentage)
{
	float x = percentage;
	float x_squared = x * x;
	float x_cubed = x_squared * x;
	return -2323.0f * x_cubed + 3504.0f * x_squared + x;
}

float convert_motor_thrust_to_cmd(float thrust)
{
	float x = thrust;
	float x_squared = x * x;
	float x_cubed = x_squared * x;
	return 0.000000027 * x_cubed + 0.0000038811 * x_squared + 0.0023461340 * x;
}
