#include <stdint.h>
#include <bound.h>

/* using polynomial functions for thrust curve line fitting */

#define THRUST_MAX 845.0 //[g]

float coeff_c_to_t[6] = {464.28, -2529.4, 3358.5, -969.42, 556.48, -5.4205};
float coeff_t_to_c[6] = {-1.92e-15, 2.601e-12, 7.873e-10, -2.901e-06, 2.369e-03, 8.467e-03};

//FIXME: Current range is 0~0.1 rather than 0%~100%
float convert_motor_cmd_to_thrust(float percentage)
{
	bound_float(&percentage, 100.0f, 0.0f);

	float x = percentage;
	float x_pow2 = x * x;
	float x_pow3 = x_pow2 * x;
	float x_pow4 = x_pow3 * x;
	float x_pow5 = x_pow4 * x;

	float thrust = coeff_c_to_t[0] * x_pow5 +
	               coeff_c_to_t[1] * x_pow4 +
	               coeff_c_to_t[2] * x_pow3 +
	               coeff_c_to_t[3] * x_pow2 +
	               coeff_c_to_t[4] * x +
	               coeff_c_to_t[5];

	bound_float(&thrust, THRUST_MAX, 0);

	return thrust;
}

//FIXME: Current range is 0~0.1 rather than 0%~100%
float convert_motor_thrust_to_cmd(float thrust)
{
	bound_float(&thrust, THRUST_MAX, 0.0f);

	float x = thrust;
	float x_pow2 = x * x;
	float x_pow3 = x_pow2 * x;
	float x_pow4 = x_pow3 * x;
	float x_pow5 = x_pow4 * x;

	float percentage = coeff_t_to_c[0] * x_pow5 +
	                   coeff_t_to_c[1] * x_pow4 +
	                   coeff_t_to_c[2] * x_pow3 +
	                   coeff_t_to_c[3] * x_pow2 +
	                   coeff_t_to_c[4] * x +
	                   coeff_t_to_c[5];

	bound_float(&percentage, 1.0f, 0.0f);

	return percentage;
}
