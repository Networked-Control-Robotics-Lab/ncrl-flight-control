#include <stdint.h>
#include <bound.h>

/* using polynomial functions for thrust curve line fitting */
float thrust_max = 845.0f; //[g]
float coeff_c_to_t[6] = {-2842.8f, 3951.7f, -1925.4f, 1381.3f, 257.37f, -7.0118f};
float coeff_t_to_c[6] = {1.169e-14, 2.264e-11, 1.697e-08, -6.715e-06, 2.336e-03, 3.082e-02};

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

	bound_float(&thrust, thrust_max, 0);

	return thrust;
}

//FIXME: Current range is 0~0.1 rather than 0%~100%
float convert_motor_thrust_to_cmd(float thrust)
{
	bound_float(&thrust, thrust_max, 0.0f);

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
