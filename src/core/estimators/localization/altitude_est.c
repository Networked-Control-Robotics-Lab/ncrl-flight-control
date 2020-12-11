#include "debug_link.h"
#include "optitrack.h"
#include "barometer.h"

float alt_rate_last = 0.0f;
float alt_rate_predict = 0.0f;
float alt_rate_fused = 0.0f;

float alt_last = 0.0f;
float alt_predict = 0.0f;
float alt_fused = 0.0f;

float alt_bias = -0.75;
float alt_rate_bias = -0.9;

/* altitude rate estimation using barometer and acceleromter with complementary filter algorithm */
void barometer_alt_rate_estimate(float *dcm, float barometer_alt, float barometer_alt_rate,
                                 float *accel_body, float dt)
{
	/****************************
	 * altitude rate estimation *
	 ****************************/

	/* convert measured acceleration from body-fixed frame to earth frame and
	 * take the z-direction component out only */
	float accel_earth_z = dcm[2*3 + 0] * accel_body[0] +
	                      dcm[2*3 + 1] * accel_body[1] +
	                      dcm[2*3 + 2] * accel_body[2];

	/* gravity cancellation */
	accel_earth_z -= -9.81;
	accel_earth_z *= -1; //convert from NED frame to ENU frame

	/* predict altitude rate by doing numerical integration */
	alt_rate_predict = alt_rate_last + (accel_earth_z * dt);

	/* complementary filter for velocity estimation */
	const float a_vel = 0.997;
	alt_rate_fused = (a_vel * alt_rate_predict) + ((1.0f - a_vel) * barometer_alt_rate);

	/* save fused altitude rate for next iteration */
	alt_rate_last = alt_rate_fused;

	/***********************
	 * altitude estimation *
	 ***********************/

	/* predict altitude by doing numerical integration */
	alt_predict = alt_last + (alt_rate_fused * dt) + 0.5f * (accel_earth_z * dt * dt);

	/* complementary filter for altitude estimation */
	const float a_alt = 0.997;
	alt_fused = (a_alt * alt_predict) + ((1.0f - a_alt) * barometer_alt);

	/* save fused altitude for next iteration */
	alt_last = alt_fused;
}

float get_fused_barometer_relative_altitude(void)
{
	return alt_fused - alt_bias;
}

float get_fused_barometer_relative_altitude_rate(void)
{
	return alt_rate_fused - alt_rate_bias;
}

void send_alt_est_debug_message(debug_msg_t *payload)
{
	float altitude = 0.0f;
	float optitrack_z = 0.0f;
	//altitude = barometer_get_relative_altitude();
	altitude = get_fused_barometer_relative_altitude();
	optitrack_read_pos_z(&optitrack_z);

	float altitude_rate = 0.0f;
	float optitrack_vz = 0.0f;
	optitrack_read_vel_z(&optitrack_vz);
	altitude_rate = get_fused_barometer_relative_altitude_rate();

	pack_debug_debug_message_header(payload, MESSAGE_ID_ALT_EST);
	/* position */
	pack_debug_debug_message_float(&altitude, payload);
	pack_debug_debug_message_float(&optitrack_z, payload);
	/* velocity */
	pack_debug_debug_message_float(&altitude_rate, payload);
	pack_debug_debug_message_float(&optitrack_vz, payload);
}
