#include <stdbool.h>
#include <stdint.h>
#include "gps_to_enu.h"
#include "position_state.h"
#include "barometer.h"
#include "compass.h"
#include "ins_comp_filter.h"
#include "gps.h"
#include "led.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "proj_config.h"
#include "ublox_m8n.h"
#include "ins_sensor_sync.h"
#include "sys_time.h"

#define INS_LOOP_PERIOD 0.0025f //400Hz

float pos_enu_raw[3];
float vel_enu_raw[3];
float pos_enu_fused[3];
float vel_enu_fused[3];

void ins_init(void)
{
	ins_sync_buffer_init();

#if (SELECT_INS == INS_COMPLEMENTARY_FILTER)
	ins_comp_filter_init(INS_LOOP_PERIOD);
#endif
}

void ins_state_estimate(void)
{
	/* check sensor status */
	bool gps_ready = is_gps_available();
	bool compass_ready = is_compass_available();
	bool barometer_ready = is_barometer_available();

	bool sensor_all_ready = gps_ready && compass_ready && barometer_ready;

	/* change led state to indicate the sensor status */
	if(sensor_all_ready == true) {
		led_on(LED_G);
	} else {
		led_off(LED_G);
	}

	float longitude, latitude, gps_msl_height;
	float gps_ned_vx, gps_ned_vy, gps_ned_vz;
	float barometer_height, barometer_height_rate;

	bool recvd_gps =
	        ins_gps_sync_buffer_pop(&longitude, &latitude, &gps_msl_height,
	                                &gps_ned_vx, &gps_ned_vy, &gps_ned_vz);
	bool recvd_barometer =
	        ins_barometer_sync_buffer_pop(&barometer_height, &barometer_height_rate);

	/* predict position and velocity with kinematics equations (400Hz) */
	ins_comp_filter_predict(pos_enu_fused, vel_enu_fused,
	                        gps_ready, barometer_ready);

	if(recvd_barometer == true) {
		pos_enu_raw[2] = barometer_height;
		vel_enu_raw[2] = barometer_height_rate;

		//run barometer correction (~50Hz)
		ins_comp_filter_barometer_correct(pos_enu_raw, vel_enu_raw,
		                                  pos_enu_fused, vel_enu_fused);

		if(recvd_gps == true) {
			/* convert gps data from geographic coordinate system to
			 * enu frame */
			float dummy_z;
			longitude_latitude_to_enu(
			        longitude, latitude, 0,
			        &pos_enu_raw[0], &pos_enu_raw[1], &dummy_z);

			if(gps_home_is_set() == false) {
				set_home_longitude_latitude(
				        longitude, latitude, barometer_height);
			}

			/* convert gps velocity from ned frame to enu frame */
			vel_enu_raw[0] = gps_ned_vy; //x_enu = y_ned
			vel_enu_raw[1] = gps_ned_vx; //y_enu = x_ned

			//run gps correction (~5Hz)
			ins_comp_filter_gps_correct(pos_enu_raw, vel_enu_raw,
			                            pos_enu_fused, vel_enu_fused);
		}
	}
}

/* raw position getters */
float ins_get_raw_position_x(void)
{
	return pos_enu_raw[0];
}

float ins_get_raw_position_y(void)
{
	return pos_enu_raw[1];
}

float ins_get_raw_position_z(void)
{
	return pos_enu_raw[2];
}

/* raw velocity getters */
float ins_get_raw_velocity_x(void)
{
	return vel_enu_raw[0];
}

float ins_get_raw_velocity_y(void)
{
	return vel_enu_raw[1];
}

float ins_get_raw_velocity_z(void)
{
	return vel_enu_raw[2];
}

/* ins fused position getters */
float ins_get_fused_position_x(void)
{
	return pos_enu_fused[0];
}

float ins_get_fused_position_y(void)
{
	return pos_enu_fused[1];
}

float ins_get_fused_position_z(void)
{
	return pos_enu_fused[2];
}

/* ins fused velocity getters */
float ins_get_fused_velocity_x(void)
{
	return vel_enu_fused[0];
}

float ins_get_fused_velocity_y(void)
{
	return vel_enu_fused[1];
}

float ins_get_fused_velocity_z(void)
{
	return vel_enu_fused[2];
}
