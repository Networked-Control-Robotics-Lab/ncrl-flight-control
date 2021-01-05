#include <stdbool.h>
#include <stdint.h>
#include "gps_to_enu.h"
#include "position_sensor.h"
#include "barometer.h"
#include "compass.h"
#include "ins_complementary.h"
#include "gps.h"
#include "led.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "proj_config.h"
#include "ublox_m8n.h"
#include "ins_sensor_sync.h"

#define INS_LOOP_PERIOD 0.0025f //400Hz

float pos_enu_raw[3];
float vel_enu_raw[3];
float pos_enu_fused[3];
float vel_enu_fused[3];

void ins_init(void)
{
	ins_sync_buffer_init();

#if (SELECT_INS == INS_COMPLEMENTARY_FILTER)
	comp_nav_init(INS_LOOP_PERIOD);
#endif
}

bool ins_check_sensor_status(void)
{
	bool sensor_all_ready;

	/* check if home position is set */
	bool home_set = gps_home_is_set();

	/* set home position when gps signal is stable */
	if((home_set == false) && (get_gps_fix_type() == GPS_FIX_TYPE_3D_FIX)) {
		float longitude, latitude, gps_height;
		get_gps_longitude_latitude_height(&longitude, &latitude, &gps_height);

		float barometer_height;
		barometer_height = barometer_get_relative_altitude();

		set_home_longitude_latitude(longitude, latitude, barometer_height);
	}

	/* check sensor status */
	bool gps_ready = is_gps_available();
	bool compass_ready = is_compass_available();
	bool barometer_ready = is_barometer_available();

	sensor_all_ready =
	        home_set && gps_ready && compass_ready && barometer_ready;

	/* change led state to indicate the sensor status */
	if(sensor_all_ready == true) {
		led_on(LED_G);
	} else {
		led_off(LED_G);
	}

	return sensor_all_ready;
}

void ins_state_estimate(void)
{
	ins_check_sensor_status();

	if(gps_home_is_set() == false) {
		return;
	}

	/* predict position and velocity with kinematic equations */
	pos_vel_complementary_filter_predict(pos_enu_fused, vel_enu_fused);

	/* correct x-y position and velocity with gps data */
	float gps_longitude, gps_latitude, gps_msl_height;
	float gps_ned_vx, gps_ned_vy, gps_ned_vz;
	bool recvd_gps =
	        ins_gps_sync_buffer_pop(&gps_longitude, &gps_latitude, &gps_msl_height,
	                                &gps_ned_vx, &gps_ned_vy, &gps_ned_vz);
	if(recvd_gps == true) {
		/* convert gps data from geographic coordinate system to enu frame */
		longitude_latitude_to_enu(gps_longitude, gps_latitude, gps_msl_height,
		                          &pos_enu_raw[0], &pos_enu_raw[1], &pos_enu_raw[2]);
		pos_enu_raw[2] = 0; //ommit z direction in coordinate transform

		/* read gps velocity and convert it from ned frame into enu frame */
		get_gps_velocity_ned(&gps_ned_vx, &gps_ned_vy, &gps_ned_vz);
		vel_enu_raw[0] = gps_ned_vy; //x_enu = y_ned
		vel_enu_raw[1] = gps_ned_vx; //y_enu = x_ned

		//run gps correction
		pos_vel_complementary_filter_gps_correct(pos_enu_raw, vel_enu_raw,
		                pos_enu_fused, vel_enu_fused);
	}

	/* correct z position and velocity with barometer (~50Hz) */
	bool recvd_barometer =
	        ins_barometer_sync_buffer_pop(&pos_enu_raw[2], &vel_enu_raw[2]);
	if(recvd_barometer == true) {
		//run barometer correction
		pos_vel_complementary_filter_barometer_correct(
		        pos_enu_raw, vel_enu_raw, pos_enu_fused, vel_enu_fused);
	}
}

void ins_get_raw_position(float *pos_enu)
{
	pos_enu[0] = pos_enu_raw[0];
	pos_enu[1] = pos_enu_raw[1];
	pos_enu[2] = pos_enu_raw[2];
}

void ins_get_raw_velocity(float *vel_enu)
{
	vel_enu[0] = vel_enu_raw[0];
	vel_enu[1] = vel_enu_raw[1];
	vel_enu[2] = vel_enu_raw[2];
}

void ins_get_fused_position(float *pos_enu)
{
	pos_enu[0] = pos_enu_fused[0];
	pos_enu[1] = pos_enu_fused[1];
	pos_enu[2] = pos_enu_fused[2];
}

void ins_get_fused_velocity(float *vel_enu)
{
	vel_enu[0] = vel_enu_fused[0];
	vel_enu[1] = vel_enu_fused[1];
	vel_enu[2] = vel_enu_fused[2];
}
