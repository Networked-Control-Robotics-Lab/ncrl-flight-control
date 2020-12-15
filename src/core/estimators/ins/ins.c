#include <stdbool.h>
#include <stdint.h>
#include "gps_to_enu.h"
#include "position_sensor.h"
#include "barometer.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"

float ins_raw_enu_x;
float ins_raw_enu_y;
float ins_raw_enu_z;

void ins_init(void)
{
}

bool ins_check_gps_status(void)
{
	/* check if home position is set */
	if(gps_home_is_set() == false) {
		if(get_gps_fix_type() == GPS_FIX_TYPE_3D_FIX) {
			float longitude, latitude, gps_height;
			get_gps_longitude_latitude_height(&longitude, &latitude, &gps_height);

			float barometer_height;
			barometer_height = barometer_get_relative_altitude();

			set_home_longitude_latitude(longitude, latitude, barometer_height);
		} else {
			return false; //gps receiver is not ready
		}
	}

	return true; //good
}

void ins_state_estimate(void)
{
	if(ins_check_gps_status() == true) {
		float longitude, latitude, gps_height;
		get_gps_longitude_latitude_height(&longitude, &latitude, &gps_height);

		float barometer_height;
		barometer_height = barometer_get_relative_altitude();

		longitude_latitude_to_enu(longitude, latitude, barometer_height,
		                          &ins_raw_enu_x, &ins_raw_enu_y, &ins_raw_enu_z);
	}
}

void ins_get_raw_position(float *pos_enu)
{
	pos_enu[0] = ins_raw_enu_x;
	pos_enu[1] = ins_raw_enu_y;
	pos_enu[2] = ins_raw_enu_z;
}
