#include <math.h>
#include "debug_link.h"
#include "optitrack.h"
#include "imu.h"
#include "barometer.h"
#include "gps_to_enu.h"
#include "system_state.h"
#include "ins_comp_filter.h"
#include "ahrs.h"
#include "ins_sensor_sync.h"
#include "compass.h"
#include "optitrack.h"
#include "led.h"
#include "proj_config.h"
#include "vins_mono.h"
#include "gps.h"
#include "rangefinder.h"

/* position (enu) */
float pos_last[3] = {0.0f};

/* velocity (enu) */
float vel_last[3] = {0.0f};

float pos_a[3];
float vel_a[3];

float dt = 0;
float half_dt_squared = 0;

void ins_comp_filter_init(float _dt)
{
	dt = _dt;
	half_dt_squared = (_dt * _dt) / 2.0f;

	/* position fusion weights */
	pos_a[0] = 0.2f; //weight of using gps raw x position
	pos_a[1] = 0.2f; //weight of using gps raw y position
	pos_a[2] = 0.1f; //weight of using rangefinder distance
	//pos_a[2] = 0.05f; //weight of using barometer height

	/* velocity fusion weights */
	vel_a[0] = 0.2f; //weight of using gps raw x velocity
	vel_a[1] = 0.2f; //weight of using gps raw y velocity
	vel_a[2] = 0.05; //weight of using rangefinder velocity
	//vel_a[2] = 0.05f; //weight of using barometer velocity
}

/* estimate position and velocity using complementary filter */
void ins_comp_filter_predict(float *pos_enu_out, float *vel_enu_out,
                             bool gps_available, bool height_available)
{
	/* read rotation matrix of current attitude */
	float *Rt;
	get_rotation_matrix_b2i(&Rt);

	/* read accelerometer (which is represented in body-fixed frame) */
	float accel_b_ned[3];
	get_accel_lpf(accel_b_ned);

	/* convert from body-fixed frame to inertial frame */
	float accel_i_ned[3];
	accel_i_ned[0] = Rt[0*3 + 0] * accel_b_ned[0] +
	                 Rt[0*3 + 1] * accel_b_ned[1] +
	                 Rt[0*3 + 2] * accel_b_ned[2];
	accel_i_ned[1] = Rt[1*3 + 0] * accel_b_ned[0] +
	                 Rt[1*3 + 1] * accel_b_ned[1] +
	                 Rt[1*3 + 2] * accel_b_ned[2];
	accel_i_ned[2] = Rt[2*3 + 0] * accel_b_ned[0] +
	                 Rt[2*3 + 1] * accel_b_ned[1] +
	                 Rt[2*3 + 2] * accel_b_ned[2];

	/* gravity compensation */
	accel_i_ned[2] += 9.78f;

	/* covert from ned frame to enu frame */
	float accel_i_enu[3];
	accel_i_enu[0] =  accel_i_ned[1];
	accel_i_enu[1] =  accel_i_ned[0];
	accel_i_enu[2] = -accel_i_ned[2];

	/* kinematics update with px, py, vx, vy if gps receiver in presence */
	if(gps_available == true) {
		vel_enu_out[0] = vel_last[0] + (accel_i_enu[0] * dt);
		vel_enu_out[1] = vel_last[1] + (accel_i_enu[1] * dt);

		pos_enu_out[0] = pos_last[0] + (vel_enu_out[0] * dt) +
		                 (accel_i_enu[0] * half_dt_squared);
		pos_enu_out[1] = pos_last[1] + (vel_enu_out[1] * dt) +
		                 (accel_i_enu[1] * half_dt_squared);
	}

	/* kinematics update with pz, vz if complementary height sensor in presence */
	if(height_available == true) {
		vel_enu_out[2] = vel_last[2] + (accel_i_enu[2] * dt);
		pos_enu_out[2] = pos_last[2] + (vel_enu_out[2] * dt) +
		                 (accel_i_enu[2] * half_dt_squared);
	}

	/* save predicted result */
	pos_last[0] = pos_enu_out[0];
	pos_last[1] = pos_enu_out[1];
	pos_last[2] = pos_enu_out[2];
	vel_last[0] = vel_enu_out[0];
	vel_last[1] = vel_enu_out[1];
	vel_last[2] = vel_enu_out[2];
}

/* estimate position and velocity using complementary filter */
void ins_comp_filter_gps_correct(float px_correct, float py_correct,
                                 float vx_correct, float vy_correct,
                                 float *pos_enu_out, float *vel_enu_out)
{
	/* fusion */
	pos_enu_out[0] = (pos_a[0] * px_correct) + ((1.0f - pos_a[0]) * pos_last[0]);
	pos_enu_out[1] = (pos_a[1] * py_correct) + ((1.0f - pos_a[1]) * pos_last[1]);
	vel_enu_out[0] = (vel_a[0] * vx_correct) + ((1.0f - vel_a[0]) * vel_last[0]);
	vel_enu_out[1] = (vel_a[1] * vy_correct) + ((1.0f - vel_a[1]) * vel_last[1]);

	/* save fused result for next iteration */
	pos_last[0] = pos_enu_out[0];
	pos_last[1] = pos_enu_out[1];
	vel_last[0] = vel_enu_out[0];
	vel_last[1] = vel_enu_out[1];
}

void ins_comp_filter_height_correct(float pz_correct, float vz_correct,
                                    float *pos_enu_out, float *vel_enu_out)
{
	/* fusion */
	pos_enu_out[2] = (pos_a[2] * pz_correct) + ((1.0f - pos_a[2]) * pos_last[2]);
	vel_enu_out[2] = (vel_a[2] * vz_correct) + ((1.0f - vel_a[2]) * vel_last[2]);

	/* save fused result for next iteration */
	pos_last[2] = pos_enu_out[2];
	vel_last[2] = vel_enu_out[2];
}

void ins_complementary_filter_estimate(float *pos_enu_raw, float *vel_enu_raw,
                                       float *pos_enu_fused, float *vel_enu_fused)
{
#if (SELECT_POSITION_SENSOR == POSITION_FUSION_USE_OPTITRACK)
	set_rgb_led_service_navigation_on_flag(optitrack_available());
	return;
#elif (SELECT_POSITION_SENSOR == POSITION_FUSION_USE_VINS_MONO)
	set_rgb_led_service_navigation_on_flag(vins_mono_available());
	return;
#endif

	/* check sensor status */
	bool gps_ready = is_gps_available();
	bool compass_ready = is_compass_available();
#if (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_BAROMETER)
	bool height_sensor_ready = is_barometer_available();
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_RANGEFINDER)
	bool height_sensor_ready = rangefinder_available();
#else
	bool height_sensor_ready = false;
	(void)height_sensor_ready; //suppress warning
#endif

	bool sensor_all_ready = gps_ready && compass_ready && height_sensor_ready;

	/* change led state to indicate the sensor status */
	set_rgb_led_service_navigation_on_flag(sensor_all_ready);

	int32_t longitude, latitude;
	float gps_msl_height;
	float gps_ned_vx, gps_ned_vy, gps_ned_vz;

	/* predict position and velocity with kinematics equations (400Hz) */
	ins_comp_filter_predict(pos_enu_fused, vel_enu_fused,
	                        gps_ready, height_sensor_ready);

#if (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_BAROMETER)
	/* height correction (barometer) */
	bool recvd_barometer = ins_barometer_sync_buffer_available();
	if(recvd_barometer == true) {
		float barometer_height, barometer_height_rate;

		/* get barometer data from sync buffer */
		ins_barometer_sync_buffer_pop(&barometer_height,
		                              &barometer_height_rate);
		pos_enu_raw[2] = barometer_height;
		vel_enu_raw[2] = barometer_height_rate;

		//run barometer correction (~50Hz)
		ins_comp_filter_height_correct(barometer_height, barometer_height_rate,
		                               pos_enu_fused, vel_enu_fused);
	}
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_RANGEFINDER)
	/* height correction (rangefinder) */
	bool recvd_rangefinder = ins_rangefinder_sync_buffer_available();
	if(recvd_rangefinder == true) {
		float rangefinder_height, rangefinder_height_rate; //inertial frame height

		/* get rangefinder data from sync buffer */
		ins_rangefinder_sync_buffer_pop(&rangefinder_height,
		                                &rangefinder_height_rate);

		pos_enu_raw[2] = rangefinder_height;
		vel_enu_raw[2] = rangefinder_height_rate;

		//run rangefinder correction (~50Hz)
		ins_comp_filter_height_correct(rangefinder_height, rangefinder_height_rate,
		                               pos_enu_fused, vel_enu_fused);
	}
#endif

	/* position correction (gps module) */
	bool recvd_gps = ins_gps_sync_buffer_available();
	if(recvd_gps == true) {
		/* get gps data from sync buffer */
		ins_gps_sync_buffer_pop(&longitude, &latitude, &gps_msl_height,
		                        &gps_ned_vx, &gps_ned_vy, &gps_ned_vz);

		/* convert gps data from geographic coordinate system to
		 * enu frame */
		float dummy_z;
		longitude_latitude_to_enu(
		        longitude, latitude, 0,
		        &pos_enu_raw[0], &pos_enu_raw[1], &dummy_z);

		if(gps_home_is_set() == false) {
			set_home_longitude_latitude(
			        longitude, latitude, 0/*barometer_height*/); //XXX
		}

		/* convert gps velocity from ned frame to enu frame */
		vel_enu_raw[0] = gps_ned_vy; //x_enu = y_ned
		vel_enu_raw[1] = gps_ned_vx; //y_enu = x_ned

		//run gps correction (~5Hz)
		ins_comp_filter_gps_correct(pos_enu_raw[0], pos_enu_raw[1],
		                            vel_enu_raw[0], vel_enu_raw[1],
		                            pos_enu_fused, vel_enu_fused);
	}
}
