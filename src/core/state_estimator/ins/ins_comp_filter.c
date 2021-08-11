#include <math.h>
#include "debug_link.h"
#include "optitrack.h"
#include "imu.h"
#include "barometer.h"
#include "geographic_transform.h"
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

/* position (ned) */
float pos_last[3] = {0.0f};

/* velocity (ned) */
float vel_last[3] = {0.0f};

/* fusion weight */
float weight_pos[3];
float weight_vel[3];

float dt = 0;
float half_dt_squared = 0;

void ins_comp_filter_init(float _dt)
{
	dt = _dt;
	half_dt_squared = (_dt * _dt) / 2.0f;

	/* position fusion weights */
	weight_pos[0] = 0.2f; //weight of using gps raw x position
	weight_pos[1] = 0.2f; //weight of using gps raw y position
	weight_pos[2] = 0.1f; //weight of using rangefinder distance
	//weight_pos[2] = 0.05f; //weight of using barometer height

	/* velocity fusion weights */
	weight_vel[0] = 0.2f; //weight of using gps raw x velocity
	weight_vel[1] = 0.2f; //weight of using gps raw y velocity
	weight_vel[2] = 0.05; //weight of using rangefinder velocity
	//weight_vel[2] = 0.05f; //weight of using barometer velocity
}

void set_ins_complementary_filter_state(float *pos_ned, float *vel_ned)
{
	pos_last[0] = pos_ned[0];
	pos_last[1] = pos_ned[1];
	pos_last[2] = pos_ned[2];
	vel_last[0] = vel_ned[0];
	vel_last[1] = vel_ned[1];
	vel_last[2] = vel_ned[2];
}

/* estimate position and velocity using complementary filter */
void ins_comp_filter_predict(float *pos_ned_out, float *vel_ned_out,
                             bool gps_available, bool height_available)
{
	/* read rotation matrix of current attitude */
	float *Rt;
	get_rotation_matrix_b2i(&Rt);

	/* read accelerometer (body-fixed frame) */
	float accel_b_ned[3];
	get_accel_lpf(accel_b_ned);

	/* convert from body-fixed frame to inertial frame */
	float accel_i[3];
	accel_i[0] = Rt[0*3 + 0] * accel_b_ned[0] +
	             Rt[0*3 + 1] * accel_b_ned[1] +
	             Rt[0*3 + 2] * accel_b_ned[2];
	accel_i[1] = Rt[1*3 + 0] * accel_b_ned[0] +
	             Rt[1*3 + 1] * accel_b_ned[1] +
	             Rt[1*3 + 2] * accel_b_ned[2];
	accel_i[2] = Rt[2*3 + 0] * accel_b_ned[0] +
	             Rt[2*3 + 1] * accel_b_ned[1] +
	             Rt[2*3 + 2] * accel_b_ned[2];

	/* gravity compensation */
	accel_i[2] += 9.78f;

	/* kinematics update of px, py, vx, vy if gps receiver in presence */
	if(gps_available == true) {
		vel_ned_out[0] = vel_last[0] + (accel_i[0] * dt);
		vel_ned_out[1] = vel_last[1] + (accel_i[1] * dt);

		pos_ned_out[0] = pos_last[0] + (vel_ned_out[0] * dt) +
		                 (accel_i[0] * half_dt_squared);
		pos_ned_out[1] = pos_last[1] + (vel_ned_out[1] * dt) +
		                 (accel_i[1] * half_dt_squared);
	}

	/* kinematics update of pz, vz if complementary height sensor in presence */
	if(height_available == true) {
		vel_ned_out[2] = vel_last[2] + (accel_i[2] * dt);
		pos_ned_out[2] = pos_last[2] + (vel_ned_out[2] * dt) +
		                 (accel_i[2] * half_dt_squared);
	}

	/* save predicted result */
	pos_last[0] = pos_ned_out[0];
	pos_last[1] = pos_ned_out[1];
	pos_last[2] = pos_ned_out[2];
	vel_last[0] = vel_ned_out[0];
	vel_last[1] = vel_ned_out[1];
	vel_last[2] = vel_ned_out[2];
}

/* estimate position and velocity using complementary filter */
void ins_comp_filter_gps_correct(float px_correct, float py_correct,
                                 float vx_correct, float vy_correct,
                                 float *pos_ned_out, float *vel_ned_out)
{
	/* fusion */
	pos_ned_out[0] = (weight_pos[0] * px_correct) + ((1.0f - weight_pos[0]) * pos_last[0]);
	pos_ned_out[1] = (weight_pos[1] * py_correct) + ((1.0f - weight_pos[1]) * pos_last[1]);
	vel_ned_out[0] = (weight_vel[0] * vx_correct) + ((1.0f - weight_vel[0]) * vel_last[0]);
	vel_ned_out[1] = (weight_vel[1] * vy_correct) + ((1.0f - weight_vel[1]) * vel_last[1]);

	/* save fused result for next iteration */
	pos_last[0] = pos_ned_out[0];
	pos_last[1] = pos_ned_out[1];
	vel_last[0] = vel_ned_out[0];
	vel_last[1] = vel_ned_out[1];
}

void ins_comp_filter_height_correct(float pz_correct, float vz_correct,
                                    float *pos_ned_out, float *vel_ned_out)
{
	/* fusion */
	pos_ned_out[2] = (weight_pos[2] * pz_correct) + ((1.0f - weight_pos[2]) * pos_last[2]);
	vel_ned_out[2] = (weight_vel[2] * vz_correct) + ((1.0f - weight_vel[2]) * vel_last[2]);

	/* save fused result for next iteration */
	pos_last[2] = pos_ned_out[2];
	vel_last[2] = vel_ned_out[2];
}

bool ins_complementary_filter_ready(void)
{
	bool gps_ready = is_gps_available();
	bool compass_ready = is_compass_available();
#if (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_BAROMETER)
	bool height_ready = is_barometer_available();
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_RANGEFINDER)
	bool height_ready = rangefinder_available();
#else
	bool height_ready = false;
#endif

	return gps_ready && compass_ready && height_ready;
}

void ins_complementary_filter_estimate(float *pos_ned_raw, float *vel_ned_raw,
                                       float *pos_ned_fused, float *vel_ned_fused)
{
	/* check sensor status */
	bool gps_compass_ready = is_gps_available() && is_compass_available();
#if (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_BAROMETER)
	bool height_ready = is_barometer_available();
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_RANGEFINDER)
	bool height_ready = rangefinder_available();
#else
	bool height_ready = false;
#endif

	int32_t longitude, latitude;
	float gps_msl_height;
	float gps_ned_vx, gps_ned_vy, gps_ned_vz;

	/* predict position and velocity with kinematics equations (400Hz) */
	ins_comp_filter_predict(pos_ned_fused, vel_ned_fused,
	                        gps_compass_ready, height_ready);

#if (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_BAROMETER)
	/* height correction (barometer) */
	bool recvd_barometer = ins_barometer_sync_buffer_available();
	if(recvd_barometer == true) {
		float barometer_height, barometer_height_rate;

		/* get barometer data from sync buffer */
		ins_barometer_sync_buffer_pop(&barometer_height,
		                              &barometer_height_rate);
		pos_ned_raw[2] = -barometer_height; //ned frame
		vel_ned_raw[2] = -barometer_height_rate;

		//run barometer correction (~50Hz)
		ins_comp_filter_height_correct(pos_ned_raw[2], vel_ned_raw[2],
		                               pos_ned_fused, vel_ned_fused);
	}
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_RANGEFINDER)
	/* height correction (rangefinder) */
	bool recvd_rangefinder = ins_rangefinder_sync_buffer_available();
	if(recvd_rangefinder == true) {
		float rangefinder_height, rangefinder_height_rate; //inertial frame height

		/* get rangefinder data from sync buffer */
		ins_rangefinder_sync_buffer_pop(&rangefinder_height,
		                                &rangefinder_height_rate);

		pos_ned_raw[2] = -rangefinder_height; //ned frame
		vel_ned_raw[2] = -rangefinder_height_rate;

		//run rangefinder correction (~40Hz)
		ins_comp_filter_height_correct(pos_ned_raw[2], vel_ned_raw[2],
		                               pos_ned_fused, vel_ned_fused);
	}
#endif

	/* position correction (gps) */
	bool recvd_gps = ins_gps_sync_buffer_available();
	if(recvd_gps == true) {
		/* get gps data from sync buffer */
		ins_gps_sync_buffer_pop(&longitude, &latitude, &gps_msl_height,
		                        &gps_ned_vx, &gps_ned_vy, &gps_ned_vz);

		/* convert gps data from geographic coordinate system to ned frame */
		longitude_latitude_to_ned(pos_ned_raw, longitude, latitude, 0);

		if(gps_home_is_set() == false) {
			set_home_longitude_latitude(longitude, latitude, 0);
		}

		vel_ned_raw[0] = gps_ned_vx;
		vel_ned_raw[1] = gps_ned_vy;

		//run gps correction (~5Hz)
		ins_comp_filter_gps_correct(pos_ned_raw[0], pos_ned_raw[1],
		                            vel_ned_raw[0], vel_ned_raw[1],
		                            pos_ned_fused, vel_ned_fused);
	}
}
