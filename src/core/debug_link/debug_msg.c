#include <stdint.h>
#include <stdbool.h>
#include "debug_link.h"
#include "sys_time.h"
#include "imu.h"
#include "compass.h"
#include "barometer.h"
#include "system_state.h"
#include "ins.h"
#include "gps.h"
#include "optitrack.h"
#include "rangefinder.h"
#include "proj_config.h"
#include "vins_mono.h"
#include "vio.h"
#include "se3_math.h"
#include "quaternion.h"
#include "ins_eskf.h"

void send_alt_est_debug_message(debug_msg_t *payload)
{
	float altitude = ins_get_fused_position_enu_z();
	float optitrack_z = optitrack_get_position_enu_z();
	float altitude_rate = ins_get_fused_velocity_enu_z();
	float optitrack_vz = optitrack_get_velocity_enu_z();

	pack_debug_debug_message_header(payload, MESSAGE_ID_ALT_EST);
	pack_debug_debug_message_float(&altitude, payload);
	pack_debug_debug_message_float(&optitrack_z, payload);
	pack_debug_debug_message_float(&altitude_rate, payload);
	pack_debug_debug_message_float(&optitrack_vz, payload);
}

void send_ins_sensor_debug_message(debug_msg_t *payload)
{
	/* record sensor data of accelerometer, gyroscope,
	 * magnetometer, gps and barometer */

	float current_time_ms;
	//float gps_last_update_time_ms;
	float accel_lpf[3];
	float gyro_raw[3];
	float mag_raw[3];
	int32_t longitude, latitude, gps_height;
	float vx, vy, vz;
	float height = 0, height_velocity = 0;

	current_time_ms = get_sys_time_ms();
	//gps_last_update_time_ms =  get_gps_last_update_time_ms();
	get_accel_lpf(accel_lpf);
	get_gyro_raw(gyro_raw);
	get_compass_raw(mag_raw);
	get_gps_longitude_latitude_height_s32(&longitude, &latitude, &gps_height);
	get_gps_velocity_ned(&vx, &vy, &vz);

#if (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_BAROMETER)
	height = barometer_get_relative_altitude();
	height_velocity = barometer_get_relative_altitude_rate();
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_RANGEFINDER)
	height = rangefinder_get_distance();
	height_velocity = rangefinder_get_velocity();
#endif

	//XXX
	gyro_raw[0] *= 0.0174533;
	gyro_raw[1] *= 0.0174533;
	gyro_raw[2] *= 0.0174533;

	pack_debug_debug_message_header(payload, MESSAGE_ID_INS_SENSOR);
	pack_debug_debug_message_float(&current_time_ms, payload);
	//pack_debug_debug_message_float(&gps_last_update_time_ms, payload);
	pack_debug_debug_message_float(&accel_lpf[0], payload);
	pack_debug_debug_message_float(&accel_lpf[1], payload);
	pack_debug_debug_message_float(&accel_lpf[2], payload);
	pack_debug_debug_message_float(&gyro_raw[0], payload);
	pack_debug_debug_message_float(&gyro_raw[1], payload);
	pack_debug_debug_message_float(&gyro_raw[2], payload);
	pack_debug_debug_message_float(&mag_raw[0], payload);
	pack_debug_debug_message_float(&mag_raw[1], payload);
	pack_debug_debug_message_float(&mag_raw[2], payload);
	pack_debug_debug_message_int32(&longitude, payload);
	pack_debug_debug_message_int32(&latitude, payload);
	pack_debug_debug_message_int32(&gps_height, payload);
	pack_debug_debug_message_float(&vx, payload);
	pack_debug_debug_message_float(&vy, payload);
	pack_debug_debug_message_float(&vz, payload);
	pack_debug_debug_message_float(&height, payload);
	pack_debug_debug_message_float(&height_velocity, payload);
}

void send_ins_raw_position_debug_message(debug_msg_t *payload)
{
	float pos_enu[3]; //x, y from gps, z from height sensor
	pos_enu[0] = ins_get_raw_position_enu_x();
	pos_enu[1] = ins_get_raw_position_enu_y();
	pos_enu[2] = ins_get_raw_position_enu_z();

	pack_debug_debug_message_header(payload, MESSAGE_ID_INS_RAW_POSITION);
	pack_debug_debug_message_float(&pos_enu[0], payload);
	pack_debug_debug_message_float(&pos_enu[1], payload);
	pack_debug_debug_message_float(&pos_enu[2], payload);
}

void send_ins_fusion_debug_message(debug_msg_t *payload)
{
	float curr_time_ms = get_sys_time_ms();
	float satellite_num = get_gps_satellite_numbers();
	float gps_update_rate = get_gps_update_freq();

	float pos_raw_x = ins_get_raw_position_enu_x();
	float pos_raw_y = ins_get_raw_position_enu_y();
	float pos_raw_z = ins_get_raw_position_enu_z();

	float pos_fused_x = ins_get_fused_position_enu_x();
	float pos_fused_y = ins_get_fused_position_enu_y();
	float pos_fused_z = ins_get_fused_position_enu_z();

	float vel_raw_x = ins_get_raw_velocity_enu_x();
	float vel_raw_y = ins_get_raw_velocity_enu_y();
	float vel_raw_z = ins_get_raw_velocity_enu_z();

	float vel_fused_x = ins_get_fused_velocity_enu_x();
	float vel_fused_y = ins_get_fused_velocity_enu_y();
	float vel_fused_z = ins_get_fused_velocity_enu_z();

	pack_debug_debug_message_header(payload, MESSAGE_ID_INS_FUSION);

	pack_debug_debug_message_float(&curr_time_ms, payload);
	pack_debug_debug_message_float(&satellite_num, payload);
	pack_debug_debug_message_float(&gps_update_rate, payload);
	pack_debug_debug_message_float(&pos_raw_x, payload);
	pack_debug_debug_message_float(&pos_raw_y, payload);
	pack_debug_debug_message_float(&pos_raw_z, payload);
	pack_debug_debug_message_float(&pos_fused_x, payload);
	pack_debug_debug_message_float(&pos_fused_y, payload);
	pack_debug_debug_message_float(&pos_fused_z, payload);
	pack_debug_debug_message_float(&vel_raw_x, payload);
	pack_debug_debug_message_float(&vel_raw_y, payload);
	pack_debug_debug_message_float(&vel_raw_z, payload);
	pack_debug_debug_message_float(&vel_fused_x, payload);
	pack_debug_debug_message_float(&vel_fused_y, payload);
	pack_debug_debug_message_float(&vel_fused_z, payload);
}

void send_gps_accuracy_debug_message(debug_msg_t *payload)
{
	float curr_time_ms = get_sys_time_ms();

	float h_acc, v_acc;
	get_gps_position_uncertainty(&h_acc, &v_acc);

	/* convert unit from [mm] to [m] */
	h_acc *= 0.001;
	v_acc *= 0.001;

	pack_debug_debug_message_header(payload, MESSAGE_ID_GPS_ACCURACY);
	pack_debug_debug_message_float(&curr_time_ms, payload);
	pack_debug_debug_message_float(&h_acc, payload);
	pack_debug_debug_message_float(&v_acc, payload);
}

void send_optitrack_vio_debug_message(debug_msg_t *payload)
{
	/* get system time */
	float curr_time_ms = get_sys_time_ms();

	/* get position and quaternion from optitrack */
	float p_optitrack[3], q_optitrack[4];
	optitrack_get_position_enu(p_optitrack);
	optitrack_get_quaternion(q_optitrack);
	//ins_get_fused_position_enu(p_optitrack);
	//ins_eskf_get_attitude_quaternion(q_optitrack);

	/* get position and quaternion from vio */
	float p_vio[3], q_vio[4];
	vio_get_position_enu(p_vio);
	vio_get_quaternion(q_vio);

	/* convert quaternion to eulers angle */
	euler_t euler_optitrack, euler_vio;
	quat_to_euler(q_optitrack, &euler_optitrack);
	quat_to_euler(q_vio, &euler_vio);

	/* convert eulers angle from radian to degree */
	euler_optitrack.roll = rad_to_deg(euler_optitrack.roll);
	euler_optitrack.pitch = rad_to_deg(euler_optitrack.pitch);
	euler_optitrack.yaw = rad_to_deg(euler_optitrack.yaw);
	euler_vio.roll = rad_to_deg(euler_vio.roll);
	euler_vio.pitch = rad_to_deg(euler_vio.pitch);
	euler_vio.yaw = rad_to_deg(euler_vio.yaw);

	pack_debug_debug_message_header(payload, MESSAGE_ID_OPTITRACK_VIO);
	pack_debug_debug_message_float(&curr_time_ms, payload);
	pack_debug_debug_message_float(&p_optitrack[0], payload);
	pack_debug_debug_message_float(&p_optitrack[1], payload);
	pack_debug_debug_message_float(&p_optitrack[2], payload);
	pack_debug_debug_message_float(&p_vio[0], payload);
	pack_debug_debug_message_float(&p_vio[1], payload);
	pack_debug_debug_message_float(&p_vio[2], payload);
	pack_debug_debug_message_float(&euler_optitrack.roll, payload);
	pack_debug_debug_message_float(&euler_optitrack.pitch, payload);
	pack_debug_debug_message_float(&euler_optitrack.yaw, payload);
	pack_debug_debug_message_float(&euler_vio.roll, payload);
	pack_debug_debug_message_float(&euler_vio.pitch, payload);
	pack_debug_debug_message_float(&euler_vio.yaw, payload);
}

void send_gnss_ins_cov_norm_debug_message(debug_msg_t *payload)
{
	float h_acc, v_acc;
	get_gps_position_uncertainty(&h_acc, &v_acc);
	float eskf_cov_norm = ins_eskf_get_covariance_matrix_norm();

	/* convert unit from [mm] to [m] */
	h_acc *= 0.001;

	pack_debug_debug_message_header(payload, MESSAGE_ID_ESKF_COV_NORM);
	pack_debug_debug_message_float(&h_acc, payload);
	pack_debug_debug_message_float(&eskf_cov_norm, payload);
}
