#include <stdint.h>
#include <stdbool.h>
#include "debug_link.h"
#include "sys_time.h"
#include "imu.h"
#include "compass.h"
#include "barometer.h"
#include "position_state.h"
#include "ins.h"
#include "gps.h"

void send_ins_sensor_debug_message(debug_msg_t *payload)
{
	/* record sensor data of accelerometer, gyroscope,
	 * magnetometer, gps and barometer */

	float current_time_ms;
	float accel_lpf[3];
	float gyro_raw[3];
	float mag_raw[3];
	float longitude, latitude, gps_height;
	float vx, vy, vz;
	float barometer_height, barometer_velocity;

	current_time_ms = get_sys_time_ms();
	get_accel_lpf(accel_lpf);
	get_gyro_raw(gyro_raw);
	get_compass_raw(mag_raw);
	get_gps_longitude_latitude_height(&longitude, &latitude, &gps_height);
	get_gps_velocity_ned(&vx, &vy, &vz);
	barometer_height = barometer_get_relative_altitude();
	barometer_velocity = barometer_get_relative_altitude_rate();

	//XXX
	gyro_raw[0] *= 0.0174533;
	gyro_raw[1] *= 0.0174533;
	gyro_raw[2] *= 0.0174533;

	pack_debug_debug_message_header(payload, MESSAGE_ID_INS_SENSOR);
	pack_debug_debug_message_float(&current_time_ms, payload);
	pack_debug_debug_message_float(&accel_lpf[0], payload);
	pack_debug_debug_message_float(&accel_lpf[1], payload);
	pack_debug_debug_message_float(&accel_lpf[2], payload);
	pack_debug_debug_message_float(&gyro_raw[0], payload);
	pack_debug_debug_message_float(&gyro_raw[1], payload);
	pack_debug_debug_message_float(&gyro_raw[2], payload);
	pack_debug_debug_message_float(&mag_raw[0], payload);
	pack_debug_debug_message_float(&mag_raw[1], payload);
	pack_debug_debug_message_float(&mag_raw[2], payload);
	pack_debug_debug_message_float(&longitude, payload);
	pack_debug_debug_message_float(&latitude, payload);
	pack_debug_debug_message_float(&gps_height, payload);
	pack_debug_debug_message_float(&vx, payload);
	pack_debug_debug_message_float(&vy, payload);
	pack_debug_debug_message_float(&vz, payload);
	pack_debug_debug_message_float(&barometer_height, payload);
	pack_debug_debug_message_float(&barometer_velocity, payload);
}

void send_ins_raw_position_debug_message(debug_msg_t *payload)
{
	float pos_enu[3]; //x, y from gps, z from height sensor
	ins_get_raw_position(pos_enu);

	pack_debug_debug_message_header(payload, MESSAGE_ID_INS_RAW_POSITION);
	pack_debug_debug_message_float(&pos_enu[0], payload);
	pack_debug_debug_message_float(&pos_enu[1], payload);
	pack_debug_debug_message_float(&pos_enu[2], payload);
}

void send_ins_fusion_debug_message(debug_msg_t *payload)
{
	float pos_enu_raw[3], vel_enu_raw[3];
	ins_get_raw_position(pos_enu_raw);
	ins_get_raw_velocity(vel_enu_raw);

	float pos_enu_fused[3], vel_enu_fused[3];
	ins_get_fused_position(pos_enu_fused);
	ins_get_fused_velocity(vel_enu_fused);

	float curr_time_ms = get_sys_time_ms();
	float satellite_num = get_gps_satellite_numbers();
	float gps_update_rate = get_gps_update_freq();

	pack_debug_debug_message_header(payload, MESSAGE_ID_INS_FUSION);

	pack_debug_debug_message_float(&curr_time_ms, payload);
	pack_debug_debug_message_float(&satellite_num, payload);
	pack_debug_debug_message_float(&gps_update_rate, payload);
	pack_debug_debug_message_float(&pos_enu_raw[0], payload);
	pack_debug_debug_message_float(&pos_enu_raw[1], payload);
	pack_debug_debug_message_float(&pos_enu_raw[2], payload);
	pack_debug_debug_message_float(&pos_enu_fused[0], payload);
	pack_debug_debug_message_float(&pos_enu_fused[1], payload);
	pack_debug_debug_message_float(&pos_enu_fused[2], payload);
	pack_debug_debug_message_float(&vel_enu_raw[0], payload);
	pack_debug_debug_message_float(&vel_enu_raw[1], payload);
	pack_debug_debug_message_float(&vel_enu_raw[2], payload);
	pack_debug_debug_message_float(&vel_enu_fused[0], payload);
	pack_debug_debug_message_float(&vel_enu_fused[1], payload);
	pack_debug_debug_message_float(&vel_enu_fused[2], payload);
}
