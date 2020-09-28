#include <stdint.h>
#include <stdbool.h>
#include "debug_link.h"
#include "sys_time.h"
#include "imu.h"
#include "compass.h"
#include "barometer.h"
#include "position_sensor.h"

void send_ins_sensor_debug_message(debug_msg_t *payload)
{
	/* record sensor data of accelerometer, gyroscope,
         * magnetometer, gps and barometer */

	float current_time_ms;
	float accel_lpf[3];
	float gyro_raw[3];
	float mag_raw[3];
	float longitude, latitude, height;
	float vx, vy, vz;

	current_time_ms = get_sys_time_ms();
	get_accel_lpf(accel_lpf);
	get_gyro_raw(gyro_raw);
	get_compass_raw(mag_raw);
	get_gps_longitude_latitude_height(&longitude, &latitude, &height);
	height = barometer_get_relative_altitude();

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
        pack_debug_debug_message_float(&height, payload);
        pack_debug_debug_message_float(&vx, payload);
        pack_debug_debug_message_float(&vy, payload);
        pack_debug_debug_message_float(&vz, payload);
}
