#include "ins_sensor_sync.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define DUMMY_COMPASS_PRESCALER_RELOAD    8   //50Hz
#define DUMMY_BAROMETER_PRESCALER_RELOAD  8   //50Hz
#define DUMMY_GPS_PRESCALER_RELOAD        40  //10Hz

void dummy_compass_handler(BaseType_t *higher_priority_task_woken)
{
	float dummy_mag[3] = {45.0f, 0.0f, 0.0f};
	ins_compass_sync_buffer_push_from_isr(dummy_mag, higher_priority_task_woken);
}

void dummy_barometer_handler(BaseType_t *higher_priority_task_woken)
{
	float dummy_height = 0.0f;
	float dummy_height_rate = 0.0f;
	ins_barometer_sync_buffer_push_from_isr(dummy_height, dummy_height_rate,
	                                        higher_priority_task_woken);
}

void dummy_gps_handler(BaseType_t *higher_priority_task_woken)
{
	int32_t dummy_longitude = 1207299564; //120.7299564
	int32_t dummy_latitude = 242500496;   //24.2500496
	float dummy_height_msl = 0.0f;
	float dummy_vx_ned = 0.0f;
	float dummy_vy_ned = 0.0f;
	float dummy_vz_ned = 0.0f;
	ins_gps_sync_buffer_push_from_isr(dummy_longitude, dummy_latitude, dummy_height_msl,
	                                  dummy_vx_ned, dummy_vy_ned, dummy_vz_ned,
	                                  higher_priority_task_woken);
}

void dummy_sensors_update_isr_handler(BaseType_t *higher_priority_task_woken)
{
	/* this function is called by timer3 isr handler with 500Hz */

	static int compass_prescaler = DUMMY_COMPASS_PRESCALER_RELOAD;
	static int barometer_prescaler = DUMMY_BAROMETER_PRESCALER_RELOAD;
	static int gps_prescaler = DUMMY_GPS_PRESCALER_RELOAD;
	compass_prescaler--;

	if(compass_prescaler == 0) {
		compass_prescaler = DUMMY_COMPASS_PRESCALER_RELOAD;
		dummy_compass_handler(higher_priority_task_woken);
	}

	barometer_prescaler--;
	if(barometer_prescaler == 0) {
		barometer_prescaler = DUMMY_BAROMETER_PRESCALER_RELOAD;
		dummy_barometer_handler(higher_priority_task_woken);
	}

	gps_prescaler--;
	if(gps_prescaler == 0) {
		gps_prescaler = DUMMY_GPS_PRESCALER_RELOAD;
		dummy_gps_handler(higher_priority_task_woken);
	}
}
