#include <stdbool.h>
#include "ins_sensor_sync.h"
#include "sys_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define INS_SYNC_BAROMETER_BUF_SIZE   50
#define INS_SYNC_GPS_BUF_SIZE         10
#define INS_SYNC_COMPASS_BUF_SIZE     50
#define INS_SYNC_RANGEFINDER_BUF_SIZE 50

QueueHandle_t ins_sync_barometer_queue;
QueueHandle_t ins_sync_gps_queue;
QueueHandle_t ins_sync_compass_queue;
QueueHandle_t ins_sync_rangefinder_queue;

bool sync_buffer_is_ready = false;

void ins_sync_buffer_init(void)
{
	ins_sync_barometer_queue =
	        xQueueCreate(INS_SYNC_BAROMETER_BUF_SIZE,
	                     sizeof(ins_sync_barometer_item_t));

	ins_sync_gps_queue =
	        xQueueCreate(INS_SYNC_GPS_BUF_SIZE, sizeof(ins_sync_gps_item_t));

	ins_sync_compass_queue =
	        xQueueCreate(INS_SYNC_COMPASS_BUF_SIZE, sizeof(ins_sync_compass_item_t));

	ins_sync_rangefinder_queue =
	        xQueueCreate(INS_SYNC_RANGEFINDER_BUF_SIZE, sizeof(ins_sync_rangefinder_item_t));

	sync_buffer_is_ready = true;
}

bool ins_sync_buffer_is_ready(void)
{
	return sync_buffer_is_ready;
}

/*===========*
 * barometer *
 *===========*/

bool ins_barometer_sync_buffer_available(void)
{
	UBaseType_t free_size = uxQueueSpacesAvailable(ins_sync_barometer_queue);
	UBaseType_t available_size = INS_SYNC_BAROMETER_BUF_SIZE - free_size;

	if(available_size > 0) {
		return true;
	} else {
		return false;
	}
}

void ins_barometer_sync_buffer_push(float height, float height_rate)
{
	ins_sync_barometer_item_t barometer_item = {
		.timestamp_s = get_sys_time_s(),
		.height = height,
		.height_rate = height_rate
	};

	xQueueSendToBack(ins_sync_barometer_queue, &barometer_item, 0);
}

void ins_barometer_sync_buffer_push_from_isr(float height, float height_rate,
                BaseType_t *higher_priority_task_woken)
{
	ins_sync_barometer_item_t barometer_item = {
		.timestamp_s = get_sys_time_s(),
		.height = height,
		.height_rate = height_rate
	};

	xQueueSendToBackFromISR(ins_sync_barometer_queue, &barometer_item, higher_priority_task_woken);
}

bool ins_barometer_sync_buffer_pop(float *height, float *height_rate)
{
	ins_sync_barometer_item_t recvd_barometer_item;
	if(xQueueReceive(ins_sync_barometer_queue, &recvd_barometer_item, 0) == pdTRUE) {
		*height = recvd_barometer_item.height;
		*height_rate = recvd_barometer_item.height_rate;
		return true;
	} else {
		return false;
	}
}

/*==============*
 * gps receiver *
 *==============*/

bool ins_gps_sync_buffer_available(void)
{
	UBaseType_t free_size = uxQueueSpacesAvailable(ins_sync_gps_queue);
	UBaseType_t available_size = INS_SYNC_GPS_BUF_SIZE - free_size;

	if(available_size > 0) {
		return true;
	} else {
		return false;
	}
}

void ins_gps_sync_buffer_push(int32_t longitude, int32_t latitude, float height_msl,
                              float vx_ned, float vy_ned, float vz_ned)
{
	ins_sync_gps_item_t gps_item = {
		.timestamp_s = get_sys_time_s(),
		.longitude = longitude,
		.latitude = latitude,
		.height_msl = height_msl,
		.vx_ned = vx_ned,
		.vy_ned = vy_ned,
		.vz_ned = vz_ned
	};

	xQueueSendToBack(ins_sync_gps_queue, &gps_item, 0);
}

void ins_gps_sync_buffer_push_from_isr(int32_t longitude, int32_t latitude, float height_msl,
                                       float vx_ned, float vy_ned, float vz_ned,
                                       BaseType_t *higher_priority_task_woken)
{
	ins_sync_gps_item_t gps_item = {
		.timestamp_s = get_sys_time_s(),
		.longitude = longitude,
		.latitude = latitude,
		.height_msl = height_msl,
		.vx_ned = vx_ned,
		.vy_ned = vy_ned,
		.vz_ned = vz_ned
	};

	xQueueSendToBackFromISR(ins_sync_gps_queue, &gps_item, higher_priority_task_woken);
}

bool ins_gps_sync_buffer_pop(int32_t *longitude, int32_t *latitude, float *height_msl,
                             float *vx_ned, float *vy_ned, float *vz_ned)
{
	ins_sync_gps_item_t recvd_gps_item;
	if(xQueueReceive(ins_sync_gps_queue, &recvd_gps_item, 0) == pdTRUE) {
		*longitude = recvd_gps_item.longitude;
		*latitude = recvd_gps_item.latitude;
		*height_msl = recvd_gps_item.height_msl;
		*vx_ned = recvd_gps_item.vx_ned;
		*vy_ned = recvd_gps_item.vy_ned;
		*vz_ned = recvd_gps_item.vz_ned;
		return true;
	} else {
		return false;
	}
}

/*=========*
 * compass *
 *=========*/

bool ins_compass_sync_buffer_available(void)
{
	UBaseType_t free_size = uxQueueSpacesAvailable(ins_sync_compass_queue);
	UBaseType_t available_size = INS_SYNC_COMPASS_BUF_SIZE - free_size;

	if(available_size > 0) {
		return true;
	} else {
		return false;
	}
}

void ins_compass_sync_buffer_push(float *mag)
{
	ins_sync_compass_item_t compass_item = {
		.timestamp_s = get_sys_time_s(),
		.mag_x = mag[0],
		.mag_y = mag[1],
		.mag_z = mag[2]
	};

	xQueueSendToBack(ins_sync_compass_queue, &compass_item, 0);
}

void ins_compass_sync_buffer_push_from_isr(float *mag, BaseType_t *higher_priority_task_woken)
{
	ins_sync_compass_item_t compass_item = {
		.timestamp_s = get_sys_time_s(),
		.mag_x = mag[0],
		.mag_y = mag[1],
		.mag_z = mag[2]
	};

	xQueueSendToBackFromISR(ins_sync_compass_queue, &compass_item, higher_priority_task_woken);
}

bool ins_compass_sync_buffer_pop(float *mag)
{
	ins_sync_compass_item_t recvd_compass_item;
	if(xQueueReceive(ins_sync_compass_queue, &recvd_compass_item, 0) == pdTRUE) {
		mag[0] = recvd_compass_item.mag_x;
		mag[1] = recvd_compass_item.mag_y;
		mag[2] = recvd_compass_item.mag_z;
		return true;
	} else {
		return false;
	}
}

/*=============*
 * rangefinder *
 *=============*/

bool ins_rangefinder_sync_buffer_available(void)
{
	UBaseType_t free_size = uxQueueSpacesAvailable(ins_sync_rangefinder_queue);
	UBaseType_t available_size = INS_SYNC_RANGEFINDER_BUF_SIZE - free_size;

	if(available_size > 0) {
		return true;
	} else {
		return false;
	}
}

void ins_rangefinder_sync_buffer_push(float height, float height_rate)
{
	ins_sync_rangefinder_item_t rangefinder_item = {
		.timestamp_s = get_sys_time_s(),
		.height = height,
		.height_rate = height_rate
	};

	xQueueSendToBack(ins_sync_rangefinder_queue, &rangefinder_item, 0);
}

void ins_rangefinder_sync_buffer_push_from_isr(float height, float height_rate,
                BaseType_t *higher_priority_task_woken)
{
	ins_sync_rangefinder_item_t rangefinder_item = {
		.timestamp_s = get_sys_time_s(),
		.height = height,
		.height_rate = height_rate
	};

	xQueueSendToBackFromISR(ins_sync_rangefinder_queue, &rangefinder_item, higher_priority_task_woken);
}

bool ins_rangefinder_sync_buffer_pop(float *height, float *height_rate)
{
	ins_sync_rangefinder_item_t recvd_rangefinder_item;
	if(xQueueReceive(ins_sync_rangefinder_queue, &recvd_rangefinder_item, 0) == pdTRUE) {
		*height = recvd_rangefinder_item.height;
		*height_rate = recvd_rangefinder_item.height_rate;
		return true;
	} else {
		return false;
	}
}
