#include "FreeRTOS.h"
#include "task.h"
#include "calibration_task.h"
#include "accel_calibration.h"
#include "compass_calibration.h"

TaskHandle_t calib_task_handle;
int calibration_type;

void wakeup_calibration_task(int type)
{
	if(calibration_type == NO_CALIBRATION) {
		calibration_type = type;
		vTaskResume(calib_task_handle);
	}
}

void task_calibration(void *param)
{
	while(1) {
		switch(calibration_type) {
		case ACCEL_SCALE_CALIBRATION:
			mavlink_accel_scale_calibration_handler();
			break;
		case ACCEL_OFFSET_CALIBRATION:
			mavlink_accel_offset_calibration_handler();
			break;
		case COMPASS_CALIBRATION:
			mavlink_compass_calibration_handler();
			break;
		case RADIO_CALIBRATION:
			break;
		}

		/* calibration finished, freeze the task */
		calibration_type = NO_CALIBRATION;
		vTaskSuspend(NULL);
	}
}
