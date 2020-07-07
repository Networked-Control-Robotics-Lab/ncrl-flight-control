#include "FreeRTOS.h"
#include "task.h"
#include "calibration_task.h"
#include "accel_calibration.h"
#include "compass_calibration.h"

TaskHandle_t calib_task_handle;
int calibration_type;

void wakeup_calibration_task(int _calibration_type)
{
	if(calibration_type == NO_CALIBRATION) {
		calibration_type = _calibration_type;
		vTaskResume(calib_task_handle);
	}
}

void task_calibration(void *param)
{
	while(1) {
		switch(calibration_type) {
		case ACCEL_CALIBRATION:
			mavlink_accel_calibration_handler();
			break;
		case COMPASS_CALIBRATION:
			break;
		case RADIO_CALIBRATION:
			break;
		}

		/* calibration finished, freeze the calibration task */
		calibration_type = NO_CALIBRATION;
		vTaskSuspend(NULL);
	}
}
