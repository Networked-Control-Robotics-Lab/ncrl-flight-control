#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "calibration_task.h"
#include "accel_calibration.h"
#include "compass_calibration.h"

TaskHandle_t calib_task_handle;
int calibration_type;
int calibration_is_cancelled = false;

void wakeup_calibration_task(int type)
{
	if(calibration_type == NO_CALIBRATION) {
		calibration_type = type;
		vTaskResume(calib_task_handle);
	}
}

void cancel_device_calibration(void)
{
	if(calibration_type != NO_CALIBRATION) {
		calibration_is_cancelled = true;
	}
}

bool is_device_calibration_cancelled(void)
{
	return calibration_is_cancelled;
}

void reset_calibration_cancelled_state(void)
{
	calibration_is_cancelled = false;
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

void calibration_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                               UBaseType_t priority)
{
	xTaskCreate(task_calibration, task_name, stack_size, NULL, priority, &calib_task_handle);
	vTaskSuspend(calib_task_handle);
}

