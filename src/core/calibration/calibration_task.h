#ifndef __CALIBRATION_TASK_H__
#define __CALIBRATION_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

enum {
	NO_CALIBRATION = 0,
	ACCEL_SCALE_CALIBRATION = 1,
	ACCEL_OFFSET_CALIBRATION = 2,
	COMPASS_CALIBRATION = 3,
	RADIO_CALIBRATION = 4
} CALIBRATION_TYPE;

void cancel_device_calibration(void);
bool is_device_calibration_cancelled(void);
void reset_calibration_cancelled_state(void);

void wakeup_calibration_task(int type);

void calibration_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                               UBaseType_t priority);

#endif
