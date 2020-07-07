#ifndef __CALIBRATION_TASK_H__
#define __CALIBRATION_TASK_H__

enum {
	NO_CALIBRATION = 0,
	ACCEL_CALIBRATION = 1,
	COMPASS_CALIBRATION = 2,
	RADIO_CALIBRATION = 3
} CALIBRATION_TYPE;

void wakeup_calibration_task(int type);
void task_calibration(void *param);

#endif
