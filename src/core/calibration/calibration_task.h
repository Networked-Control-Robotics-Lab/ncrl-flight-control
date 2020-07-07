#ifndef __CALIBRATION_TASK_H__
#define __CALIBRATION_TASK_H__

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

void task_calibration(void *param);

#endif
