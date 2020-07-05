#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"
#include "imu.h"

enum {
	ACCEL_CALIB_FRONT,
	ACCEL_CALIB_BACK,
	ACCEL_CALIB_UP,
	ACCEL_CALIB_DOWN,
	ACCEL_CALIB_LEFT,
	ACCEL_CALIB_RIGHT,
	ACCEL_CALIB_DIR_UNKNOWN
} ACCEL_CALIB_DIR;

int detect_accel_orientation(float *accel)
{
	bool x_is_neg = accel[0] < 0.0f ? true : false;
	bool y_is_neg = accel[1] < 0.0f ? true : false;
	bool z_is_neg = accel[2] < 0.0f ? true : false;

	float x_norm = fabs(accel[0]);
	float y_norm = fabs(accel[1]);
	float z_norm = fabs(accel[2]);

	if(x_norm > y_norm && x_norm > z_norm) {
		/* x norm is the biggest */
		return x_is_neg == true ? ACCEL_CALIB_FRONT : ACCEL_CALIB_BACK;
	} else if(y_norm > x_norm && y_norm > z_norm) {
		/* y norm is the biggest */
		return y_is_neg == true ? ACCEL_CALIB_RIGHT : ACCEL_CALIB_LEFT ;
	} else if(z_norm > x_norm && z_norm > y_norm) {
		/* z norm is the biggest */
		return z_is_neg == true ? ACCEL_CALIB_DOWN : ACCEL_CALIB_UP;
	}

	return ACCEL_CALIB_DIR_UNKNOWN;
}

void mavlink_accel_calibration_handler(void)
{
	freertos_task_delay(1000);

	bool front_finished = false;
	bool back_finished = false;
	bool left_finished = false;
	bool right_finished = false;
	bool up_finished = false;
	bool down_finished = false;

	float accel[3] = {0.0f};

	send_mavlink_status_text("[cal] calibration started: 2 accel", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] back orientation detected", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] back side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] progress <16>", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] front orientation detected", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] front side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] progress <32>", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] left orientation detected", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] left side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] progress <48>", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] right orientation detected", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] right side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] progress <64>", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] up orientation detected", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] up side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] progress <80>", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] down orientation detected", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] down side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);
	send_mavlink_status_text("[cal] progress <100>", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] calibration done: accel", 6, 0, 0);
	freertos_task_delay(1000);
	return;

	while(1) {
		get_imu_filtered_accel(accel);
		int orientation = detect_accel_orientation(accel);

		switch(orientation) {
		case ACCEL_CALIB_FRONT:
			if(front_finished == true) break;
			break;
		case ACCEL_CALIB_BACK:
			if(back_finished == true) break;
			break;
		case ACCEL_CALIB_UP:
			if(up_finished == true) break;
			break;
		case ACCEL_CALIB_DOWN:
			if(down_finished == true) break;
			break;
		case ACCEL_CALIB_LEFT:
			if(left_finished == true) break;
			break;
		case ACCEL_CALIB_RIGHT:
			if(right_finished == true) break;
			break;
		default:
			break;
		}

		/* finish collecting datas */
		if(front_finished == true && back_finished == true &&
		    up_finished == true && down_finished == true &&
		    left_finished == true && right_finished == true) {
			return;
		}
	}
}
