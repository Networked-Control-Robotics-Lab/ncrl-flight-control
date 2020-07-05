#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"
#include "imu.h"
#include "sys_time.h"

enum {
	ACCEL_CALIB_FRONT,
	ACCEL_CALIB_BACK,
	ACCEL_CALIB_UP,
	ACCEL_CALIB_DOWN,
	ACCEL_CALIB_LEFT,
	ACCEL_CALIB_RIGHT,
	ACCEL_CALIB_DIR_UNKNOWN
} ACCEL_CALIB_DIR;

static int detect_accel_orientation(float *accel)
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

static float capture_accel_gavity_extreme_vaule_x(bool cap_neg)
{
	float accel[3];
	float accel_x;

	float curr_time;
	float last_time = get_sys_time_s();

	/* init */
	get_imu_filtered_accel(accel);
	float extreme = accel[0];

	while(1) {
		curr_time = get_sys_time_s();
		if((curr_time - last_time) > 5.0f) {
			break;
		}

		get_imu_filtered_accel(accel);
		accel_x = accel[0];

		if(cap_neg == true) {
			if(extreme > accel_x) {
				extreme = accel_x;
			}
		} else {
			if(extreme < accel_x) {
				extreme = accel_x;
			}
		}
	}

	return extreme;
}

static float capture_accel_gavity_extreme_vaule_y(bool cap_neg)
{
	float accel[3];
	float accel_y;

	float curr_time;
	float last_time = get_sys_time_s();

	/* init */
	get_imu_filtered_accel(accel);
	float extreme = accel[1];

	while(1) {
		curr_time = get_sys_time_s();
		if((curr_time - last_time) > 5.0f) {
			break;
		}

		get_imu_filtered_accel(accel);
		accel_y = accel[1];

		if(cap_neg == true) {
			if(extreme > accel_y) {
				extreme = accel_y;
			}
		} else {
			if(extreme < accel_y) {
				extreme = accel_y;
			}
		}
	}

	return extreme;
}


static float capture_accel_gavity_extreme_vaule_z(bool cap_neg)
{
	float accel[3];
	float accel_z;

	float curr_time;
	float last_time = get_sys_time_s();

	/* init */
	get_imu_filtered_accel(accel);
	float extreme = accel[2];

	while(1) {
		curr_time = get_sys_time_s();
		if((curr_time - last_time) > 5.0f) {
			break;
		}

		get_imu_filtered_accel(accel);
		accel_z = accel[2];

		if(cap_neg == true) {
			if(extreme > accel_z) {
				extreme = accel_z;
			}
		} else {
			if(extreme < accel_z) {
				extreme = accel_z;
			}
		}
	}

	return extreme;
}

bool detect_accel_motion(float *accel)
{
	static float accel_last[3] = {0.0f};

	float accel_change[3];
	accel_change[0] = accel[0] - accel_last[0];
	accel_change[1] = accel[1] - accel_last[1];
	accel_change[2] = accel[2] - accel_last[2];

	float accel_change_norm = sqrt(accel_change[0] * accel_change[0] +
	                               accel_change[1] * accel_change[1] +
	                               accel_change[2] * accel_change[2]);

	accel_last[0] = accel[0];
	accel_last[1] = accel[1];
	accel_last[2] = accel[2];

	if(accel_change_norm > 0.98) {
		return true;
	} else {
		return false;
	}
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

	volatile float calib_x_p, calib_x_n;
	volatile float calib_y_p, calib_y_n;
	volatile float calib_z_p, calib_z_n;

	float accel[3] = {0.0f};

	send_mavlink_status_text("[cal] calibration started: 2 accel", 6, 0, 0);
	freertos_task_delay(1000);

	float curr_time;
	float last_time = get_sys_time_s();

	while(1) {
		get_imu_filtered_accel(accel);

		curr_time = get_sys_time_s();

		if(detect_accel_motion(accel) == true) {
			last_time = get_sys_time_s();
		}

		if((curr_time - last_time) < 5.0f) {
			continue;
		}

		last_time = curr_time;

		int orientation = detect_accel_orientation(accel);

		switch(orientation) {
		case ACCEL_CALIB_FRONT: {
			if(front_finished == true) break;

			send_mavlink_status_text("[cal] front orientation detected", 6, 0, 0);
			calib_x_p = capture_accel_gavity_extreme_vaule_x(true);
			send_mavlink_status_text("[cal] front side done, rotate to a different side", 6, 0, 0);

			front_finished = true;
			break;
		}
		case ACCEL_CALIB_BACK: {
			if(back_finished == true) break;

			send_mavlink_status_text("[cal] back orientation detected", 6, 0, 0);
			calib_x_n = capture_accel_gavity_extreme_vaule_x(false);
			send_mavlink_status_text("[cal] back side done, rotate to a different side", 6, 0, 0);

			back_finished = true;
			break;
		}
		case ACCEL_CALIB_UP: {
			if(up_finished == true) break;

			send_mavlink_status_text("[cal] up orientation detected", 6, 0, 0);
			calib_z_p = capture_accel_gavity_extreme_vaule_z(false);
			send_mavlink_status_text("[cal] up side done, rotate to a different side", 6, 0, 0);

			up_finished = true;
			break;
		}
		case ACCEL_CALIB_DOWN: {
			if(down_finished == true) break;

			send_mavlink_status_text("[cal] down orientation detected", 6, 0, 0);
			calib_z_n = capture_accel_gavity_extreme_vaule_z(true);
			send_mavlink_status_text("[cal] down side done, rotate to a different side", 6, 0, 0);

			down_finished = true;
			break;
		}
		case ACCEL_CALIB_LEFT: {
			if(left_finished == true) break;

			send_mavlink_status_text("[cal] left orientation detected", 6, 0, 0);
			calib_y_p = capture_accel_gavity_extreme_vaule_y(false);
			send_mavlink_status_text("[cal] left side done, rotate to a different side", 6, 0, 0);

			left_finished = true;
			break;
		}
		case ACCEL_CALIB_RIGHT: {
			if(right_finished == true) break;

			send_mavlink_status_text("[cal] right orientation detected", 6, 0, 0);
			calib_y_n = capture_accel_gavity_extreme_vaule_y(true);
			send_mavlink_status_text("[cal] right side done, rotate to a different side", 6, 0, 0);

			right_finished = true;
			break;
		}
		default:
			break;
		}

		/* finish collecting datas */
		if(front_finished == true && back_finished == true &&
		    up_finished == true && down_finished == true &&
		    left_finished == true && right_finished == true) {
			send_mavlink_status_text("[cal] calibration done: accel", 6, 0, 0);
			config_imu_accel_scale_calib_setting(calib_x_p, calib_x_n,
			                                     calib_y_p, calib_y_n,
			                                     calib_z_p, calib_z_n);
			return;
		}
	}
}
