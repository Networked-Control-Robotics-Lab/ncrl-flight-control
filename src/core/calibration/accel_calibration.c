#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "imu.h"
#include "sys_time.h"
#include "quadshell.h"
#include "mavlink_task.h"
#include "sys_param.h"
#include "common_list.h"
#include "calibration_task.h"
#include "uart.h"
#include "board_porting.h"

#define ACCEL_CALIB_SAMPLING_TIMES 2000

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

static float capture_accel_gavity_vaule_x(bool cap_neg)
{
	float accel[3];

	float gravity = 0.0f;

	int i;
	for(i = 0; i < ACCEL_CALIB_SAMPLING_TIMES; i++) {
		/* read sensor */
		get_accel_lpf(accel);
		gravity += accel[0] / ACCEL_CALIB_SAMPLING_TIMES;
		freertos_task_delay(2.5);
	}

	return gravity;
}

static float capture_accel_gavity_vaule_y(bool cap_neg)
{
	float accel[3];

	float gravity = 0.0f;

	int i;
	for(i = 0; i < ACCEL_CALIB_SAMPLING_TIMES; i++) {
		/* read sensor */
		get_accel_lpf(accel);
		gravity += accel[1] / ACCEL_CALIB_SAMPLING_TIMES;
		freertos_task_delay(2.5);
	}

	return gravity;
}


static float capture_accel_gavity_vaule_z(bool cap_neg)
{
	float accel[3];

	float gravity = 0.0f;

	int i;
	for(i = 0; i < ACCEL_CALIB_SAMPLING_TIMES; i++) {
		/* read sensor */
		get_accel_lpf(accel);
		gravity += accel[2] / ACCEL_CALIB_SAMPLING_TIMES;
		freertos_task_delay(2.5);
	}

	return gravity;
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

void send_progress_update_message(int stage)
{
	switch(stage) {
	case 1:
		send_mavlink_calibration_status_text("[cal] progress <16>");
		break;
	case 2:
		send_mavlink_calibration_status_text("[cal] progress <32>");
		break;
	case 3:
		send_mavlink_calibration_status_text("[cal] progress <48>");
		break;
	case 4:
		send_mavlink_calibration_status_text("[cal] progress <64>");
		break;
	case 5:
		send_mavlink_calibration_status_text("[cal] progress <80>");
		break;
	case 6:
		send_mavlink_calibration_status_text("[cal] progress <100>");
		break;
	}
}

void mavlink_accel_scale_calibration_handler(void)
{
	reset_accel_scale_factor();
	reset_accel_bias();

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

	/* trigger qgroundcontrol to show the calibration window */
	send_mavlink_calibration_status_text("[cal] calibration started: 2 accel");

	float curr_time;
	float last_time = get_sys_time_s();

	float reset_start_time = get_sys_time_s();

	int stage = 0;

	while(1) {
		if(is_device_calibration_cancelled() == true) {
			reset_calibration_cancelled_state();
			send_mavlink_calibration_status_text("[cal] calibration cancelled");
			return;
		}

		/* read sensor data */
		get_accel_lpf(accel);

		/* read current time */
		curr_time = get_sys_time_s();

		/* timeout (10 seconds) */
		if((curr_time - reset_start_time) > 10.0f) {
			send_mavlink_calibration_status_text(
			        "[cal] calibration failed: timeout: no motion");
			return;
		}

		/* detect if imu is motionless */
		if(detect_accel_motion(accel) == true) {
			last_time = get_sys_time_s();
		}

		/* collect calibration data if imu is motionless over 5 seconds */
		if((curr_time - last_time) < 3.0f) {
			continue;
		}

		/* update timer */
		last_time = curr_time;

		/* detect imu orientation */
		int orientation = detect_accel_orientation(accel);

		/* collect calibration data */
		switch(orientation) {
		case ACCEL_CALIB_FRONT: {
			if(front_finished == true) break;

			send_mavlink_calibration_status_text("[cal] front orientation detected");
			calib_x_n = capture_accel_gavity_vaule_x(true);
			send_mavlink_calibration_status_text("[cal] front side done, rotate to a different side");

			stage++;
			send_progress_update_message(stage);

			reset_start_time = get_sys_time_s(); //reset timeout timer

			front_finished = true;
			break;
		}
		case ACCEL_CALIB_BACK: {
			if(back_finished == true) break;

			send_mavlink_calibration_status_text("[cal] back orientation detected");
			calib_x_p = capture_accel_gavity_vaule_x(false);
			send_mavlink_calibration_status_text("[cal] back side done, rotate to a different side");

			stage++;
			send_progress_update_message(stage);

			reset_start_time = get_sys_time_s(); //reset timeout timer

			back_finished = true;
			break;
		}
		case ACCEL_CALIB_UP: {
			if(up_finished == true) break;

			send_mavlink_calibration_status_text("[cal] up orientation detected");
			calib_z_p = capture_accel_gavity_vaule_z(false);
			send_mavlink_calibration_status_text("[cal] up side done, rotate to a different side");

			stage++;
			send_progress_update_message(stage);

			reset_start_time = get_sys_time_s(); //reset timeout timer

			up_finished = true;
			break;
		}
		case ACCEL_CALIB_DOWN: {
			if(down_finished == true) break;

			send_mavlink_calibration_status_text("[cal] down orientation detected");
			calib_z_n = capture_accel_gavity_vaule_z(true);
			send_mavlink_calibration_status_text("[cal] down side done, rotate to a different side");

			stage++;
			send_progress_update_message(stage);

			reset_start_time = get_sys_time_s(); //reset timeout timer

			down_finished = true;
			break;
		}
		case ACCEL_CALIB_LEFT: {
			if(left_finished == true) break;

			send_mavlink_calibration_status_text("[cal] left orientation detected");
			calib_y_p = capture_accel_gavity_vaule_y(false);
			send_mavlink_calibration_status_text("[cal] left side done, rotate to a different side");

			stage++;
			send_progress_update_message(stage);

			reset_start_time = get_sys_time_s(); //reset timeout timer

			left_finished = true;
			break;
		}
		case ACCEL_CALIB_RIGHT: {
			if(right_finished == true) break;

			send_mavlink_calibration_status_text("[cal] right orientation detected");
			calib_y_n = capture_accel_gavity_vaule_y(true);
			send_mavlink_calibration_status_text("[cal] right side done, rotate to a different side");

			stage++;
			send_progress_update_message(stage);

			reset_start_time = get_sys_time_s(); //reset timeout timer

			right_finished = true;
			break;
		}
		default:
			break;
		}

		/* apply calibration result if calibration is finished */
		if(front_finished == true && back_finished == true &&
		    up_finished == true && down_finished == true &&
		    left_finished == true && right_finished == true) {
			float x_scale = (2.0f * 9.81) / (calib_x_p - calib_x_n);
			float y_scale = (2.0f * 9.81) / (calib_y_p - calib_y_n);
			float z_scale = (2.0f * 9.81) / (calib_z_p - calib_z_n);

			set_accel_scale_factor(x_scale, y_scale, z_scale);

			set_sys_param_float(IMU_FINISH_CALIB, 1.0f);
			set_sys_param_float(CAL_ACC0_ID, 1);
			set_sys_param_float(CAL_ACC1_ID, 2);
			set_sys_param_float(CAL_ACC0_XSCALE, x_scale);
			set_sys_param_float(CAL_ACC0_YSCALE, y_scale);
			set_sys_param_float(CAL_ACC0_ZSCALE, z_scale);

			/* update parameter list to flash */
			save_param_list_to_flash();

			send_mavlink_calibration_status_text("[cal] calibration done: accel");

			return;
		}
	}
}

void mavlink_accel_offset_calibration_handler(void)
{
	reset_accel_bias();

	send_mavlink_calibration_status_text("[cal] calibration started: 2 level");

	float accel[3];
	get_accel_lpf(accel);

	float x_offset = accel[0] - 0.0f;
	float y_offset = accel[1] - 0.0f;
	float z_offset = accel[2] - (-9.8f);

	set_accel_bias(x_offset, y_offset, z_offset);

	set_sys_param_float(CAL_ACC0_XOFF, x_offset);
	set_sys_param_float(CAL_ACC0_YOFF, y_offset);
	set_sys_param_float(CAL_ACC0_ZOFF, z_offset);

	/* update parameter list to flash */
	save_param_list_to_flash();

	send_mavlink_calibration_status_text("[cal] calibration done: level");
}

void shell_accel_calibration_handler(void)
{
	reset_accel_scale_factor();
	reset_accel_bias();

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

	shell_puts("press [q] to stop.\n\r"
	           "[cal] calibration started: 2 accel\n\r");

	float curr_time;
	float last_time = get_sys_time_s();

	char c;

	/* acceleromter scale calibration */
	while(1) {
		if(debug_link_getc(&c, 0) == true) {
			if(c == 'q') return;
		}

		/* read sensor data */
		get_accel_lpf(accel);

		/* read current time */
		curr_time = get_sys_time_s();

		/* detect if imu is motionless */
		if(detect_accel_motion(accel) == true) {
			last_time = get_sys_time_s();
		}

		/* collect calibration data if imu is motionless over 5 seconds */
		if((curr_time - last_time) < 3.0f) {
			continue;
		}

		/* update timer */
		last_time = curr_time;

		/* detect imu orientation */
		int orientation = detect_accel_orientation(accel);

		/* collect calibration data */
		switch(orientation) {
		case ACCEL_CALIB_FRONT: {
			if(front_finished == true) break;

			shell_puts("[cal] front orientation detected\n\r");
			calib_x_n = capture_accel_gavity_vaule_x(true);
			shell_puts("[cal] front side done, rotate to a different side\n\r");

			front_finished = true;
			break;
		}
		case ACCEL_CALIB_BACK: {
			if(back_finished == true) break;

			shell_puts("[cal] back orientation detected\n\r");
			calib_x_p = capture_accel_gavity_vaule_x(false);
			shell_puts("[cal] back side done, rotate to a different side\n\r");

			back_finished = true;
			break;
		}
		case ACCEL_CALIB_UP: {
			if(up_finished == true) break;

			shell_puts("[cal] up orientation detected\n\r");
			calib_z_p = capture_accel_gavity_vaule_z(false);
			shell_puts("[cal] up side done, rotate to a different side\n\r");

			up_finished = true;
			break;
		}
		case ACCEL_CALIB_DOWN: {
			if(down_finished == true) break;

			shell_puts("[cal] down orientation detected\n\r");
			calib_z_n = capture_accel_gavity_vaule_z(true);
			shell_puts("[cal] down side done, rotate to a different side\n\r");

			down_finished = true;
			break;
		}
		case ACCEL_CALIB_LEFT: {
			if(left_finished == true) break;

			shell_puts("[cal] left orientation detected\n\r");
			calib_y_p = capture_accel_gavity_vaule_y(false);
			shell_puts("[cal] left side done, rotate to a different side\n\r");

			left_finished = true;
			break;
		}
		case ACCEL_CALIB_RIGHT: {
			if(right_finished == true) break;

			shell_puts("[cal] right orientation detected\n\r");
			calib_y_n = capture_accel_gavity_vaule_y(true);
			shell_puts("[cal] right side done, rotate to a different side\n\r");

			right_finished = true;
			break;
		}
		default:
			break;
		}

		/* apply calibration result if calibration is finished */
		if(front_finished == true && back_finished == true &&
		    up_finished == true && down_finished == true &&
		    left_finished == true && right_finished == true) {
			float x_scale = (2.0f * 9.81) / (calib_x_p - calib_x_n);
			float y_scale = (2.0f * 9.81) / (calib_y_p - calib_y_n);
			float z_scale = (2.0f * 9.81) / (calib_z_p - calib_z_n);

			set_accel_scale_factor(x_scale, y_scale, z_scale);

			set_sys_param_float(IMU_FINISH_CALIB, 1.0f);
			set_sys_param_float(CAL_ACC0_ID, 1);
			set_sys_param_float(CAL_ACC1_ID, 2);
			set_sys_param_float(CAL_ACC0_XSCALE, x_scale);
			set_sys_param_float(CAL_ACC0_YSCALE, y_scale);
			set_sys_param_float(CAL_ACC0_ZSCALE, z_scale);

			char s[300] = {0};
			sprintf(s, "[cal] calibration done: accel\n\r"
			        "x scale: %f\n\r"
			        "y scale: %f\n\r"
			        "z scale: %f\n\r",
			        x_scale, y_scale, z_scale);
			shell_puts(s);

			break;
		}
	}

	/* accelerometer offset calibration */
	shell_puts("press enter to start level horizon calibration\n\r");

	while(1) {
		if(debug_link_getc(&c, 0) == true) {
			if(c == 13) break;
		}
	}

	shell_puts("[cal] calibration started: 2 level\n\r");

	get_accel_lpf(accel);

	float x_offset = accel[0] - 0.0f;
	float y_offset = accel[1] - 0.0f;
	float z_offset = accel[2] - (-9.81f);

	set_accel_bias(x_offset, y_offset, z_offset);

	set_sys_param_float(CAL_ACC0_XOFF, x_offset);
	set_sys_param_float(CAL_ACC0_YOFF, y_offset);
	set_sys_param_float(CAL_ACC0_ZOFF, z_offset);

	/* update parameter list to flash */
	save_param_list_to_flash();

	char s[300] = {0};
	sprintf(s, "[cal] calibration done: level\n\r"
	        "x offset: %f\n\r"
	        "y offset: %f\n\r"
	        "z offset: %f\n\r",
	        x_offset, y_offset, z_offset);
	shell_puts(s);
}
