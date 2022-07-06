#include "motor.h"
#include "sbus_radio.h"
#include "led.h"
#include "delay.h"
#include "board_porting.h"

float do_esc_calib = false;

float do_force_testing = false;
float force_testing_percentage = 0.0f;

/**************************************************************************************
 * electronic speed controller range calibration (triggered by shell or groundstation)*
 **************************************************************************************/

void trigger_esc_range_calibration(void)
{
	do_esc_calib = true;
}

bool is_esc_range_calibration_triggered(void)
{
	return do_esc_calib;
}

void esc_range_calibration(void)
{
	/* caution:
	 * 1.remove all propellers berfore doing esc calibration!
	 * 2.after the calibration you should shut down the power and reset
	 * the system before flying. */

	set_rgb_led_calibration_mode_flag(true);

	radio_t rc;
	float calib_signal = 0.0f;

	while(1) {
		sbus_rc_read(&rc);

		calib_signal = rc.throttle * 0.01f;

		set_all_motor_value(calib_signal);

		freertos_task_delay(1);
	}
}

/*******************************************************
 * motor force testing (can only be triggered by shell)*
 *******************************************************/

void trigger_motor_force_testing(void)
{
	do_force_testing = true;
	force_testing_percentage = 0.0f;
}

bool is_motor_force_testing_triggered(void)
{
	return do_force_testing;
}

void set_motor_force_testing_percentage(float percentage)
{
	force_testing_percentage = percentage;
}

float get_motor_force_testing_percentage(void)
{
	return force_testing_percentage;
}

void motor_force_testing(void)
{
	/* caution:
	 * since this function is dangerous, the testing loop will never be
	 * end until you turn off the power */

	set_rgb_led_calibration_mode_flag(true);

	while(1) {
		set_motor_value(MOTOR1, force_testing_percentage);

		freertos_task_delay(1);
	}
}
