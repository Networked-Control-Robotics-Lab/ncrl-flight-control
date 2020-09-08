#include "motor.h"
#include "sbus_radio.h"
#include "led.h"

float do_esc_calib = false;

void trigger_esc_calibration(void)
{
	do_esc_calib = true;
}

bool is_esc_calibration_triggered(void)
{
	return do_esc_calib;
}

void esc_calibration(void)
{
	/* caution:
	 * 1.remove all propellers berfore doing esc calibration!
	 * 2.after the calibration you should shut down the power and reset
	 * the system before flying. */

	led_on(LED_R);
	led_off(LED_G);
	led_on(LED_B);

	radio_t rc;
	float calib_signal = 0.0f;

	while(1) {
		sbus_rc_read(&rc);

		calib_signal = rc.throttle * 0.01f;

		set_motor_value(MOTOR1, calib_signal);
		set_motor_value(MOTOR2, calib_signal);
		set_motor_value(MOTOR3, calib_signal);
		set_motor_value(MOTOR4, calib_signal);
		set_motor_value(MOTOR5, calib_signal);
		set_motor_value(MOTOR6, calib_signal);
	}
}
