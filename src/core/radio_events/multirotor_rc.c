#include "arm_math.h"
#include "sbus_radio.h"
#include "autopilot.h"
#include "system_state.h"
#include "sys_time.h"
#include "led.h"
#include "esc_calibration.h"
#include "board_porting.h"
#include "proj_config.h"
#include "system_state.h"
#include "sensor_switching.h"

void multirotor_free_fall_rc(radio_t *rc);
void multirotor_navigation_switch_test(radio_t *rc);

void multirotor_rc_special_function_handler(radio_t *rc)
{
	//call the test function you want to use here:
	//multirotor_free_fall_rc(rc);
	//multirotor_navigation_switch_test(rc);
}

/* stability testing function: quadrotor free fall recovery
 * function mapping to aux1_mode position: upper (off), middle (start), lower (off) */
void multirotor_free_fall_rc(radio_t *rc)
{
	static int aux1_mode_last = RC_AUX_MODE1;

	const float recovery_height = 1.5f; //[m]

	/* aux1 button: upper position */
	if(rc->aux1_mode == RC_AUX_MODE1) {
		autopilot_unlock_motor();
	}

	/* aux1 button: middle position */
	if(rc->aux1_mode == RC_AUX_MODE2 && aux1_mode_last != RC_AUX_MODE2) {
		if(rc->auto_flight == true) {
			autopilot_lock_motor();
		}
	}

	/* aux1 button: lower position */
	if(rc->aux1_mode == RC_AUX_MODE3) {
		autopilot_unlock_motor();
	}

	float height = get_enu_position_z();
	if((height <= recovery_height) && (autopilot_get_mode() == AUTOPILOT_MOTOR_LOCKED_MODE)) {
		autopilot_unlock_motor();
	}

	aux1_mode_last = rc->aux1_mode;
}

void multirotor_navigation_switch_test(radio_t *rc)
{
	static int aux1_mode_last = RC_AUX_MODE1;

	/* aux1 button: upper position */
	if(rc->aux1_mode == RC_AUX_MODE1 && aux1_mode_last != RC_AUX_MODE1) {
	}

	/* aux1 button: middle position */
	if(rc->aux1_mode == RC_AUX_MODE2 && aux1_mode_last != RC_AUX_MODE2) {
		/* middle position is used to trigger camera and imu only,
		   the code is implemented in the flight control task */
		switch_navigation_system(NAV_GNSS_INS);
	}

	/* aux1 button: lower position */
	if(rc->aux1_mode == RC_AUX_MODE3 && aux1_mode_last != RC_AUX_MODE3) {
		//switch_navigation_system(NAV_LOCAL_VIO);
		switch_navigation_system(NAV_GLOBAL_VIO);

		/* beside of navigation system switching, the camera trigger
		   and imu data sending are implemented in the flight control
		   task */
	}

	aux1_mode_last = rc->aux1_mode;
}

void rc_wait_unlock_geasture(void)
{
	radio_t rc;

	float time_last = 0.0f;
	float time_current = 0.0f;

	set_rgb_led_service_motor_lock_flag(true);

	while(1) {
		sbus_rc_read(&rc);

		time_current = get_sys_time_ms();
		if(time_current - time_last > 100.0f) {
			time_last = time_current;
		}

		//force to leave the loop if user triggered the motor esc range calibration
		if(is_esc_range_calibration_triggered() == true) break;

		//force to leave the loop if user triggered the motor thrust testing
		if(is_motor_force_testing_triggered() == true) break;

		if(rc.throttle < 5.0f && rc.pitch < -30.0f &&
		    rc.yaw > +30.0f && rc.roll < -30.0f) {
			break;
		}

		vTaskDelay(1);
	}

	set_rgb_led_service_motor_lock_flag(false);
}

bool rc_unlock_geasture_handler(radio_t *rc, float *accel_lpf)
{
	static bool motor_lock = false;

	if(rc->safety == true) {
		float accel_norm_squared = accel_lpf[0]*accel_lpf[0] +
		                           accel_lpf[1]*accel_lpf[1] +
		                           accel_lpf[2]*accel_lpf[2];
		float accel_norm;
		arm_sqrt_f32(accel_norm_squared, &accel_norm);

		float accel_translational = accel_norm - 9.78f;

		if(accel_translational < 0.5) {
			motor_lock = true;
		}
	} else {
		if(motor_lock == true &&
		    rc->throttle < 5.0f && rc->pitch < -30.0f &&
		    rc->yaw > +30.0f && rc->roll < -30.0f) {
			motor_lock = false;
		}

	}

	return motor_lock;
}
