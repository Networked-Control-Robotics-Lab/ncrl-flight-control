#include "sbus_radio.h"
#include "autopilot.h"
#include "position_state.h"

void multirotor_free_fall_rc(radio_t *rc);

void multirotor_rc_special_function_handler(radio_t *rc)
{
	//call the test function you want to use here:
	//multirotor_free_fall_rc(rc);
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

	float height = get_enu_height();
	if(height <= recovery_height && autopilot_motor_ls_lock() == true) {
		autopilot_unlock_motor();
	}

	aux1_mode_last = rc->aux1_mode;
}
