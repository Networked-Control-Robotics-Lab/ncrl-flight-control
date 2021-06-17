#include <stdint.h>
#include "autopilot.h"
#include "system_state.h"

extern autopilot_t autopilot;

int autopilot_trigger_auto_landing(void)
{
	if(autopilot.mode == AUTOPILOT_HOVERING_MODE) {
		autopilot.mode = AUTOPILOT_LANDING_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NOT_IN_HOVERING_MODE;
	}
}

int autopilot_trigger_auto_takeoff(void)
{
	//TODO: replace the hard coded threshold value
	if(get_enu_position_z() < 0.2) {
		autopilot.mode = AUTOPILOT_TAKEOFF_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_ALREADY_TAKEOFF;
	}
}

void autopilot_takeoff_handler(void)
{
	/* slowly change the height setpoint for takeoff */
	autopilot.ctrl_target.pos[2] += autopilot.takeoff_speed * autopilot.period;
	autopilot_assign_zero_vel_target();
	autopilot_assign_zero_acc_feedforward();
	if(autopilot.ctrl_target.pos[2] > autopilot.takeoff_height) {
		autopilot.mode = AUTOPILOT_HOVERING_MODE;
		autopilot.ctrl_target.pos[2] = autopilot.takeoff_height;
	}
}

void autopilot_landing_handler(float *curr_pos)
{
	autopilot_assign_zero_vel_target();
	autopilot_assign_zero_acc_feedforward();

	/* check if the height setpoint is lower than the height accepted to land */
	if(autopilot.ctrl_target.pos[2] < autopilot.landing_accept_height_lower) {
		autopilot.ctrl_target.pos[2] = autopilot.landing_accept_height_lower;
		autopilot.land_avaliable = true;
	} else {
		/* slowly change the height setpoint for landing */
		autopilot.ctrl_target.pos[2] -= autopilot.landing_speed * autopilot.period;
	}

	/* check if the height of the uav is lower than the height accepted to land */
	if((autopilot.land_avaliable == true) &&
	    (curr_pos[2] < autopilot.landing_accept_height_upper)) {
		autopilot.mode = AUTOPILOT_MOTOR_LOCKED_MODE;
		autopilot.land_avaliable = false;
	}
}
