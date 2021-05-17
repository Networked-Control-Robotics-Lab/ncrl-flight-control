#include <stdint.h>
#include "autopilot.h"

extern autopilot_t autopilot;

void autopilot_takeoff_handler(void)
{
	/* slowly change the height setpoint for takeoff */
	autopilot.wp_now.pos[2] += autopilot.takeoff_speed;
	autopilot_assign_zero_vel_target();
	autopilot_assign_zero_acc_feedforward();
	if(autopilot.wp_now.pos[2] > autopilot.takeoff_height) {
		autopilot.mode = AUTOPILOT_HOVERING_MODE;
		autopilot.wp_now.pos[2] = autopilot.takeoff_height;
	}
}

void autopilot_landing_handler(void)
{
	autopilot_assign_zero_vel_target();
	autopilot_assign_zero_acc_feedforward();

	/* check if the height setpoint is lower than the height accepted to land */
	if(autopilot.wp_now.pos[2] < autopilot.landing_accept_height_lower) {
		autopilot.wp_now.pos[2] = autopilot.landing_accept_height_lower;
		autopilot.land_avaliable = true;
	} else {
		/* slowly change the height setpoint for landing */
		autopilot.wp_now.pos[2] -= autopilot.landing_speed;
	}

	/* check if the height of the uav is lower than the height accepted to land */
	if(autopilot.land_avaliable == true &&
	    autopilot.uav_state.pos[2] < autopilot.landing_accept_height_upper) {
		autopilot.mode = AUTOPILOT_MOTOR_LOCKED_MODE;
		autopilot.land_avaliable = false;
	}
}
