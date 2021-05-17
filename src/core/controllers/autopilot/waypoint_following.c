#include <stdint.h>
#include "autopilot.h"
#include "fence.h"
#include "sys_time.h"

extern autopilot_t autopilot;

int autopilot_get_waypoint_count(void)
{
	return autopilot.wp_num;
}

bool autopilot_get_waypoint_gps_mavlink(int index, int32_t *latitude, int32_t *longitude, float *height, uint16_t *cmd)
{
	if(index >= autopilot.wp_num) {
		return false; //invalid waypoint index
	} else {
		*latitude = autopilot.wp_list[index].latitude;
		*longitude = autopilot.wp_list[index].longitude;
		*height = autopilot.wp_list[index].height;
		*cmd = autopilot.wp_list[index].command;
		return true;
	}
}

int autopilot_add_new_waypoint(float pos[3], float heading, float halt_time_sec, float radius)
{
	if(autopilot_test_point_in_rectangular_fence(pos) == false) {
		return AUTOPILOT_WAYPOINT_OUT_OF_FENCE;
	} else if(autopilot.wp_num <= TRAJ_WP_MAX_NUM) {
		autopilot.wp_list[autopilot.wp_num].pos[0] = pos[0];
		autopilot.wp_list[autopilot.wp_num].pos[1] = pos[1];
		autopilot.wp_list[autopilot.wp_num].pos[2] = pos[2];
		autopilot.wp_list[autopilot.wp_num].heading = heading;
		autopilot.wp_list[autopilot.wp_num].halt_time_sec = halt_time_sec;
		autopilot.wp_list[autopilot.wp_num].touch_radius = radius;
		autopilot.wp_num++;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WAYPOINT_LIST_FULL;
	}
}

int autopilot_add_new_waypoint_gps_mavlink(int32_t latitude, int32_t longitude, float height, uint16_t cmd)
{
	//TODO: add geo-fence protection

	if(autopilot.wp_num <= TRAJ_WP_MAX_NUM) {
		autopilot.wp_list[autopilot.wp_num].latitude = latitude;
		autopilot.wp_list[autopilot.wp_num].longitude = longitude;
		autopilot.wp_list[autopilot.wp_num].height = height;
		autopilot.wp_list[autopilot.wp_num].command = cmd;

		//TODO: no idea why mavlink don't support these parameters,
		//      set default for now
		autopilot.wp_list[autopilot.wp_num].heading = 0.0;
		autopilot.wp_list[autopilot.wp_num].halt_time_sec = 3.0;
		autopilot.wp_list[autopilot.wp_num].touch_radius = 50.0;

		autopilot.wp_num++;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WAYPOINT_LIST_FULL;
	}
}

int autopilot_clear_waypoint_list(void)
{
	if(autopilot.mode != AUTOPILOT_FOLLOW_WAYPOINT_MODE &&
	    autopilot.mode != AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE) {
		autopilot.wp_num = 0;
		autopilot.curr_wp = 0;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WAYPOINT_FOLLOWING_BUSY;
	}
}

int autopilot_goto_waypoint_now(float pos[3], bool change_height)
{
	bool in_fence = autopilot_test_point_in_rectangular_fence(pos);

	if(in_fence == true) {
		autopilot.mode = AUTOPILOT_HOVERING_MODE;
		(autopilot.wp_now).pos[0] = pos[0];
		(autopilot.wp_now).pos[1] = pos[1];
		if(change_height == true) {
			(autopilot.wp_now).pos[2] = pos[2];
		}
		autopilot.curr_wp = 0; //reset waypoint list pointer
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WAYPOINT_OUT_OF_FENCE;
	}
}

int autopilot_halt_waypoint_mission(void)
{
	if(autopilot.mode == AUTOPILOT_FOLLOW_WAYPOINT_MODE ||
	    autopilot.mode == AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE) {
		autopilot.halt_flag = true;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NOT_IN_WAYPOINT_MODE;
	}
}

int autopilot_resume_waypoint_mission(void)
{
	if(autopilot.curr_wp != 0 && autopilot.curr_wp != (autopilot.wp_num - 1)) {
		autopilot.mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NO_HALTED_WAYPOINT_MISSION;
	}
}

int autopilot_waypoint_mission_start(bool loop_mission)
{
	if(autopilot.mode != AUTOPILOT_HOVERING_MODE) {
		return AUTOPILOT_NOT_IN_HOVERING_MODE;
	}

	if(autopilot.wp_num >= 1) {
		autopilot.curr_wp = 0;
		/* change to waypoint following mode */
		autopilot.mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
		autopilot.loop_mission = loop_mission;
		/* update position/velocity setpoint to controller */
		float x_target = autopilot.wp_list[autopilot.curr_wp].pos[0];
		float y_target = autopilot.wp_list[autopilot.curr_wp].pos[1];
		float z_target = autopilot.wp_list[autopilot.curr_wp].pos[2];
		autopilot_assign_pos_target(x_target, y_target, z_target);
		autopilot_assign_zero_vel_target();
		autopilot_assign_zero_acc_feedforward();
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WAYPOINT_LIST_EMPYT;
	}
}

void autopilot_mission_reset(void)
{
	autopilot.curr_wp = 0;
}

void autopilot_wait_next_waypoint_handler(void)
{
	float curr_time = get_sys_time_s();
	/* check if the time is up */
	if((curr_time - autopilot.waypoint_wait_timer) >
	    autopilot.wp_list[autopilot.curr_wp].halt_time_sec) {
		/* continue next waypoint if exist */
		if(autopilot.curr_wp < (autopilot.wp_num - 1)) {
			autopilot.mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
			autopilot.curr_wp++;
		} else {
			/* check if user ask to loop the mission */
			if(autopilot.loop_mission == true) {
				/* start waypointmission again */
				autopilot.mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
				autopilot.curr_wp = 0;
			} else {
				/* end of the mission, do hovering */
				autopilot.mode = AUTOPILOT_HOVERING_MODE;
			}
		}

		/* update position/velocity setpoint to controller */
		float x_target = autopilot.wp_list[autopilot.curr_wp].pos[0];
		float y_target = autopilot.wp_list[autopilot.curr_wp].pos[1];
		float z_target = autopilot.wp_list[autopilot.curr_wp].pos[2];
		autopilot_assign_pos_target(x_target, y_target, z_target);
		autopilot_assign_zero_vel_target();
		autopilot_assign_zero_acc_feedforward();
	}
}

void autopilot_follow_waypoint_handler(void)
{
	/* calculate 2-norm to check if enter the waypoint touch zone or not */
	float curr_dist[3];
	curr_dist[0] = autopilot.uav_state.pos[0] - autopilot.wp_list[autopilot.curr_wp].pos[0];
	curr_dist[0] *= curr_dist[0];
	curr_dist[1] = autopilot.uav_state.pos[1] - autopilot.wp_list[autopilot.curr_wp].pos[1];
	curr_dist[1] *= curr_dist[1];
	curr_dist[2] = autopilot.uav_state.pos[2] - autopilot.wp_list[autopilot.curr_wp].pos[2];
	curr_dist[2] *= curr_dist[2];

	float accept_dist = autopilot.wp_list[autopilot.curr_wp].touch_radius;
	accept_dist *= accept_dist;

	if((curr_dist[0] + curr_dist[1] + curr_dist[2]) < accept_dist) {
		/* start the timer */
		autopilot.waypoint_wait_timer = get_sys_time_s();
		autopilot.mode = AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE;
	}
}
