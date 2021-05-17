#include <stdint.h>
#include "autopilot.h"
#include "fence.h"
#include "sys_time.h"

extern autopilot_t autopilot;

int autopilot_get_waypoint_count(void)
{
	return autopilot.waypoint_num;
}

bool autopilot_get_waypoint_gps_mavlink(int index, int32_t *latitude, int32_t *longitude, float *height, uint16_t *cmd)
{
	if(index >= autopilot.waypoint_num) {
		return false; //invalid waypoint index
	} else {
		*latitude = autopilot.waypoints[index].latitude;
		*longitude = autopilot.waypoints[index].longitude;
		*height = autopilot.waypoints[index].height;
		*cmd = autopilot.waypoints[index].command;
		return true;
	}
}

int autopilot_add_new_waypoint(float pos[3], float heading, float halt_time_sec, float radius)
{
	if(autopilot_test_point_in_rectangular_fence(pos) == false) {
		return AUTOPILOT_WAYPOINT_OUT_OF_FENCE;
	} else if(autopilot.waypoint_num <= TRAJ_WP_MAX_NUM) {
		int curr_waypoint_num = autopilot.curr_waypoint;
		autopilot.waypoints[curr_waypoint_num].pos[0] = pos[0];
		autopilot.waypoints[curr_waypoint_num].pos[1] = pos[1];
		autopilot.waypoints[curr_waypoint_num].pos[2] = pos[2];
		autopilot.waypoints[curr_waypoint_num].heading = heading;
		autopilot.waypoints[curr_waypoint_num].halt_time_sec = halt_time_sec;
		autopilot.waypoints[curr_waypoint_num].touch_radius = radius;
		autopilot.waypoint_num++;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WAYPOINT_LIST_FULL;
	}
}

int autopilot_add_new_waypoint_gps_mavlink(int32_t latitude, int32_t longitude, float height, uint16_t cmd)
{
	//TODO: add geo-fence protection

	if(autopilot.waypoint_num <= TRAJ_WP_MAX_NUM) {
		int curr_waypoint_num = autopilot.curr_waypoint;
		autopilot.waypoints[curr_waypoint_num].latitude = latitude;
		autopilot.waypoints[curr_waypoint_num].longitude = longitude;
		autopilot.waypoints[curr_waypoint_num].height = height;
		autopilot.waypoints[curr_waypoint_num].command = cmd;

		//TODO: no idea why mavlink doesn't support these parameters,
		//      set default for now
		autopilot.waypoints[curr_waypoint_num].heading = 0.0;
		autopilot.waypoints[curr_waypoint_num].halt_time_sec = 3.0;
		autopilot.waypoints[curr_waypoint_num].touch_radius = 50.0;

		autopilot.waypoint_num++;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WAYPOINT_LIST_FULL;
	}
}

int autopilot_clear_waypoint_list(void)
{
	if(autopilot.mode != AUTOPILOT_FOLLOW_WAYPOINT_MODE &&
	    autopilot.mode != AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE) {
		autopilot.waypoint_num = 0;
		autopilot.curr_waypoint = 0;
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
		autopilot.ctrl_target.pos[0] = pos[0];
		autopilot.ctrl_target.pos[1] = pos[1];
		if(change_height == true) {
			autopilot.ctrl_target.pos[2] = pos[2];
		}
		autopilot.curr_waypoint = 0; //reset waypoint index to zero
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
	if(autopilot.curr_waypoint != 0 && autopilot.curr_waypoint != (autopilot.waypoint_num - 1)) {
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

	if(autopilot.waypoint_num >= 1) {
		autopilot.curr_waypoint = 0;
		/* change to waypoint following mode */
		autopilot.mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
		autopilot.loop_mission = loop_mission;
		/* update position/velocity setpoint to the controller */
		int curr_waypoint_num = autopilot.curr_waypoint;
		float x_target = autopilot.waypoints[curr_waypoint_num].pos[0];
		float y_target = autopilot.waypoints[curr_waypoint_num].pos[1];
		float z_target = autopilot.waypoints[curr_waypoint_num].pos[2];
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
	autopilot.curr_waypoint = 0;
}

void autopilot_wait_next_waypoint_handler(void)
{
	int curr_waypoint_num = autopilot.curr_waypoint;
	float curr_time = get_sys_time_s();

	/* check if the time is up */
	if((curr_time - autopilot.waypoint_wait_timer) >
	    autopilot.waypoints[curr_waypoint_num].halt_time_sec) {
		/* continue next waypoint if exist */
		if(curr_waypoint_num < (autopilot.waypoint_num - 1)) {
			autopilot.mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
			autopilot.curr_waypoint++;
		} else {
			/* check if user ask to loop the mission */
			if(autopilot.loop_mission == true) {
				/* start waypointmission again */
				autopilot.mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
				autopilot.curr_waypoint = 0;
			} else {
				/* end of the mission, do hovering */
				autopilot.mode = AUTOPILOT_HOVERING_MODE;
			}
		}

		/* update position/velocity setpoint to the controller */
		float x_target = autopilot.waypoints[curr_waypoint_num].pos[0];
		float y_target = autopilot.waypoints[curr_waypoint_num].pos[1];
		float z_target = autopilot.waypoints[curr_waypoint_num].pos[2];
		autopilot_assign_pos_target(x_target, y_target, z_target);
		autopilot_assign_zero_vel_target();
		autopilot_assign_zero_acc_feedforward();
	}
}

void autopilot_follow_waypoint_handler(float *curr_pos)
{
	/* calculate 2-norm to check if enter the waypoint touch zone or not */
	float curr_dist[3];
	int curr_waypoint_num = autopilot.curr_waypoint;
	curr_dist[0] = curr_pos[0] - autopilot.waypoints[curr_waypoint_num].pos[0];
	curr_dist[1] = curr_pos[1] - autopilot.waypoints[curr_waypoint_num].pos[1];
	curr_dist[2] = curr_pos[2] - autopilot.waypoints[curr_waypoint_num].pos[2];
	curr_dist[0] *= curr_dist[0];
	curr_dist[1] *= curr_dist[1];
	curr_dist[2] *= curr_dist[2];

	float accept_dist = autopilot.waypoints[curr_waypoint_num].touch_radius;
	accept_dist *= accept_dist;

	if((curr_dist[0] + curr_dist[1] + curr_dist[2]) < accept_dist) {
		/* start the timer */
		autopilot.waypoint_wait_timer = get_sys_time_s();
		autopilot.mode = AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE;
	}
}
