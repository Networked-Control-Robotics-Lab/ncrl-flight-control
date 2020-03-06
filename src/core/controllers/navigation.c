#include <stdbool.h>
#include "arm_math.h"
#include "sys_time.h"
#include "navigation.h"

#define EARTH_RADIUS 6371 //[km]

nav_t *nav_ptr;

void latitude_longitude_to_cartesian(float latitude, float longitude, float *x, float *y)
{
	/* haversine formula, the error is ~0.5% (by assuming earth is spherical symmetry),
	 * can be improved by using vincenty formula if necessary */
	*x = EARTH_RADIUS * arm_cos_f32(latitude) * arm_cos_f32(longitude);
	*y = EARTH_RADIUS * arm_cos_f32(latitude) * arm_sin_f32(longitude);
	//*z = EARTH_RADIUS * arm_sin_f32(latitude);
}

void nav_init(nav_t *_nav)
{
	nav_ptr = _nav;
	nav_ptr->mode = NAV_MANUAL_FLIGHT_MODE;
	nav_ptr->landing_speed = 0.13;
	nav_ptr->takeoff_speed = 0.08;
	nav_ptr->takeoff_height = 150;          //[cm]
	nav_ptr->landing_accept_height = 20.0f; //[cm]
}

void nav_update_uav_info(float pos[3], float vel[3])
{
	nav_ptr->uav_info.pos[0] = pos[1];
	nav_ptr->uav_info.pos[1] = pos[0];
	nav_ptr->uav_info.pos[2] = pos[2]; //FIXME
}

void nav_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height)
{
	nav_ptr->geo_fence.lx = lx * 100.0f; //convet unit from [m] to [cm]
	nav_ptr->geo_fence.ly = ly * 100.0f;
	nav_ptr->geo_fence.height = height * 100.0f;
}

static bool nav_test_point_in_rectangular_fence(float p[3])
{
	if((p[0] <= (+nav_ptr->geo_fence.lx + nav_ptr->geo_fence.origin[0])) &&
	    (p[0] >= (-nav_ptr->geo_fence.lx + nav_ptr->geo_fence.origin[0])) &&
	    (p[1] <= (+nav_ptr->geo_fence.ly + nav_ptr->geo_fence.origin[1])) &&
	    (p[1] >= (-nav_ptr->geo_fence.ly + nav_ptr->geo_fence.origin[1])) &&
	    (p[2] >= 0.0f) && (p[2] <= nav_ptr->geo_fence.height)) {
		return true;
	} else {
		return false;
	}
}

int nav_add_new_waypoint(float pos[3], float heading, float halt_time_sec)
{
	if(nav_ptr->wp_num <= WAYPOINT_NUM_MAX) {
		nav_ptr->wp_list[nav_ptr->wp_num].pos[0] = pos[0];
		nav_ptr->wp_list[nav_ptr->wp_num].pos[1] = pos[1];
		nav_ptr->wp_list[nav_ptr->wp_num].pos[2] = pos[2];
		nav_ptr->wp_list[nav_ptr->wp_num].heading = heading;
		nav_ptr->wp_num++;
		return NAV_SET_SUCCEED;
	} else {
		return NAV_WP_LIST_FULL;
	}
}

int nav_clear_waypoint_list(void)
{
	if(nav_ptr->mode != NAV_FOLLOW_WAYPOINT_MODE &&
	    nav_ptr->mode != NAV_WAIT_NEXT_WAYPOINT_MODE) {
		nav_ptr->wp_num = 0;
		nav_ptr->curr_wp = 0;
		return NAV_SET_SUCCEED;
	} else {
		return NAV_MISSION_EXECUTING;
	}
}

int nav_goto_waypoint_now(float pos[3], bool change_height)
{
	bool in_fence = nav_test_point_in_rectangular_fence(pos);

	if(in_fence == true) {
		nav_ptr->mode = NAV_HOVERING_MODE;
		(nav_ptr->wp_now).pos[0] = pos[0];
		(nav_ptr->wp_now).pos[1] = pos[1];
		if(change_height == true) {
			(nav_ptr->wp_now).pos[2] = pos[2];
		}
		nav_ptr->curr_wp = 0; //reset waypoint list pointer
		return NAV_SET_SUCCEED;
	} else {
		return NAV_WP_OUT_OF_FENCE;
	}
}

int nav_halt_waypoint_mission(void)
{
	if(nav_ptr->mode == NAV_FOLLOW_WAYPOINT_MODE) {
		nav_ptr->halt_flag = true;
		return NAV_SET_SUCCEED;
	} else {
		return NAV_NO_EXECUTING_MISSION;
	}
}

int nav_resume_waypoint_mission(void)
{
	if(nav_ptr->curr_wp != 0 && nav_ptr->curr_wp != (nav_ptr->wp_num - 1)) {
		nav_ptr->mode = NAV_FOLLOW_WAYPOINT_MODE;
		return NAV_SET_SUCCEED;
	} else {
		return NAV_NO_EXECUTING_MISSION;
	}
}

int nav_waypoint_mission_start(void)
{
	if(nav_ptr->wp_num >= 1) {
		nav_ptr->curr_wp = 0;
		nav_ptr->mode = NAV_FOLLOW_WAYPOINT_MODE;
		return NAV_SET_SUCCEED;
	} else {
		return NAV_WP_LIST_EMPYT;
	}
}

int nav_trigger_auto_landing(void)
{
	if(nav_ptr->mode == NAV_HOVERING_MODE) {
		nav_ptr->mode = NAV_LANDING_MODE;
		return NAV_SET_SUCCEED;
	} else {
		return NAV_POSITION_NOT_FIXED;
	}
}

int nav_trigger_auto_takeoff(void)
{
	/* FIXME: should test motor output rather than absoulte height */
	if(nav_ptr->uav_info.pos[2] < 15.0f) {
		nav_ptr->mode = NAV_TAKEOFF_MODE;
		return NAV_SET_SUCCEED;
	} else {
		return NAV_UAV_ALREADY_TAKEOFF;
	}
}

void nav_waypoint_handler(void)
{
	static float start_time = 0.0f;
	float curr_time = 0.0f;

	/* if receive halt command */
	if(nav_ptr->halt_flag == true) {
		/* hovering at current position */
		nav_ptr->wp_now.pos[0] = nav_ptr->uav_info.pos[0];
		nav_ptr->wp_now.pos[1] = nav_ptr->uav_info.pos[1];
		nav_ptr->wp_now.pos[2] = nav_ptr->uav_info.pos[2];
		nav_ptr->halt_flag = false;
		nav_ptr->mode = NAV_HOVERING_MODE;
	}

	switch(nav_ptr->mode) {
	case NAV_MANUAL_FLIGHT_MODE:
	case NAV_TRAJECTORY_FOLLOWING_MODE:
	case NAV_HOVERING_MODE:
		return;
	case NAV_LANDING_MODE: {
		/* slowly change the height setpoint to indicate uav to the sky */
		nav_ptr->wp_now.pos[2] -= nav_ptr->landing_speed;
		if(nav_ptr->uav_info.pos[2] < nav_ptr->landing_accept_height) {
			nav_ptr->mode = NAV_MOTOR_LOCKED_MODE;
		}
		break;
	}
	case NAV_TAKEOFF_MODE: {
		/* slowly change the height setpoint to indicate uav to the ground */
		nav_ptr->wp_now.pos[2] += nav_ptr->takeoff_speed;
		if(nav_ptr->uav_info.pos[2] > nav_ptr->takeoff_height) {
			nav_ptr->mode = NAV_HOVERING_MODE;
			nav_ptr->uav_info.pos[2] = nav_ptr->takeoff_height;
		}
		break;
	}
	case NAV_WAIT_NEXT_WAYPOINT_MODE: {
		curr_time = get_sys_time_s();
		/* check if time is up */
		if((curr_time - start_time) > nav_ptr->wp_list[nav_ptr->curr_wp].halt_time_sec) {
			/* continue next waypoint if exist */
			if(nav_ptr->curr_wp < (nav_ptr->wp_num - 1)) {
				nav_ptr->mode = NAV_FOLLOW_WAYPOINT_MODE;
				nav_ptr->curr_wp++;
			} else {
				/* there is no more waypoints, hovering at the last one's position */
				nav_ptr->wp_now.pos[0] = nav_ptr->wp_list[nav_ptr->curr_wp].pos[0];
				nav_ptr->wp_now.pos[1] = nav_ptr->wp_list[nav_ptr->curr_wp].pos[1];
				nav_ptr->wp_now.pos[2] = nav_ptr->wp_list[nav_ptr->curr_wp].pos[2];
				nav_ptr->mode = NAV_HOVERING_MODE;
			}
		}
		break;
	}
	case NAV_FOLLOW_WAYPOINT_MODE: {
		/* calculate 2-norm to check if enter the waypoint touch zone or not */
		float curr_dist[3];
		curr_dist[0] = nav_ptr->uav_info.pos[0] - nav_ptr->wp_list[nav_ptr->curr_wp].pos[0];
		curr_dist[0] *= curr_dist[0];
		curr_dist[1] = nav_ptr->uav_info.pos[1] - nav_ptr->wp_list[nav_ptr->curr_wp].pos[1];
		curr_dist[1] *= curr_dist[1];
		curr_dist[2] = nav_ptr->uav_info.pos[2] - nav_ptr->wp_list[nav_ptr->curr_wp].pos[2];
		curr_dist[2] *= curr_dist[2];

		float accept_dist = nav_ptr->wp_list[nav_ptr->curr_wp].touch_radius;
		accept_dist *= accept_dist;

		if((curr_dist[0] + curr_dist[1] + curr_dist[2]) < accept_dist) {
			/* start the timer */
			start_time = get_sys_time_s();
			nav_ptr->mode = NAV_WAIT_NEXT_WAYPOINT_MODE;
		}
		break;
	}
	}
}
