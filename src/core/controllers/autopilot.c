#include <stdio.h>
#include <stdbool.h>
#include "arm_math.h"
#include "sys_time.h"
#include "polynomial.h"
#include "autopilot.h"
#include "uart.h"
#include "delay.h"

#define EARTH_RADIUS 6371 //[km]

autopilot_t *autopilot_ptr;

void latitude_longitude_to_cartesian(float latitude, float longitude, float *x, float *y)
{
	/* haversine formula, the error is ~0.5% (by assuming earth is spherical symmetry),
	 * can be improved by using vincenty formula if necessary */
	*x = EARTH_RADIUS * arm_cos_f32(latitude) * arm_cos_f32(longitude);
	*y = EARTH_RADIUS * arm_cos_f32(latitude) * arm_sin_f32(longitude);
	//*z = EARTH_RADIUS * arm_sin_f32(latitude);
}

void assign_vector_3x1_eun_to_ned(float *ned, float *enu)
{
	ned[0] = enu[1];
	ned[1] = enu[0];
	ned[2] = -enu[2];
}

void autopilot_init(autopilot_t *_autopilot)
{
	autopilot_ptr = _autopilot;
	autopilot_ptr->mode = AUTOPILOT_MANUAL_FLIGHT_MODE;
	autopilot_ptr->armed = false;
	autopilot_ptr->landing_speed = 0.13;
	autopilot_ptr->takeoff_speed = 0.08;
	autopilot_ptr->takeoff_height = 100;          //[cm]
	autopilot_ptr->landing_accept_height = 15.0f; //[cm]
	autopilot_ptr->traj_update_time_last = 0.0f;
}

void autopilot_update_uav_info(float pos_enu[3], float vel_enu[3])
{
	autopilot_ptr->uav_info.pos[0] = pos_enu[0];
	autopilot_ptr->uav_info.pos[1] = pos_enu[1];
	autopilot_ptr->uav_info.pos[2] = pos_enu[2];
}

void autopilot_assign_trajactory_waypoint(float time)
{
	//TODO: unifiy the unit
	int curr_traj = autopilot_ptr->curr_traj;
	autopilot_ptr->wp_now.pos[0] = 100.0f *
		calc_7th_polynomial(autopilot_ptr->trajectory_segments[curr_traj].x_poly_coeff, time);
	autopilot_ptr->wp_now.pos[1] = 100.0f *
		calc_7th_polynomial(autopilot_ptr->trajectory_segments[curr_traj].y_poly_coeff, time);
	autopilot_ptr->wp_now.vel[0] = 100.0f *
		calc_6th_polynomial(autopilot_ptr->trajectory_segments[curr_traj].vx_poly_coeff, time);
	autopilot_ptr->wp_now.vel[1] = 100.0f *
		calc_6th_polynomial(autopilot_ptr->trajectory_segments[curr_traj].vy_poly_coeff, time);
	autopilot_ptr->wp_now.vel[2] = 0.0f;

	//TODO: altitude fixed mode
	//TODO: yaw motion planning
}

void autopilot_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height)
{
	autopilot_ptr->geo_fence.lx = lx * 100.0f; //convet unit from [m] to [cm]
	autopilot_ptr->geo_fence.ly = ly * 100.0f;
	autopilot_ptr->geo_fence.height = height * 100.0f;
}

static bool autopilot_test_point_in_rectangular_fence(float p[3])
{
	if((p[0] <= (+autopilot_ptr->geo_fence.lx + autopilot_ptr->geo_fence.origin[0])) &&
	    (p[0] >= (-autopilot_ptr->geo_fence.lx + autopilot_ptr->geo_fence.origin[0])) &&
	    (p[1] <= (+autopilot_ptr->geo_fence.ly + autopilot_ptr->geo_fence.origin[1])) &&
	    (p[1] >= (-autopilot_ptr->geo_fence.ly + autopilot_ptr->geo_fence.origin[1])) &&
	    (p[2] >= 0.0f) && (p[2] <= autopilot_ptr->geo_fence.height)) {
		return true;
	} else {
		return false;
	}
}

void autopilot_set_mode(int new_mode)
{
	autopilot_ptr->mode = new_mode;
}

int autopilot_get_mode(void)
{
	return autopilot_ptr->mode;
}

void autopilot_set_armed(void)
{
	autopilot_ptr->armed = true;
}

void autopilot_set_disarmed(void)
{
	autopilot_ptr->armed = false;
}

bool autopilot_get_is_armed(void)
{
	return autopilot_ptr->armed;
}

void autopilot_mission_reset(void)
{
	autopilot_ptr->curr_wp = 0;
}

int autopilot_add_new_waypoint(float pos[3], float heading, float halt_time_sec, float radius)
{
	if(autopilot_test_point_in_rectangular_fence(pos) == false) {
		return AUTOPILOT_WP_OUT_OF_FENCE;
	} else if(autopilot_ptr->wp_num <= WAYPOINT_NUM_MAX) {
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].pos[0] = pos[0];
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].pos[1] = pos[1];
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].pos[2] = pos[2];
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].heading = heading;
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].halt_time_sec = halt_time_sec;
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].touch_radius = radius;
		autopilot_ptr->wp_num++;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WP_LIST_FULL;
	}
}

int autopilot_set_x_trajectory(int index, float *x_traj_coeff, float fligt_time)
{
	//TODO: check trajectory list size
	//TODO: check autopilot mode
	//TODO: derive velocity coefficients from position coefficients
	copy_7th_polynomial_coefficients(autopilot_ptr->trajectory_segments[index].x_poly_coeff,
	                                 x_traj_coeff);
	autopilot_ptr->trajectory_segments[index].flight_time = fligt_time;
	differentiate_7th_polynomial(autopilot_ptr->trajectory_segments[index].x_poly_coeff,
                                     autopilot_ptr->trajectory_segments[index].vx_poly_coeff);

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_set_y_trajectory(int index, float *y_traj_coeff, float fligt_time)
{
	//TODO: check trajectory list size
	//TODO: check autopilot mode
	//TODO: derive velocity coefficients from position coefficients
	copy_7th_polynomial_coefficients(autopilot_ptr->trajectory_segments[index].y_poly_coeff,
	                                 y_traj_coeff);
	autopilot_ptr->trajectory_segments[index].flight_time = fligt_time;
	differentiate_7th_polynomial(autopilot_ptr->trajectory_segments[index].y_poly_coeff,
                                     autopilot_ptr->trajectory_segments[index].vy_poly_coeff);

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_set_z_trajectory(int index, float *z_traj_coeff, float fligt_time)
{
	//TODO: check trajectory list size
	//TODO: check autopilot mode
	//TODO: derive velocity coefficients from position coefficients
	copy_7th_polynomial_coefficients(autopilot_ptr->trajectory_segments[index].z_poly_coeff,
	                                 z_traj_coeff);
	autopilot_ptr->trajectory_segments[index].flight_time = fligt_time;
	differentiate_7th_polynomial(autopilot_ptr->trajectory_segments[index].z_poly_coeff,
                                     autopilot_ptr->trajectory_segments[index].vz_poly_coeff);

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_set_yaw_trajectory(int index, float *yaw_traj_coeff, float fligt_time)
{
	//TODO: check trajectory list size
	//TODO: check autopilot mode
	//TODO: derive velocity coefficients from position coefficients
	copy_3th_polynomial_coefficients(autopilot_ptr->trajectory_segments[index].yaw_poly_coeff,
	                                 yaw_traj_coeff);
	autopilot_ptr->trajectory_segments[index].flight_time = fligt_time;
	differentiate_3th_polynomial(autopilot_ptr->trajectory_segments[index].yaw_poly_coeff,
                                     autopilot_ptr->trajectory_segments[index].yaw_rate_poly_coeff);

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_config_trajectory_following(int traj_num, bool z_traj, bool yaw_traj)
{
	//TODO: check trajectory list size
	//TODO: check autopilot mode
	//TODO: derive velocity coefficients from position coefficients
	autopilot_ptr->traj_num = traj_num;
	autopilot_ptr->z_traj = z_traj;
	autopilot_ptr->yaw_traj = yaw_traj;
	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_clear_waypoint_list(void)
{
	if(autopilot_ptr->mode != AUTOPILOT_FOLLOW_WAYPOINT_MODE &&
	    autopilot_ptr->mode != AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE) {
		autopilot_ptr->wp_num = 0;
		autopilot_ptr->curr_wp = 0;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_MISSION_EXECUTING;
	}
}

int autopilot_goto_waypoint_now(float pos[3], bool change_height)
{
	bool in_fence = autopilot_test_point_in_rectangular_fence(pos);

	if(in_fence == true) {
		autopilot_ptr->mode = AUTOPILOT_HOVERING_MODE;
		(autopilot_ptr->wp_now).pos[0] = pos[0];
		(autopilot_ptr->wp_now).pos[1] = pos[1];
		if(change_height == true) {
			(autopilot_ptr->wp_now).pos[2] = pos[2];
		}
		autopilot_ptr->curr_wp = 0; //reset waypoint list pointer
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WP_OUT_OF_FENCE;
	}
}

int autopilot_halt_waypoint_mission(void)
{
	if(autopilot_ptr->mode == AUTOPILOT_FOLLOW_WAYPOINT_MODE ||
	    autopilot_ptr->mode == AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE) {
		autopilot_ptr->halt_flag = true;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NO_EXECUTING_MISSION;
	}
}

int autopilot_resume_waypoint_mission(void)
{
	if(autopilot_ptr->curr_wp != 0 && autopilot_ptr->curr_wp != (autopilot_ptr->wp_num - 1)) {
		autopilot_ptr->mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NO_EXECUTING_MISSION;
	}
}

int autopilot_waypoint_mission_start(bool loop_mission)
{
	if(autopilot_ptr->mode != AUTOPILOT_HOVERING_MODE) {
		return AUTOPILOT_NOT_IN_HOVERING_MODE;
	}

	if(autopilot_ptr->wp_num >= 1) {
		autopilot_ptr->curr_wp = 0;
		autopilot_ptr->mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
		autopilot_ptr->wp_now.pos[0] = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[0];
		autopilot_ptr->wp_now.pos[1] = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[1];
		autopilot_ptr->wp_now.pos[2] = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[2];
		autopilot_ptr->loop_mission = loop_mission;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WP_LIST_EMPYT;
	}
}

int autopilot_trajectory_following_start(bool loop_trajectory)
{
	/* return error if trajectory list is empty */
	if(autopilot_ptr->traj_num <= 0) {
		return AUTOPILOT_TRAJ_LIST_EMPTY;
	}

	/* trajectory following mode can only be triggered if uav is hovering at a
	 * fixed point */
	if(autopilot_ptr->mode == AUTOPILOT_HOVERING_MODE) {
		autopilot_ptr->loop_mission = loop_trajectory;
		autopilot_ptr->curr_traj = 0;
		autopilot_ptr->traj_start_time = get_sys_time_s();
		autopilot_ptr->mode = AUTOPILOT_TRAJECTORY_FOLLOWING_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NOT_IN_HOVERING_MODE;
	}
}

int autopilot_trajectory_following_stop(void)
{
	if(autopilot_ptr->mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		autopilot_ptr->mode = AUTOPILOT_HOVERING_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NOT_IN_TRAJECTORY_MODE;
	}
}

int autopilot_trigger_auto_landing(void)
{
	if(autopilot_ptr->mode == AUTOPILOT_HOVERING_MODE) {
		autopilot_ptr->mode = AUTOPILOT_LANDING_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_POSITION_NOT_FIXED;
	}
}

int autopilot_trigger_auto_takeoff(void)
{
	/* FIXME: should test motor output rather than absoulte height */
	if(autopilot_ptr->uav_info.pos[2] < 15.0f) {
		autopilot_ptr->mode = AUTOPILOT_TAKEOFF_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_UAV_ALREADY_TAKEOFF;
	}
}

void autopilot_waypoint_handler(void)
{
	static float start_time = 0.0f;
	float curr_time = 0.0f;

	/* if receive halt command */
	if(autopilot_ptr->halt_flag == true) {
		/* hovering at current position */
		autopilot_ptr->wp_now.pos[0] = autopilot_ptr->uav_info.pos[0];
		autopilot_ptr->wp_now.pos[1] = autopilot_ptr->uav_info.pos[1];
		autopilot_ptr->wp_now.pos[2] = autopilot_ptr->uav_info.pos[2];
		autopilot_ptr->halt_flag = false;
		autopilot_ptr->mode = AUTOPILOT_HOVERING_MODE;
	}

	switch(autopilot_ptr->mode) {
	case AUTOPILOT_MANUAL_FLIGHT_MODE:
	case AUTOPILOT_HOVERING_MODE:
		return;
	case AUTOPILOT_TRAJECTORY_FOLLOWING_MODE: {
		/* converting trajectory polynomial to waypoint according to the update frequency*/
		float current_time = get_sys_time_s();
		if((current_time - autopilot_ptr->traj_update_time_last) < TRAJECTORY_WP_UPDATE_TIME) {
			break;
		} else {
			autopilot_ptr->traj_update_time_last = current_time;
		}

		float elapsed_time = current_time - autopilot_ptr->traj_start_time;
		if(elapsed_time >=
		    autopilot_ptr->trajectory_segments[autopilot_ptr->curr_traj].flight_time) {
			/* continue next trajectory if exist */
			if(autopilot_ptr->curr_traj < (autopilot_ptr->traj_num - 1)) {
				autopilot_ptr->curr_traj++;
				autopilot_ptr->traj_start_time = current_time;
			} else {
				/* check if user ask to loop the mission */
				if(autopilot_ptr->loop_mission == true) {
					/* start trajectory mission again */
					autopilot_ptr->curr_traj = 0;
					autopilot_ptr->traj_start_time = get_sys_time_s();
				} else {
					/* end of the mission, do hovering */
					autopilot_ptr->mode = AUTOPILOT_HOVERING_MODE;
				}
			}
		}

		autopilot_assign_trajactory_waypoint(elapsed_time); //update setpoint

		break;
	}
	case AUTOPILOT_LANDING_MODE: {
		/* slowly change the height setpoint to indicate uav to the sky */
		autopilot_ptr->wp_now.pos[2] -= autopilot_ptr->landing_speed;
		if(autopilot_ptr->uav_info.pos[2] < autopilot_ptr->landing_accept_height) {
			autopilot_ptr->mode = AUTOPILOT_MOTOR_LOCKED_MODE;
		}
		break;
	}
	case AUTOPILOT_TAKEOFF_MODE: {
		/* slowly change the height setpoint to indicate uav to the ground */
		autopilot_ptr->wp_now.pos[2] += autopilot_ptr->takeoff_speed;
		if(autopilot_ptr->uav_info.pos[2] > autopilot_ptr->takeoff_height) {
			autopilot_ptr->mode = AUTOPILOT_HOVERING_MODE;
			autopilot_ptr->uav_info.pos[2] = autopilot_ptr->takeoff_height;
		}
		break;
	}
	case AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE: {
		curr_time = get_sys_time_s();
		/* check if time is up */
		if((curr_time - start_time) > autopilot_ptr->wp_list[autopilot_ptr->curr_wp].halt_time_sec) {
			/* continue next waypoint if exist */
			if(autopilot_ptr->curr_wp < (autopilot_ptr->wp_num - 1)) {
				autopilot_ptr->mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
				autopilot_ptr->curr_wp++;
			} else {
				/* check if user ask to loop the mission */
				if(autopilot_ptr->loop_mission == true) {
					/* start waypointmission again */
					autopilot_ptr->mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
					autopilot_ptr->curr_wp = 0;
				} else {
					/* end of the mission, do hovering */
					autopilot_ptr->mode = AUTOPILOT_HOVERING_MODE;
				}
			}

			autopilot_ptr->wp_now.pos[0] = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[0];
			autopilot_ptr->wp_now.pos[1] = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[1];
			autopilot_ptr->wp_now.pos[2] = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[2];
		}
		break;
	}
	case AUTOPILOT_FOLLOW_WAYPOINT_MODE: {
		/* calculate 2-norm to check if enter the waypoint touch zone or not */
		float curr_dist[3];
		curr_dist[0] = autopilot_ptr->uav_info.pos[0] - autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[0];
		curr_dist[0] *= curr_dist[0];
		curr_dist[1] = autopilot_ptr->uav_info.pos[1] - autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[1];
		curr_dist[1] *= curr_dist[1];
		curr_dist[2] = autopilot_ptr->uav_info.pos[2] - autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[2];
		curr_dist[2] *= curr_dist[2];

		float accept_dist = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].touch_radius;
		accept_dist *= accept_dist;

		if((curr_dist[0] + curr_dist[1] + curr_dist[2]) < accept_dist) {
			/* start the timer */
			start_time = get_sys_time_s();
			autopilot_ptr->mode = AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE;
		}
		break;
	}
	}
}

void debug_print_waypoint_list(void)
{
	char *prompt = "waypoint list:\n\r";
	uart3_puts(prompt, strlen(prompt));

	char s[200] = {0};
	int i;
	for(i = 0; i < autopilot_ptr->wp_num; i++) {
		sprintf(s, "wp #%d: x=%.1f, y=%.1f, z=%.1f, heading=%.1f,  stay_time=%.1f, radius=%.1f\n\r",
		        i, autopilot_ptr->wp_list[i].pos[0], autopilot_ptr->wp_list[i].pos[1],
		        autopilot_ptr->wp_list[i].pos[2], autopilot_ptr->wp_list[i].heading,
		        autopilot_ptr->wp_list[i].halt_time_sec, autopilot_ptr->wp_list[i].touch_radius);
		uart3_puts(s, strlen(s));
	}
}

void debug_print_waypoint_status(void)
{
	if(autopilot_ptr->mode != AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE && autopilot_ptr->mode != AUTOPILOT_FOLLOW_WAYPOINT_MODE) {
		char *no_executing_s = "autopilot off, no executing waypoint mission.\n\r";
		uart3_puts(no_executing_s, strlen(no_executing_s));
		return;
	}

	char s[200] = {'\0'};
	int curr_wp_num = autopilot_ptr->curr_wp;
	sprintf(s, "current waypoint = #%d, x=%.1fm, y=%.1fm, z=%.1fm,"
	        " heading=%.1f, stay_time=%.1f, radius=%.1fm\n\r",
	        curr_wp_num,
	        autopilot_ptr->wp_list[curr_wp_num].pos[0] * 0.01,
	        autopilot_ptr->wp_list[curr_wp_num].pos[1] * 0.01,
	        autopilot_ptr->wp_list[curr_wp_num].pos[2] * 0.01,
	        autopilot_ptr->wp_list[curr_wp_num].heading,
	        autopilot_ptr->wp_list[curr_wp_num].halt_time_sec,
	        autopilot_ptr->wp_list[curr_wp_num].touch_radius * 0.01);
	uart3_puts(s, strlen(s));
	freertos_task_delay(1);
}
