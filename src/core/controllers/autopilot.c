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
	autopilot_ptr->motor_locked = false;
	autopilot_ptr->landing_speed = 0.0013;        //[m/s]
	autopilot_ptr->takeoff_speed = 0.0008;        //[m/s]
	autopilot_ptr->takeoff_height = 1.0f;         //[m]
	autopilot_ptr->landing_accept_height = 0.15f; //[m]
}

void autopilot_update_uav_state(float pos_enu[3], float vel_enu[3])
{
	autopilot_ptr->uav_state.pos[0] = pos_enu[0];
	autopilot_ptr->uav_state.pos[1] = pos_enu[1];
	autopilot_ptr->uav_state.pos[2] = pos_enu[2];
}

bool autopilot_is_manual_flight_mode(void)
{
	if(autopilot_ptr->mode == AUTOPILOT_MANUAL_FLIGHT_MODE) {
		return true;
	} else {
		return false;
	}
}

bool autopilot_is_motor_locked_mode(void)
{
	if(autopilot_ptr->mode == AUTOPILOT_MOTOR_LOCKED_MODE) {
		return true;
	} else {
		return false;
	}
}

bool autopilot_is_auto_flight_mode(void)
{
	if(autopilot_ptr->mode != AUTOPILOT_MANUAL_FLIGHT_MODE &&
	    autopilot_ptr->mode != AUTOPILOT_MOTOR_LOCKED_MODE) {
		return true;
	} else {
		return false;
	}
}

void autopilot_assign_pos_target_x(float x)
{
	autopilot_ptr->wp_now.pos[0] = x;
}

void autopilot_assign_pos_target_y(float y)
{
	autopilot_ptr->wp_now.pos[1] = y;
}

void autopilot_assign_pos_target_z(float z)
{
	autopilot_ptr->wp_now.pos[2] = z;
}

void autopilot_assign_pos_target(float x, float y, float z)
{
	autopilot_ptr->wp_now.pos[0] = x;
	autopilot_ptr->wp_now.pos[1] = y;
	autopilot_ptr->wp_now.pos[2] = z;
}

void autopilot_assign_vel_target(float vx, float vy, float vz)
{
	autopilot_ptr->wp_now.vel[0] = vx;
	autopilot_ptr->wp_now.vel[1] = vy;
	autopilot_ptr->wp_now.vel[2] = vz;
}

void autopilot_assign_zero_vel_target(void)
{
	autopilot_ptr->wp_now.vel[0] = 0.0f;
	autopilot_ptr->wp_now.vel[1] = 0.0f;
	autopilot_ptr->wp_now.vel[2] = 0.0f;
}

void autopilot_assign_acc_feedforward(float ax, float ay, float az)
{
	autopilot_ptr->wp_now.acc_feedforward[0] = ax;
	autopilot_ptr->wp_now.acc_feedforward[1] = ay;
	autopilot_ptr->wp_now.acc_feedforward[2] = az;
}

void autopilot_assign_zero_acc_feedforward(void)
{
	autopilot_ptr->wp_now.acc_feedforward[0] = 0.0f;
	autopilot_ptr->wp_now.acc_feedforward[1] = 0.0f;
	autopilot_ptr->wp_now.acc_feedforward[2] = 0.0f;
}

void autopilot_assign_trajactory_waypoint(float time)
{
	int curr_traj = autopilot_ptr->curr_traj;
	float *x_traj_coeff = autopilot_ptr->trajectory_segments[curr_traj].x_poly_coeff;
	float *y_traj_coeff = autopilot_ptr->trajectory_segments[curr_traj].y_poly_coeff;
	float *z_traj_coeff = autopilot_ptr->trajectory_segments[curr_traj].z_poly_coeff;

	float *vx_traj_coeff = autopilot_ptr->trajectory_segments[curr_traj].vx_poly_coeff;
	float *vy_traj_coeff = autopilot_ptr->trajectory_segments[curr_traj].vy_poly_coeff;
	float *vz_traj_coeff = autopilot_ptr->trajectory_segments[curr_traj].vz_poly_coeff;

	float *ax_traj_coeff = autopilot_ptr->trajectory_segments[curr_traj].ax_poly_coeff;
	float *ay_traj_coeff = autopilot_ptr->trajectory_segments[curr_traj].ay_poly_coeff;
	float *az_traj_coeff = autopilot_ptr->trajectory_segments[curr_traj].az_poly_coeff;

	/* update position/velocity setpoint to controller */
	float x_target = calc_7th_polynomial(x_traj_coeff, time);
	float y_target = calc_7th_polynomial(y_traj_coeff, time);
	float z_target;

	float vx_target = calc_6th_polynomial(vx_traj_coeff, time);
	float vy_target = calc_6th_polynomial(vy_traj_coeff, time);
	float vz_target;

	float ax_feedforward = calc_5th_polynomial(ax_traj_coeff, time);
	float ay_feedforward = calc_5th_polynomial(ay_traj_coeff, time);
	float az_feedforward;

	if(autopilot_ptr->z_traj == false) {
		//z_target = set by remote controller
		vz_target = 0.0f;      //vz_target = zero speed
		az_feedforward = 0.0f; //az_target = zero acceleration
	} else {
		z_target = calc_7th_polynomial(z_traj_coeff, time);
		vz_target = calc_6th_polynomial(vz_traj_coeff, time);
		az_feedforward = calc_5th_polynomial(az_traj_coeff, time);

		autopilot_assign_pos_target_y(z_target);
	}

	autopilot_assign_pos_target_x(x_target);
	autopilot_assign_pos_target_y(y_target);
	autopilot_assign_vel_target(vx_target, vy_target, vz_target);
	autopilot_assign_acc_feedforward(ax_feedforward, ay_feedforward, az_feedforward);
	//autopilot_assign_zero_acc_feedforward(); //disable acceleration feedforward control
}

void autopilot_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height)
{
	autopilot_ptr->geo_fence.lx = lx; //[m]
	autopilot_ptr->geo_fence.ly = ly; //[m]
	autopilot_ptr->geo_fence.height = height; //[m]
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

bool autopilot_is_armed(void)
{
	return autopilot_ptr->armed;
}

void autopilot_mission_reset(void)
{
	autopilot_ptr->curr_wp = 0;
}

void autopilot_get_pos_setpoint(float *pos_set)
{
	pos_set[0] = autopilot_ptr->wp_now.pos[0];
	pos_set[1] = autopilot_ptr->wp_now.pos[1];
	pos_set[2] = autopilot_ptr->wp_now.pos[2];
}

void autopilot_get_vel_setpoint(float *vel_set)
{
	vel_set[0] = autopilot_ptr->wp_now.vel[0];
	vel_set[1] = autopilot_ptr->wp_now.vel[1];
	vel_set[2] = autopilot_ptr->wp_now.vel[2];
}

void autopilot_get_accel_feedforward(float *accel_ff)
{
	accel_ff[0] = autopilot_ptr->wp_now.acc_feedforward[0];
	accel_ff[1] = autopilot_ptr->wp_now.acc_feedforward[1];
	accel_ff[2] = autopilot_ptr->wp_now.acc_feedforward[2];
}

int autopilot_get_waypoint_count(void)
{
	return autopilot_ptr->wp_num;
}

bool autopilot_get_waypoint_gps_mavlink(int index, int32_t *latitude, int32_t *longitude,
                                        float *height, uint16_t *cmd)
{
	if(index >= autopilot_ptr->wp_num) {
		return false; //invalid waypoint index
	} else {
		*latitude = autopilot_ptr->wp_list[index].latitude;
		*longitude = autopilot_ptr->wp_list[index].longitude;
		*height = autopilot_ptr->wp_list[index].height;
		*cmd = autopilot_ptr->wp_list[index].command;
		return true;
	}
}

int autopilot_add_new_waypoint(float pos[3], float heading, float halt_time_sec, float radius)
{
	if(autopilot_test_point_in_rectangular_fence(pos) == false) {
		return AUTOPILOT_WAYPOINT_OUT_OF_FENCE;
	} else if(autopilot_ptr->wp_num <= TRAJ_WP_MAX_NUM) {
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].pos[0] = pos[0];
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].pos[1] = pos[1];
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].pos[2] = pos[2];
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].heading = heading;
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].halt_time_sec = halt_time_sec;
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].touch_radius = radius;
		autopilot_ptr->wp_num++;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WAYPOINT_LIST_FULL;
	}
}

int autopilot_add_new_waypoint_gps_mavlink(int32_t latitude, int32_t longitude,
                float height, uint16_t cmd)
{
	//TODO: add geo-fence protection

	if(autopilot_ptr->wp_num <= TRAJ_WP_MAX_NUM) {
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].latitude = latitude;
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].longitude = longitude;
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].height = height;
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].command = cmd;

		//TODO: no idea why mavlink don't support these parameters,
		//      set default for now
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].heading = 0.0;
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].halt_time_sec = 3.0;
		autopilot_ptr->wp_list[autopilot_ptr->wp_num].touch_radius = 50.0;

		autopilot_ptr->wp_num++;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WAYPOINT_LIST_FULL;
	}
}

int autopilot_set_x_trajectory(int index, float *x_traj_coeff, float fligt_time)
{
	if(index >= (TRAJ_WP_MAX_NUM - 1)) {
		return AUTOPILOT_TRAJACTORY_LIST_FULL;
	}

	if(autopilot_ptr->mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		return AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY;
	}

	/* save position trajectory */
	copy_7th_polynomial_coefficients(autopilot_ptr->trajectory_segments[index].x_poly_coeff,
	                                 x_traj_coeff);
	/* calculate velocity trajectory */
	differentiate_7th_polynomial(autopilot_ptr->trajectory_segments[index].x_poly_coeff,
	                             autopilot_ptr->trajectory_segments[index].vx_poly_coeff);
	/* calculate acceleration trajectory */
	differentiate_6th_polynomial(autopilot_ptr->trajectory_segments[index].vx_poly_coeff,
	                             autopilot_ptr->trajectory_segments[index].ax_poly_coeff);

	autopilot_ptr->trajectory_segments[index].flight_time = fligt_time;

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_set_y_trajectory(int index, float *y_traj_coeff, float fligt_time)
{
	if(index >= (TRAJ_WP_MAX_NUM - 1)) {
		return AUTOPILOT_TRAJACTORY_LIST_FULL;
	}

	if(autopilot_ptr->mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		return AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY;
	}

	/* save position trajectory */
	copy_7th_polynomial_coefficients(autopilot_ptr->trajectory_segments[index].y_poly_coeff,
	                                 y_traj_coeff);
	/* calculate velocity trajectory */
	differentiate_7th_polynomial(autopilot_ptr->trajectory_segments[index].y_poly_coeff,
	                             autopilot_ptr->trajectory_segments[index].vy_poly_coeff);
	/* calculate acceleration trajectotry */
	differentiate_6th_polynomial(autopilot_ptr->trajectory_segments[index].vy_poly_coeff,
	                             autopilot_ptr->trajectory_segments[index].ay_poly_coeff);

	autopilot_ptr->trajectory_segments[index].flight_time = fligt_time;

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_set_z_trajectory(int index, float *z_traj_coeff, float fligt_time)
{
	if(index >= (TRAJ_WP_MAX_NUM - 1)) {
		return AUTOPILOT_TRAJACTORY_LIST_FULL;
	}

	if(autopilot_ptr->mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		return AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY;
	}

	/* save position trajectory */
	copy_7th_polynomial_coefficients(autopilot_ptr->trajectory_segments[index].z_poly_coeff,
	                                 z_traj_coeff);
	/* calculate velocity trajectory */
	differentiate_7th_polynomial(autopilot_ptr->trajectory_segments[index].z_poly_coeff,
	                             autopilot_ptr->trajectory_segments[index].vz_poly_coeff);
	/* calculate acceleration trajectory */
	differentiate_6th_polynomial(autopilot_ptr->trajectory_segments[index].vz_poly_coeff,
	                             autopilot_ptr->trajectory_segments[index].az_poly_coeff);

	autopilot_ptr->trajectory_segments[index].flight_time = fligt_time;

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_set_yaw_trajectory(int index, float *yaw_traj_coeff, float fligt_time)
{
	if(index >= (TRAJ_WP_MAX_NUM - 1)) {
		return AUTOPILOT_TRAJACTORY_LIST_FULL;
	}

	if(autopilot_ptr->mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		return AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY;
	}

	/* save yaw trajectory */
	copy_3th_polynomial_coefficients(autopilot_ptr->trajectory_segments[index].yaw_poly_coeff,
	                                 yaw_traj_coeff);
	/* calculate yaw rate trajectory */
	differentiate_3th_polynomial(autopilot_ptr->trajectory_segments[index].yaw_poly_coeff,
	                             autopilot_ptr->trajectory_segments[index].yaw_rate_poly_coeff);

	autopilot_ptr->trajectory_segments[index].flight_time = fligt_time;

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_config_trajectory_following(int traj_num, bool z_traj, bool yaw_traj)
{
	if(traj_num >= TRAJ_WP_MAX_NUM) {
		return AUTOPILOT_TRAJACTORY_LIST_TOO_LARGE;
	}

	if(autopilot_ptr->mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		return AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY;
	}

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
		return AUTOPILOT_WAYPOINT_FOLLOWING_BUSY;
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
		return AUTOPILOT_WAYPOINT_OUT_OF_FENCE;
	}
}

int autopilot_halt_waypoint_mission(void)
{
	if(autopilot_ptr->mode == AUTOPILOT_FOLLOW_WAYPOINT_MODE ||
	    autopilot_ptr->mode == AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE) {
		autopilot_ptr->halt_flag = true;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NOT_IN_WAYPOINT_MODE;
	}
}

int autopilot_resume_waypoint_mission(void)
{
	if(autopilot_ptr->curr_wp != 0 && autopilot_ptr->curr_wp != (autopilot_ptr->wp_num - 1)) {
		autopilot_ptr->mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NO_HALTED_WAYPOINT_MISSION;
	}
}

int autopilot_waypoint_mission_start(bool loop_mission)
{
	if(autopilot_ptr->mode != AUTOPILOT_HOVERING_MODE) {
		return AUTOPILOT_NOT_IN_HOVERING_MODE;
	}

	if(autopilot_ptr->wp_num >= 1) {
		autopilot_ptr->curr_wp = 0;
		/* change to waypoint following mode */
		autopilot_ptr->mode = AUTOPILOT_FOLLOW_WAYPOINT_MODE;
		autopilot_ptr->loop_mission = loop_mission;
		/* update position/velocity setpoint to controller */
		float x_target = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[0];
		float y_target = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[1];
		float z_target = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[2];
		autopilot_assign_pos_target(x_target, y_target, z_target);
		autopilot_assign_zero_vel_target();
		autopilot_assign_zero_acc_feedforward();
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_WAYPOINT_LIST_EMPYT;
	}
}

int autopilot_trajectory_following_start(bool loop_trajectory)
{
	/* return error if trajectory list is empty */
	if(autopilot_ptr->traj_num <= 0) {
		return AUTOPILOT_TRAJACTORY_LIST_EMPTY;
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
		autopilot_ptr->halt_flag = true;
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
		return AUTOPILOT_NOT_IN_HOVERING_MODE;
	}
}

bool autopilot_motor_ls_lock(void)
{
	return autopilot_ptr->motor_locked;
}
void autopilot_lock_motor(void)
{
	/* caution:dangerous function, carefully use! */
	autopilot_ptr->motor_locked = true;
}

void autopilot_unlock_motor(void)
{
	/* caution:dangerous function, carefully use! */
	autopilot_ptr->motor_locked = false;
}

int autopilot_trigger_auto_takeoff(void)
{
	if(autopilot_ptr->uav_state.pos[2] < 0.2) {
		autopilot_ptr->mode = AUTOPILOT_TAKEOFF_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_ALREADY_TAKEOFF;
	}
}

void autopilot_guidance_handler(void)
{
	static float start_time = 0.0f;
	float curr_time = 0.0f;

	/* if receive halt command */
	if(autopilot_ptr->halt_flag == true) {
		autopilot_ptr->halt_flag = false;
		autopilot_ptr->mode = AUTOPILOT_HOVERING_MODE;

		/* hovering at current position */
		autopilot_assign_pos_target(
		        autopilot_ptr->uav_state.pos[0],
		        autopilot_ptr->uav_state.pos[1],
		        autopilot_ptr->uav_state.pos[2]);
		autopilot_assign_zero_vel_target();
		autopilot_assign_zero_acc_feedforward();
	}

	switch(autopilot_ptr->mode) {
	case AUTOPILOT_MANUAL_FLIGHT_MODE:
	case AUTOPILOT_HOVERING_MODE:
		return;
	case AUTOPILOT_TRAJECTORY_FOLLOWING_MODE: {
		/* converte trajectory polynomial to waypoint according to the update frequency */
		float current_time = get_sys_time_s();
		float elapsed_time = current_time - autopilot_ptr->traj_start_time;
		if(elapsed_time >=
		    autopilot_ptr->trajectory_segments[autopilot_ptr->curr_traj].flight_time) {
			elapsed_time = 0.0f; //reset trajectory time variable

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
		autopilot_assign_zero_vel_target();
		autopilot_assign_zero_acc_feedforward();
		if(autopilot_ptr->uav_state.pos[2] < autopilot_ptr->landing_accept_height) {
			autopilot_ptr->mode = AUTOPILOT_MOTOR_LOCKED_MODE;
		}
		break;
	}
	case AUTOPILOT_TAKEOFF_MODE: {
		/* slowly change the height setpoint to indicate uav to the ground */
		autopilot_ptr->wp_now.pos[2] += autopilot_ptr->takeoff_speed;
		if(autopilot_ptr->uav_state.pos[2] > autopilot_ptr->takeoff_height) {
			autopilot_ptr->mode = AUTOPILOT_HOVERING_MODE;
			autopilot_ptr->uav_state.pos[2] = autopilot_ptr->takeoff_height;
			autopilot_assign_zero_vel_target();
			autopilot_assign_zero_acc_feedforward();
		}
		break;
	}
	case AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE: {
		curr_time = get_sys_time_s();
		/* check if time is up */
		if((curr_time - start_time) >
		    autopilot_ptr->wp_list[autopilot_ptr->curr_wp].halt_time_sec) {
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

			/* update position/velocity setpoint to controller */
			float x_target = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[0];
			float y_target = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[1];
			float z_target = autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[2];
			autopilot_assign_pos_target(x_target, y_target, z_target);
			autopilot_assign_zero_vel_target();
			autopilot_assign_zero_acc_feedforward();
		}
		break;
	}
	case AUTOPILOT_FOLLOW_WAYPOINT_MODE: {
		/* calculate 2-norm to check if enter the waypoint touch zone or not */
		float curr_dist[3];
		curr_dist[0] = autopilot_ptr->uav_state.pos[0] - autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[0];
		curr_dist[0] *= curr_dist[0];
		curr_dist[1] = autopilot_ptr->uav_state.pos[1] - autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[1];
		curr_dist[1] *= curr_dist[1];
		curr_dist[2] = autopilot_ptr->uav_state.pos[2] - autopilot_ptr->wp_list[autopilot_ptr->curr_wp].pos[2];
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
