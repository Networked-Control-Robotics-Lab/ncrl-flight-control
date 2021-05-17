#include <stdio.h>
#include <stdbool.h>
#include "arm_math.h"
#include "sys_time.h"
#include "polynomial.h"
#include "autopilot.h"
#include "uart.h"
#include "delay.h"
#include "sbus_radio.h"
#include "ahrs.h"
#include "attitude_state.h"

#define EARTH_RADIUS 6371 //[km]

autopilot_t autopilot;

bool check_motor_lock_condition(bool condition)
{
	return condition; //true: lock, false: unlock
}

void assign_vector_3x1_enu_to_ned(float *ned, float *enu)
{
	ned[0] = enu[1];
	ned[1] = enu[0];
	ned[2] = -enu[2];
}

void autopilot_init(void)
{
	autopilot.mode = AUTOPILOT_MANUAL_FLIGHT_MODE;
	autopilot.armed = false;
	autopilot.motor_locked = false;
	autopilot.landing_speed = 0.0013; //[m/s]
	autopilot.takeoff_speed = 0.0008; //[m/s]
	autopilot.takeoff_height = 1.0f;  //[m]
	autopilot.landing_accept_height_lower = 0.10f; //[m]
	autopilot.landing_accept_height_upper = 0.12f; //[m]
	autopilot.land_avaliable = false;
}

void autopilot_update_uav_state(float pos_enu[3], float vel_enu[3])
{
	autopilot.uav_state.pos[0] = pos_enu[0];
	autopilot.uav_state.pos[1] = pos_enu[1];
	autopilot.uav_state.pos[2] = pos_enu[2];
}

bool autopilot_is_manual_flight_mode(void)
{
	if(autopilot.mode == AUTOPILOT_MANUAL_FLIGHT_MODE) {
		return true;
	} else {
		return false;
	}
}

bool autopilot_is_motor_locked_mode(void)
{
	if(autopilot.mode == AUTOPILOT_MOTOR_LOCKED_MODE) {
		return true;
	} else {
		return false;
	}
}

bool autopilot_is_auto_flight_mode(void)
{
	if(autopilot.mode != AUTOPILOT_MANUAL_FLIGHT_MODE &&
	    autopilot.mode != AUTOPILOT_MOTOR_LOCKED_MODE) {
		return true;
	} else {
		return false;
	}
}

bool autopilot_is_armed(void)
{
	return autopilot.armed;
}

void autopilot_lock_motor(void)
{
	/* caution:dangerous function, carefully use! */
	autopilot.motor_locked = true;
}

void autopilot_unlock_motor(void)
{
	/* caution:dangerous function, carefully use! */
	autopilot.motor_locked = false;
}

void autopilot_assign_pos_target_x(float x)
{
	autopilot.wp_now.pos[0] = x;
}

void autopilot_assign_pos_target_y(float y)
{
	autopilot.wp_now.pos[1] = y;
}

void autopilot_assign_pos_target_z(float z)
{
	autopilot.wp_now.pos[2] = z;
}

void autopilot_assign_pos_target(float x, float y, float z)
{
	autopilot.wp_now.pos[0] = x;
	autopilot.wp_now.pos[1] = y;
	autopilot.wp_now.pos[2] = z;
}

void autopilot_assign_vel_target(float vx, float vy, float vz)
{
	autopilot.wp_now.vel[0] = vx;
	autopilot.wp_now.vel[1] = vy;
	autopilot.wp_now.vel[2] = vz;
}

void autopilot_assign_zero_vel_target(void)
{
	autopilot.wp_now.vel[0] = 0.0f;
	autopilot.wp_now.vel[1] = 0.0f;
	autopilot.wp_now.vel[2] = 0.0f;
}

void autopilot_assign_acc_feedforward(float ax, float ay, float az)
{
	autopilot.wp_now.acc_feedforward[0] = ax;
	autopilot.wp_now.acc_feedforward[1] = ay;
	autopilot.wp_now.acc_feedforward[2] = az;
}

void autopilot_assign_zero_acc_feedforward(void)
{
	autopilot.wp_now.acc_feedforward[0] = 0.0f;
	autopilot.wp_now.acc_feedforward[1] = 0.0f;
	autopilot.wp_now.acc_feedforward[2] = 0.0f;
}

void autopilot_assign_trajactory_waypoint(float time)
{
	int curr_traj = autopilot.curr_traj;
	float *x_traj_coeff = autopilot.trajectory_segments[curr_traj].x_poly_coeff;
	float *y_traj_coeff = autopilot.trajectory_segments[curr_traj].y_poly_coeff;
	float *z_traj_coeff = autopilot.trajectory_segments[curr_traj].z_poly_coeff;

	float *vx_traj_coeff = autopilot.trajectory_segments[curr_traj].vx_poly_coeff;
	float *vy_traj_coeff = autopilot.trajectory_segments[curr_traj].vy_poly_coeff;
	float *vz_traj_coeff = autopilot.trajectory_segments[curr_traj].vz_poly_coeff;

	float *ax_traj_coeff = autopilot.trajectory_segments[curr_traj].ax_poly_coeff;
	float *ay_traj_coeff = autopilot.trajectory_segments[curr_traj].ay_poly_coeff;
	float *az_traj_coeff = autopilot.trajectory_segments[curr_traj].az_poly_coeff;

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

	if(autopilot.z_traj == false) {
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
	autopilot.geo_fence.lx = lx; //[m]
	autopilot.geo_fence.ly = ly; //[m]
	autopilot.geo_fence.height = height; //[m]
}

static bool autopilot_test_point_in_rectangular_fence(float p[3])
{
	if((p[0] <= (+autopilot.geo_fence.lx + autopilot.geo_fence.origin[0])) &&
	    (p[0] >= (-autopilot.geo_fence.lx + autopilot.geo_fence.origin[0])) &&
	    (p[1] <= (+autopilot.geo_fence.ly + autopilot.geo_fence.origin[1])) &&
	    (p[1] >= (-autopilot.geo_fence.ly + autopilot.geo_fence.origin[1])) &&
	    (p[2] >= 0.0f) && (p[2] <= autopilot.geo_fence.height)) {
		return true;
	} else {
		return false;
	}
}

void autopilot_set_mode(int new_mode)
{
	autopilot.mode = new_mode;
}

void autopilot_set_armed(void)
{
	autopilot.armed = true;
}

void autopilot_set_disarmed(void)
{
	autopilot.armed = false;
}

int autopilot_get_mode(void)
{
	return autopilot.mode;
}

int autopilot_get_waypoint_count(void)
{
	return autopilot.wp_num;
}

void autopilot_get_pos_setpoint(float *pos_set)
{
	pos_set[0] = autopilot.wp_now.pos[0];
	pos_set[1] = autopilot.wp_now.pos[1];
	pos_set[2] = autopilot.wp_now.pos[2];
}

void autopilot_get_vel_setpoint(float *vel_set)
{
	vel_set[0] = autopilot.wp_now.vel[0];
	vel_set[1] = autopilot.wp_now.vel[1];
	vel_set[2] = autopilot.wp_now.vel[2];
}

void autopilot_get_accel_feedforward(float *accel_ff)
{
	accel_ff[0] = autopilot.wp_now.acc_feedforward[0];
	accel_ff[1] = autopilot.wp_now.acc_feedforward[1];
	accel_ff[2] = autopilot.wp_now.acc_feedforward[2];
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

int autopilot_set_x_trajectory(int index, float *x_traj_coeff, float fligt_time)
{
	if(index >= (TRAJ_WP_MAX_NUM - 1)) {
		return AUTOPILOT_TRAJACTORY_LIST_FULL;
	}

	if(autopilot.mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		return AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY;
	}

	/* save position trajectory */
	copy_7th_polynomial_coefficients(autopilot.trajectory_segments[index].x_poly_coeff,
	                                 x_traj_coeff);
	/* calculate velocity trajectory */
	differentiate_7th_polynomial(autopilot.trajectory_segments[index].x_poly_coeff,
	                             autopilot.trajectory_segments[index].vx_poly_coeff);
	/* calculate acceleration trajectory */
	differentiate_6th_polynomial(autopilot.trajectory_segments[index].vx_poly_coeff,
	                             autopilot.trajectory_segments[index].ax_poly_coeff);

	autopilot.trajectory_segments[index].flight_time = fligt_time;

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_set_y_trajectory(int index, float *y_traj_coeff, float fligt_time)
{
	if(index >= (TRAJ_WP_MAX_NUM - 1)) {
		return AUTOPILOT_TRAJACTORY_LIST_FULL;
	}

	if(autopilot.mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		return AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY;
	}

	/* save position trajectory */
	copy_7th_polynomial_coefficients(autopilot.trajectory_segments[index].y_poly_coeff,
	                                 y_traj_coeff);
	/* calculate velocity trajectory */
	differentiate_7th_polynomial(autopilot.trajectory_segments[index].y_poly_coeff,
	                             autopilot.trajectory_segments[index].vy_poly_coeff);
	/* calculate acceleration trajectotry */
	differentiate_6th_polynomial(autopilot.trajectory_segments[index].vy_poly_coeff,
	                             autopilot.trajectory_segments[index].ay_poly_coeff);

	autopilot.trajectory_segments[index].flight_time = fligt_time;

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_set_z_trajectory(int index, float *z_traj_coeff, float fligt_time)
{
	if(index >= (TRAJ_WP_MAX_NUM - 1)) {
		return AUTOPILOT_TRAJACTORY_LIST_FULL;
	}

	if(autopilot.mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		return AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY;
	}

	/* save position trajectory */
	copy_7th_polynomial_coefficients(autopilot.trajectory_segments[index].z_poly_coeff,
	                                 z_traj_coeff);
	/* calculate velocity trajectory */
	differentiate_7th_polynomial(autopilot.trajectory_segments[index].z_poly_coeff,
	                             autopilot.trajectory_segments[index].vz_poly_coeff);
	/* calculate acceleration trajectory */
	differentiate_6th_polynomial(autopilot.trajectory_segments[index].vz_poly_coeff,
	                             autopilot.trajectory_segments[index].az_poly_coeff);

	autopilot.trajectory_segments[index].flight_time = fligt_time;

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_set_yaw_trajectory(int index, float *yaw_traj_coeff, float fligt_time)
{
	if(index >= (TRAJ_WP_MAX_NUM - 1)) {
		return AUTOPILOT_TRAJACTORY_LIST_FULL;
	}

	if(autopilot.mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		return AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY;
	}

	/* save yaw trajectory */
	copy_3th_polynomial_coefficients(autopilot.trajectory_segments[index].yaw_poly_coeff,
	                                 yaw_traj_coeff);
	/* calculate yaw rate trajectory */
	differentiate_3th_polynomial(autopilot.trajectory_segments[index].yaw_poly_coeff,
	                             autopilot.trajectory_segments[index].yaw_rate_poly_coeff);

	autopilot.trajectory_segments[index].flight_time = fligt_time;

	return AUTOPILOT_SET_SUCCEED;
}

int autopilot_config_trajectory_following(int traj_num, bool z_traj, bool yaw_traj)
{
	if(traj_num >= TRAJ_WP_MAX_NUM) {
		return AUTOPILOT_TRAJACTORY_LIST_TOO_LARGE;
	}

	if(autopilot.mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		return AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY;
	}

	autopilot.traj_num = traj_num;
	autopilot.z_traj = z_traj;
	autopilot.yaw_traj = yaw_traj;
	return AUTOPILOT_SET_SUCCEED;
}


int autopilot_trajectory_following_start(bool loop_trajectory)
{
	/* return error if trajectory list is empty */
	if(autopilot.traj_num <= 0) {
		return AUTOPILOT_TRAJACTORY_LIST_EMPTY;
	}

	/* trajectory following mode can only be triggered if uav is hovering at a
	 * fixed point */
	if(autopilot.mode == AUTOPILOT_HOVERING_MODE) {
		autopilot.loop_mission = loop_trajectory;
		autopilot.curr_traj = 0;
		autopilot.traj_start_time = get_sys_time_s();
		autopilot.mode = AUTOPILOT_TRAJECTORY_FOLLOWING_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NOT_IN_HOVERING_MODE;
	}
}

int autopilot_trajectory_following_stop(void)
{
	if(autopilot.mode == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
		autopilot.halt_flag = true;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_NOT_IN_TRAJECTORY_MODE;
	}
}

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
	if(autopilot.uav_state.pos[2] < 0.2) {
		autopilot.mode = AUTOPILOT_TAKEOFF_MODE;
		return AUTOPILOT_SET_SUCCEED;
	} else {
		return AUTOPILOT_ALREADY_TAKEOFF;
	}
}

void autopilot_hovering_position_trimming_handler(void)
{
	const float dt = 0.0001;

	/* position setpoint increment in ned body-fixed frame  */
	float x_increment_b = 0.0f;
	float y_increment_b = 0.0f;

	/* position setpoint increment in ned inertial frame */
	float x_increment_i = 0.0f;
	float y_increment_i = 0.0f;

	radio_t rc;
	sbus_rc_read(&rc);

	/* pitch */
	if(rc.pitch > 5.0f || rc.pitch < -5.0f) {
		x_increment_b = rc.pitch * dt;
	}

	/* roll */
	if(rc.roll > 5.0f || rc.roll < -5.0f) {
		y_increment_b = -rc.roll * dt; //TODO: unifying rc sign
	}

	float *R_b2i;
	get_rotation_matrix_b2i(&R_b2i);

	/* position increment in ned inertial frame */
	x_increment_i = R_b2i[0*3 + 0] * x_increment_b + (R_b2i[0*3 + 1] * y_increment_b);
	y_increment_i = R_b2i[1*3 + 0] * x_increment_b + (R_b2i[1*3 + 1] * y_increment_b);

	/* apply increment to autopilot position target variables (enu frame) */
	autopilot.wp_now.pos[0] += y_increment_i;
	autopilot.wp_now.pos[1] += x_increment_i;
}

void autopilot_guidance_handler(void)
{
	/* receive and handle remote controller commands */
	switch(autopilot.mode) {
	case AUTOPILOT_HOVERING_MODE:
		autopilot_hovering_position_trimming_handler();
		break;
	}

	static float start_time = 0.0f;
	float curr_time = 0.0f;

	/* if receive halt command */
	if(autopilot.halt_flag == true) {
		autopilot.halt_flag = false;
		autopilot.mode = AUTOPILOT_HOVERING_MODE;

		/* hovering at current position */
		autopilot_assign_pos_target(
		        autopilot.uav_state.pos[0],
		        autopilot.uav_state.pos[1],
		        autopilot.uav_state.pos[2]);
		autopilot_assign_zero_vel_target();
		autopilot_assign_zero_acc_feedforward();
	}

	/* autopilot */
	switch(autopilot.mode) {
	case AUTOPILOT_MANUAL_FLIGHT_MODE:
	case AUTOPILOT_HOVERING_MODE:
		return;
	case AUTOPILOT_TRAJECTORY_FOLLOWING_MODE: {
		/* converte trajectory polynomial to waypoint according to the update frequency */
		float current_time = get_sys_time_s();
		float elapsed_time = current_time - autopilot.traj_start_time;
		if(elapsed_time >=
		    autopilot.trajectory_segments[autopilot.curr_traj].flight_time) {
			elapsed_time = 0.0f; //reset trajectory time variable

			/* continue next trajectory if exist */
			if(autopilot.curr_traj < (autopilot.traj_num - 1)) {
				autopilot.curr_traj++;
				autopilot.traj_start_time = current_time;
			} else {
				/* check if user ask to loop the mission */
				if(autopilot.loop_mission == true) {
					/* start trajectory mission again */
					autopilot.curr_traj = 0;
					autopilot.traj_start_time = get_sys_time_s();
				} else {
					/* end of the mission, do hovering */
					autopilot.mode = AUTOPILOT_HOVERING_MODE;
				}
			}
		}

		autopilot_assign_trajactory_waypoint(elapsed_time); //update setpoint

		break;
	}
	case AUTOPILOT_LANDING_MODE: {
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
		break;
	}
	case AUTOPILOT_TAKEOFF_MODE: {
		/* slowly change the height setpoint for takeoff */
		autopilot.wp_now.pos[2] += autopilot.takeoff_speed;
		autopilot_assign_zero_vel_target();
		autopilot_assign_zero_acc_feedforward();
		if(autopilot.wp_now.pos[2] > autopilot.takeoff_height) {
			autopilot.mode = AUTOPILOT_HOVERING_MODE;
			autopilot.wp_now.pos[2] = autopilot.takeoff_height;
		}
		break;
	}
	case AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE: {
		curr_time = get_sys_time_s();
		/* check if the time is up */
		if((curr_time - start_time) >
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
		break;
	}
	case AUTOPILOT_FOLLOW_WAYPOINT_MODE: {
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
			start_time = get_sys_time_s();
			autopilot.mode = AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE;
		}
		break;
	}
	}
}

void debug_print_waypoint_list(void)
{
	char *prompt = "waypoint list:\n\r";
	uart1_puts(prompt, strlen(prompt));

	char s[200] = {0};
	int i;
	for(i = 0; i < autopilot.wp_num; i++) {
		sprintf(s, "wp #%d: x=%.1f, y=%.1f, z=%.1f, heading=%.1f,  stay_time=%.1f, radius=%.1f\n\r",
		        i, autopilot.wp_list[i].pos[0], autopilot.wp_list[i].pos[1],
		        autopilot.wp_list[i].pos[2], autopilot.wp_list[i].heading,
		        autopilot.wp_list[i].halt_time_sec, autopilot.wp_list[i].touch_radius);
		uart1_puts(s, strlen(s));
	}
}

void debug_print_waypoint_status(void)
{
	if(autopilot.mode != AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE && autopilot.mode != AUTOPILOT_FOLLOW_WAYPOINT_MODE) {
		char *no_executing_s = "autopilot off, no executing waypoint mission.\n\r";
		uart1_puts(no_executing_s, strlen(no_executing_s));
		return;
	}

	char s[200] = {'\0'};
	int curr_wp_num = autopilot.curr_wp;
	sprintf(s, "current waypoint = #%d, x=%.1fm, y=%.1fm, z=%.1fm,"
	        " heading=%.1f, stay_time=%.1f, radius=%.1fm\n\r",
	        curr_wp_num,
	        autopilot.wp_list[curr_wp_num].pos[0] * 0.01,
	        autopilot.wp_list[curr_wp_num].pos[1] * 0.01,
	        autopilot.wp_list[curr_wp_num].pos[2] * 0.01,
	        autopilot.wp_list[curr_wp_num].heading,
	        autopilot.wp_list[curr_wp_num].halt_time_sec,
	        autopilot.wp_list[curr_wp_num].touch_radius * 0.01);
	uart1_puts(s, strlen(s));
	freertos_task_delay(1);
}
