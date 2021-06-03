#include "polynomial.h"
#include "sys_time.h"
#include "autopilot.h"

autopilot_t autopilot;

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

void autopilot_trajectory_following_handler(void)
{
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
}
