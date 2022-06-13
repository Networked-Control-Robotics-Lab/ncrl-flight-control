#ifndef __AUTOPILOT_H__
#define __AUTOPILOT_H__

#include <stdint.h>
#include <stdbool.h>
#include "sbus_radio.h"

#define TRAJ_WP_MAX_NUM 50

enum {
	/* user manual flight mode */
	AUTOPILOT_MANUAL_FLIGHT_MODE,
	/* hovering at a waypoint */
	AUTOPILOT_HOVERING_MODE,
	/* fly to a given position */
	AUTOPILOT_GOTO_MODE,
	/* follow the waypoint list to fly */
	AUTOPILOT_FOLLOW_WAYPOINT_MODE,
	/* hovering at current waypoint for some time before traveling to next one */
	AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE,
	/* follow a trajectory by given position, velocity and acceleration */
	AUTOPILOT_TRAJECTORY_FOLLOWING_MODE,
	/* auto-takeoff mode */
	AUTOPILOT_TAKEOFF_MODE,
	/* auto-landing mode */
	AUTOPILOT_LANDING_MODE,
	/* motor locked mode */
	AUTOPILOT_MOTOR_LOCKED_MODE
} AUTOPILOT_MODE;

enum {
	AUTOPILOT_SET_SUCCEED,
	AUTOPILOT_WAYPOINT_OUT_OF_FENCE,
	AUTOPILOT_WAYPOINT_LIST_FULL,
	AUTOPILOT_WAYPOINT_FOLLOWING_BUSY,
	AUTOPILOT_WAYPOINT_LIST_EMPYT,
	AUTOPILOT_NO_HALTED_WAYPOINT_MISSION,
	AUTOPILOT_TRAJACTORY_LIST_EMPTY,
	AUTOPILOT_TRAJACTORY_LIST_FULL,
	AUTOPILOT_TRAJACTORY_LIST_TOO_LARGE,
	AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY,
	AUTOPILOT_NOT_IN_HOVERING_MODE,
	AUTOPILOT_NOT_IN_WAYPOINT_MODE,
	AUTOPILOT_NOT_IN_TRAJECTORY_MODE,
	AUTOPILOT_ALREADY_TAKEOFF
} AUTOPILOT_RETVAL;

struct trajectory_segment_t {
	float x_poly_coeff[8];
	float y_poly_coeff[8];
	float z_poly_coeff[8];

	float vx_poly_coeff[7];
	float vy_poly_coeff[7];
	float vz_poly_coeff[7];

	float ax_poly_coeff[6];
	float ay_poly_coeff[6];
	float az_poly_coeff[6];

	float yaw_poly_coeff[4];
	float yaw_rate_poly_coeff[3];

	float flight_time;
};

struct waypoint_t {
	float pos[3];        //[m]
	float heading;       //[deg]
	float halt_time_sec; //[s]

	/* compatible with mavlink design */
	int32_t latitude;  //[deg], scaled by 1/1e7
	int32_t longitude; //[deg], scaled by 1/1e7
	float height;      //[m]
	uint16_t command;  //check MAV_CMD enum
};

/* every entities in autopilot_t is defined in enu frame */
typedef struct {
	/* control target (enu frame) */
	struct {
		float pos[3];             //[m]
		float vel[3];             //[m/s]
		float acc_feedforward[3]; //[m/s^2]
		float heading;            //[deg]
	} ctrl_target;

	/* rectangular geo-fence (enu frame) */
	struct {
		float origin[3];
		float lx;
		float ly;
		float height;
	} geo_fence;

	/* autopilot datas and flags */
	float period; //executing period
	int mode;
	bool halt_flag;
	bool loop_mission;
	bool armed;
	bool motor_locked;
	float waypoint_touch_radius;

	/* 1. auto-takeoff and landing */
	bool land_avaliable;
	float landing_speed;
	float landing_accept_height_upper;
	float landing_accept_height_lower;
	float takeoff_speed;
	float takeoff_height;
	float tracking_speed;

	/* 2. goto mission */
	struct {
		float pos[3];  //[m]
		float vel[3];  //[m/s]
		float heading; //[deg]
		bool change_height;
	} goto_target;

	/* 3. waypoint mission */
	struct waypoint_t waypoints[TRAJ_WP_MAX_NUM]; //waypoint list (enu frame)
	int curr_waypoint;       //index of current waypoint to track
	int waypoint_num;        //total waypoint numbers
	int waypoint_wait_timer; //used for delay between waypoints

	/* 4. trajectory mission */
	struct trajectory_segment_t trajectory_segments[TRAJ_WP_MAX_NUM]; //trajectory list
	float trajectory_update_time;
	float traj_start_time;
	int curr_traj;  //trajectory segment index, indicates which trajectory to track
	int traj_num;   //total trajectory number
	bool z_traj;
	bool yaw_traj;
} autopilot_t;

bool check_motor_lock_condition(bool condition);
void assign_vector_3x1_enu_to_ned(float *ned, float *enu);

void autopilot_init(void);

bool autopilot_is_auto_flight_mode(void);
bool autopilot_is_armed(void);

void autopilot_lock_motor(void);
void autopilot_unlock_motor(void);

void autopilot_assign_heading_target(float heading);
void autopilot_assign_pos_target(float x, float y, float z);
void autopilot_assign_pos_target_x(float x);
void autopilot_assign_pos_target_y(float y);
void autopilot_assign_pos_target_z(float z);
void autopilot_assign_vel_target(float vx, float vy, float vz);
void autopilot_assign_vel_target_x(float vx);
void autopilot_assign_vel_target_y(float vy);
void autopilot_assign_vel_target_z(float vz);
void autopilot_assign_zero_vel_target(void);
void autopilot_assign_acc_feedforward(float ax, float ay, float az);
void autopilot_assign_zero_acc_feedforward(void);

void autopilot_set_mode(int new_mode);
void autopilot_set_armed(void);
void autopilot_set_disarmed(void);

int autopilot_get_mode(void);
float autopilot_get_heading_setpoint(void);
void autopilot_get_pos_setpoint(float *pos_set);
float autopilot_get_pos_setpoint_x(void);
float autopilot_get_pos_setpoint_y(void);
float autopilot_get_pos_setpoint_z(void);
void autopilot_get_vel_setpoint(float *vel_set);
float autopilot_get_vel_setpoint_x(void);
float autopilot_get_vel_setpoint_y(void);
float autopilot_get_vel_setpoint_z(void);
void autopilot_get_accel_feedforward(float *accel_ff);

void autopilot_guidance_handler(radio_t *rc, float *curr_pos_enu, float *curr_vel_enu);

void debug_print_waypoint_list(void);
void debug_print_waypoint_status(void);

#endif
