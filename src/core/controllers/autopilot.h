#ifndef __AUTOPILOT_H__
#define __AUTOPILOT_H__

#include <stdbool.h>

#define WAYPOINT_NUM_MAX 50

#define TRAJECTORY_WP_UPDATE_FREQ 100
#define TRAJECTORY_WP_UPDATE_TIME (1.0f / TRAJECTORY_WP_UPDATE_FREQ)

enum {
	/* user manual flight mode */
	AUTOPILOT_MANUAL_FLIGHT_MODE,
	/* hovering at a waypoint */
	AUTOPILOT_HOVERING_MODE,
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
	AUTOPILOT_WP_OUT_OF_FENCE,
	AUTOPILOT_WP_LIST_FULL,
	AUTOPILOT_MISSION_EXECUTING,
	AUTOPILOT_NO_EXECUTING_MISSION,
	AUTOPILOT_WP_LIST_EMPYT,
	AUTOPILOT_TRAJ_LIST_EMPTY,
	AUTOPILOT_POSITION_NOT_FIXED,
	AUTOPILOT_UAV_ALREADY_TAKEOFF,
	AUTOPILOT_NOT_IN_HOVERING_MODE,
	AUTOPILOT_NOT_IN_TRAJECTORY_MODE
} AUTOPILOT_SET_RETVAL;

struct trajectory_segment_t
{
	float x_poly_coeff[8];
	float y_poly_coeff[8];
	float z_poly_coeff[8];

	float vx_poly_coeff[7];
	float vy_poly_coeff[7];
	float vz_poly_coeff[7];

	float ax_poly_coeff[7];
	float ay_poly_coeff[7];
	float az_poly_coeff[7];

	float yaw_poly_coeff[4];
	float yaw_rate_poly_coeff[3];

	float flight_time;
};

struct waypoint_t {
	float pos[3];        //[m]
	float heading;       //[deg]
	float halt_time_sec; //[s]
	float touch_radius;  //[m]
};

/* every entities in autopilot_t is defined in enu frame */
typedef struct {
	struct {
		float pos[3];
		float vel[3];
	} uav_state; /* current position and velocity of the uav */

	struct {
		float pos[3];             //[m]
		float vel[3];             //[m/s]
		float acc_feedforward[3]; //[m/s^2]
		float heading;            //[deg]
	} wp_now; /* autopilot provides these to controller as desired setpoint */

	struct {
		float origin[3];
		float lx;
		float ly;
		float height;
	} geo_fence; /* rectangular geo-fence in enu frame */

	float landing_speed;
	float landing_accept_height;
	float takeoff_speed;
	float takeoff_height;
	
	int mode;
	bool halt_flag;
	bool loop_mission;
	bool armed;
	bool motor_locked;

	/* for waypoint following (representing setpoint with waypoints) */
	struct waypoint_t wp_list[WAYPOINT_NUM_MAX]; //enu frame
	int curr_wp; //waypoint index, indicates which waypoint to track
	int wp_num;  //total waypoint number

	/* for trajectory following (representing setpoint with 7th ordered polynomials) */
	struct trajectory_segment_t trajectory_segments[WAYPOINT_NUM_MAX];
	float trajectory_update_time;
	int curr_traj;  //trajectory segment index, indicates which trajectory to track
	int traj_num;   //total trajectory number
	float traj_start_time;
	bool z_traj;
	bool yaw_traj;
} autopilot_t;

void autopilot_init(autopilot_t *_autopilot);
void autopilot_update_uav_state(float pos_enu[3], float vel_enu[3]);

bool autopilot_is_manual_flight_mode(void);
bool autopilot_is_motor_locked_mode(void);
bool autopilot_is_auto_flight_mode(void);

void autopilot_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height);
void autopilot_set_mode(int new_mode);
void autopilot_set_armed(void);
void autopilot_set_disarmed(void);
void autopilot_mission_reset(void);

bool autopilot_is_armed(void);
int autopilot_get_mode(void);
void autopilot_get_pos_setpoint(float *pos_set);
void autopilot_get_vel_setpoint(float *vel_set);
void autopilot_get_accel_feedforward(float *accel_ff);

int autopilot_set_x_trajectory(int index, float *x_traj_coeff, float fligt_time);
int autopilot_set_y_trajectory(int index, float *y_traj_coeff, float fligt_time);
int autopilot_set_z_trajectory(int index, float *z_traj_coeff, float fligt_time);
int autopilot_set_yaw_trajectory(int index, float *yaw_traj_coeff, float fligt_time);
int autopilot_config_trajectory_following(int traj_num, bool z_traj, bool yaw_traj);
int autopilot_trajectory_following_start(bool loop_trajectory);
int autopilot_trajectory_following_stop(void);

int autopilot_add_new_waypoint(float pos[3], float heading, float halt_time_sec, float radius);
int autopilot_clear_waypoint_list(void);
int autopilot_goto_waypoint_now(float pos[3], bool change_height);
int autopilot_halt_waypoint_mission(void);
int autopilot_resume_waypoint_mission(void);
int autopilot_waypoint_mission_start(bool loop_mission);
int autopilot_trigger_auto_landing(void);
int autopilot_trigger_auto_takeoff(void);

bool autopilot_motor_ls_lock(void);
void autopilot_lock_motor(void);
void autopilot_unlock_motor(void);

void autopilot_guidance_handler(void);

void assign_vector_3x1_eun_to_ned(float *ned, float *enu);

void debug_print_waypoint_list(void);
void debug_print_waypoint_status(void);

#endif
