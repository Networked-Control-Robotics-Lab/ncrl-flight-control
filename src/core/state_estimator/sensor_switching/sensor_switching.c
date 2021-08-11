#include <stdbool.h>
#include "sensor_switching.h"
#include "autopilot.h"
#include "vio.h"
#include "ins.h"
#include "quaternion.h"
#include "vio.h"
#include "gps.h"
#include "vins_mono.h"
#include "compass.h"
#include "proj_config.h"
#include "system_state.h"
#include "optitrack.h"
#include "ins_eskf.h"

#define ENABLE_SENSOR_SWITCHING 0

navigation_manager_t navigation_manager;

void sensor_combination_gnss_ins(void)
{
	navigation_manager.current_system = NAV_GNSS_INS;

#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK)
	set_heading_sensor(HEADING_FUSION_USE_OPTITRACK);
	set_position_sensor(POSITION_FUSION_USE_OPTITRACK);
	set_height_sensor(HEIGHT_FUSION_USE_OPTITRACK);
#else
	set_heading_sensor(HEADING_FUSION_USE_COMPASS);
	set_position_sensor(POSITION_FUSION_USE_GPS);
	set_height_sensor(HEIGHT_FUSION_USE_RANGEFINDER);
#endif
}

void sensors_combination_vio(void)
{
	navigation_manager.current_system = NAV_LOCAL_VIO;
	set_heading_sensor(HEADING_FUSION_USE_VINS_MONO);
	set_position_sensor(POSITION_FUSION_USE_VINS_MONO);
	set_height_sensor(HEIGHT_FUSION_USE_VINS_MONO);
}

void navigation_manager_init(void)
{
#if (ENABLE_SENSOR_SWITCHING == 0)
	return;
#endif

#if 1
	/* initial start with the gnss/ins */
	sensor_combination_gnss_ins();
#else
	/* initial start with the vio */
	sensors_combination_vio();
#endif
}

static void switch_gnss_ins_to_global_vio(void)
{
	navigation_manager.current_system = NAV_GLOBAL_VIO;

	/*=====================*
	 * vio frame alignment *
	 *=====================*/

	vio_calculate_frame_alignment();
	vio_enable_frame_alignment();

	/*=========================*
	 * update reference signal *
	 *=========================*/

	set_heading_sensor(HEADING_FUSION_USE_VINS_MONO);
	set_position_sensor(POSITION_FUSION_USE_VINS_MONO);
	set_height_sensor(HEIGHT_FUSION_USE_VINS_MONO);

	/*=========================*
	 * update control setpoint *
	 *=========================*/

	/* position state from the gnss aligned vio */
	float p_global_vio[3];
	vio_get_position_enu(p_global_vio);

	/* position state from the gnss ins */
	float p_gnss_ins[3];
#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK)
	optitrack_get_position_enu(p_gnss_ins);
#else
	ins_get_fused_position_enu(p_gnss_ins);
#endif

	/* calculate translation */
	float p_translation[3];
	p_translation[0] = p_global_vio[0] - p_gnss_ins[0];
	p_translation[1] = p_global_vio[1] - p_gnss_ins[1];
	p_translation[2] = p_global_vio[2] - p_gnss_ins[2];

	float xd[3];
	autopilot_get_pos_setpoint(xd);

	/* calculate coordinate transform */
	xd[0] += p_translation[0];
	xd[1] += p_translation[1];
	xd[2] += p_translation[2];

	/* apply coordinate transform to the controller */
	autopilot_assign_pos_target(xd[0], xd[1], xd[2]);
}

static void switch_local_vio_to_gnss_ins(void)
{
	navigation_manager.current_system = NAV_GNSS_INS;

	/* calculate vio frame alignment parameters */
	vio_calculate_frame_alignment();

	/* get frame alignment parameters */
	float q_l2g[4], R_l2g[3*3], p_l2g[3];
	vio_get_frame_alignment_quaternion(q_l2g);
	vio_get_frame_alignment_rotation_matrix(R_l2g);
	vio_get_frame_alignment_translation(p_l2g);

	/*=========================*
	 * update reference signal *
	 *=========================*/

#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK)
	set_heading_sensor(HEADING_FUSION_USE_OPTITRACK);
	set_position_sensor(POSITION_FUSION_USE_OPTITRACK);
	set_height_sensor(HEIGHT_FUSION_USE_OPTITRACK);
#else
	set_heading_sensor(HEADING_FUSION_USE_COMPASS);
	set_position_sensor(POSITION_FUSION_USE_GPS);
	set_height_sensor(HEIGHT_FUSION_USE_RANGEFINDER);
#endif

	/*=========================*
	 * update control setpoint *
	 *=========================*/

	/* calculate yaw angle from the quaternion */
	float yaw_transform = rad_to_deg(calc_yaw_from_quat(q_l2g));

	/* get desired values from autopilot */
	float xd_enu[3], xd[3], vd_enu[3], vd[3], yaw_d;
	autopilot_get_pos_setpoint(xd_enu);
	autopilot_get_vel_setpoint(vd_enu);
	yaw_d = autopilot_get_heading_setpoint();

	/* XXX: convert xd and vd from enu to ned */
	xd[0] =  xd_enu[1];
	xd[1] =  xd_enu[0];
	xd[2] = -xd_enu[2];
	vd[0] =  vd_enu[1];
	vd[1] =  vd_enu[0];
	vd[2] = -vd_enu[2];

	/* calculate new xd */
	float new_xd_ned[3], new_xd[3];
	calc_matrix_multiply_vector_3d(new_xd_ned, xd, R_l2g);
	new_xd_ned[0] += p_l2g[0];
	new_xd_ned[1] += p_l2g[1];
	new_xd_ned[2] += p_l2g[2];
	/* XXX: convert ned back to enu */
	new_xd[0] =  new_xd_ned[1];
	new_xd[1] =  new_xd_ned[0];
	new_xd[2] = -new_xd_ned[2];

	/* calculate new vd */
	float new_vd_ned[3], new_vd[3];
	calc_matrix_multiply_vector_3d(new_vd_ned, vd, R_l2g);
	/* XXX: convert ned back to enu */
	new_vd[0] =  new_vd_ned[1];
	new_vd[1] =  new_vd_ned[0];
	new_vd[2] = -new_vd_ned[2];

	/* calculate new yaw_d */
	yaw_d += yaw_transform;

	/* bound the yaw angle in the range of [-180, +180] */
	if(yaw_d > +180.0f) {
		yaw_d -= 360.0f;
	} else if(yaw_d <= -180.0f) {
		yaw_d += 360.0f;
	}

	/* apply coordinate transform to the controller */
	autopilot_assign_pos_target(new_xd[0], new_xd[1], new_xd[2]);
	autopilot_assign_vel_target(new_vd[0], new_vd[1], new_vd[2]);
	autopilot_assign_heading_target(yaw_d);

	/* activate vio frame alignment */
	vio_enable_frame_alignment();
}

static void switch_global_vio_to_gnss_ins(void)
{
	navigation_manager.current_system = NAV_GNSS_INS;

	/*=========================*
	 * update reference signal *
	 *=========================*/

#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK)
	set_heading_sensor(HEADING_FUSION_USE_OPTITRACK);
	set_position_sensor(POSITION_FUSION_USE_OPTITRACK);
	set_height_sensor(HEIGHT_FUSION_USE_OPTITRACK);
#else
	set_heading_sensor(HEADING_FUSION_USE_COMPASS);
	set_position_sensor(POSITION_FUSION_USE_GPS);
	set_height_sensor(HEIGHT_FUSION_USE_RANGEFINDER);
#endif

	/*=========================*
	 * update control setpoint *
	 *=========================*/

	/* position state from the gnss aligned vio */
	float p_global_vio[3];
	vio_get_position_enu(p_global_vio);

	/* position state from the gnss ins */
	float p_gnss_ins[3];
#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK)
	optitrack_get_position_enu(p_gnss_ins);
#else
	ins_get_fused_position_enu(p_gnss_ins);
#endif

	/* calculate translation */
	float p_translation[3];
	p_translation[0] = p_gnss_ins[0] - p_global_vio[0];
	p_translation[1] = p_gnss_ins[1] - p_global_vio[1];
	p_translation[2] = p_gnss_ins[2] - p_global_vio[2];

	float xd[3];
	autopilot_get_pos_setpoint(xd);

	/* calculate coordinate transform */
	xd[0] += p_translation[0];
	xd[1] += p_translation[1];
	xd[2] += p_translation[2];

	/* apply coordinate transform to the controller */
	autopilot_assign_pos_target(xd[0], xd[1], xd[2]);
}

void switch_navigation_system(int new_system)
{
	navigation_manager.new_system = new_system;
	navigation_manager.require_update = true;
}

void navigation_manager_handler(void)
{
#if (ENABLE_SENSOR_SWITCHING == 0)
	return;
#endif

	/* XXX: currently we can not handle the navigation system failure
	   properly. If the case is happened, we turn off the auto-flight
	   mode and let the user control the uav manually */

	/* TODO: vio failure recovery procedure:
	 * 1. save last position and quaternion as extrinsic parameters
	 * 2. restart the vio, the origin is set at the new position in space
	 * 3. coordinate transform with the extrinsic parameters */

	/* TODO: gnss/ins failure recovery */

	/* check if gnss/ins and vio are both vaild */
#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK)
	bool gnss_ins_ready = optitrack_available(); //switch between vio and optitrack
#else
	bool gnss_ins_ready = ins_eskf_is_stable();  //switch between vio and gnss/ins
#endif
	bool vio_ready = vio_available();

	if(navigation_manager.require_update == false || !gnss_ins_ready || !vio_ready) {
		return;
	}
	navigation_manager.require_update = false;

	/* switch navigation system */
	int current_system = navigation_manager.current_system;
	int new_system = navigation_manager.new_system;

	if(current_system == NAV_GNSS_INS && new_system == NAV_GLOBAL_VIO) {
		switch_gnss_ins_to_global_vio();
	} else if(current_system == NAV_LOCAL_VIO && new_system == NAV_GNSS_INS) {
		switch_local_vio_to_gnss_ins();
	} else if(current_system == NAV_GLOBAL_VIO && new_system == NAV_GNSS_INS) {
		switch_global_vio_to_gnss_ins();
	}
}
