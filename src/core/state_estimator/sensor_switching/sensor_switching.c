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

#define ENABLE_SENSOR_SWITCHING 1

nav_sys_manager_t nav_sys_manager;

void navigation_system_manager_init(void)
{
#if (ENABLE_SENSOR_SWITCHING == 0)
	return;
#endif

#if 1   /* initial start with the gnss/ins */
	nav_sys_manager.curr_nav_sys = NAV_GNSS_INS;
	//change_heading_sensor_src(HEADING_FUSION_USE_COMPASS);
	//change_position_sensor_src(POSITION_FUSION_USE_GPS);
	//change_height_sensor_src(HEIGHT_FUSION_USE_RANGEFINDER);
#else   /* initial start with the vio */
	nav_sys_manager.curr_nav_sys = NAV_LOCAL_VIO;
	change_heading_sensor_src(HEADING_FUSION_USE_VINS_MONO);
	change_position_sensor_src(POSITION_FUSION_USE_VINS_MONO);
	change_height_sensor_src(HEIGHT_FUSION_USE_VINS_MONO);
#endif
}

static void nav_switch_gnss_ins_to_aligned_vio(void)
{
	nav_sys_manager.curr_nav_sys = NAV_GNSS_ALIGNED_VIO;

	/*=====================*
	 * vio frame alignment *
	 *=====================*/

	vio_calc_frame_alignment_transform();
	vio_enable_frame_alignment();

	/*=========================*
	 * update reference signal *
	 *=========================*/

	change_heading_sensor_src(HEADING_FUSION_USE_VINS_MONO);
	change_position_sensor_src(POSITION_FUSION_USE_VINS_MONO);
	change_height_sensor_src(HEIGHT_FUSION_USE_VINS_MONO);

	/*=========================*
	 * update control setpoint *
	 *=========================*/

	/* position state from the gnss aligned vio */
	float p_global_vio[3];
	vio_get_position(p_global_vio);

	/* position state from the gnss ins */
	float p_gnss_ins[3];
	//ins_get_fused_position(p_gnss_ins);
	optitrack_read_pos(p_gnss_ins); //XXX: indoor test only

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

static void nav_switch_non_aligned_vio_to_gnss_ins(void)
{
	nav_sys_manager.curr_nav_sys = NAV_GNSS_INS;

	/*=========================*
	 * update reference signal *
	 *=========================*/
	change_heading_sensor_src(HEADING_FUSION_USE_COMPASS);
	change_position_sensor_src(POSITION_FUSION_USE_GPS);
	change_height_sensor_src(HEIGHT_FUSION_USE_RANGEFINDER);

	/*=========================*
	 * update control setpoint *
	 *=========================*/

	/* position and quaternion state from the gnss ins */
	float p_gnss_ins[3], q_gnss_ins[4];
	//ins_get_fused_position(p_gnss_ins);
	optitrack_read_pos(p_gnss_ins); //XXX: indoor test only
	ins_ahrs_get_attitude_quaternion(q_gnss_ins);

	/* position and quaternion state from the nonaligned vio */
	float p_local_vio[3], q_local_vio[4];
	vio_get_position(p_local_vio);
	vio_get_quaternion(q_local_vio);

	/* calculate yaw angle from the quaternion */
	float yaw_gnss_ins = calc_yaw_from_quat(q_gnss_ins);
	float yaw_nonaligned_vio = calc_yaw_from_quat(q_local_vio);

	/* calculate translation */
	float p_translation[3], yaw_transform;
	p_translation[0] = p_gnss_ins[0] - p_local_vio[0];
	p_translation[1] = p_gnss_ins[1] - p_local_vio[1];
	p_translation[2] = p_gnss_ins[2] - p_local_vio[2];
	yaw_transform = yaw_gnss_ins - yaw_nonaligned_vio;

	float xd[3], yaw_d;
	autopilot_get_pos_setpoint(xd);
	yaw_d = autopilot_get_heading_setpoint();

	/* calculate coordinate transform */
	xd[0] += p_translation[0];
	xd[1] += p_translation[1];
	xd[2] += p_translation[2];
	yaw_d += yaw_transform;

	/* bound the yaw angle in the range of [-180, +180] */
	if(yaw_d > +180.0f) {
		yaw_d -= 360.0f;
	} else if(yaw_d < -180.0f) {
		yaw_d += 360.0f;
	}

	/* apply coordinate transform to the controller */
	autopilot_assign_pos_target(xd[0], xd[1], xd[2]);
	autopilot_assign_heading_target(yaw_d);
}

static void nav_switch_aligned_vio_to_gnss_ins(void)
{
	nav_sys_manager.curr_nav_sys = NAV_GNSS_INS;

	/*=========================*
	 * update reference signal *
	 *=========================*/

	//change_heading_sensor_src(HEADING_FUSION_USE_COMPASS);
	//change_position_sensor_src(POSITION_FUSION_USE_GPS);
	//change_height_sensor_src(HEIGHT_FUSION_USE_RANGEFINDER);
	change_heading_sensor_src(HEADING_FUSION_USE_OPTITRACK);
	change_position_sensor_src(POSITION_FUSION_USE_OPTITRACK);
	change_height_sensor_src(HEIGHT_FUSION_USE_OPTITRACK);

	/*=========================*
	 * update control setpoint *
	 *=========================*/

	/* position state from the gnss aligned vio */
	float p_global_vio[3];
	vio_get_position(p_global_vio);

	/* position state from the gnss ins */
	float p_gnss_ins[3];
	//ins_get_fused_position(p_gnss_ins);
	optitrack_read_pos(p_gnss_ins); //XXX: indoor test only

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

void switch_navigation_system(int new_nav_sys)
{
	nav_sys_manager.new_nav_sys = new_nav_sys;
	nav_sys_manager.require_update = true;
}

void sensor_switching_handler(void)
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

	bool gnss_ins_ready = true;//is_gps_available() && is_compass_available(); //XXX
	bool vio_ready = vins_mono_available();
	bool sensors_all_ready = gnss_ins_ready && vio_ready;

	if(nav_sys_manager.require_update == false && sensors_all_ready == true) {
		return;
	}
	nav_sys_manager.require_update = false;

	int old_select = nav_sys_manager.curr_nav_sys;
	int new_select = nav_sys_manager.new_nav_sys;

	if(old_select == NAV_GNSS_INS && new_select == NAV_GNSS_ALIGNED_VIO) {
		nav_switch_gnss_ins_to_aligned_vio();
	} else if(old_select == NAV_LOCAL_VIO && new_select == NAV_GNSS_INS) {
		nav_switch_non_aligned_vio_to_gnss_ins();
	} else if(old_select == NAV_GNSS_ALIGNED_VIO && new_select == NAV_GNSS_INS) {
		nav_switch_aligned_vio_to_gnss_ins();
	}
}
