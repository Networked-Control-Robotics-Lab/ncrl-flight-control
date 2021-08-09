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

navigation_manager_t navigation_manager;

void navigation_manager_init(void)
{
#if (ENABLE_SENSOR_SWITCHING == 0)
	return;
#endif

#if 1   /* initial start with the gnss/ins */
	navigation_manager.current_system = NAV_GNSS_INS;
	//set_heading_sensor(HEADING_FUSION_USE_COMPASS);
	//set_position_sensor(POSITION_FUSION_USE_GPS);
	//set_height_sensor(HEIGHT_FUSION_USE_RANGEFINDER);
#else   /* initial start with the vio */
	navigation_manager.current_system = NAV_LOCAL_VIO;
	set_heading_sensor(HEADING_FUSION_USE_VINS_MONO);
	set_position_sensor(POSITION_FUSION_USE_VINS_MONO);
	set_height_sensor(HEIGHT_FUSION_USE_VINS_MONO);
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

static void switch_local_vio_to_gnss_ins(void)
{
	navigation_manager.current_system = NAV_GNSS_INS;

	/*=========================*
	 * update reference signal *
	 *=========================*/
	//set_heading_sensor(HEADING_FUSION_USE_COMPASS);
	//set_position_sensor(POSITION_FUSION_USE_GPS);
	//set_height_sensor(HEIGHT_FUSION_USE_RANGEFINDER);
	set_heading_sensor(HEADING_FUSION_USE_OPTITRACK);
	set_position_sensor(POSITION_FUSION_USE_OPTITRACK);
	set_height_sensor(HEIGHT_FUSION_USE_OPTITRACK);

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

static void switch_global_vio_to_gnss_ins(void)
{
	navigation_manager.current_system = NAV_GNSS_INS;

	/*=========================*
	 * update reference signal *
	 *=========================*/

	//set_heading_sensor(HEADING_FUSION_USE_COMPASS);
	//set_position_sensor(POSITION_FUSION_USE_GPS);
	//set_height_sensor(HEIGHT_FUSION_USE_RANGEFINDER);
	set_heading_sensor(HEADING_FUSION_USE_OPTITRACK);
	set_position_sensor(POSITION_FUSION_USE_OPTITRACK);
	set_height_sensor(HEIGHT_FUSION_USE_OPTITRACK);

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

	vio_disable_frame_alignment();
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
	bool gnss_ins_ready = true; //eskf_ins_is_stable(); //XXX
	bool vio_ready = vio_available();

	if(navigation_manager.require_update == false && gnss_ins_ready && vio_ready) {
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
