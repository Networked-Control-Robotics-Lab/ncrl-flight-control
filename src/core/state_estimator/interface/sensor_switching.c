#include <stdbool.h>
#include "sensor_switching.h"
#include "autopilot.h"
#include "vio_interface.h"
#include "ins.h"
#include "quaternion.h"

nav_sys_manager_t nav_sys_mananger;

void navigation_system_manager_init(int select)
{
	nav_sys_mananger.select = select;
}

static void nav_switch_gnss_ins_to_aligned_vio(void)
{
	/* select new navigation system */
	nav_sys_mananger.select = NAV_GNSS_ALIGNED_VIO;

	/* position state from the gnss aligned vio */
	float p_aligned_vio[3];
	vio_get_quaternion(p_aligned_vio);

	/* position state from the gnss ins */
	float p_gnss_ins[3];
	ins_get_fused_position(p_gnss_ins);

	/* calculate translation */
	float p_transform[3];
	p_transform[0] = p_aligned_vio[0] - p_gnss_ins[0];
	p_transform[1] = p_aligned_vio[1] - p_gnss_ins[1];
	p_transform[2] = p_aligned_vio[2] - p_gnss_ins[2];

	float xd[3];
	autopilot_get_pos_setpoint(xd);

	/* calculate coordinate transform */
	xd[0] += p_transform[0];
	xd[1] += p_transform[1];
	xd[2] += p_transform[2];

	/* apply coordinate transform to the controller */
	autopilot_assign_pos_target(xd[0], xd[1], xd[2]);
}

static void nav_switch_non_aligned_vio_to_gnss_ins(void)
{
	/* select new navigation system */
	nav_sys_mananger.select = NAV_GNSS_INS;

	/* position and quaternion state from the gnss ins */
	float p_gnss_ins[3], q_gnss_ins[4];
	ins_get_fused_position(p_gnss_ins);
	ins_ahrs_get_attitude_quaternion(q_gnss_ins);

	/* position and quaternion state from the nonaligned vio */
	float p_nonaligned_vio[3], q_nonaligned_vio[4];
	vio_get_position(p_nonaligned_vio);
	vio_get_quaternion(q_nonaligned_vio);

	/* calculate yaw angle from the quaternion */
	float yaw_gnss_ins = calc_yaw_from_quat(q_gnss_ins);
	float yaw_nonaligned_vio = calc_yaw_from_quat(q_nonaligned_vio);

	/* calculate translation */
	float p_transform[3], yaw_transform;
	p_transform[0] = p_gnss_ins[0] - p_nonaligned_vio[0];
	p_transform[1] = p_gnss_ins[1] - p_nonaligned_vio[1];
	p_transform[2] = p_gnss_ins[2] - p_nonaligned_vio[2];
	yaw_transform = yaw_gnss_ins - yaw_nonaligned_vio;

	float xd[3], yaw_d;
	autopilot_get_pos_setpoint(xd);
	yaw_d = autopilot_get_heading_setpoint();

	/* calculate coordinate transform */
	xd[0] += p_transform[0];
	xd[1] += p_transform[1];
	xd[2] += p_transform[2];
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
	/* select new navigation system */
	nav_sys_mananger.select = NAV_GNSS_INS;

	/* position state from the gnss aligned vio */
	float p_aligned_vio[3];
	vio_get_position(p_aligned_vio);

	/* position state from the gnss ins */
	float p_gnss_ins[3];
	ins_get_fused_position(p_gnss_ins);

	/* calculate translation */
	float p_transform[3];
	p_transform[0] = p_gnss_ins[0] - p_aligned_vio[0];
	p_transform[1] = p_gnss_ins[1] - p_aligned_vio[1];
	p_transform[2] = p_gnss_ins[2] - p_aligned_vio[2];

	float xd[3];
	autopilot_get_pos_setpoint(xd);

	/* calculate coordinate transform */
	xd[0] += p_transform[0];
	xd[1] += p_transform[1];
	xd[2] += p_transform[2];

	/* apply coordinate transform to the controller */
	autopilot_assign_pos_target(xd[0], xd[1], xd[2]);
}

bool switch_navigation_system(int new_select)
{
	int old_select = nav_sys_mananger.select;

	if(old_select == NAV_GNSS_INS && new_select == NAV_GNSS_ALIGNED_VIO) {
		nav_switch_gnss_ins_to_aligned_vio();
		return true;
	} else if(old_select == NAV_LOCAL_VIO && new_select == NAV_GNSS_INS) {
		nav_switch_non_aligned_vio_to_gnss_ins();
		return true;
	} else if(old_select == NAV_GNSS_ALIGNED_VIO && new_select == NAV_GNSS_INS) {
		nav_switch_aligned_vio_to_gnss_ins();
		return true;
	}

	return false;
}
