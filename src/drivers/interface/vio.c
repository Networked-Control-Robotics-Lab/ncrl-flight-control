#include <stdbool.h>
#include <stdint.h>
#include "se3_math.h"
#include "vio.h"
#include "vins_mono.h"
#include "quaternion.h"
#include "gps.h"
#include "compass.h"
#include "ins.h"

vio_t vio;

bool vio_available(void)
{
	return vins_mono_available();
}

void vio_enable_frame_alignment(void)
{
	vio.frame_align_on = true;
}

void vio_disable_frame_alignment(void)
{
	vio.frame_align_on = false;
}

void vio_calc_frame_alignment_transform(void)
{
	float q_gnss[4];
	ins_ahrs_get_attitude_quaternion(q_gnss);

	float q_local_vio[4];
	vins_mono_read_quaternion(q_local_vio);

	/* calculate inv(q_local_vio) */
	float q_local_vio_inv[4];
	quaternion_conj(q_local_vio, q_local_vio_inv);

	/* calculate vio.q_align = inv(q_local_vio) * q_gnss */
	quaternion_mult(q_local_vio_inv, q_gnss, vio.q_align);

	vio.frame_align_on = true;
}

void vio_get_quaternion(float *q)
{
	if(vio.frame_align_on == true) {
		/* read local vio quaternion */
		float q_local_vio[4];
		vins_mono_read_quaternion(q_local_vio);

		/* calculate q_gnss = q_local_vio * vio.q_align */
		quaternion_mult(q_local_vio, vio.q_align, q);
	} else {
		vins_mono_read_quaternion(q);
	}
}

void vio_get_position(float *pos)
{
	if(vio.frame_align_on == true) {
		/* read local vio position */
		float p_local_vio[3];
		vins_mono_read_pos(p_local_vio);

		/* calculate Rt(a_align) */
		float R[3*3], Rt[3*3];
		quat_to_rotation_matrix(vio.q_align, R, Rt);

		/* calculate p_gnss = Rt(q_align) * p_local_vio */
		calc_matrix_multiply_vector_3d(pos, p_local_vio, Rt);
	} else {
		vins_mono_read_pos(pos);
	}
}

float vio_get_position_x(void)
{
	return vins_mono_read_pos_x();
}

float vio_get_position_y(void)
{
	return vins_mono_read_pos_y();
}

float vio_get_position_z(void)
{
	return vins_mono_read_pos_z();
}

void vio_get_velocity(float *vel)
{
	vins_mono_read_vel(vel);
}

float vio_get_velocity_x(void)
{
	return vins_mono_read_vel_x();
}

float vio_get_velocity_y(void)
{
	return vins_mono_read_vel_y();
}

float vio_get_velocity_z(void)
{
	return vins_mono_read_vel_z();
}
