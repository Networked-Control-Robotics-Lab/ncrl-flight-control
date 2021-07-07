#include <stdbool.h>
#include <stdint.h>
#include "se3_math.h"
#include "vio_interface.h"
#include "vins_mono.h"
#include "quaternion.h"
#include "gps.h"
#include "compass.h"
#include "ins.h"

vio_manager_t vio_manager;

void vio_frame_alignment_init(void);

void vio_update_handler(void)
{
	/* check vio sensor state */
	if(vins_mono_available() == true) {
		/* align vio to the earth frame if possible */
		if(vio_manager.gnss_align_on == false) {
			vio_frame_alignment_init();
		}
	} else {
		vio_manager.gnss_align_on = false;
	}
}

void vio_frame_alignment_init(void)
{
	bool gps_ready = is_gps_available();
	bool compass_ready = is_compass_available();

	if(gps_ready == false || compass_ready == false) {
		return;
	}

	float q_gnss[4];
	ins_ahrs_get_attitude_quaternion(q_gnss);

	float q_local_vio[4];
	vins_mono_read_quaternion(q_local_vio);

	/* calculate inv(q_local_vio) */
	float q_local_vio_inv[4];
	quaternion_conj(q_local_vio, q_local_vio_inv);

	/* calculate vio_manager.q_align = inv(q_local_vio) * q_gnss */
	quaternion_mult(q_local_vio_inv, q_gnss, vio_manager.q_align);

	vio_manager.gnss_align_on = true;
}

void vio_get_quaternion(float *q)
{
	if(vio_manager.gnss_align_on == true) {
		/* read local vio quaternion */
		float q_local_vio[4];
		vins_mono_read_quaternion(q_local_vio);

		/* calculate q_gnss = q_local_vio * vio_manager.q_align */
		quaternion_mult(q_local_vio, vio_manager.q_align, q);
	} else {
		vins_mono_read_quaternion(q);
	}
}

void vio_get_position(float *p)
{
	if(vio_manager.gnss_align_on == true) {
		/* read local vio position */
		float p_local_vio[3];
		vins_mono_read_pos(p_local_vio);

		/* calculate Rt(a_align) */
		float R[3*3], Rt[3*3];
		quat_to_rotation_matrix(vio_manager.q_align, R, Rt);

		/* calculate p_gnss = Rt(q_align) * p_local_vio */
		calc_matrix_multiply_vector_3d(p, p_local_vio, Rt);
	} else {
		vins_mono_read_pos(p);
	}
}
