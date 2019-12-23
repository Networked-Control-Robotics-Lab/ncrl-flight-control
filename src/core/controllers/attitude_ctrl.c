#include "arm_math.h"
#include "led.h"
#include "ahrs.h"
#include "matrix.h"

/* geometry controller's gains  */
#define krx 0.0f
#define kry 0.0f
#define krz 0.0f
#define kwx 0.0f
#define kwy 0.0f
#define kwz 0.0f

void euler_to_rotation_matrix(euler_t *euler, float *r)
{
	float phi = euler->roll;
	float theta = euler->pitch;
	float psi = euler->yaw;

	r[0*0] = arm_cos_f32(theta)*arm_cos_f32(psi);
	r[0*1] = -arm_cos_f32(phi)*arm_sin_f32(psi) + arm_sin_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi);
	r[0*2] = arm_sin_f32(phi)*arm_sin_f32(psi) + arm_cos_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi);

	r[1*0] = arm_cos_f32(theta)*arm_sin_f32(psi);
	r[1*1] = arm_cos_f32(phi)*arm_cos_f32(psi) + arm_sin_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi);
	r[1*2] = -arm_sin_f32(phi)*arm_cos_f32(psi) + arm_cos_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi);

	r[2*0] = -arm_sin_f32(theta);
	r[2*1] = arm_sin_f32(phi)*arm_cos_f32(theta);
	r[2*2] = arm_cos_f32(phi)*arm_cos_f32(theta);
}

void quat_to_rotation_matrix(float q[4], float *r)
{
	r[0*0] = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
	r[0*1] = 2.0f * (q[1]*q[2] - q[0]*q[3]);
	r[0*2] = 2.0f * (q[0]*q[2] - q[1]*q[3]);

	r[1*0] = 2.0f * (q[1]*q[2] + q[0]*q[3]);
	r[1*1] = 1.0f - 2.0f * (q[1]*q[1] + q[3]*q[3]);
	r[1*2] = 2.0f * (q[2]*q[3] - q[0]*q[1]);

	r[2*0] = 2.0f * (q[1]*q[3] - q[0]*q[2]);
	r[2*1] = 2.0f * (q[0]*q[1] + q[2]*q[3]);
	r[2*2] = 1.0f - 2.0f * (q[1]*q[1]+q[2]*q[2]);
}

void vee_map_3x3(float *mat, float vec[3])
{
	vec[0] = mat[2*1];
	vec[1] = mat[0*2];
	vec[2] = mat[1*0];
}

void hat_map_3x3(float vec[3], float *mat)
{
	mat[0*0] = 0.0f;
	mat[0*1] = -vec[2];
	mat[0*2] = +vec[1];
	mat[1*0] = +vec[2];
	mat[1*1] = 0.0f;
	mat[1*2] = -vec[0];
	mat[2*0] = -vec[1];
	mat[2*1] = +vec[0];
	mat[2*2] = 0.0f;
}

void cross_product_3x1(float vec_a[3], float vec_b[3], float vec_result[3])
{
#if 0   //matrix approach
	float mat_a_arr[3][3];
	arm_matrix_instance_f32 mat_a, mat_b, mat_c;
	arm_mat_init_f32(mat_a, 3, 3, mat_a_arr);
	arm_mat_init_f32(mat_b, 3, 1, vec_b);
	arm_mat_init_f32(mat_c, 3, 1, vec_result);
	hat_map_3x3(vec_a, _mat_(mat_a));
	arm_mat_mult_f32(&mat_a, &mat_b, &mat_result);
#endif
	vec_result[0] = vec_a[1]*vec_b[2] - vec_a[2]*vec_b[1];
	vec_result[1] = vec_a[2]*vec_b[0] - vec_a[0]*vec_b[2];
	vec_result[2] = vec_a[0]*vec_b[1] - vec_a[1]*vec_b[0];
}

void geometry_ctrl(euler_t *rc, float attitude_q[4], float *output_forces, float *output_moments)
{
	MAT_ALLOC_INIT(R, 3, 3);
	MAT_ALLOC_INIT(Rd, 3, 3);
	MAT_ALLOC_INIT(Rt, 3, 3);
	MAT_ALLOC_INIT(Rtd, 3, 3);
	MAT_ALLOC_INIT(RtdR, 3, 3);
	MAT_ALLOC_INIT(RtRd, 3, 3);
	MAT_ALLOC_INIT(RtRdWd, 3, 3);
	MAT_ALLOC_INIT(W, 3, 1);
	MAT_ALLOC_INIT(Wd, 3, 1);
	MAT_ALLOC_INIT(eR_mat_double, 3, 3);
	MAT_ALLOC_INIT(eR_mat, 3, 3);
	MAT_ALLOC_INIT(eR, 3, 1);
	MAT_ALLOC_INIT(eW, 3, 1);

	/* convert attitude (quaternion) to rotation matrix */
	quat_to_rotation_matrix(&attitude_q[0], _mat_(R));

	/* convert radio command (euler angle) to rotation matrix */
	euler_to_rotation_matrix(rc, _mat_(Rd));

	/* calculate attitude error eR */
	MAT_MULT(&Rtd, &R, &RtdR);
	MAT_MULT(&Rt, &Rd, &RtRd); //XXX: duplicated term
	MAT_SUB(&RtdR, &RtRd, &eR_mat_double);
	MAT_SCALE(&eR_mat_double, 0.5f, &eR_mat);
	vee_map_3x3(_mat_(eR_mat), _mat_(eR));

	/* calculate attitude rate error eW */
	MAT_MULT(&Rt, &Rd, &RtRd);
	MAT_MULT(&RtRd, &Wd, &RtRdWd); //XXX: duplicated term
	MAT_SUB(&W, &RtRdWd, &eW);

	output_moments[0] = -krx*_mat_(eR)[0] -kwx*_mat_(eW)[0];
	output_moments[1] = -kry*_mat_(eR)[1] -kwy*_mat_(eW)[1];
	output_moments[2] = -krz*_mat_(eR)[2] -kwz*_mat_(eW)[2];
}

void thrust_allocate_quadrotor(float *forces, float *moments, float *motors)
{
}
