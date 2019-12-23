#include "arm_math.h"
#include "ahrs.h"
#include "matrix.h"

void euler_to_rotation_matrix(euler_t *euler, float r[3][3])
{
	float phi = euler->roll;
	float theta = euler->pitch;
	float psi = euler->yaw;

	r[0][0] = arm_cos_f32(theta)*arm_cos_f32(psi);
	r[0][1] = -arm_cos_f32(phi)*arm_sin_f32(psi) + arm_sin_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi);
	r[0][2] = arm_sin_f32(phi)*arm_sin_f32(psi) + arm_cos_f32(phi)*arm_sin_f32(theta)*arm_cos_f32(psi);

	r[1][0] = arm_cos_f32(theta)*arm_sin_f32(psi);
	r[1][1] = arm_cos_f32(phi)*arm_cos_f32(psi) + arm_sin_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi);
	r[1][2] = -arm_sin_f32(phi)*arm_cos_f32(psi) + arm_cos_f32(phi)*arm_sin_f32(theta)*arm_sin_f32(psi);

	r[2][0] = -arm_sin_f32(theta);
	r[2][1] = arm_sin_f32(phi)*arm_cos_f32(theta);
	r[2][2] = arm_cos_f32(phi)*arm_cos_f32(theta);
}

void quat_to_rotation_matrix(float q[4], float r[3][3])
{
	r[0][0] = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
	r[0][1] = 2.0f * (q[1]*q[2] - q[0]*q[3]);
	r[0][2] = 2.0f * (q[0]*q[2] - q[1]*q[3]);

	r[1][0] = 2.0f * (q[1]*q[2] + q[0]*q[3]);
	r[1][1] = 1.0f - 2.0f * (q[1]*q[1] + q[3]*q[3]);
	r[1][2] = 2.0f * (q[2]*q[3] - q[0]*q[1]);

	r[2][0] = 2.0f * (q[1]*q[3] - q[0]*q[2]);
	r[2][1] = 2.0f * (q[0]*q[1] + q[2]*q[3]);
	r[2][2] = 1.0f - 2.0f * (q[1]*q[1]+q[2]*q[2]);
}

void vee_map_3x3(float mat[3][3], float vec[3])
{
	vec[0] = mat[2][1];
	vec[1] = mat[0][2];
	vec[2] = mat[1][0];
}

void hat_map_3x3(float vec[3], float mat[3][3])
{
	mat[0][0] = 0.0f;
	mat[0][1] = -vec[2];
	mat[0][2] = +vec[1];
	mat[1][0] = +vec[2];
	mat[1][1] = 0.0f;
	mat[1][2] = -vec[0];
	mat[2][0] = -vec[1];
	mat[2][1] = +vec[0];
	mat[2][2] = 0.0f;
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

void geometry_ctrl(euler_t rc_cmd, float attitude_q[4])
{
}
