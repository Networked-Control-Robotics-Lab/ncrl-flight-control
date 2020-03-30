#include "arm_math.h"
#include "se3_math.h"

void quaternion_copy(float *q_dest, float *q_src)
{
	q_dest[0] = q_src[0];
	q_dest[1] = q_src[1];
	q_dest[2] = q_src[2];
	q_dest[3] = q_src[3];
}

void quaternion_mult(float *q1, float *q2, float *q_mult)
{
	q_mult[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
	q_mult[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
	q_mult[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
	q_mult[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void quaternion_conj(float *q, float *q_conj)
{
	q_conj[0] = q[0];
	q_conj[1] = -q[1];
	q_conj[2] = -q[2];
	q_conj[3] = -q[3];
}

void quat_normalize(float *q)
{
	float sq_sum = (q[0])*(q[0]) + (q[1])*(q[1]) + (q[2])*(q[2]) + (q[3])*(q[3]);
	float norm;
	arm_sqrt_f32(sq_sum, &norm);
	q[0] /= norm;
	q[1] /= norm;
	q[2] /= norm;
	q[3] /= norm;
}

//in: quaterion, out: euler angle [radian]
void quat_to_euler(float *q, euler_t *euler)
{
	euler->roll = atan2(2.0*(q[0]*q[1] + q[2]*q[3]), 1.0-2.0*(q[1]*q[1] + q[2]*q[2]));
	euler->pitch = asin(2.0*(q[0]*q[2] - q[3]*q[1]));
	euler->yaw = atan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0-2.0*(q[2]*q[2] + q[3]*q[3]));
}

//in: euler angle [radian], out: quaternion
void euler_to_quat(euler_t *euler, float *q)
{
	float phi = euler->roll * 0.5f;
	float theta = euler->pitch * 0.5f;
	float psi = euler->yaw * 0.5f;

	q[0] = arm_cos_f32(phi) * arm_cos_f32(theta) * arm_cos_f32(psi) +
	       arm_sin_f32(phi) * arm_sin_f32(theta) * arm_sin_f32(psi);
	q[1] = arm_sin_f32(phi) * arm_cos_f32(theta) * arm_cos_f32(psi) -
	       arm_cos_f32(phi) * arm_sin_f32(theta) * arm_sin_f32(psi);
	q[2] = arm_cos_f32(phi) * arm_sin_f32(theta) * arm_cos_f32(psi) +
	       arm_sin_f32(phi) * arm_cos_f32(theta) * arm_sin_f32(psi);
	q[3] = arm_cos_f32(phi) * arm_cos_f32(theta) * arm_sin_f32(psi) -
	       arm_sin_f32(phi) * arm_sin_f32(theta) * arm_cos_f32(psi);
}
