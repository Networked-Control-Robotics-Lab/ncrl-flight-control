#include <arm_math.h>
#include <vector.h>

void vector3d_normalize(float *v)
{
	float sq_sum = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
	float norm;
	arm_sqrt_f32(sq_sum, &norm);
	v[0] /= norm;
	v[1] /= norm;
	v[2] /= norm;
}
