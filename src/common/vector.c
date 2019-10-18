#include <arm_math.h>
#include <vector.h>

void vector3d_normalize(vector3d_f_t *v)
{
	float sq_sum = (v->x)*(v->x) + (v->y)*(v->y) + (v->z)*(v->z);
	float norm;
	arm_sqrt_f32(sq_sum, &norm);
	v->x /= norm;
	v->y /= norm;
	v->z /= norm;
}
