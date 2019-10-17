#include <string.h>

#include "vector.h"

void lpf(vector3d_f_t *new, vector3d_f_t *last, vector3d_f_t *filtered, float a)
{
	filtered->x = (new->x * a) + (last->x * (1 - a));
	filtered->y = (new->y * a) + (last->y * (1 - a));
	filtered->z = (new->z * a) + (last->z * (1 - a));
	last->x = filtered->x;
	last->y = filtered->y;
	last->z = filtered->z;
}
