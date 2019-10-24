#include <string.h>

#include "vector.h"

void lpf(vector3d_f_t *new, vector3d_f_t *filtered, float a)
{
	/* filtered = (new * a) + (old * (1.0 - a))  */
	filtered->x = (new->x * a) + (filtered->x * (1.0f - a));
	filtered->y = (new->y * a) + (filtered->y * (1.0f - a));
	filtered->z = (new->z * a) + (filtered->z * (1.0f - a));
}
