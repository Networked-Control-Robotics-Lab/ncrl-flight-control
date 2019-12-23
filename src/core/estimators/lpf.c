#include <string.h>

#include "vector.h"

void lpf(float new, float *filtered, float a)
{
	*filtered = (new * a) + (*filtered * (1.0f - a));
}
