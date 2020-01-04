#include "arm_math.h"
#include "navigation.h"

#define EARTH_RADIUS 6371 //[km]

void latitude_longitude_to_cartesian(float latitude, float longitude, float *x, float *y)
{
	/* haversine formula, the error is ~0.5% (by assuming earth is spherical symmetry),
	 * can be improved by using vincenty formula if necessary */
	*x = EARTH_RADIUS * arm_cos_f32(latitude) * arm_cos_f32(longitude);
	*y = EARTH_RADIUS * arm_cos_f32(latitude) * arm_sin_f32(longitude);
	//*z = EARTH_RADIUS * arm_sin_f32(latitude);
}
