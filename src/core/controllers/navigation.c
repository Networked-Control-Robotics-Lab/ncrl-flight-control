#include "arm_math.h"
#include "navigation.h"

#define EARTH_RADIUS 6371 //[km]

nav_t *nav;

void nav_init(nav_t *_nav)
{
	nav = _nav;
}

void latitude_longitude_to_cartesian(float latitude, float longitude, float *x, float *y)
{
	/* haversine formula, the error is ~0.5% (by assuming earth is spherical symmetry),
	 * can be improved by using vincenty formula if necessary */
	*x = EARTH_RADIUS * arm_cos_f32(latitude) * arm_cos_f32(longitude);
	*y = EARTH_RADIUS * arm_cos_f32(latitude) * arm_sin_f32(longitude);
	//*z = EARTH_RADIUS * arm_sin_f32(latitude);
}


int nav_add_new_waypoint(float *pos, float heading)
{
	return 0;
}

int nav_goto_waypoint_now(float *pos, float heading)
{
	nav->wp_now.pos[0] = pos[0];
	nav->wp_now.pos[1] = pos[1];
	nav->wp_now.pos[2] = pos[2];
	nav->wp_now.heading = heading;
	return 0;
}
