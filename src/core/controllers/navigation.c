#include <stdbool.h>
#include "arm_math.h"
#include "navigation.h"

#define EARTH_RADIUS 6371 //[km]

nav_t *nav_ptr;

void latitude_longitude_to_cartesian(float latitude, float longitude, float *x, float *y)
{
	/* haversine formula, the error is ~0.5% (by assuming earth is spherical symmetry),
	 * can be improved by using vincenty formula if necessary */
	*x = EARTH_RADIUS * arm_cos_f32(latitude) * arm_cos_f32(longitude);
	*y = EARTH_RADIUS * arm_cos_f32(latitude) * arm_sin_f32(longitude);
	//*z = EARTH_RADIUS * arm_sin_f32(latitude);
}

void nav_init(nav_t *_nav)
{
	nav_ptr = _nav;
}

void nav_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height)
{
	nav_ptr->geo_fence.lx = lx * 100.0f; //convet unit from [m] to [cm]
	nav_ptr->geo_fence.ly = ly * 100.0f;
	nav_ptr->geo_fence.height = height * 100.0f;
}

static bool nav_test_point_in_rectangular_fence(float p[3])
{
	if((p[0] < (+nav_ptr->geo_fence.lx + nav_ptr->geo_fence.origin[0])) &&
	    (p[0] > (-nav_ptr->geo_fence.lx + nav_ptr->geo_fence.origin[0])) &&
	    (p[1] < (+nav_ptr->geo_fence.ly + nav_ptr->geo_fence.origin[1])) &&
	    (p[1] > (-nav_ptr->geo_fence.ly + nav_ptr->geo_fence.origin[1])) &&
	    (p[2] > 0.0f) && (p[2] < nav_ptr->geo_fence.height)) {
		return true;
	} else {
		return false;
	}
}

int nav_add_new_waypoint(float pos[3], float heading)
{
	return 0;
}

int nav_goto_waypoint_now(float pos[3], float heading)
{
	bool in_fence = nav_test_point_in_rectangular_fence(pos);

	if(in_fence == true) {
		(nav_ptr->wp_now).pos[0] = pos[0];
		(nav_ptr->wp_now).pos[1] = pos[1];
		(nav_ptr->wp_now).pos[2] = pos[2];
		(nav_ptr->wp_now).heading = heading;
		return WP_SET_SUCCEED;
	} else {
		return WP_SET_OUT_OF_FENCE;
	}
}
