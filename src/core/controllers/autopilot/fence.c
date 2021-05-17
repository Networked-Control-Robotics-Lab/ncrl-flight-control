#include <stdint.h>
#include "autopilot.h"

extern autopilot_t autopilot;

void autopilot_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height)
{
	autopilot.geo_fence.lx = lx; //[m]
	autopilot.geo_fence.ly = ly; //[m]
	autopilot.geo_fence.height = height; //[m]
}

bool autopilot_test_point_in_rectangular_fence(float p[3])
{
	if((p[0] <= (+autopilot.geo_fence.lx + autopilot.geo_fence.origin[0])) &&
	    (p[0] >= (-autopilot.geo_fence.lx + autopilot.geo_fence.origin[0])) &&
	    (p[1] <= (+autopilot.geo_fence.ly + autopilot.geo_fence.origin[1])) &&
	    (p[1] >= (-autopilot.geo_fence.ly + autopilot.geo_fence.origin[1])) &&
	    (p[2] >= 0.0f) && (p[2] <= autopilot.geo_fence.height)) {
		return true;
	} else {
		return false;
	}
}
