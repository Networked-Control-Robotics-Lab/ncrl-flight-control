#ifndef __FENCE_H__
#define __FENCE_H__

void autopilot_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height);
bool autopilot_test_point_in_rectangular_fence(float p[3]);

#endif
