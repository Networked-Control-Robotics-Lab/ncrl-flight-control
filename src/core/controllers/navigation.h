#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

#define WAYPOINT_NUM_MAX 50

enum {
	WP_SET_SUCCEED,
	WP_SET_OUT_OF_FENCE,
} WP_SET_RETVAL;

struct waypoint_t {
	float pos[3];
	float heading;
	float halt_time;
};

typedef struct {
	struct waypoint_t wp_now; //enu frame!

	//earth-north-up
	struct rect_fence {
		float origin[3];
		float lx;
		float ly;
		float height;
	} geo_fence; /* rectangular geo-fence in enu frame */

	struct waypoint_t wp_list[WAYPOINT_NUM_MAX]; //enu frame!
	int curr_wp_num;
} nav_t;

void nav_init(nav_t *_nav);
void nav_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height);
int nav_add_new_waypoint(float pos[3], float heading);
int nav_goto_waypoint_now(float pos[3], float heading);

#endif
