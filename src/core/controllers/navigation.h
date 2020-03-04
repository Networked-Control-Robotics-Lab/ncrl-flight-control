#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

#define WAYPOINT_NUM_MAX 50

struct waypoint_t {
	float pos[3];
	float heading;
	float halt_time;
};

typedef struct {
	struct waypoint_t wp_now;

	struct waypoint_t wp_list[WAYPOINT_NUM_MAX];
	int curr_wp_num;

	struct {
		float len_x;
		float len_y;
	} geo_fence; /* rectangular geo-fence */
} nav_t;

void nav_init(nav_t *_nav);
int nav_add_new_waypoint(float *pos, float heading);
int nav_goto_waypoint_now(float *pos, float heading);

#endif
