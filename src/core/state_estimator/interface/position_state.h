#ifndef __POSITION_SENSOR_H__
#define __POSITION_SENSOR_H__

typedef struct {
	int heading_src;
	int height_src;
	int position_src;
} sensor_manager_t;

bool is_heading_available(void);
bool is_xy_position_info_available(void);
bool is_height_info_available(void);

void get_enu_position(float *pos);
float get_enu_position_x(void);
float get_enu_position_y(void);
float get_enu_position_z(void);
void get_enu_velocity(float *vel);
float get_enu_velocity_x(void);
float get_enu_velocity_y(void);
float get_enu_velocity_z(void);

void change_heading_sensor_src(int new_src);
void change_height_sensor_src(int new_src);
void change_position_sensor_src(int new_src);

#endif
