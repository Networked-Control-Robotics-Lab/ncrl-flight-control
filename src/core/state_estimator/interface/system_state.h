#ifndef __SYSTEM_STATE_H__
#define __SYSTEM_STATE_H__

typedef struct {
	int heading_sensor;
	int height_sensor;
	int position_sensor;
} sensor_manager_t;

bool is_heading_available(void);
bool is_xy_position_available(void);
bool is_height_available(void);

void get_attitude_euler_angles(float *roll, float *pitch, float *yaw);
void get_attitude_quaternion(float *q);
void get_rotation_matrix_b2i(float **R_b2i);
void get_rotation_matrix_i2b(float **R_i2b);

void get_enu_position(float *pos);
float get_enu_position_x(void);
float get_enu_position_y(void);
float get_enu_position_z(void);

void get_enu_velocity(float *vel);
float get_enu_velocity_x(void);
float get_enu_velocity_y(void);
float get_enu_velocity_z(void);

int get_heading_sensor(void);
int get_height_sensor(void);
int get_position_sensor(void);

void set_heading_sensor(int new_heading_sensor);
void set_height_sensor(int new_height_sensor);
void set_position_sensor(int new_position_sensor);

#endif
