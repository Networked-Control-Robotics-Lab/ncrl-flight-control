#ifndef __VIO_H__
#define __VIO_H__

typedef struct {
	float q_l2g[4];   //frame rotation from global to local
	float R_l2g[3*3]; //frame rotation from local to global
	float p_l2g[3];   //frame translation from local to global

	bool frame_align; //flag to enable vio frame alignment
} vio_t;

bool vio_available(void);

void vio_enable_frame_alignment(void);
void vio_disable_frame_alignment(void);

void vio_calculate_frame_alignment(void);

void vio_get_frame_alignment_quaternion(float *q_l2g);
void vio_get_frame_alignment_rotation_matrix(float *R_l2g);
void vio_get_frame_alignment_translation(float *p_l2g);

void vio_get_quaternion(float *q);

void vio_get_position_ned(float *pos);
float vio_get_position_ned_x(void);
float vio_get_position_ned_y(void);
float vio_get_position_ned_z(void);

void vio_get_velocity_ned(float *vel);
float vio_get_velocity_ned_x(void);
float vio_get_velocity_ned_y(void);
float vio_get_velocity_ned_z(void);

void vio_get_position_enu(float *pos);
float vio_get_position_enu_x(void);
float vio_get_position_enu_y(void);
float vio_get_position_enu_z(void);

void vio_get_velocity_enu(float *vel);
float vio_get_velocity_enu_x(void);
float vio_get_velocity_enu_y(void);
float vio_get_velocity_enu_z(void);

#endif
