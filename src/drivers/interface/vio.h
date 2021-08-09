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

void vio_get_quaternion(float *q);
void vio_get_position(float *pos);
float vio_get_position_x(void);
float vio_get_position_y(void);
float vio_get_position_z(void);
void vio_get_velocity(float *vel);
float vio_get_velocity_x(void);
float vio_get_velocity_y(void);
float vio_get_velocity_z(void);

#endif
