#ifndef __VIO_H__
#define __VIO_H__

typedef struct {
	float q_align[4];
	bool frame_align_on;
} vio_t;

bool vio_available(void);

void vio_enable_frame_alignment(void);
void vio_disable_frame_alignment(void);
void vio_calc_frame_alignment_transform(void);

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
