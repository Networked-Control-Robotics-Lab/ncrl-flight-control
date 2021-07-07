#ifndef __VIO_INTERFACE_H__
#define __VIO_INTERFACE_H__

typedef struct {
	float q_align[4];
	bool gnss_align_on;
} vio_manager_t;

void vio_get_quaternion(float *q);
void vio_get_position(float *p);

#endif
