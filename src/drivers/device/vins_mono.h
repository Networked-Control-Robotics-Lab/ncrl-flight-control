#ifndef __VINS_MONO_H__
#define __VINS_MONO_H__

#include "debug_link.h"

#define VINS_MONO_SERIAL_MSG_SIZE 44

typedef struct {
	uint8_t id;

	/* position [m] */
	float pos_enu[3];

	/* velocity [m/s] */
	float vel_enu[3];

	/* orientation (quaternion) */
	float q[4];

	float time_now;
	float time_last;
	float update_rate;

	volatile int buf_pos;
	uint8_t buf[VINS_MONO_SERIAL_MSG_SIZE];
	bool vel_ready;
} vins_mono_t ;


void vins_mono_init(int id);

/* reception of vins-mon pose and velocity information */
int vins_mono_serial_decoder(uint8_t *buf);
void vins_mono_isr_handler(uint8_t c);

/* transmission of imu information for vins-mono */
void send_vins_mono_imu_msg(void);
void vins_mono_send_imu_200hz(void);

/* vins-mono camera triggering */
void vins_mono_camera_trigger_20hz(void);

void vins_mono_update(void);
bool vins_mono_available(void);

/* vins_mono getters */
void vins_mono_get_position_enu(float *pos);
float vins_mono_get_position_enu_x(void);
float vins_mono_get_position_enu_y(void);
float vins_mono_get_position_enu_z(void);

void vins_mono_get_velocity_enu(float *vel);
float vins_mono_get_velocity_enu_x(void);
float vins_mono_get_velocity_enu_y(void);
float vins_mono_get_velocity_enu_z(void);

void vins_mono_get_position_ned(float *pos);
float vins_mono_get_position_ned_x(void);
float vins_mono_get_position_ned_y(void);
float vins_mono_get_position_ned_z(void);

void vins_mono_get_velocity_ned(float *vel);
float vins_mono_get_velocity_ned_x(void);
float vins_mono_get_velocity_ned_y(void);
float vins_mono_get_velocity_ned_z(void);

void vins_mono_get_quaternion(float *q);

/* vins-mono debug messages */
void send_vins_mono_position_debug_message(debug_msg_t *payload);
void send_vins_mono_quaternion_debug_message(debug_msg_t *payload);
void send_vins_mono_velocity_debug_message(debug_msg_t *payload);

#endif
