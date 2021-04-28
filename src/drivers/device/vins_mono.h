#ifndef __VINS_MONO_H__
#define __VINS_MONO_H__

#define VINS_MONO_SERIAL_MSG_SIZE 44

typedef struct {
	uint8_t id;

	/* position [m] */
	float pos[3];

	/* velocity (numerical differentiation) [m/s] */
	float vel_raw[3];
	float vel_filtered[3];

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
void vins_mono_send_imu_50hz(void);

/* vins-mono camera triggering */
void vins_mono_camera_trigger_20hz(void);

void vins_mono_update(void);
bool vins_mono_available(void);

float vins_mono_read_pos_x(void);
float vins_mono_read_pos_y(void);
float vins_mono_read_pos_z(void);
float vins_mono_read_vel_x(void);
float vins_mono_read_vel_y(void);
float vins_mono_read_vel_z(void);

/* vins-mono debug messages */
void send_vins_mono_position_debug_message(debug_msg_t *payload);
void send_vins_mono_quaternion_debug_message(debug_msg_t *payload);
void send_vins_mono_velocity_debug_message(debug_msg_t *payload);

#endif
