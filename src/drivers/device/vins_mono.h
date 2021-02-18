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
	float pos_last[3];
	bool vel_ready;
} vins_mono_t ;


//for receiving pose & vel message
void vins_mono_init(int id);
int vins_mono_serial_decoder(uint8_t *buf);
void vins_mono_isr_handler(uint8_t c);	//vins_mono rx interrupt

void vins_mono_update(void);
bool vins_mono_available(void);		//vins_mono rx get info stablely

void vins_mono_read_pos_x(float *x);
void vins_mono_read_pos_y(float *y);
void vins_mono_read_pos_z(float *z);
void vins_mono_read_vel_x(float *vx);
void vins_mono_read_vel_y(float *vy);
void vins_mono_read_vel_z(float *vz);

//for sending IMU message			
void send_vins_mono_imu_msg(void);
void vins_mono_send_imu_50hz(void);

//for vins_mono camera triggering		
void vins_mono_camera_trigger_20hz(void);

//for Debug message 					
void send_vins_mono_position_debug_message(debug_msg_t *payload);
void send_vins_mono_quaternion_debug_message(debug_msg_t *payload);
void send_vins_mono_velocity_debug_message(debug_msg_t *payload);

#endif
