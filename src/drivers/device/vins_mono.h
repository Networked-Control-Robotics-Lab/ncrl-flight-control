#ifndef __VINS_MONO_H__
#define __VINS_MONO_H__

void send_vins_mono_imu_msg(void);
void vins_mono_send_imu_50hz(void);
void vins_mono_camera_trigger_20hz(void);

#endif
