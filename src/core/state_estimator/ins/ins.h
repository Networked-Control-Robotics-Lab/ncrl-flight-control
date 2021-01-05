#ifndef __INS_H__
#define __INS_H__

void ins_init(void);
bool ins_check_sensor_status(void);
void ins_state_estimate(void);

void ins_get_raw_position(float *pos_enu);
void ins_get_raw_velocity(float *vel_enu_raw);
void ins_get_fused_position(float *pos_enu_fused);
void ins_get_fused_velocity(float *vel_enu_fused);

#endif
