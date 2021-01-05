#ifndef __INS_H__
#define __INS_H__

void ins_init(void);
bool ins_check_sensor_status(void);
void ins_state_estimate(void);

/* raw position getters */
float ins_get_raw_position_x(void);
float ins_get_raw_position_y(void);
float ins_get_raw_position_z(void);

/* raw velocity getters */
float ins_get_raw_velocity_x(void);
float ins_get_raw_velocity_y(void);
float ins_get_raw_velocity_z(void);

/* ins fused position getters */
float ins_get_fused_position_x(void);
float ins_get_fused_position_y(void);
float ins_get_fused_position_z(void);

/* ins fused velocity getters */
float ins_get_fused_velocity_x(void);
float ins_get_fused_velocity_y(void);
float ins_get_fused_velocity_z(void);

#endif
