#ifndef __COMP_NAV_H__
#define __COMP_NAV_H__

void comp_nav_init(float _dt);
void comp_nav_estimate(void);

void pos_vel_complementary_filter(float *pos_enu_raw, float *vel_enu_raw);
void get_complementary_fused_position(float *pos_enu);
void get_complementary_fused_velocity(float *vel_enu);

#endif
