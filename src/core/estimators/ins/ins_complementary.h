#ifndef __COMP_NAV_H__
#define __COMP_NAV_H__

void comp_nav_init(float _dt);
void comp_nav_estimate(void);

void pos_vel_complementary_filter_predict(float *pos_enu_out, float *vel_enu_out);
void pos_vel_complementary_filter_gps_correct(float *pos_enu_in,  float *vel_enu_in,
                                              float *pos_enu_out, float *vel_enu_out);
void pos_vel_complementary_filter_barometer_correct(float *pos_enu_in,  float *vel_enu_in,
                                                    float *pos_enu_out, float *vel_enu_out);

#endif
