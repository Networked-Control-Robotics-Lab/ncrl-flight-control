#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

void nav_velocity_predict(float *dcm_b2i, float *imu_acc);
void nav_velocity_correct(float *vel_ref, float *vel_filtered);

#endif
