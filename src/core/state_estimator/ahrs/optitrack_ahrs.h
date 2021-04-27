#ifndef __OPTITRACK_AHRS_H__
#define __OPTITRACK_AHRS_H__

void optitrack_ahrs_init(float ahrs_dt);
void ahrs_optitrack_imu_fuse_estimate(float *q_out, float *gyro);

#endif
