#ifndef __MULTIROTOR_RC_H__
#define __MULTIROTOR_RC_H__

void multirotor_rc_special_function_handler(radio_t *rc);

void rc_wait_unlock_geasture(void);
bool rc_unlock_geasture_handler(radio_t *rc, float *accel_lpf);

#endif
