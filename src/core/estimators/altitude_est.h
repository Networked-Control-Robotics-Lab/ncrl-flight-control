#ifndef __ALTITUDE_EST_H__
#define __ALTITUDE_EST_H__

float barometer_alt_rate_estimate(float *dcm, float alt_rate, float *accel_body, float dt);
void send_alt_est_debug_message(debug_msg_t *payload);

#endif
