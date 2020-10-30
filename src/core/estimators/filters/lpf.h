#ifndef __LPF_H__
#define __LPF_H__

void lpf_first_order_init(float *ret_gain, float sampling_time, float cutoff_freq);
void lpf_first_order(float new, float *filtered, float alpha);

#endif
