#include <math.h>
#include <string.h>
#include "lpf.h"

void lpf_first_order_init(float *ret_gain, float sampling_time, float cutoff_freq)
{
	//return the a value of the first order low pass filter
	*ret_gain = sampling_time / (sampling_time + 2 * M_PI * cutoff_freq);
}

void lpf_first_order(float new, float *filtered, float alpha)
{
	*filtered = (new * alpha) + (*filtered * (1.0f - alpha));
}
