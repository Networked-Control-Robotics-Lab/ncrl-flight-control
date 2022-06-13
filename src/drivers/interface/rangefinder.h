#ifndef __RANGEFINDER_H__
#define __RANGEFINDER_H__

bool rangefinder_available(void);
float rangefinder_get_distance(void);
float rangefinder_get_velocity(void);
float rangefinder_get_update_freq(void);

#endif

