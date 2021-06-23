#ifndef __RANGEFINDER_H__
#define __RANGEFINDER_H__

bool rangefinder_available(void);
float rangefinder_get_(void);
float rangefinder_get_relative_altitude(void);
float rangefinder_get_update_freq(void);

#endif
 
