#ifndef __TAKEOFF_LANDING_H__
#define __TAKEOFF_LANDING_H__

int autopilot_trigger_auto_landing(void);
int autopilot_trigger_auto_takeoff(void);

void autopilot_takeoff_handler(void);
void autopilot_landing_handler(float *curr_pos);

#endif
