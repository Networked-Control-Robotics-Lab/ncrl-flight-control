#ifndef __PERF_LIST_H__
#define __PREF_LIST_H__

/* enumerate performace counter id for executuin time profiling */
enum {
	PERF_AHRS_INS,
	PERF_CONTROLLER,
	PERF_FLIGHT_CONTROL_LOOP,
	PERF_FLIGHT_CONTROL_TRIGGER_TIME
} PERF_LIST;

#endif
