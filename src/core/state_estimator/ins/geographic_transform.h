#ifndef __GEOGRAPHIC_TRANSFORM_H__
#define __GEOGRAPHIC_TRANSFORM_H__

typedef struct {
	int32_t home_longitude_s32;
	int32_t home_latitude_s32;

	double home_longitude;
	double home_latitude;

	double home_ecef[3];

	bool home_is_set;
} gps_home_t;

bool gps_home_is_set(void);

void set_home_longitude_latitude(int32_t longitude, int32_t latitude, float height_msl);
void get_home_longitude_latitude(int32_t *longitude, int32_t *latitude);

void longitude_latitude_to_ned(float *pos_ned, int32_t _longitude,
                               int32_t _latitude, float height_msl);

#endif
