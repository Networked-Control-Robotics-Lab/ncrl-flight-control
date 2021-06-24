#ifndef __UBLOX_M8N_H__
#define __UBLOX_M8N_H__

#include <stdbool.h>

#define UBLOX_SV_NUM_MAX 255

#define UBX_BUFFER_SIZE 2000

#define UBX_NAV_PVT_SET_LEN 16
#define UBX_NAV_SAT_SET_LEN 16
#define UBX_NMEA_GXGGA_SET_LEN 16
#define UBX_NMEA_GXGGL_SET_LEN 16
#define UBX_NMEA_GXGSA_SET_LEN 16
#define UBX_NMEA_GXGSV_SET_LEN 16
#define UBX_NMEA_GXRMC_SET_LEN 16
#define UBX_NMEA_GXVTG_SET_LEN 16
#define UBX_2G_MODE_SET_LEN 44
#define UBX_CFG_RATE_SET_LEN 14
#define UBX_SAVE_ROM_CMD_LEN 21

enum {
	UBX_STATE_WAIT_SYNC_C1 = 0,
	UBX_STATE_WAIT_SYNC_C2 = 1,
	UBX_STATE_RECEIVE_CLASS = 2,
	UBX_STATE_RECEIVE_ID = 3,
	UBX_STATE_RECEIVE_LEN1 = 4,
	UBX_STATE_RECEIVE_LEN2 = 5,
	UBX_STATE_RECEIVE_PAYLOAD = 6,
	UBX_STATE_RECEIVE_CK1 = 7,
	UBX_STATE_RECEIVE_CK2 = 8
} UBX_PARSE_STATE;

struct sat_payload_item {
	uint8_t gnss_id;
	uint8_t sv_id;
	uint8_t cno;
	//int8_t elev;
	//int16_t azim;
	int16_t pr_res;
	uint32_t flags;
};

typedef struct {
	int parse_state;
	uint8_t recept_class;
	uint8_t recept_id;
	uint8_t recept_len_buf[2];
	uint16_t recept_len;
	uint8_t recept_ck[2];
	uint8_t calc_ck[2];
	uint8_t recept_buf[UBX_BUFFER_SIZE];
	uint16_t recept_buf_ptr;

	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	int32_t longitude;
	int32_t latitude;
	int32_t height_ellipsoid; //height above ellipsoid
	int32_t height_msl;       //height above mean sea level
	int32_t h_acc;            //horizontal accuracy
	int32_t v_acc;            //vertical accuracy
	int32_t vel_n;
	int32_t vel_e;
	int32_t vel_d;
	int32_t ground_speed;
	int32_t heading;
	uint8_t fix_type;
	uint8_t num_sv;
	uint16_t pdop;

	struct sat_payload_item sat_payload_list[UBLOX_SV_NUM_MAX];

	float last_read_time;
	float update_freq;
} ublox_t;

void ublox_m8n_init(void);
void ublox_m8n_isr_handler(uint8_t c);
bool ublox_available(void);
void ublox_m8n_gps_update(void);

void ublox_m8n_get_longitude_latitude_height_s32(int32_t *longitude, int32_t *latitude, int32_t *height_msl);
void ublox_m8n_get_longitude_latitude_height(float *longitude, float *latitude, float *height_msl);
void ublox_m8n_get_velocity_ned(float *vx, float *vy, float *vz);
int ublox_m8n_get_satellite_numbers(void);
void ublox_m8n_get_dilution_of_precision(float *pdop, float *hdop, float *vdop);
uint8_t ublox_m8n_get_fix_type(void);
void ublox_m8n_get_position_uncertainty(float *h_acc, float *v_acc);
float ublox_m8n_get_ground_speed(void);
float ublox_m8n_get_heading(void);
float ublox_m8n_get_update_freq(void);
float ublox_m8n_get_last_update_time_ms(void);

#endif
