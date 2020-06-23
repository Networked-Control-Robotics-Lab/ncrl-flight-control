#ifndef __UBLOX_M8N_H__
#define __UBLOX_M8N_H__

#define UBX_BUFFER_SIZE 200

#define UBX_NAV_PVT_SET_LEN 16
#define UBX_NMEA_GXGGA_SET_LEN 16
#define UBX_NMEA_GXGGL_SET_LEN 16
#define UBX_NMEA_GXGSA_SET_LEN 16
#define UBX_NMEA_GXGSV_SET_LEN 16
#define UBX_NMEA_GXGMC_SET_LEN 16
#define UBX_NMEA_GXVTG_SET_LEN 16
#define UBX_2G_MODE_SET_LEN 44
#define UBX_UTC_TIME_SET_LEN 14
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

typedef struct {
	int parse_state;
	uint8_t recept_class;
	uint8_t recept_id;
	uint8_t recept_len_buf[2];
	uint16_t recept_len;
	uint8_t recept_ck[2];
	uint8_t recept_buf[UBX_BUFFER_SIZE];
	uint16_t recept_buf_ptr;

	uint16_t year;
	uint16_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	int32_t longitude;
	int32_t latitude;
	int32_t vel_n;
	int32_t vel_e;
	uint8_t fix_type;
	uint8_t num_sv;
	uint16_t pdop;
} ublox_t;

void ublox_m8n_init(void);
void ublox_m8n_isr_handler(uint8_t c);
void ublox_m8n_gps_update(void);

#endif
