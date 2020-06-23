#include <string.h>
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ublox_m8n.h"
#include "uart.h"
#include "delay.h"

#define UBLOX_M8N_QUEUE_SIZE 1000

#define UBX_SYNC_C1 0xb5
#define UBX_SYNC_C2 0x62

uint8_t ubx_nav_pvt_set[] = {0xB5, 0x62, 0x6, 0x01, 0x08, 0x00, 0x01, 0x7,
                             0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1};
uint8_t ubx_nmea_gxgga_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
uint8_t ubx_nmea_gxggl_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
uint8_t ubx_nmea_gxgsa_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
uint8_t ubx_nmea_gxgsv_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
uint8_t ubx_nmea_gxgmc_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04,
                                0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x07, 0x4F};
uint8_t ubx_nmea_gxvtg_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05,
                                0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x08, 0x56};
uint8_t ubx_2g_mode_set[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07,
                             0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                             0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E,
                             0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x2A};
uint8_t ubx_utc_time_set[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01,
                              0x00, 0x00, 0x00, 0xDD, 0x68};
uint8_t ubx_save_rom_cmd[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x17, 0x31, 0xBF};
                              
typedef struct {
	char c;
} ublox_m8n_buf_c_t;

QueueHandle_t ublox_m8n_queue;

ublox_t ublox;

void ublox_command_send(uint8_t *cmd, int size)
{
	usart_puts(UART7, (char *)cmd, size);
	blocked_delay_ms(100);
}

void ublox_m8n_init(void)
{
	ublox_m8n_queue = xQueueCreate(UBLOX_M8N_QUEUE_SIZE, sizeof(ublox_m8n_buf_c_t));

#if 1
	/* enable nav_pvt message output */
	ublox_command_send(ubx_nav_pvt_set, UBX_NAV_PVT_SET_LEN);

	/* diable nmea messages output */
	ublox_command_send(ubx_nmea_gxgga_set, UBX_NMEA_GXGGA_SET_LEN);
	ublox_command_send(ubx_nmea_gxggl_set, UBX_NMEA_GXGGL_SET_LEN);
	ublox_command_send(ubx_nmea_gxgsa_set, UBX_NMEA_GXGSA_SET_LEN);
	ublox_command_send(ubx_nmea_gxgsv_set, UBX_NMEA_GXGSV_SET_LEN);
	ublox_command_send(ubx_nmea_gxgmc_set, UBX_NMEA_GXGMC_SET_LEN);
	ublox_command_send(ubx_nmea_gxvtg_set, UBX_NMEA_GXVTG_SET_LEN);

	/* set dynamic range to 2g  */
	ublox_command_send(ubx_2g_mode_set, UBX_2G_MODE_SET_LEN);

	/* use UTC time */
	ublox_command_send(ubx_utc_time_set, UBX_UTC_TIME_SET_LEN);

	/* save configurations to rom */
	//ublox_command_send(ubx_save_rom_cmd, UBX_SAVE_ROM_CMD_LEN);
#endif
}

void ublox_checksum_calc(uint8_t *result, uint8_t *payload, uint16_t len)
{
	result[0] = 0;
	result[1] = 0;

	int i;
	for(i = 0; i < len; i++) {
		result[0] += payload[i];
		result[1] += result[0];
	}
}

void ublox_m8n_isr_handler(uint8_t c)
{
	ublox_m8n_buf_c_t ublox_m8n_queue_item;
	ublox_m8n_queue_item.c = c;

	BaseType_t higher_priority_task_woken = pdFALSE;
	xQueueSendToBackFromISR(ublox_m8n_queue, &ublox_m8n_queue_item,
	                        &higher_priority_task_woken);
	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

void ublox_decode_nav_pvt_msg(void)
{
	if((ublox.recept_class != 0x01) || (ublox.recept_id != 0x07) ||
	    (ublox.recept_len != 92)) {
		return;
	}

	uint8_t calc_ck[2];
	ublox_checksum_calc(calc_ck, ublox.recept_buf, 92);
	if(calc_ck != ublox.recept_ck) {
		return;
	}

	memcpy(&ublox.year, (ublox.recept_buf + 4), sizeof(uint16_t));
	memcpy(&ublox.month, (ublox.recept_buf + 6), sizeof(uint16_t));
	memcpy(&ublox.day, (ublox.recept_buf + 7), sizeof(uint8_t));
	memcpy(&ublox.hour, (ublox.recept_buf + 8), sizeof(uint8_t));
	memcpy(&ublox.minute, (ublox.recept_buf + 9), sizeof(uint8_t));
	memcpy(&ublox.second, (ublox.recept_buf + 10), sizeof(uint8_t));
	memcpy(&ublox.longitude, (ublox.recept_buf + 24), sizeof(int32_t));
	memcpy(&ublox.latitude, (ublox.recept_buf + 28), sizeof(int32_t));
	memcpy(&ublox.vel_n, (ublox.recept_buf + 48), sizeof(int32_t));
	memcpy(&ublox.vel_e, (ublox.recept_buf + 52), sizeof(int32_t));
	memcpy(&ublox.fix_type, (ublox.recept_buf + 20), sizeof(uint8_t));
	memcpy(&ublox.num_sv, (ublox.recept_buf + 23), sizeof(uint8_t));
	memcpy(&ublox.pdop, (ublox.recept_buf + 76), sizeof(uint16_t));
}

void ublox_m8n_gps_update(void)
{
	/* ubx message protocol:
	   +---------+---------+-------+----+-----+----------+-----------+------------+
	   | sync c1 | sync c2 | class | id | len | payloads | checksum1 | checksum 2 |
	   +---------+---------+-------+----+-----+----------+-----------+------------+*/

	ublox_m8n_buf_c_t recept_c;
	while(xQueueReceive(ublox_m8n_queue, &recept_c, 0) == pdTRUE) {
		uint8_t c = recept_c.c;

		switch(ublox.parse_state) {
		case UBX_STATE_WAIT_SYNC_C1:
			if(c == UBX_SYNC_C1) {
				ublox.parse_state = UBX_STATE_WAIT_SYNC_C2;
				ublox.recept_buf_ptr = 0;
			}
			break;
		case UBX_STATE_WAIT_SYNC_C2:
			if(c == UBX_SYNC_C2) {
				ublox.parse_state = UBX_STATE_RECEIVE_CLASS;
			} else {
				ublox.parse_state = UBX_STATE_WAIT_SYNC_C1;
			}
			break;
		case UBX_STATE_RECEIVE_CLASS:
			ublox.recept_class = c;
			ublox.parse_state = UBX_STATE_RECEIVE_ID;
			break;
		case UBX_STATE_RECEIVE_ID:
			ublox.recept_id = c;
			ublox.parse_state = UBX_STATE_RECEIVE_LEN1;
			break;
		case UBX_STATE_RECEIVE_LEN1:
			ublox.recept_len_buf[0] = c;
			ublox.parse_state = UBX_STATE_RECEIVE_LEN2;
			break;
		case UBX_STATE_RECEIVE_LEN2:
			ublox.recept_len_buf[1] = c;
			ublox.recept_len = *(uint16_t *)ublox.recept_len_buf;
			ublox.parse_state = UBX_STATE_RECEIVE_PAYLOAD;
			break;
		case UBX_STATE_RECEIVE_PAYLOAD: {
			if(ublox.recept_buf_ptr == (ublox.recept_len - 1)) {
				ublox.recept_buf[ublox.recept_buf_ptr] = c;
				ublox.parse_state = UBX_STATE_RECEIVE_CK1;
			} else {
				ublox.recept_buf[ublox.recept_buf_ptr] = c;
				ublox.recept_buf_ptr++;
			}
			break;
		}
		case UBX_STATE_RECEIVE_CK1:
			ublox.recept_ck[0] = c;
			ublox.parse_state = UBX_STATE_RECEIVE_CK2;
			break;
		case UBX_STATE_RECEIVE_CK2:
			ublox.recept_ck[1] = c;
			ublox_decode_nav_pvt_msg();
			ublox.parse_state = UBX_STATE_WAIT_SYNC_C1;
			break;
		}
	}
}
