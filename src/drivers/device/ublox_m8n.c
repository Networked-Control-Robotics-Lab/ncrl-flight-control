#include <stdio.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ublox_m8n.h"
#include "uart.h"
#include "delay.h"
#include "sys_time.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "ins_sensor_sync.h"

#define UBLOX_M8N_QUEUE_SIZE 2000

#define UBX_SYNC_C1 0xb5
#define UBX_SYNC_C2 0x62

uint8_t ubx_nav_pvt_set[] = {0xB5, 0x62, 0x6, 0x01, 0x08, 0x00, 0x01, 0x7,
                             0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1
                            };
uint8_t ubx_nmea_gxgga_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23
                               };
uint8_t ubx_nmea_gxggl_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A
                               };
uint8_t ubx_nmea_gxgsa_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31
                               };
uint8_t ubx_nmea_gxgsv_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38
                               };
uint8_t ubx_nmea_gxrmc_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F
                               };
uint8_t ubx_nmea_gxvtg_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46
                               };
uint8_t ubx_2g_mode_set[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07,
                             0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                             0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E,
                             0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x2A
                            };
uint8_t ubx_utc_time_set[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01,
                              0x00, 0x00, 0x00, 0xDD, 0x68
                             };
uint8_t ubx_save_rom_cmd[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x17, 0x31, 0xBF
                             };

typedef struct {
	char c;
} ublox_m8n_buf_c_t;

QueueHandle_t ublox_m8n_queue;

ublox_t ublox;

bool ublox_available(void)
{
	//timeout if no gps data available more than 1000ms
	float current_time = get_sys_time_s();
	if((current_time - ublox.last_read_time) > 1.0) {
		return false;
	}
	return true;
}

void ublox_command_send(uint8_t *cmd, int size)
{
	usart_puts(UART7, (char *)cmd, size);
	blocked_delay_ms(100);
}

void ublox_m8n_init(void)
{
	ublox_m8n_queue = xQueueCreate(UBLOX_M8N_QUEUE_SIZE, sizeof(ublox_m8n_buf_c_t));

	blocked_delay_ms(500); //wait until uart finished initialization

	/* enable nav_pvt message output */
	ublox_command_send(ubx_nav_pvt_set, UBX_NAV_PVT_SET_LEN);

	/* diable nmea messages output */
	ublox_command_send(ubx_nmea_gxgga_set, UBX_NMEA_GXGGA_SET_LEN);
	ublox_command_send(ubx_nmea_gxggl_set, UBX_NMEA_GXGGL_SET_LEN);
	ublox_command_send(ubx_nmea_gxgsa_set, UBX_NMEA_GXGSA_SET_LEN);
	ublox_command_send(ubx_nmea_gxgsv_set, UBX_NMEA_GXGSV_SET_LEN);
	ublox_command_send(ubx_nmea_gxrmc_set, UBX_NMEA_GXRMC_SET_LEN);
	ublox_command_send(ubx_nmea_gxvtg_set, UBX_NMEA_GXVTG_SET_LEN);

	/* set dynamic range to 2g  */
	ublox_command_send(ubx_2g_mode_set, UBX_2G_MODE_SET_LEN);

	/* use UTC time */
	ublox_command_send(ubx_utc_time_set, UBX_UTC_TIME_SET_LEN);

	/* save configurations to rom */
	//ublox_command_send(ubx_save_rom_cmd, UBX_SAVE_ROM_CMD_LEN);
}

/* get raw longitude/ latitude/ height from gps receiver */
void ublox_m8n_get_longitude_latitude_height_s32(int32_t *longitude, int32_t *latitude, int32_t *height_msl)
{
	*longitude = ublox.longitude;   //[deg/1e7]
	*latitude = ublox.latitude;     //[deg/1e7]
	*height_msl = ublox.height_msl; //[mm]
}

void ublox_m8n_get_longitude_latitude_height(float *longitude, float *latitude, float *height_msl)
{
	*longitude = (float)ublox.longitude * 1e-7;   //[deg]
	*latitude = (float)ublox.latitude * 1e-7;     //[deg]
	*height_msl = (float)ublox.height_msl * 1e-3; //[m]
}

void ublox_m8n_get_velocity_ned(float *vx, float *vy, float *vz)
{
	*vx = (float)ublox.vel_n * 1e-3; //[m/s]
	*vy = (float)ublox.vel_e * 1e-3; //[m/s]
	*vz = (float)ublox.vel_d * 1e-3; //[m/s]
}

void ublox_m8n_get_dilution_of_precision(float *pdop, float *hdop, float *vdop)
{
	*pdop = (float)ublox.pdop * 1e2;
	*hdop = (float)ublox.pdop * 1e2;
	*vdop = (float)ublox.pdop * 1e2;
}

int ublox_m8n_get_satellite_numbers(void)
{
	return ublox.num_sv;
}

/* get gps gix type and return in mavlink defined enum */
uint8_t ublox_m8n_get_fix_type(void)
{
	switch(ublox.fix_type) {
	case 0: //no fix
		return GPS_FIX_TYPE_NO_FIX;
	case 1: //dead reckoning only
		return GPS_FIX_TYPE_NO_FIX;
	case 2: //2d fix
		return GPS_FIX_TYPE_2D_FIX;
	case 3: //3d fix
		return GPS_FIX_TYPE_3D_FIX;
	case 4: //GNSS + dead reckoning combined
		return GPS_FIX_TYPE_3D_FIX;
	case 5: //time only fix
		return GPS_FIX_TYPE_NO_FIX;
	default:
		return GPS_FIX_TYPE_NO_GPS;
	}
}

void ublox_m8n_get_position_uncertainty(float *h_acc, float *v_acc)
{
	*h_acc = (float)ublox.h_acc; //[mm], horizontal accuracy (uncertainty)
	*v_acc = (float)ublox.v_acc; //[mm], vertical accuracy (uncertainty)
}

float ublox_m8n_get_ground_speed(void)
{
	return (float)ublox.ground_speed; //[mm/s]
}

float ublox_m8n_get_heading(void)
{
	return (float)ublox.heading * 1e5; //[deg]
}

float ublox_m8n_get_update_freq(void)
{
	return ublox.update_freq;
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

	/* length of nav_pvt_msg's payload is 92, reserve 4 for header */
	ublox_checksum_calc(ublox.calc_ck, ublox.recept_buf, 92 + 4);
	if(*(uint16_t *)ublox.calc_ck != *(uint16_t *)ublox.recept_ck) {
		return;
	}

	uint8_t *ublox_payload_addr = ublox.recept_buf + 4;
	//memcpy(&ublox.year, (ublox_payload_addr + 4), sizeof(uint16_t));
	//memcpy(&ublox.month, (ublox_payload_addr + 6), sizeof(uint8_t));
	//memcpy(&ublox.day, (ublox_payload_addr + 7), sizeof(uint8_t));
	//memcpy(&ublox.hour, (ublox_payload_addr + 8), sizeof(uint8_t));
	//memcpy(&ublox.minute, (ublox_payload_addr + 9), sizeof(uint8_t));
	//memcpy(&ublox.second, (ublox_payload_addr + 10), sizeof(uint8_t));
	memcpy(&ublox.longitude, (ublox_payload_addr + 24), sizeof(int32_t));
	memcpy(&ublox.latitude, (ublox_payload_addr + 28), sizeof(int32_t));
	memcpy(&ublox.height_ellipsoid, (ublox_payload_addr + 32), sizeof(int32_t));
	memcpy(&ublox.height_msl, (ublox_payload_addr + 36), sizeof(int32_t));
	memcpy(&ublox.vel_n, (ublox_payload_addr + 48), sizeof(int32_t));
	memcpy(&ublox.vel_e, (ublox_payload_addr + 52), sizeof(int32_t));
	memcpy(&ublox.vel_d, (ublox_payload_addr + 56), sizeof(int32_t));
	memcpy(&ublox.ground_speed, (ublox_payload_addr + 60), sizeof(int32_t));
	memcpy(&ublox.h_acc, (ublox_payload_addr + 40), sizeof(uint32_t));
	memcpy(&ublox.v_acc, (ublox_payload_addr + 44), sizeof(uint32_t));
	memcpy(&ublox.heading, (ublox_payload_addr + 64), sizeof(int32_t));
	memcpy(&ublox.fix_type, (ublox_payload_addr + 20), sizeof(uint8_t));
	memcpy(&ublox.num_sv, (ublox_payload_addr + 23), sizeof(uint8_t));
	memcpy(&ublox.pdop, (ublox_payload_addr + 76), sizeof(uint16_t));

	/* push gps data to ins sync buffer if satellite number >= 6 and
	 * fix mode = 3D fix mode */
	if((ublox.num_sv >= 6) && (ublox.fix_type == 3)) {
		/* set ublox state to be available */
		float curr_time = get_sys_time_s();
		ublox.update_freq = 1.0f / (curr_time - ublox.last_read_time);
		ublox.last_read_time = curr_time;

		float longitude = ublox.longitude * 1e-7;
		float latitude = ublox.latitude * 1e-7;
		float height_msl = ublox.height_msl * 1e2;
		float vel_n = ublox.vel_n * 1e-3;
		float vel_e = ublox.vel_e * 1e-3;
		float vel_d = ublox.vel_d * 1e-3;
		ins_gps_sync_buffer_push(longitude, latitude, height_msl,
		                         vel_n, vel_e, vel_d);
	}
}

void ublox_m8n_gps_update(void)
{
	ublox_m8n_buf_c_t recept_c;

#if 0   /* test print */
	while(1) {
		while(xQueueReceive(ublox_m8n_queue, &recept_c, 0) != pdTRUE);
		uart_putc(USART3, recept_c.c);
	}
#endif

	/* ubx message protocol:
	   +---------+---------+-------+----+-----+----------+-----------+------------+
	   | sync c1 | sync c2 | class | id | len | payloads | checksum1 | checksum 2 |
	   +---------+---------+-------+----+-----+----------+-----------+------------+*/

	while(xQueueReceive(ublox_m8n_queue, &recept_c, 0) == pdTRUE) {
		uint8_t c = recept_c.c;

		switch(ublox.parse_state) {
		case UBX_STATE_WAIT_SYNC_C1:
			if(c == UBX_SYNC_C1) {
				ublox.parse_state = UBX_STATE_WAIT_SYNC_C2;
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

			/* save class to buffer for checksum calculation */
			ublox.recept_buf_ptr = 0;
			ublox.recept_buf[ublox.recept_buf_ptr] = c;
			ublox.recept_buf_ptr++;

			ublox.parse_state = UBX_STATE_RECEIVE_ID;

			break;
		case UBX_STATE_RECEIVE_ID:
			ublox.recept_id = c;

			/* save id to buffer for checksum calculation */
			ublox.recept_buf[ublox.recept_buf_ptr] = c;
			ublox.recept_buf_ptr++;

			ublox.parse_state = UBX_STATE_RECEIVE_LEN1;
			break;
		case UBX_STATE_RECEIVE_LEN1:
			ublox.recept_len_buf[0] = c;

			/* save length to buffer for checksum calculation */
			ublox.recept_buf[ublox.recept_buf_ptr] = c;
			ublox.recept_buf_ptr++;

			ublox.parse_state = UBX_STATE_RECEIVE_LEN2;
			break;
		case UBX_STATE_RECEIVE_LEN2:
			ublox.recept_len_buf[1] = c;
			ublox.recept_len = *(uint16_t *)ublox.recept_len_buf;

			/* save length to buffer for checksum calculation */
			ublox.recept_buf[ublox.recept_buf_ptr] = c;
			ublox.recept_buf_ptr++;

			ublox.parse_state = UBX_STATE_RECEIVE_PAYLOAD;
			break;
		case UBX_STATE_RECEIVE_PAYLOAD: {
			/* reserve 4 for class, id and len */
			if(ublox.recept_buf_ptr == (ublox.recept_len - 1 + 4)) {
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
			/* decode phase */
			ublox.recept_ck[1] = c;
			ublox_decode_nav_pvt_msg();
			ublox.parse_state = UBX_STATE_WAIT_SYNC_C1;
		}
	}
}
