#include <stdint.h>
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "./publisher.h"
#include "sys_param.h"

void mav_param_request_list(mavlink_message_t *received_msg)
{
	mavlink_message_t msg;

	uint16_t param_cnt = get_sys_param_list_size();

	char *param_name;
	float param_val = 0.0f;
	uint8_t param_type = 0;

	uint8_t data_u8;
	int8_t data_s8;
	uint16_t data_u16;
	int16_t data_s16;
	uint32_t data_u32;
	int32_t data_s32;
	float data_float;

	int i;
	for(i = 0; i < param_cnt; i++) {
		get_sys_param_name(i, &param_name);
		get_sys_param_type(i, &param_type);

		switch(param_type) {
		case SYS_PARAM_U8:
			get_sys_param_u8(i, &data_u8);
			param_val = (float)data_u8;
		case SYS_PARAM_S8:
			get_sys_param_s8(i, &data_s8);
			param_val = (float)data_s8;
		case SYS_PARAM_U16:
			get_sys_param_u16(i, &data_u16);
			param_val = (float)data_u16;
		case SYS_PARAM_S16:
			get_sys_param_s16(i, &data_s16);
			param_val = (float)data_s16;
		case SYS_PARAM_U32:
			get_sys_param_u32(i, &data_u32);
			param_val = (float)data_u32;
		case SYS_PARAM_S32:
			get_sys_param_s32(i, &data_s32);
			param_val = (float)data_s32;
		case SYS_PARAM_FLOAT:
			get_sys_param_float(i, &data_float);
			param_val = (float)data_float;
		default:
			return;
		}

		mavlink_msg_param_value_pack_chan(1, 1, MAVLINK_COMM_1, &msg, param_name,
		                                  param_val, param_type, param_cnt, i);
		send_mavlink_msg_to_uart(&msg);
	}
}

void mav_param_request_read(mavlink_message_t *received_msg)
{
	mavlink_param_request_read_t mav_param_rq;
	mavlink_msg_param_request_read_decode(received_msg, &mav_param_rq);

	mavlink_message_t msg;

	int param_list_size = get_sys_param_list_size();

	char *param_name;
	float param_val = 0.0f;
	uint8_t param_type = 0;

	uint8_t data_u8;
	int8_t data_s8;
	uint16_t data_u16;
	int16_t data_s16;
	uint32_t data_u32;
	int32_t data_s32;
	float data_float;

	/* find the parameter ground station ask to read */
	int i;
	for(i = 0 ; i < param_list_size; i++) {
		get_sys_param_name(i, &param_name);

		/* compare paramameter name */
		if(strcmp(param_name, mav_param_rq.param_id) != 0) continue;

		get_sys_param_type(i, &param_type);

		switch(param_type) {
		case SYS_PARAM_U8:
			get_sys_param_u8(i, &data_u8);
			param_val = (float)data_u8;
		case SYS_PARAM_S8:
			get_sys_param_s8(i, &data_s8);
			param_val = (float)data_s8;
		case SYS_PARAM_U16:
			get_sys_param_u16(i, &data_u16);
			param_val = (float)data_u16;
		case SYS_PARAM_S16:
			get_sys_param_s16(i, &data_s16);
			param_val = (float)data_s16;
		case SYS_PARAM_U32:
			get_sys_param_u32(i, &data_u32);
			param_val = (float)data_u32;
		case SYS_PARAM_S32:
			get_sys_param_s32(i, &data_s32);
			param_val = (float)data_s32;
		case SYS_PARAM_FLOAT:
			get_sys_param_float(i, &data_float);
			param_val = (float)data_float;
		default:
			return;
		}

		mavlink_msg_param_value_pack_chan(1, 1, MAVLINK_COMM_1, &msg, param_name,
		                                  param_val, param_type, param_list_size, i);
		send_mavlink_msg_to_uart(&msg);
	}
}

void mav_param_set(mavlink_message_t *received_msg)
{
	mavlink_param_set_t mav_param_set;
	mavlink_msg_param_set_decode(received_msg, &mav_param_set);

	mavlink_message_t msg;

	int param_list_size = get_sys_param_list_size();

	char *param_name;
	float param_val = 0.0f;
	uint8_t param_type = 0;

	uint8_t data_u8;
	int8_t data_s8;
	uint16_t data_u16;
	int16_t data_s16;
	uint32_t data_u32;
	int32_t data_s32;
	float data_float;

	/* find the parameter ground station ask to write */
	int i;
	for(i = 0 ; i < param_list_size; i++) {
		get_sys_param_name(i, &param_name);

		/* compare paramameter name */
		if(strcmp(param_name, mav_param_set.param_id) != 0) continue;

		get_sys_param_type(i, &param_type);

		switch(param_type) {
		case SYS_PARAM_U8:
			set_sys_param_u8(i, (uint8_t)mav_param_set.param_value);
			get_sys_param_u8(i, &data_u8);
			param_val = (float)data_u8;
		case SYS_PARAM_S8:
			set_sys_param_s8(i, (int8_t)mav_param_set.param_value);
			get_sys_param_s8(i, &data_s8);
			param_val = (float)data_s8;
		case SYS_PARAM_U16:
			set_sys_param_u16(i, (uint16_t)mav_param_set.param_value);
			get_sys_param_u16(i, &data_u16);
			param_val = (float)data_u16;
		case SYS_PARAM_S16:
			set_sys_param_s16(i, (int16_t)mav_param_set.param_value);
			get_sys_param_s16(i, &data_s16);
			param_val = (float)data_s16;
		case SYS_PARAM_U32:
			set_sys_param_u32(i, (uint32_t)mav_param_set.param_value);
			get_sys_param_u32(i, &data_u32);
			param_val = (float)data_u32;
		case SYS_PARAM_S32:
			set_sys_param_s32(i, (int32_t)mav_param_set.param_value);
			get_sys_param_s32(i, &data_s32);
			param_val = (float)data_s32;
		case SYS_PARAM_FLOAT:
			set_sys_param_float(i, (float)mav_param_set.param_value);
			get_sys_param_float(i, &data_float);
			param_val = (float)data_float;
		default:
			return;
		}

		mavlink_msg_param_value_pack_chan(1, 1, MAVLINK_COMM_1, &msg, param_name,
		                                  param_val, param_type, param_list_size, i);
		send_mavlink_msg_to_uart(&msg);
	}
}
