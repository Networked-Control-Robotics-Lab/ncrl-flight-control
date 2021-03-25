#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "./mavlink/mav_publisher.h"
#include "sys_param.h"
#include "delay.h"
#include "hash.h"
#include "sys_param.h"
#include "common_list.h"

bool send_param = true;
int send_index = 0;
int send_count = 0;

void mav_param_request_list(mavlink_message_t *received_msg)
{
	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* decode param_request_list message */
	mavlink_param_request_list_t param_request_list;
	mavlink_msg_param_request_list_decode(received_msg, &param_request_list);

	/* ignore the message if the target id not matched to the system id */
	if((uint8_t)sys_id != param_request_list.target_system) {
		return;
	}

	send_count = get_sys_param_list_size();
	send_index = 0;
	send_param = true;
}

void mav_param_resend_missing_item(int index)
{
	mavlink_message_t msg;

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

	get_sys_param_name(index, &param_name);
	get_sys_param_type(index, &param_type);

	switch(param_type) {
	case SYS_PARAM_U8:
		get_sys_param_u8(index, &data_u8);
		param_val = (float)data_u8;
		break;
	case SYS_PARAM_S8:
		get_sys_param_s8(index, &data_s8);
		param_val = (float)data_s8;
		break;
	case SYS_PARAM_U16:
		get_sys_param_u16(index, &data_u16);
		param_val = (float)data_u16;
		break;
	case SYS_PARAM_S16:
		get_sys_param_s16(index, &data_s16);
		param_val = (float)data_s16;
		break;
	case SYS_PARAM_U32:
		get_sys_param_u32(index, &data_u32);
		param_val = (float)data_u32;
		break;
	case SYS_PARAM_S32:
		get_sys_param_s32(index, &data_s32);
		param_val = (float)data_s32;
		break;
	case SYS_PARAM_FLOAT:
		get_sys_param_float(index, &data_float);
		param_val = (float)data_float;
		break;
	default:
		return;
	}

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_msg_param_value_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg, param_name,
	                                  param_val, param_type, send_count, index);
	send_mavlink_msg_to_uart(&msg);
}


void mav_param_request_read(mavlink_message_t *received_msg)
{
	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* decode param_request_read message */
	mavlink_param_request_read_t mav_param_rq;
	mavlink_msg_param_request_read_decode(received_msg, &mav_param_rq);

	/* ignore the message if the target id not matched to the system id */
	if((uint8_t)sys_id != mav_param_rq.target_system) {
		return;
	}

	/* empty param_id means qgs requests parameter resending */
	if(mav_param_rq.param_id[0] == '\0') {
		mav_param_resend_missing_item(mav_param_rq.param_index);
		return;
	}

	mavlink_message_t msg;

	int param_list_size = get_sys_param_list_size();

	char *param_name;
	float param_val = 0.0f;
	uint8_t param_type = 0;
	unsigned long param_hash = 0;

	uint8_t data_u8;
	int8_t data_s8;
	uint16_t data_u16;
	int16_t data_s16;
	uint32_t data_u32;
	int32_t data_s32;
	float data_float;

	/* warning: mavlink char array doesn't deal with string end symbol \0, therefore
	 * the following preprocessing is required! */
	char proper_str[50] = {0};
	strncpy(proper_str, mav_param_rq.param_id, 16);

	unsigned long search_hash = hash_djb2((unsigned char *)proper_str);

	/* find the parameter ground station ask to read */
	int i;
	for(i = 0 ; i < param_list_size; i++) {
		/* use hash code comparison to accelerate parameter searching */
		get_sys_param_hash(i, &param_hash);
		if(search_hash != param_hash) continue;

		/* compare paramameter name */
		get_sys_param_name(i, &param_name);
		if(strcmp(param_name, proper_str) != 0) continue;

		get_sys_param_type(i, &param_type);

		switch(param_type) {
		case SYS_PARAM_U8:
			get_sys_param_u8(i, &data_u8);
			param_val = (float)data_u8;
			break;
		case SYS_PARAM_S8:
			get_sys_param_s8(i, &data_s8);
			param_val = (float)data_s8;
			break;
		case SYS_PARAM_U16:
			get_sys_param_u16(i, &data_u16);
			param_val = (float)data_u16;
			break;
		case SYS_PARAM_S16:
			get_sys_param_s16(i, &data_s16);
			param_val = (float)data_s16;
			break;
		case SYS_PARAM_U32:
			get_sys_param_u32(i, &data_u32);
			param_val = (float)data_u32;
			break;
		case SYS_PARAM_S32:
			get_sys_param_s32(i, &data_s32);
			param_val = (float)data_s32;
			break;
		case SYS_PARAM_FLOAT:
			get_sys_param_float(i, &data_float);
			param_val = (float)data_float;
			break;
		default:
			return;
		}

		mavlink_msg_param_value_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg, param_name,
		                                  param_val, param_type, param_list_size, i);
		send_mavlink_msg_to_uart(&msg);
	}
}

void mav_param_set(mavlink_message_t *received_msg)
{
	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* decode param_request_set message */
	mavlink_param_set_t mav_param_set;
	mavlink_msg_param_set_decode(received_msg, &mav_param_set);

	/* ignore the message if the target id not matched to the system id */
	if((uint8_t)sys_id != mav_param_set.target_system) {
		return;
	}

	mavlink_message_t msg;

	int param_list_size = get_sys_param_list_size();

	char *param_name;
	float param_val = 0.0f;
	uint8_t param_type = 0;
	unsigned long param_hash = 0;

	uint8_t data_u8;
	int8_t data_s8;
	uint16_t data_u16;
	int16_t data_s16;
	uint32_t data_u32;
	int32_t data_s32;
	float data_float;

	/* warning: mavlink char array doesn't deal with string end symbol \0, therefore
	 * the following preprocessing is required! */
	char proper_str[50] = {0};
	strncpy(proper_str, mav_param_set.param_id, 16);

	unsigned long search_hash = hash_djb2((unsigned char *)proper_str);

	/* find the parameter ground station ask to write */
	int i;
	for(i = 0 ; i < param_list_size; i++) {
		/* use hash code comparison to accelerate parameter searching */
		get_sys_param_hash(i, &param_hash);
		if(search_hash != param_hash) continue;

		/* compare paramameter name */
		get_sys_param_name(i, &param_name);
		if(strcmp(param_name, proper_str) != 0) continue;

		get_sys_param_type(i, &param_type);

		switch(param_type) {
		case SYS_PARAM_U8:
			set_sys_param_u8(i, (uint8_t)mav_param_set.param_value);
			get_sys_param_u8(i, &data_u8);
			param_val = (float)data_u8;
			break;
		case SYS_PARAM_S8:
			set_sys_param_s8(i, (int8_t)mav_param_set.param_value);
			get_sys_param_s8(i, &data_s8);
			param_val = (float)data_s8;
			break;
		case SYS_PARAM_U16:
			set_sys_param_u16(i, (uint16_t)mav_param_set.param_value);
			get_sys_param_u16(i, &data_u16);
			param_val = (float)data_u16;
			break;
		case SYS_PARAM_S16:
			set_sys_param_s16(i, (int16_t)mav_param_set.param_value);
			get_sys_param_s16(i, &data_s16);
			param_val = (float)data_s16;
			break;
		case SYS_PARAM_U32:
			set_sys_param_u32(i, (uint32_t)mav_param_set.param_value);
			get_sys_param_u32(i, &data_u32);
			param_val = (float)data_u32;
			break;
		case SYS_PARAM_S32:
			set_sys_param_s32(i, (int32_t)mav_param_set.param_value);
			get_sys_param_s32(i, &data_s32);
			param_val = (float)data_s32;
			break;
		case SYS_PARAM_FLOAT:
			set_sys_param_float(i, (float)mav_param_set.param_value);
			get_sys_param_float(i, &data_float);
			param_val = (float)data_float;
			break;
		default:
			return;
		}

		/* update parameter list to flash */
		save_param_list_to_flash();

		mavlink_msg_param_value_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg, param_name,
		                                  param_val, param_type, param_list_size, i);
		send_mavlink_msg_to_uart(&msg);
	}
}

void paramater_microservice_handler(void)
{
	if(send_param == false) return;

	mavlink_message_t msg;

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

	get_sys_param_name(send_index, &param_name);
	get_sys_param_type(send_index, &param_type);

	switch(param_type) {
	case SYS_PARAM_U8:
		get_sys_param_u8(send_index, &data_u8);
		param_val = (float)data_u8;
		break;
	case SYS_PARAM_S8:
		get_sys_param_s8(send_index, &data_s8);
		param_val = (float)data_s8;
		break;
	case SYS_PARAM_U16:
		get_sys_param_u16(send_index, &data_u16);
		param_val = (float)data_u16;
		break;
	case SYS_PARAM_S16:
		get_sys_param_s16(send_index, &data_s16);
		param_val = (float)data_s16;
		break;
	case SYS_PARAM_U32:
		get_sys_param_u32(send_index, &data_u32);
		param_val = (float)data_u32;
		break;
	case SYS_PARAM_S32:
		get_sys_param_s32(send_index, &data_s32);
		param_val = (float)data_s32;
		break;
	case SYS_PARAM_FLOAT:
		get_sys_param_float(send_index, &data_float);
		param_val = (float)data_float;
		break;
	default:
		return;
	}

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_msg_param_value_pack_chan((uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg, param_name,
	                                  param_val, param_type, send_count, send_index);
	send_mavlink_msg_to_uart(&msg);

	//select next param to send
	send_index++;

	/* last param had sent, close the protocol */
	if(send_index == send_count) {
		send_param = false;
	}
}
