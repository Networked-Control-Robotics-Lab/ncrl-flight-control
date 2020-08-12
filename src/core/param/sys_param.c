#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "crc.h"
#include "sys_param.h"
#include "hash.h"

sys_param_data *sys_param_list = NULL;
int list_size = 0;
int list_last_index = 0;

void init_sys_param_list(sys_param_data *list, int _list_size)
{
	sys_param_list = list;
	list_size = _list_size;
	if(_list_size > 0) {
		list_last_index = list_size - 1;
	}
}

void reset_sys_param_list_to_default(void)
{
	int i;
	for(i = 0; i < list_size; i++) {
		sys_param_list[i].curr = sys_param_list[i]._default;
	}
}

int get_sys_param_list_size(void)
{
	return list_size;
}

int get_sys_param_name(int index, char **name)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*name = sys_param_list[index].name;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_hash(int index, unsigned long *param_hash)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*param_hash = sys_param_list[index].hash;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_type(int index, uint8_t *type)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*type = sys_param_list[index].type;

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_update_var_addr(int index, void *var_addr)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].update_var_ptr = var_addr;

	return SYS_PARAM_SUCCEED;
}

/****************************
 * system parameter getting *
 ****************************/
int get_sys_param_u8(int index, uint8_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].curr.u8_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_s8(int index, int8_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].curr.s8_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_u16(int index, uint16_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].curr.u16_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_s16(int index, int16_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].curr.s16_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_u32(int index, uint32_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].curr.u32_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_s32(int index, int32_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].curr.s32_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_float(int index, float *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].curr.float_val;

	return SYS_PARAM_SUCCEED;
}

/***********************************
 * system parameter initialization *
 ************************************/
int init_sys_param_u8(int index, char *name, uint8_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].name = name;
	sys_param_list[index].curr.u8_val = val;
	sys_param_list[index]._default.u8_val = val;
	sys_param_list[index].type = SYS_PARAM_U8;
	sys_param_list[index].hash = hash_djb2((unsigned char *)name);
	sys_param_list[index].update_var_ptr = NULL;

	return SYS_PARAM_SUCCEED;
}

int init_sys_param_s8(int index, char *name, int8_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].name = name;
	sys_param_list[index].curr.s8_val = val;
	sys_param_list[index]._default.s8_val = val;
	sys_param_list[index].type = SYS_PARAM_S8;
	sys_param_list[index].hash = hash_djb2((unsigned char *)name);
	sys_param_list[index].update_var_ptr = NULL;

	return SYS_PARAM_SUCCEED;
}

int init_sys_param_u16(int index, char *name, uint16_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].name = name;
	sys_param_list[index].curr.u16_val = val;
	sys_param_list[index]._default.u16_val = val;
	sys_param_list[index].type = SYS_PARAM_U16;
	sys_param_list[index].hash = hash_djb2((unsigned char *)name);
	sys_param_list[index].update_var_ptr = NULL;

	return SYS_PARAM_SUCCEED;
}

int init_sys_param_s16(int index, char *name, int16_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].name = name;
	sys_param_list[index].curr.s16_val = val;
	sys_param_list[index]._default.s16_val = val;
	sys_param_list[index].type = SYS_PARAM_S16;
	sys_param_list[index].hash = hash_djb2((unsigned char *)name);
	sys_param_list[index].update_var_ptr = NULL;

	return SYS_PARAM_SUCCEED;
}

int init_sys_param_u32(int index, char *name, uint32_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].name = name;
	sys_param_list[index].curr.u32_val = val;
	sys_param_list[index]._default.u32_val = val;
	sys_param_list[index].type = SYS_PARAM_U32;
	sys_param_list[index].hash = hash_djb2((unsigned char *)name);
	sys_param_list[index].update_var_ptr = NULL;

	return SYS_PARAM_SUCCEED;
}

int init_sys_param_s32(int index, char *name, int32_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].name = name;
	sys_param_list[index].curr.s32_val = val;
	sys_param_list[index]._default.s32_val = val;
	sys_param_list[index].type = SYS_PARAM_S32;
	sys_param_list[index].hash = hash_djb2((unsigned char *)name);
	sys_param_list[index].update_var_ptr = NULL;

	return SYS_PARAM_SUCCEED;
}

int init_sys_param_float(int index, char *name, float val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].name = name;
	sys_param_list[index].curr.float_val = val;
	sys_param_list[index]._default.float_val = val;
	sys_param_list[index].type = SYS_PARAM_FLOAT;
	sys_param_list[index].hash = hash_djb2((unsigned char *)name);
	sys_param_list[index].update_var_ptr = NULL;

	return SYS_PARAM_SUCCEED;
}

/****************************
 * system parameter setting *
 ****************************/
int set_sys_param_u8(int index, uint8_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].curr.u8_val = val;

	if(sys_param_list[index].update_var_ptr != NULL) {
		*(uint8_t *)sys_param_list[index].update_var_ptr = val;
	}

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_s8(int index, int8_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}
	sys_param_list[index].curr.s8_val = val;

	if(sys_param_list[index].update_var_ptr != NULL) {
		*(int8_t *)sys_param_list[index].update_var_ptr = val;
	}

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_u16(int index, uint16_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].curr.u16_val = val;

	if(sys_param_list[index].update_var_ptr != NULL) {
		*(uint16_t *)sys_param_list[index].update_var_ptr = val;
	}

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_s16(int index, int16_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].curr.s16_val = val;

	if(sys_param_list[index].update_var_ptr != NULL) {
		*(int16_t *)sys_param_list[index].update_var_ptr = val;
	}

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_u32(int index, uint32_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].curr.u32_val = val;

	if(sys_param_list[index].update_var_ptr != NULL) {
		*(uint32_t *)sys_param_list[index].update_var_ptr = val;
	}

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_s32(int index, int32_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].curr.s32_val = val;

	if(sys_param_list[index].update_var_ptr != NULL) {
		*(int32_t *)sys_param_list[index].update_var_ptr = val;
	}

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_float(int index, float val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].curr.float_val = val;

	if(sys_param_list[index].update_var_ptr != NULL) {
		*(float *)sys_param_list[index].update_var_ptr = val;
	}

	return SYS_PARAM_SUCCEED;
}

/*******************************
 * flash storage r/w functions *
 *******************************/
int save_param_list_to_flash(void)
{
	uint32_t save_buf[list_size + 2];
	memset(save_buf, 0, list_size + 2);

	int i;
	for(i = 0; i < list_size; i++) {
		switch(sys_param_list[i].type) {
		case SYS_PARAM_U8:
			memcpy(&save_buf[i + 2], &sys_param_list[i].curr.u8_val, sizeof(uint8_t));
			break;
		case SYS_PARAM_S8:
			memcpy(&save_buf[i + 2], &sys_param_list[i].curr.s8_val, sizeof(int8_t));
			break;
		case SYS_PARAM_U16:
			memcpy(&save_buf[i + 2], &sys_param_list[i].curr.u16_val, sizeof(uint16_t));
			break;
		case SYS_PARAM_S16:
			memcpy(&save_buf[i + 2], &sys_param_list[i].curr.s16_val, sizeof(int16_t));
			break;
		case SYS_PARAM_U32:
			memcpy(&save_buf[i + 2], &sys_param_list[i].curr.u32_val, sizeof(uint32_t));
			break;
		case SYS_PARAM_S32:
			memcpy(&save_buf[i + 2], &sys_param_list[i].curr.s32_val, sizeof(int32_t));
			break;
		case SYS_PARAM_FLOAT:
			memcpy(&save_buf[i + 2], &sys_param_list[i].curr.float_val, sizeof(float));
			break;
		}
	}

	/* reserve 2 word for list size and crc */
	memcpy(&save_buf[0], &list_size, sizeof(int));
	uint32_t crc = calculate_crc_of_words(save_buf + 2, list_size);
	memcpy(&save_buf[1], &crc, sizeof(uint32_t));

	/* save system parameters to sector 11 of internel flash */
	int retval = flash_write(ADDR_FLASH_SECTOR_11, save_buf, list_size + 2);

	switch(retval) {
	case FLASH_WR_DATA_INCORRECT:
		return SYS_PARAM_FLASH_WR_DATA_INCORRECT;
	case FLASH_WR_TIMEOUT:
		return SYS_PARAM_FLASH_WR_TIMEOUT;
	case FLASH_ERASE_TIMEOUT:
		return SYS_PARAM_FLASH_ERASE_TIMEOUT;
	}

	return SYS_PARAM_FLASH_WR_SUCCEED;
}

int load_param_list_from_flash(void)
{
	uint32_t read_buf[list_size];
	memset(read_buf, 0, list_size);

	int read_list_size = 0;
	int read_crc = 0;

	uint32_t flash_start_addr = ADDR_FLASH_SECTOR_11;

	/* read header (parameter list size) and crc */
	memcpy(&read_list_size, (uint32_t *)(flash_start_addr), sizeof(int));
	memcpy(&read_crc, (uint32_t *)(flash_start_addr) + 1, sizeof(uint32_t));

	/* read paramater list */
	memcpy(read_buf, (uint32_t *)(flash_start_addr) + 2, sizeof(uint32_t) * list_size);

	/* confirm list size stored in flash */
	if(read_list_size != list_size) {
		/* incorrect, save default value to flash */
		save_param_list_to_flash();
		return SYS_PARAM_FLASH_SIZE_INCORRECT;
	}

	/* confirm crc stored in flash */
	uint32_t calc_crc = calculate_crc_of_words(read_buf, list_size);
	if(read_crc != calc_crc) {
		/* incorrect, save default value to flash */
		save_param_list_to_flash();
		return SYS_PARAM_FLASH_CRC_INCORRECT;
	}

	/* load parameter list from buffer */
	int i;
	for(i = 0; i < list_size; i++) {
		switch(sys_param_list[i].type) {
		case SYS_PARAM_U8:
			memcpy(&sys_param_list[i].curr.u8_val, &read_buf[i], sizeof(uint8_t));
			break;
		case SYS_PARAM_S8:
			memcpy(&sys_param_list[i].curr.s8_val, &read_buf[i], sizeof(int8_t));
			break;
		case SYS_PARAM_U16:
			memcpy(&sys_param_list[i].curr.u16_val, &read_buf[i], sizeof(uint16_t));
			break;
		case SYS_PARAM_S16:
			memcpy(&sys_param_list[i].curr.s16_val, &read_buf[i], sizeof(int16_t));
			break;
		case SYS_PARAM_U32:
			memcpy(&sys_param_list[i].curr.u32_val, &read_buf[i], sizeof(uint32_t));
			break;
		case SYS_PARAM_S32:
			memcpy(&sys_param_list[i].curr.s32_val, &read_buf[i], sizeof(int32_t));
			break;
		case SYS_PARAM_FLOAT:
			memcpy(&sys_param_list[i].curr.float_val, &read_buf[i], sizeof(float));
			break;
		}
	}

	return SYS_PARAM_FLASH_WR_SUCCEED;
}
