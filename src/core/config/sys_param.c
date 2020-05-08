#include "stdio.h"
#include "sys_param.h"

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

int get_sys_param_type(int index, int *type)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*type = sys_param_list[index].type;

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

	*retval = sys_param_list[index].u8_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_s8(int index, int8_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].s8_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_u16(int index, uint16_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].u16_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_s16(int index, int16_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].s16_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_u32(int index, uint32_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].u32_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_s32(int index, int32_t *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].s32_val;

	return SYS_PARAM_SUCCEED;
}

int get_sys_param_float(int index, float *retval)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	*retval = sys_param_list[index].float_val;

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

	sys_param_list[index].u8_val = val;

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_s8(int index, int8_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}
	sys_param_list[index].s8_val = val;

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_u16(int index, uint16_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].u16_val = val;

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_s16(int index, int16_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].s16_val = val;

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_u32(int index, uint32_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].u32_val = val;

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_s32(int index, int32_t val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].s32_val = val;

	return SYS_PARAM_SUCCEED;
}

int set_sys_param_float(int index, float val)
{
	if((index < 0) || (index > list_last_index)) {
		return SYS_PARAM_INDEX_OUT_OF_RANGE;
	}

	sys_param_list[index].float_val = val;

	return SYS_PARAM_SUCCEED;
}
