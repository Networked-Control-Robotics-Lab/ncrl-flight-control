#ifndef __SYS_PARAM_H__
#define __SYS_PARAM_H__

#include <stdint.h>

#define DEF_SYS_PARAM_ID(id) id

#define SYS_PARAM_ATTR(_name, _type) \
	{.name = _name, .type = _type}

#define SYS_PARAM_TYPE_TO_STRING(type) #type

#define SIZE_OF_PARAM_LIST(list) (sizeof(list) / sizeof(sys_param_data))

#define INIT_SYS_PARAM_U8(id, val) init_sys_param_u8(id, #id, val)
#define INIT_SYS_PARAM_S8(id, val) init_sys_param_s8(id, #id, val)
#define INIT_SYS_PARAM_U16(id, val) init_sys_param_u16(id, #id, val)
#define INIT_SYS_PARAM_S16(id, val) init_sys_param_s16(id, #id, val)
#define INIT_SYS_PARAM_U32(id, val) init_sys_param_u32(id, #id, val)
#define INIT_SYS_PARAM_S32(id, val) init_sys_param_s32(id, #id, val)
#define INIT_SYS_PARAM_FLOAT(id, val) init_sys_param_float(id, #id, val)
#define INIT_SYS_PARAM_FLOAT(id, val) init_sys_param_float(id, #id, val)

enum {
	SYS_PARAM_SUCCEED = 0,
	SYS_PARAM_INDEX_OUT_OF_RANGE = 1
} SYS_PARARM_RETVAL;

enum {
	/* compatible with MAV_PARAM_TYPE (mavlink enum) */
	SYS_PARAM_U8 = 1,
	SYS_PARAM_S8 = 2,
	SYS_PARAM_U16 = 3,
	SYS_PARAM_S16 = 4,
	SYS_PARAM_U32 = 5,
	SYS_PARAM_S32 = 6,
	SYS_PARAM_FLOAT = 9
	                  /* not supporting uint64_t, int64_t and double */
} SYS_PARAM_TYPE;

enum {
	SYS_PARAM_FLASH_WR_SUCCEED = 0,
	SYS_PARAM_FLASH_WR_DATA_INCORRECT = 1,
	SYS_PARAM_FLASH_WR_TIMEOUT = 2,
	SYS_PARAM_FLASH_ERASE_TIMEOUT = 3,
	SYS_PARAM_FLASH_CRC_INCORRECT = 4,
	SYS_PARAM_FLASH_SIZE_INCORRECT = 5
} SYS_PARAM_FLASH_RETVAL;

typedef union {
	uint8_t u8_val;
	int8_t s8_val;
	uint16_t u16_val;
	int16_t s16_val;
	uint32_t u32_val;
	int32_t s32_val;
	float float_val;
} param_data_t;

typedef struct {
	char *name;
	uint8_t type;
	unsigned long hash;

	void *update_var_ptr;

	param_data_t curr;
	param_data_t _default;
} sys_param_data;

void init_sys_param_list(sys_param_data *list, int _list_size);
void reset_sys_param_list_to_default(void);

int get_sys_param_list_size(void);
int get_sys_param_name(int index, char **name);
int get_sys_param_hash(int index, unsigned long *param_hash);
int get_sys_param_type(int index, uint8_t *type);
int set_sys_param_update_var_addr(int index, void *var_addr);

int init_sys_param_u8(int index, char *name, uint8_t val);
int init_sys_param_s8(int index, char *name, int8_t val);
int init_sys_param_u16(int index, char *name, uint16_t val);
int init_sys_param_s16(int index, char *name, int16_t val);
int init_sys_param_u32(int index, char *name, uint32_t val);
int init_sys_param_s32(int index, char *name, int32_t val);
int init_sys_param_float(int index, char *name, float val);

int get_sys_param_u8(int index, uint8_t *retval);
int get_sys_param_s8(int index, int8_t *retval);
int get_sys_param_u16(int index, uint16_t *retval);
int get_sys_param_s16(int index, int16_t *retval);
int get_sys_param_u32(int index, uint32_t *retval);
int get_sys_param_s32(int index, int32_t *retval);
int get_sys_param_float(int index, float *retval);

int set_sys_param_u8(int index, uint8_t val);
int set_sys_param_s8(int index, int8_t val);
int set_sys_param_u16(int index, uint16_t val);
int set_sys_param_s16(int index, int16_t val);
int set_sys_param_u32(int index, uint32_t val);
int set_sys_param_s32(int index, int32_t val);
int set_sys_param_float(int index, float val);

int save_param_list_to_flash(void);
int load_param_list_from_flash(void);

#endif
