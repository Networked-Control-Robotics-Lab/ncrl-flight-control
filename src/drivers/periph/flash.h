#ifndef __FLASH_H__
#define __FLASH_H__

#define ADDR_FLASH_SECTOR_0  ((uint32_t)0x08000000) //base addrress of sector 0, 16KB
#define ADDR_FLASH_SECTOR_1  ((uint32_t)0x08004000) //base addrress of sector 1, 16KB
#define ADDR_FLASH_SECTOR_2  ((uint32_t)0x08008000) //base addrress of sector 2, 16KB
#define ADDR_FLASH_SECTOR_3  ((uint32_t)0x0800C000) //base addrress of sector 3, 16KB
#define ADDR_FLASH_SECTOR_4  ((uint32_t)0x08010000) //base addrress of sector 4, 64KB
#define ADDR_FLASH_SECTOR_5  ((uint32_t)0x08020000) //base addrress of sector 5, 128KB
#define ADDR_FLASH_SECTOR_6  ((uint32_t)0x08040000) //base addrress of sector 6, 128KB
#define ADDR_FLASH_SECTOR_7  ((uint32_t)0x08060000) //base addrress of sector 7, 128KB
#define ADDR_FLASH_SECTOR_8  ((uint32_t)0x08080000) //base addrress of sector 8, 128KB
#define ADDR_FLASH_SECTOR_9  ((uint32_t)0x080A0000) //base addrress of sector 9, 128KB
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) //base addrress of sector 10, 128KB
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) //base addrress of sector 11, 128KB

enum {
	FLASH_WR_SUCCEED = 0,
	FLASH_WR_DATA_INCORRECT = 1,
	FLASH_WR_TIMEOUT = 2,
	FLASH_ERASE_TIMEOUT = 3
} FLASH_RET_VAL;

void flash_init(void);
int flash_write(uint32_t start_addr, uint32_t *data_arr, int size);

void debug_print_flash(uint32_t start_address, int size);

#endif
