#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "flash.h"
#include "uart.h"

SemaphoreHandle_t flash_write_semphr;

uint32_t get_flash_sector_num(uint32_t addr)
{
	uint32_t sector = 0;

	if((addr < ADDR_FLASH_SECTOR_1) && (addr >= ADDR_FLASH_SECTOR_0)) {
		sector = FLASH_Sector_0;
	} else if((addr < ADDR_FLASH_SECTOR_2) && (addr >= ADDR_FLASH_SECTOR_1)) {
		sector = FLASH_Sector_1;
	} else if((addr < ADDR_FLASH_SECTOR_3) && (addr >= ADDR_FLASH_SECTOR_2)) {
		sector = FLASH_Sector_2;
	} else if((addr < ADDR_FLASH_SECTOR_4) && (addr >= ADDR_FLASH_SECTOR_3)) {
		sector = FLASH_Sector_3;
	} else if((addr < ADDR_FLASH_SECTOR_5) && (addr >= ADDR_FLASH_SECTOR_4)) {
		sector = FLASH_Sector_4;
	} else if((addr < ADDR_FLASH_SECTOR_6) && (addr >= ADDR_FLASH_SECTOR_5)) {
		sector = FLASH_Sector_5;
	} else if((addr < ADDR_FLASH_SECTOR_7) && (addr >= ADDR_FLASH_SECTOR_6)) {
		sector = FLASH_Sector_6;
	} else if((addr < ADDR_FLASH_SECTOR_8) && (addr >= ADDR_FLASH_SECTOR_7)) {
		sector = FLASH_Sector_7;
	} else if((addr < ADDR_FLASH_SECTOR_9) && (addr >= ADDR_FLASH_SECTOR_8)) {
		sector = FLASH_Sector_8;
	} else if((addr < ADDR_FLASH_SECTOR_10) && (addr >= ADDR_FLASH_SECTOR_9)) {
		sector = FLASH_Sector_9;
	} else if((addr < ADDR_FLASH_SECTOR_11) && (addr >= ADDR_FLASH_SECTOR_10)) {
		sector = FLASH_Sector_10;
	} else { /* (addr < FLASH_END_ADDR) && (addr >= ADDR_FLASH_SECTOR_11) */
		sector = FLASH_Sector_11;
	}

	return sector;
}

void flash_init(void)
{
	flash_write_semphr = xSemaphoreCreateBinary();
	xSemaphoreGive(flash_write_semphr);
}

int flash_write(uint32_t start_addr, uint32_t *data_arr, volatile int size)
{
	xSemaphoreTake(flash_write_semphr, portMAX_DELAY); //critical section start

	FLASH_Unlock();

	volatile uint32_t end_addr = start_addr + size;

	volatile uint32_t start_sector = get_flash_sector_num(start_addr);
	volatile uint32_t end_sector = get_flash_sector_num(end_addr);

	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
	                FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	int i;

	/* erase flash sector */
	for(i = start_sector; i <= end_sector; i += 8) {
		//voltage range 3 (2.7V ~ 3.6V) = word writting mode
		while(FLASH_EraseSector(i, VoltageRange_3) != FLASH_COMPLETE) {
			//XXX: timeout
		}
	}

	/* write data into flash sector */
	for(i = 0; i < size; i++) {
		volatile uint32_t write_addr = start_addr + (i * 4);
		while(FLASH_ProgramWord(write_addr, data_arr[i]) != FLASH_COMPLETE) {
			//XXX: timeout
		}
	}

	/* verify the written data */
	uint32_t data_in_flash;
	for(i = 0; i < size; i++) {
		uint32_t read_addr = start_addr + (i * 4);
		data_in_flash = *(uint32_t *)read_addr;

		if(data_in_flash != data_arr[i]) {
			FLASH_Lock();
			return FLASH_WR_DATA_INCORRECT;
		}
	}

	FLASH_Lock();

	xSemaphoreGive(flash_write_semphr); //critical section end

	return FLASH_WR_SUCCEED;
}

void debug_print_flash(uint32_t start_address, int size)
{
	uint32_t data_read;

	char s[50] = {0};

	int i;
	for(i = 0; i < size; i++) {
		uint32_t read_addr = start_address + (i * 4);
		data_read = *(uint32_t *)read_addr;

		sprintf(s, "%p: %lu\n\r", (uint32_t *)read_addr, data_read);
		uart3_puts(s, strlen(s));
	}
}
