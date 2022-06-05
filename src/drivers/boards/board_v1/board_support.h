#ifndef __BOARD_SUPPORT_H__
#define __BOARD_SUPPORT_H__

#include "uart.h"
#include "spi.h"

/* function ports */
#define mavlink_puts    uart3_puts
#define mavlink_getc    uart3_getc

#define debug_link_puts uart1_puts
#define debug_link_getc uart1_getc

#define vins_mono_puts  uart6_puts

#define imu_spi_read_write    spi1_read_write
#define imu_spi_chip_select   spi1_chip_select
#define imu_spi_chip_deselect spi1_chip_deselect

#define barometer_spi_read_write    spi3_read_write
#define barometer_spi_chip_select   spi3_chip_select
#define barometer_spi_chip_deselect spi3_chip_deselect

void board_init(void);

#endif
