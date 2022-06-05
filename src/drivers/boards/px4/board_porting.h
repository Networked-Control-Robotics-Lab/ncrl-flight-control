#ifndef __BOARD_PORTING_H__
#define __BOARD_PORTING_H__

#include "uart.h"
#include "spi.h"
#include "ncp5623c.h"

/* hardware portings */
#define MOTOR1 &TIM1->CCR4
#define MOTOR2 &TIM1->CCR3
#define MOTOR3 &TIM1->CCR2
#define MOTOR4 &TIM1->CCR1
#define MOTOR5 &TIM4->CCR2
#define MOTOR6 GPIOD, GPIO_Pin_14 //vins-mono camera triggering

#define EXT_SW GPIOB, GPIO_Pin_14 //XXX: external switch's porting is not complete

#define mavlink_puts uart3_puts
#define mavlink_getc uart3_getc

#define debug_link_puts uart2_puts
#define debug_link_getc uart2_getc

//#define vins_mono_puts  uart_puts

#define gps_puts uart4_puts

#define imu_spi_read_write    spi1_read_write
#define imu_spi_chip_select   spi1_chip_select
#define imu_spi_chip_deselect spi1_chip_deselect

#define barometer_spi_read_write    spi3_read_write //XXX: barometer's porting is not complete
#define barometer_spi_chip_select   spi3_chip_select
#define barometer_spi_chip_deselect spi3_chip_deselect

#define led_control ncp5623c_led_control
#define led_toggle  ncp5623c_led_toggle

void board_init(void);

void motor_init(void);
void motor_halt(void);

void camera_trigger_gpio_on(void);
void camera_trigger_gpio_off(void);

#endif
