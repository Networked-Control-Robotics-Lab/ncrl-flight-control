#ifndef __BOARD_PORTING_H__
#define __BOARD_PORTING_H__

#include "uart.h"
#include "spi.h"
#include "gpio.h"

/* hardware portings */
#define MOTOR1 &TIM4->CCR1
#define MOTOR2 &TIM4->CCR2
#define MOTOR3 &TIM1->CCR4
#define MOTOR4 &TIM1->CCR3
#define MOTOR5 &TIM4->CCR3
#define MOTOR6 &TIM4->CCR4
#define MOTOR8 GPIOE, GPIO_Pin_9  //vins-mono trigger

#define EXT_SW GPIOB, GPIO_Pin_14 //external switch

#define LED_R GPIOA, GPIO_Pin_2
#define LED_G GPIOA, GPIO_Pin_0
#define LED_B GPIOA, GPIO_Pin_3

#define mavlink_puts uart3_puts
#define mavlink_getc uart3_getc

#define debug_link_puts uart1_puts
#define debug_link_getc uart1_getc

#define vins_mono_puts  uart6_puts

#define gps_puts uart7_puts

#define imu_spi_read_write    spi1_read_write
#define imu_spi_chip_select   spi1_chip_select
#define imu_spi_chip_deselect spi1_chip_deselect

#define barometer_spi_read_write    spi3_read_write
#define barometer_spi_chip_select   spi3_chip_select
#define barometer_spi_chip_deselect spi3_chip_deselect

#define led_control gpio_led_control
#define led_toggle  gpio_led_toggle

void motor_init(void);
void motor_halt(void);

void camera_trigger_gpio_on(void);
void camera_trigger_gpio_off(void);

#endif
