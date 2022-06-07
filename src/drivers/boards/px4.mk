#pixhawk 2.4.6 board

PROJ_ROOT := $(dir $(lastword $(MAKEFILE_LIST)))/../..

-include $(PROJ_ROOT)/drivers/boards/board_common.mk

CFLAGS+=-DSELECT_BOARD=BOARD_PX4_V246 \
	-DSELECT_MPU_IMU=USE_MPU6000

#peripherial driver implementation for this board
SRC+=$(PROJ_ROOT)/drivers/periph/px4/uart.c \
	$(PROJ_ROOT)/drivers/periph/px4/spi.c \
	$(PROJ_ROOT)/drivers/periph/px4/pwm.c \
	$(PROJ_ROOT)/drivers/periph/px4/exti.c \
	$(PROJ_ROOT)/drivers/periph/px4/gpio.c \
	$(PROJ_ROOT)/drivers/periph/px4/i2c.c \

#all devices supported by this board
SRC+=$(PROJ_ROOT)/drivers/device/ncp5623c.c \
	$(PROJ_ROOT)/drivers/device/mpu6500.c \
	$(PROJ_ROOT)/drivers/device/ms5611.c \
	$(PROJ_ROOT)/drivers/device/sbus_radio.c \
	$(PROJ_ROOT)/drivers/device/motor.c \
	$(PROJ_ROOT)/drivers/device/optitrack.c \
	$(PROJ_ROOT)/drivers/device/sys_time.c \
	$(PROJ_ROOT)/drivers/device/ist8310.c \
	$(PROJ_ROOT)/drivers/device/ublox_m8n.c \
	$(PROJ_ROOT)/drivers/device/vins_mono.c \
	$(PROJ_ROOT)/drivers/device/led.c \
	$(PROJ_ROOT)/drivers/device/lidar_lite.c

#porting of the board
SRC+=$(PROJ_ROOT)/drivers/boards/px4/board_init.c \
	$(PROJ_ROOT)/drivers/boards/px4/board_porting.c \
	$(PROJ_ROOT)/drivers/boards/px4/system_stm32f4xx.c

CFLAGS+=-I$(PROJ_ROOT)/drivers/periph/px4
CFLAGS+=-I$(PROJ_ROOT)/drivers/boards/px4
