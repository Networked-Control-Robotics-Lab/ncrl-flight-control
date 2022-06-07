PROJ_ROOT := $(dir $(lastword $(MAKEFILE_LIST)))/../..

-include $(PROJ_ROOT)/drivers/boards/board_common.mk

#prototype board v1

CFLAGS+=-DSELECT_BOARD=BOARD_PROTOTYPE_V1 \
	-DSELECT_MPU_IMU=USE_MPU6500

#peripherial driver implementation for this board
SRC+=$(PROJ_ROOT)/drivers/periph/board_v1/uart.c \
	$(PROJ_ROOT)/drivers/periph/board_v1/spi.c \
	$(PROJ_ROOT)/drivers/periph/board_v1/pwm.c \
	$(PROJ_ROOT)/drivers/periph/board_v1/exti.c \
	$(PROJ_ROOT)/drivers/periph/board_v1/gpio.c \

#all devices supported by this board
SRC+=$(PROJ_ROOT)/drivers/device/mpu6500.c \
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
SRC+=$(PROJ_ROOT)/drivers/boards/board_v1/board_init.c \
	$(PROJ_ROOT)/drivers/boards/board_v1/board_porting.c \
	$(PROJ_ROOT)/drivers/boards/board_v1/system_stm32f4xx.c

CFLAGS+=-I$(PROJ_ROOT)/drivers/periph/board_v1
CFLAGS+=-I$(PROJ_ROOT)/drivers/boards/board_v1
