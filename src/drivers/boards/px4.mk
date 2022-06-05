PROJ_ROOT := $(dir $(lastword $(MAKEFILE_LIST)))/../..

CFLAGS+=-DSELECT_BOARD=BOARD_PX4_V246

SRC+=$(PROJ_ROOT)/drivers/periph/px4/uart.c \
	$(PROJ_ROOT)/drivers/periph/px4/spi.c \
	$(PROJ_ROOT)/drivers/periph/px4/pwm.c \
	$(PROJ_ROOT)/drivers/periph/px4/exti.c \
	$(PROJ_ROOT)/drivers/periph/px4/gpio.c \
	$(PROJ_ROOT)/drivers/periph/px4/i2c.c \
	$(PROJ_ROOT)/drivers/device/ncp5623c.c \
	$(PROJ_ROOT)/drivers/boards/px4/board_support.c

CFLAGS+=-I$(PROJ_ROOT)/drivers/periph/px4
CFLAGS+=-I$(PROJ_ROOT)/drivers/boards/px4
