PROJ_ROOT := $(dir $(lastword $(MAKEFILE_LIST)))/../..

SRC+=$(PROJ_ROOT)/drivers/periph/board_v1/uart.c \
	$(PROJ_ROOT)/drivers/periph/board_v1/spi.c \
	$(PROJ_ROOT)/drivers/periph/board_v1/pwm.c \
	$(PROJ_ROOT)/drivers/periph/board_v1/exti.c \
	$(PROJ_ROOT)/drivers/periph/board_v1/gpio.c \
	$(PROJ_ROOT)/drivers/boards/board_v1/board_support.c

CFLAGS+=-I$(PROJ_ROOT)/drivers/periph/board_v1
CFLAGS+=-I$(PROJ_ROOT)/drivers/boards/board_v1
