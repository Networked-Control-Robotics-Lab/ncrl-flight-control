PROJ_ROOT := $(dir $(lastword $(MAKEFILE_LIST)))/../..

SRC+=$(PROJ_ROOT)/drivers/periph/board_v1/sw_i2c.c

CFLAGS+=-I$(PROJ_ROOT)/drivers/periph/board_v1
