PROJ_ROOT := $(dir $(lastword $(MAKEFILE_LIST)))/../..

SRC+=$(PROJ_ROOT)/drivers/periph/common/timer.c \
	$(PROJ_ROOT)/drivers/periph/common/isr.c \
	$(PROJ_ROOT)/drivers/periph/common/flash.c \
	$(PROJ_ROOT)/drivers/periph/common/crc.c

CFLAGS+=-I$(PROJ_ROOT)/drivers/periph/common
