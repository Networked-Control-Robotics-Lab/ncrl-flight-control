EXECUTABLE=ncrl_fc.elf

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
GDB=arm-none-eabi-gdb

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS+=--specs=nano.specs --specs=nosys.specs -fno-builtin-printf -u _printf_float
CFLAGS+=-Wall -fno-strict-aliasing
CFLAGS+=-D USE_STDPERIPH_DRIVER \
	-D STM32F427xx \
	-D STM32F427_437xx \
	-D ARM_MATH_CM4 \
	-D __FPU_PRESENT=1 \
	#-D __FPU_USED=1

CFLAGS+=-Wl,-T,./sys_startup/stm32f427vi_flash.ld

LDFLAGS+=-Wl,--start-group -lm -Wl,--end-group

SRC=

SRC+=./sys_startup/system_stm32f4xx.c

SRC+=lib/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.c \
	lib/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.c \
	lib/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.c \
	lib/CMSIS/DSP_Lib/Source/StatisticsFunctions/arm_power_f32.c \
	lib/CMSIS/DSP_Lib/Source/StatisticsFunctions/arm_max_f32.c \
	lib/CMSIS/DSP_Lib/Source/BasicMathFunctions/arm_sub_f32.c \
	lib/CMSIS/DSP_Lib/Source/SupportFunctions/arm_copy_f32.c \
	lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_init_f32.c \
	lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_scale_f32.c \
	lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_add_f32.c \
	lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_sub_f32.c \
	lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_mult_f32.c \
	lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_trans_f32.c \
	lib/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_inverse_f32.c

SRC+=./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c\
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c \
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c \
	./lib/STM32F4xx_StdPeriph_Driver/src/misc.c

SRC+=lib/FreeRTOS/Source/croutine.c \
	lib/FreeRTOS/Source/event_groups.c \
	lib/FreeRTOS/Source/list.c \
	lib/FreeRTOS/Source/queue.c \
	lib/FreeRTOS/Source/tasks.c \
	lib/FreeRTOS/Source/timers.c \
	lib/FreeRTOS/Source/portable/MemMang/heap_4.c \
	lib/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \

SRC+=./core/main.c \
	./core/flight_ctl.c \
	./core/sys_time.c \
	./core/lpf.c

SRC+=./common/delay.c \
	./common/bound.c

SRC+=./driver/periph/led.c \
	./driver/periph/uart.c \
	./driver/periph/spi.c \
	./driver/periph/pwm.c \
	./driver/periph/timer.c \
	./driver/periph/isr.c \
	./driver/device/mpu6500.c \
	./driver/device/hmc5983.c \
	./driver/device/sbus_receiver.c \
	./driver/device/motor.c

CFLAGS+=-I./
CFLAGS+=-I./sys_startup
CFLAGS+=-I./core
CFLAGS+=-I./common
CFLAGS+=-I./driver/periph
CFLAGS+=-I./driver/device
CFLAGS+=-I./lib/CMSIS
CFLAGS+=-I./lib/CMSIS/Include
CFLAGS+=-I./lib/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS+=-I./lib/STM32F4xx_StdPeriph_Driver/inc
CFLAGS+=-I./lib/FreeRTOS/Source/include
CFLAGS+=-I./lib/FreeRTOS/Source/portable/GCC/ARM_CM4F

OBJS=$(SRC:.c=.o)

STARTUP=./sys_startup/startup_stm32f427.s
STARTUP_OBJ=./sys_startup/startup_stm32f427.s

all:$(EXECUTABLE)

$(EXECUTABLE): $(STARTUP_OBJ) $(OBJS)
	@echo "LD" $@
	@$(CC) $(CFLAGS) $(OBJS) $(STARTUP_OBJ) $(LDFLAGS) -o $@

%.o: %.s 
	@echo "CC" $@
	@$(CC) $(CFLAGS) $^ $(LDFLAGS) -c $<

%.o: %.c
	@echo "CC" $@
	@$(CC) $(CFLAGS) -c $< $(LDFLAGS) -o $@

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(OBJS)
	rm -rf *.orig

flash:
	openocd -f interface/stlink-v2.cfg \
	-f target/stm32f4x_stlink.cfg \
	-c "init" \
	-c "reset init" \
	-c "halt" \
	-c "flash write_image erase $(EXECUTABLE)" \
	-c "verify_image $(EXECUTABLE)" \
	-c "reset run" -c shutdown

openocd:
	openocd -s /opt/openocd/share/openocd/scripts/ -f ./gdb/openocd.cfg

gdbauto:
	cgdb -d $(GDB) -x ./gdb/openocd_gdb.gdb

astyle:
	astyle -r --exclude=lib --exclude=sys_startup --style=linux --suffix=none --indent=tab=8  *.c *.h

.PHONY:all clean flash openocd gdbauto
