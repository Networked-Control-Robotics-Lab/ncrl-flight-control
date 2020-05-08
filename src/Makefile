EXECUTABLE=ncrl_fc.elf

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
GDB=arm-none-eabi-gdb

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS+=--specs=nano.specs --specs=nosys.specs -fno-builtin-printf -u _printf_float
CFLAGS+=-Wall -Wno-address-of-packed-member -fno-strict-aliasing
CFLAGS+=-D USE_STDPERIPH_DRIVER \
	-D STM32F427xx \
	-D STM32F427_437xx \
	-D ARM_MATH_CM4 \
	-D __FPU_PRESENT=1 \
	#-D __FPU_USED=1

CFLAGS+=-Wl,-T,./platform/stm32f427vi_flash.ld

LDFLAGS+=-Wl,--start-group -lm -Wl,--end-group

SRC=

SRC+=./platform/system_stm32f4xx.c

SRC+=lib/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.c \
	lib/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.c \
	lib/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.c \
	lib/CMSIS/DSP_Lib/Source/StatisticsFunctions/arm_power_f32.c \
	lib/CMSIS/DSP_Lib/Source/StatisticsFunctions/arm_max_f32.c \
	lib/CMSIS/DSP_Lib/Source/BasicMathFunctions/arm_sub_f32.c \
	lib/CMSIS/DSP_Lib/Source/BasicMathFunctions/arm_dot_prod_f32.c \
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
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
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
	./core/estimators/lpf.c \
	./core/estimators/ahrs.c \
	./core/estimators/comp_ahrs.c \
	./core/estimators/madgwick_ahrs.c \
	./core/estimators/free_fall.c \
	./core/controllers/multirotor_pid_ctrl.c \
	./core/controllers/multirotor_geometry_ctrl.c \
	./core/controllers/motor_thrust.c \
	./core/controllers/autopilot.c \
	./core/tasks/fc_task.c \
	./core/tasks/mavlink_task.c \
	./core/tasks/debug_link_task.c \
	./core/tasks/shell_task.c \
	./core/tasks/compass_task.c \
	./core/shell/quadshell.c \
	./core/shell/shell_cmds.c \
	./core/debug_link/debug_link.c \
	./core/mavlink/publisher.c \
	./core/mavlink/parser.c \
	./core/mavlink/mission.c \
	./core/mavlink/param.c \
	./core/mavlink/trajectory.c \
	./core/perf/perf.c \
	./core/config/sys_param.c

SRC+=./common/delay.c \
	./common/bound.c \
	./common/matrix.c \
	./common/se3_math.c \
	./common/quaternion.c \
	./common/polynomial.c

SRC+=./driver/periph/uart.c \
	./driver/periph/spi.c \
	./driver/periph/pwm.c \
	./driver/periph/timer.c \
	./driver/periph/isr.c \
	./driver/periph/exti.c \
	./driver/periph/gpio.c \
	./driver/periph/sw_i2c.c \
	./driver/device/mpu6500.c \
	./driver/device/ms5611.c \
	./driver/device/sbus_receiver.c \
	./driver/device/motor.c \
	./driver/device/optitrack.c \
	./driver/device/sys_time.c \
	./driver/device/ist8310.c \
	./driver/device/led.c

CFLAGS+=-I./
CFLAGS+=-I./platform
CFLAGS+=-I./core
CFLAGS+=-I./core/estimators
CFLAGS+=-I./core/controllers
CFLAGS+=-I./core/debug_link
CFLAGS+=-I./core/tasks
CFLAGS+=-I./core/mavlink
CFLAGS+=-I./core/shell
CFLAGS+=-I./core/perf
CFLAGS+=-I./core/config
CFLAGS+=-I./common
CFLAGS+=-I./driver/periph
CFLAGS+=-I./driver/device
CFLAGS+=-I./lib/CMSIS
CFLAGS+=-I./lib/CMSIS/Include
CFLAGS+=-I./lib/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS+=-I./lib/STM32F4xx_StdPeriph_Driver/inc
CFLAGS+=-I./lib/FreeRTOS/Source/include
CFLAGS+=-I./lib/FreeRTOS/Source/portable/GCC/ARM_CM4F
CFLAGS+=-I./lib/mavlink_v2/common
CFLAGS+=-I./lib/mavlink_v2/ncrl_mavlink

OBJS=$(SRC:.c=.o)

STARTUP=./platform/startup_stm32f427.s
STARTUP_OBJ=./platform/startup_stm32f427.s

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
	astyle -r --exclude=lib --exclude=platform --style=linux --suffix=none --indent=tab=8  *.c *.h

.PHONY:all clean flash openocd gdbauto
