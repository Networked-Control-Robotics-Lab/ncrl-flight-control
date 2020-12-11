EXECUTABLE=ncrl_fc.elf

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
GDB=arm-none-eabi-gdb
SIZE=arm-none-eabi-size

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
	./lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c \
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
	./core/estimators/filters/lpf.c \
	./core/estimators/free_fall/free_fall.c \
	./core/estimators/attitude/ahrs.c \
	./core/estimators/attitude/comp_ahrs.c \
	./core/estimators/attitude/madgwick_ahrs.c \
	./core/estimators/attitude/eskf_ahrs.c \
	./core/estimators/localization/altitude_est.c \
	./core/estimators/localization/gps_ekf.c \
	./core/controllers/multirotor_pid/multirotor_pid_ctrl.c \
	./core/controllers/multirotor_pid/multirotor_pid_param.c \
	./core/controllers/multirotor_geometry/multirotor_geometry_ctrl.c \
	./core/controllers/multirotor_geometry/multirotor_geometry_param.c \
	./core/controllers/multirotor_geometry/send_debug_adaptive_ICL.c \
	./core/controllers/actuator/motor_thrust_fitting.c \
	./core/controllers/autopilot.c \
	./core/tasks/flight_ctrl_task.c \
	./core/tasks/mavlink_task.c \
	./core/tasks/debug_link_task.c \
	./core/tasks/shell_task.c \
	./core/shell/quadshell.c \
	./core/shell/shell_cmds.c \
	./core/debug_link/debug_link.c \
	./core/debug_link/debug_msg.c \
	./core/mavlink/mav_publisher.c \
	./core/mavlink/mav_parser.c \
	./core/mavlink/mav_mission.c \
	./core/mavlink/mav_param.c \
	./core/mavlink/mav_trajectory.c \
	./core/mavlink/mav_command.c \
	./core/perf/perf.c \
	./core/param/sys_param.c \
	./core/param/common_list.c \
	./core/radio_events/multirotor_rc.c \
	./core/calibration/accel_calibration.c \
	./core/calibration/compass_calibration.c \
	./core/calibration/esc_calibration.c \
	./core/calibration/calibration_task.c

SRC+=./common/delay.c \
	./common/bound.c \
	./common/matrix.c \
	./common/se3_math.c \
	./common/quaternion.c \
	./common/polynomial.c \
	./common/hash.c \
	./common/ellipsoid_least_square.c

SRC+=./drivers/periph/uart.c \
	./drivers/periph/spi.c \
	./drivers/periph/pwm.c \
	./drivers/periph/timer.c \
	./drivers/periph/isr.c \
	./drivers/periph/exti.c \
	./drivers/periph/gpio.c \
	./drivers/periph/sw_i2c.c \
	./drivers/periph/flash.c \
	./drivers/periph/crc.c \
	./drivers/device/mpu6500.c \
	./drivers/device/ms5611.c \
	./drivers/device/sbus_radio.c \
	./drivers/device/motor.c \
	./drivers/device/optitrack.c \
	./drivers/device/sys_time.c \
	./drivers/device/ist8310.c \
	./drivers/device/led.c \
	./drivers/device/ublox_m8n.c \
	./drivers/device/vins_mono.c \
	./drivers/interface/imu.c \
	./drivers/interface/position_sensor.c \
	./drivers/interface/barometer.c \
	./drivers/interface/compass.c

CFLAGS+=-I./
CFLAGS+=-I./platform
CFLAGS+=-I./core
CFLAGS+=-I./core/estimators
CFLAGS+=-I./core/estimators/filters
CFLAGS+=-I./core/estimators/free_fall
CFLAGS+=-I./core/estimators/attitude
CFLAGS+=-I./core/estimators/localization
CFLAGS+=-I./core/controllers
CFLAGS+=-I./core/controllers/multirotor_pid
CFLAGS+=-I./core/controllers/multirotor_geometry
CFLAGS+=-I./core/controllers/actuator
CFLAGS+=-I./core/debug_link
CFLAGS+=-I./core/tasks
CFLAGS+=-I./core/mavlink
CFLAGS+=-I./core/shell
CFLAGS+=-I./core/perf
CFLAGS+=-I./core/param
CFLAGS+=-I./core/radio_events
CFLAGS+=-I./core/calibration
CFLAGS+=-I./common
CFLAGS+=-I./drivers/periph
CFLAGS+=-I./drivers/device
CFLAGS+=-I./drivers/interface
CFLAGS+=-I./lib/CMSIS
CFLAGS+=-I./lib/CMSIS/Include
CFLAGS+=-I./lib/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS+=-I./lib/STM32F4xx_StdPeriph_Driver/inc
CFLAGS+=-I./lib/FreeRTOS/Source/include
CFLAGS+=-I./lib/FreeRTOS/Source/portable/GCC/ARM_CM4F
CFLAGS+=-I./lib/mavlink_v2/common
CFLAGS+=-I./lib/mavlink_v2/ncrl_mavlink

OBJS=$(SRC:.c=.o)
DEPEND=$(SRC:.c=.d)

STARTUP=./platform/startup_stm32f427.s
STARTUP_OBJ=./platform/startup_stm32f427.s

all:$(EXECUTABLE)

$(EXECUTABLE): $(STARTUP_OBJ) $(OBJS)
	@echo "LD" $@
	@$(CC) $(CFLAGS) $(OBJS) $(STARTUP_OBJ) $(LDFLAGS) -o $@

-include $(DEPEND)

%.o: %.s 
	@echo "CC" $@
	@$(CC) $(CFLAGS) $^ $(LDFLAGS) -c $<

%.o: %.c
	@echo "CC" $@
	@$(CC) $(CFLAGS) -MMD -MP -c $< $(LDFLAGS) -o $@

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(OBJS)
	rm -rf $(DEPEND)
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

size:
	$(SIZE)  $(EXECUTABLE)

.PHONY:all clean flash openocd gdbauto

