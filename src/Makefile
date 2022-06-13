EXECUTABLE=firmware.elf

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

CFLAGS+=-Wl,-T,./drivers/boards/common/stm32f427vi_flash.ld

LDFLAGS+=-Wl,--start-group -lm -Wl,--end-group

SRC=

#=================#
# board selection #
#=================#
-include ./drivers/boards/px4.mk       #use px4 2.4.6 board
#-include ./drivers/boards/board_v1.mk #use prototyope board v1

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
	./core/filters/lpf.c \
	./core/filters/median_filter.c \
	./core/state_estimator/misc/free_fall/free_fall.c \
	./core/state_estimator/ahrs/ahrs.c \
	./core/state_estimator/ahrs/ahrs_comp_filter.c \
	./core/state_estimator/ahrs/ahrs_madgwick.c \
	./core/state_estimator/ahrs/ahrs_eskf.c \
	./core/state_estimator/ahrs/ahrs_optitrack.c \
	./core/state_estimator/ins/ins_comp_filter.c \
	./core/state_estimator/ins/geographic_transform.c \
	./core/state_estimator/ins/ins.c \
	./core/state_estimator/ins/ins_eskf.c \
	./core/state_estimator/ins/ins_sensor_sync.c \
	./core/state_estimator/ins/dummy_sensors.c \
	./core/state_estimator/interface/system_state.c \
	./core/state_estimator/sensor_switching/sensor_switching.c \
	./core/controllers/multirotor_pid/multirotor_pid_ctrl.c \
	./core/controllers/multirotor_pid/multirotor_pid_param.c \
	./core/controllers/multirotor_geometry/multirotor_geometry_ctrl.c \
	./core/controllers/multirotor_geometry/multirotor_geometry_param.c \
	./core/controllers/actuator/motor_thrust_fitting.c \
	./core/controllers/autopilot/autopilot.c \
	./core/controllers/autopilot/waypoint_following.c \
	./core/controllers/autopilot/trajectory_following.c \
	./core/controllers/autopilot/takeoff_landing.c \
	./core/controllers/autopilot/fence.c \
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

SRC+=./drivers/interface/imu.c \
	./drivers/interface/barometer.c \
	./drivers/interface/compass.c \
	./drivers/interface/gps.c \
	./drivers/interface/rangefinder.c \
	./drivers/interface/vio.c

CFLAGS+=-I./
CFLAGS+=-I./drivers/boards/common
CFLAGS+=-I./core
CFLAGS+=-I./core/filters
CFLAGS+=-I./core/state_estimator
CFLAGS+=-I./core/state_estimator/ahrs
CFLAGS+=-I./core/state_estimator/ins
CFLAGS+=-I./core/state_estimator/interface
CFLAGS+=-I./core/state_estimator/sensor_switching
CFLAGS+=-I./core/state_estimator/misc/free_fall
CFLAGS+=-I./core/controllers
CFLAGS+=-I./core/controllers/multirotor_pid
CFLAGS+=-I./core/controllers/multirotor_geometry
CFLAGS+=-I./core/controllers/actuator
CFLAGS+=-I./core/controllers/autopilot
CFLAGS+=-I./core/debug_link
CFLAGS+=-I./core/tasks
CFLAGS+=-I./core/mavlink
CFLAGS+=-I./core/shell
CFLAGS+=-I./core/perf
CFLAGS+=-I./core/param
CFLAGS+=-I./core/radio_events
CFLAGS+=-I./core/calibration
CFLAGS+=-I./common
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

STARTUP=./drivers/boards/common/startup_stm32f427.s
STARTUP_OBJ=./drivers/boards/common/startup_stm32f427.s

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
	openocd -f interface/stlink.cfg \
	-f target/stm32f4x.cfg \
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
	astyle --style=linux --indent=tab=8 --exclude=lib --recursive "*.c,*.h"

autopep8:
	autopep8 -i ../tools/serial_plot.py

size:
	$(SIZE)  $(EXECUTABLE)

.PHONY: all clean flash openocd gdbauto autopep8 size
