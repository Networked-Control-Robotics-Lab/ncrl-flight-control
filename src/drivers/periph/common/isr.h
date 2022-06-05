#ifndef __ISR_H__
#define __ISR_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* isr priority list (lower value has higher priority)  */
#define SYS_TIMER_PRIORITY       (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 0)

#define IMU_EXTI_PRIORITY        (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)

#define BAROMETER_PRIORITY       (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2)
#define SW_I2C_TIMER_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2)

#define GPS_VICON_UART_PRIORITY  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3)
#define SLAM_UART_PRIORITY       (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3)

#define SBUS_UART_PRIORITY       (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 4)

#define MAVLINK_UART_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 5)
#define DEBUG_LINK_UART_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 5)

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);

#endif
