#ifndef __ISR_H__
#define __ISR_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* interrupt routine service priority list  */
#define SYS_TIMER_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 0)

#define IMU_EXTI_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)

#define BAROMETER_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2)
#define SW_I2C_TIMER_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2)

#define GPS_OPTITRACK_UART_ISR (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3)
#define GPS_UART_TX_ISR (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3)

#define SBUS_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 4)

#define UART1_TX_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 5)
#define UART1_RX_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 5)
#define UART3_TX_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 5)
#define UART3_RX_ISR_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 5)

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);

#endif
