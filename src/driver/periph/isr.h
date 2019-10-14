#ifndef __ISR_H__
#define __ISR_H__

/* interrupt routine service priority list  */
#define UART4_PRIORITY 0

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);

#endif
