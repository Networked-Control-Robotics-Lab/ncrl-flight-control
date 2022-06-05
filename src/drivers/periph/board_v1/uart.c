#include <stdio.h>
#include <unistd.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "stm32f4xx_conf.h"
#include "isr.h"
#include "sbus_radio.h"
#include "optitrack.h"
#include "vins_mono.h"
#include "ublox_m8n.h"
#include "proj_config.h"

#define UART1_QUEUE_SIZE 100
#define UART3_QUEUE_SIZE 500

typedef struct {
	char c;
} uart_c_t;

SemaphoreHandle_t uart1_tx_semphr;
SemaphoreHandle_t uart3_tx_semphr;
SemaphoreHandle_t uart7_tx_semphr;

QueueHandle_t uart1_rx_queue;
QueueHandle_t uart3_rx_queue;

/*
 * <uart1>
 * usage: log
 * tx: gpio_pin_a9 (dma2 channel4 stream7)
 * rx: gpio_pin_a10 (dma2 channel4 stream2)
 */
void uart1_init(int baudrate)
{
	uart1_tx_semphr = xSemaphoreCreateBinary();
	uart1_rx_queue = xQueueCreate(UART1_QUEUE_SIZE, sizeof(uart_c_t));

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	USART_InitTypeDef USART_InitStruct = {
		.USART_BaudRate = baudrate,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
		.USART_WordLength = USART_WordLength_8b,
		.USART_StopBits = USART_StopBits_1,
		.USART_Parity = USART_Parity_No
	};
	USART_Init(USART1, &USART_InitStruct);
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = DMA2_Stream7_IRQn,
		.NVIC_IRQChannelPreemptionPriority = DEBUG_LINK_UART_PRIORITY,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = DEBUG_LINK_UART_PRIORITY;
	NVIC_Init(&NVIC_InitStruct);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/*
 * <uart3>
 * usage: telecommunication
 * tx: gpio_pin_d8 (dma1 channel4 stream3)
 * rx: gpio_pin_d9 (dma1 channel4 stream4)
 */
void uart3_init(int baudrate)
{
	uart3_tx_semphr = xSemaphoreCreateBinary();
	uart3_rx_queue = xQueueCreate(UART3_QUEUE_SIZE, sizeof(uart_c_t));

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	USART_InitTypeDef USART_InitStruct = {
		.USART_BaudRate = baudrate,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
		.USART_WordLength = USART_WordLength_8b,
		.USART_StopBits = USART_StopBits_1,
		.USART_Parity = USART_Parity_No
	};
	USART_Init(USART3, &USART_InitStruct);
	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_TC);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = DMA1_Stream3_IRQn,
		.NVIC_IRQChannelPreemptionPriority = MAVLINK_UART_PRIORITY,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = MAVLINK_UART_PRIORITY;
	NVIC_Init(&NVIC_InitStruct);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/*
 * <uart4>
 * usage: s-bus
 * rx: gpio_pin_c11
 */
void uart4_init(int baudrate)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_11,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	USART_InitTypeDef USART_InitStruct = {
		.USART_BaudRate = baudrate,
		.USART_Mode = USART_Mode_Rx,
		.USART_WordLength = USART_WordLength_8b,
		.USART_StopBits = USART_StopBits_2,
		.USART_Parity = USART_Parity_Even,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None
	};
	USART_Init(UART4, &USART_InitStruct);
	USART_Cmd(UART4, ENABLE);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = UART4_IRQn,
		.NVIC_IRQChannelPreemptionPriority = SBUS_UART_PRIORITY,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
}

/*
 * <uart6>
 * usage: cam
 * tx: gpio_pin_c6 (dma2 channel5 stream6)
 * rx: gpio_pin_c7 (dma2 channel5 stream2)
 */
void uart6_init(int baudrate)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	USART_InitTypeDef USART_InitStruct = {
		.USART_BaudRate = baudrate,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
		.USART_WordLength = USART_WordLength_8b,
		.USART_StopBits = USART_StopBits_1,
		.USART_Parity = USART_Parity_No
	};
	USART_Init(USART6, &USART_InitStruct);
	USART_Cmd(USART6, ENABLE);
	USART_ClearFlag(USART6, USART_FLAG_TC);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = USART6_IRQn,
		.NVIC_IRQChannelPreemptionPriority = SLAM_UART_PRIORITY,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}

/*
 * <uart7>
 * usage: telecommunication
 * tx: gpio_pin_e8
 * rx: gpio_pin_e7
 */
void uart7_init(int baudrate)
{
	uart7_tx_semphr = xSemaphoreCreateBinary();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	USART_InitTypeDef USART_InitStruct = {
		.USART_BaudRate = baudrate,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
		.USART_WordLength = USART_WordLength_8b,
		.USART_StopBits = USART_StopBits_1,
		.USART_Parity = USART_Parity_No
	};
	USART_Init(UART7, &USART_InitStruct);
	USART_Cmd(UART7, ENABLE);
	USART_ClearFlag(UART7, USART_FLAG_TC);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = DMA1_Stream1_IRQn,
		.NVIC_IRQChannelPreemptionPriority = GPS_VICON_UART_PRIORITY,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);
	DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = UART7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = GPS_VICON_UART_PRIORITY;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);
}

void uart_putc(USART_TypeDef *uart, char c)
{
	while(USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
	USART_SendData(uart, c);
	while(USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET);
}

char uart_getc(USART_TypeDef *uart)
{
	while(USART_GetFlagStatus(uart, USART_FLAG_RXNE) == RESET);
	return USART_ReceiveData(uart);
}

void usart_puts(USART_TypeDef *uart, char *s, int size)
{
	int i;
	for(i = 0; i < size; i++) {
		uart_putc(uart, s[i]);
	}
}

void uart1_puts(char *s, int size)
{
	//uart1 tx: dma2 channel4 stream7
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);

	DMA_InitTypeDef DMA_InitStructure = {
		.DMA_BufferSize = (uint32_t)size,
		.DMA_FIFOMode = DMA_FIFOMode_Disable,
		.DMA_FIFOThreshold = DMA_FIFOThreshold_Full,
		.DMA_MemoryBurst = DMA_MemoryBurst_Single,
		.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
		.DMA_MemoryInc = DMA_MemoryInc_Enable,
		.DMA_Mode = DMA_Mode_Normal,
		.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR),
		.DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
		.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
		.DMA_Priority = DMA_Priority_Medium,
		.DMA_Channel = DMA_Channel_4,
		.DMA_DIR = DMA_DIR_MemoryToPeripheral,
		.DMA_Memory0BaseAddr = (uint32_t)s
	};
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

	//send data from memory to uart data register
	DMA_Cmd(DMA2_Stream7, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

	xSemaphoreTake(uart1_tx_semphr, portMAX_DELAY);
}

void uart3_puts(char *s, int size)
{
	//uart3 tx: dma1 channel4 stream3
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);

	DMA_InitTypeDef DMA_InitStructure = {
		.DMA_BufferSize = (uint32_t)size,
		.DMA_FIFOMode = DMA_FIFOMode_Disable,
		.DMA_FIFOThreshold = DMA_FIFOThreshold_Full,
		.DMA_MemoryBurst = DMA_MemoryBurst_Single,
		.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
		.DMA_MemoryInc = DMA_MemoryInc_Enable,
		.DMA_Mode = DMA_Mode_Normal,
		.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR),
		.DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
		.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
		.DMA_Priority = DMA_Priority_Medium,
		.DMA_Channel = DMA_Channel_4,
		.DMA_DIR = DMA_DIR_MemoryToPeripheral,
		.DMA_Memory0BaseAddr = (uint32_t)s
	};
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);

	//send data from memory to uart data register
	DMA_Cmd(DMA1_Stream3, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

	xSemaphoreTake(uart3_tx_semphr, portMAX_DELAY);
}

//specially designed puts function of uart6 for vins-mono
void uart6_puts(char *s, int size)
{
	static bool uart6_tx_busy = false;

	if(uart6_tx_busy == true && DMA_GetFlagStatus(DMA2_Stream6, DMA_FLAG_TCIF6) == RESET) {
		return;
	} else {
		uart6_tx_busy = false;
	}

	static uint8_t uart6_buf[100];
	memcpy(uart6_buf, s, size);

	//uart6 tx: dma2 channel5 stream6
	DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6);

	DMA_InitTypeDef DMA_InitStructure = {
		.DMA_BufferSize = (uint32_t)size,
		.DMA_FIFOMode = DMA_FIFOMode_Disable,
		.DMA_FIFOThreshold = DMA_FIFOThreshold_Full,
		.DMA_MemoryBurst = DMA_MemoryBurst_Single,
		.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
		.DMA_MemoryInc = DMA_MemoryInc_Enable,
		.DMA_Mode = DMA_Mode_Normal,
		.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR),
		.DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
		.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
		.DMA_Priority = DMA_Priority_Medium,
		.DMA_Channel = DMA_Channel_5,
		.DMA_DIR = DMA_DIR_MemoryToPeripheral,
		.DMA_Memory0BaseAddr = (uint32_t)uart6_buf
	};
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);

	//send data from memory to uart data register
	DMA_Cmd(DMA2_Stream6, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);

	uart6_tx_busy = true;

	//while(DMA_GetFlagStatus(DMA2_Stream6, DMA_FLAG_TCIF6) == RESET);
}

void uart7_puts(char *s, int size)
{
#if 0
	//uart7 tx: dma1 channel5 stream1
	DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF5);

	DMA_InitTypeDef DMA_InitStructure = {
		.DMA_BufferSize = (uint32_t)size,
		.DMA_FIFOMode = DMA_FIFOMode_Disable,
		.DMA_FIFOThreshold = DMA_FIFOThreshold_Full,
		.DMA_MemoryBurst = DMA_MemoryBurst_Single,
		.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
		.DMA_MemoryInc = DMA_MemoryInc_Enable,
		.DMA_Mode = DMA_Mode_Normal,
		.DMA_PeripheralBaseAddr = (uint32_t)(&UART7->DR),
		.DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
		.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
		.DMA_Priority = DMA_Priority_Medium,
		.DMA_Channel = DMA_Channel_5,
		.DMA_DIR = DMA_DIR_MemoryToPeripheral,
		.DMA_Memory0BaseAddr = (uint32_t)s
	};
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);

	//send data from memory to uart data register
	DMA_Cmd(DMA1_Stream1, ENABLE);
	USART_DMACmd(UART7, USART_DMAReq_Tx, ENABLE);

	xSemaphoreTake(uart7_tx_semphr, portMAX_DELAY);
#else
	usart_puts(UART7, s, size);
#endif
}

bool uart1_getc(char *c, long sleep_ticks)
{
	uart_c_t recpt_c;
	if(xQueueReceive(uart1_rx_queue, &recpt_c, sleep_ticks) == pdFALSE) {
		return false;
	} else {
		*c = recpt_c.c;
		return true;
	}
}

bool uart3_getc(char *c, long sleep_ticks)
{
	uart_c_t recpt_c;
	if(xQueueReceive(uart3_rx_queue, &recpt_c, sleep_ticks) == pdFALSE) {
		return false;
	} else {
		*c = recpt_c.c;
		return true;
	}
}

void DMA1_Stream1_IRQHandler(void)
{
	/* uart7 tx dma */
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1) == SET) {
		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);

		BaseType_t higher_priority_task_woken = pdFALSE;
		xSemaphoreGiveFromISR(uart7_tx_semphr, &higher_priority_task_woken);
		portEND_SWITCHING_ISR(higher_priority_task_woken);
	}
}

void DMA1_Stream3_IRQHandler(void)
{
	/* uart3 tx dma */
	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) == SET) {
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);

		BaseType_t higher_priority_task_woken = pdFALSE;
		xSemaphoreGiveFromISR(uart3_tx_semphr, &higher_priority_task_woken);
		portEND_SWITCHING_ISR(higher_priority_task_woken);
	}
}

void DMA2_Stream7_IRQHandler(void)
{
	/* uart1 tx dma */
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) == SET) {
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);

		BaseType_t higher_priority_task_woken = pdFALSE;
		xSemaphoreGiveFromISR(uart1_tx_semphr, &higher_priority_task_woken);
		portEND_SWITCHING_ISR(higher_priority_task_woken);
	}
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
		uart_c_t uart_queue_item;
		uart_queue_item.c = USART_ReceiveData(USART1);
		USART1->SR;

		BaseType_t higher_priority_task_woken = pdFALSE;
		xQueueSendToBackFromISR(uart1_rx_queue, &uart_queue_item, &higher_priority_task_woken);
		portEND_SWITCHING_ISR(higher_priority_task_woken)
	}
}

void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {
		uart_c_t uart_queue_item;
		uart_queue_item.c = USART_ReceiveData(USART3);
		USART3->SR;

		BaseType_t higher_priority_task_woken = pdFALSE;
		xQueueSendToBackFromISR(uart3_rx_queue, &uart_queue_item, &higher_priority_task_woken);
		portEND_SWITCHING_ISR(higher_priority_task_woken);
	}
}

void UART4_IRQHandler(void)
{
	/* using uart4 rxne interrupt to receive and parse
	   sbus message */
	uint8_t c;

	if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET) {
		c = USART_ReceiveData(UART4);
		UART4->SR;

		sbus_rc_isr_handler(c);
	}
}

void USART6_IRQHandler(void)
{
	uint8_t c;
	if(USART_GetITStatus(USART6, USART_IT_RXNE) == SET) {
		c = USART_ReceiveData(USART6);
		USART6->SR;
		vins_mono_isr_handler(c);
	}
}

void UART7_IRQHandler(void)
{
	uint8_t c;
	if(USART_GetITStatus(UART7, USART_IT_RXNE) == SET) {
		c = USART_ReceiveData(UART7);
		UART7->SR;

#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_GPS)
		ublox_m8n_isr_handler(c);
#elif (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK)
		optitrack_isr_handler(c);
#else
		(void)c; //prevent from unused variable warning
#endif
	}
}
