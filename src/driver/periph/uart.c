#include <unistd.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx_conf.h"

/*
 * <uart1>
 * usage: log
 * tx: gpio_pin_a9 (dma2 channel4 stream7)
 * rx: gpio_pin_a10 (dma2 channel4 stream2)
 */
void uart1_init(int baudrate)
{
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
}

/*
 * <uart3>
 * usage: telecommunication
 * tx: gpio_pin_d8 (dma1 channel4 stream3)
 * rx: gpio_pin_d9 (dma1 channel4 stream4)
 */
void uart3_init(int baudrate)
{
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
		.USART_StopBits = USART_StopBits_1,
		.USART_Parity = USART_Parity_No
	};
	USART_Init(UART4, &USART_InitStruct);

	USART_Cmd(UART4, ENABLE);
}

/*
 * <uart6>
 * usage: cam
 * tx: gpio_pin_c6
 * rx: gpio_pin_c7
 */
void uart6_init(int baudrate)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_6,
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
}

/*
 * <uart7>
 * usage: telecommunication
 * tx: gpio_pin_e7
 * rx: gpio_pin_e8
 */
void uart7_init(int baudrate)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
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
}

void uart_putc(USART_TypeDef *uart, char c)
{
	while(USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
	USART_SendData(uart, c);
	while(USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET);
}

uint8_t uart_getc(USART_TypeDef *uart, char c)
{
	while(USART_GetFlagStatus(uart, USART_FLAG_RXNE) == SET);
	return USART_ReceiveData(uart);
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

	while(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) == RESET);
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

	while(DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) == RESET);
}

void uart6_puts(char *s, int size)
{
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
		.DMA_Memory0BaseAddr = (uint32_t)s
	};
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);

	//send data from memory to uart data register
	DMA_Cmd(DMA2_Stream6, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);

	while(DMA_GetFlagStatus(DMA2_Stream6, DMA_FLAG_TCIF6) == RESET);
}
