#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <proj_config.h>
#include <timer.h>
#include <sys_time.h>
#include <delay.h>
#include "ncp5623c.h"
#include "i2c.h"
#include "led.h"
#include "string.h"
#include "stm32f4xx_conf.h"
#include "uart.h"
#include <stdio.h>
#define UART2_QUEUE_SIZE 100
/*
 * <uart2>
 * usage: log
 * tx: gpio_pin_a9 (dma2 channel4 stream7)
 * rx: gpio_pin_a10 (dma2 channel4 stream2)
 * tx: gpio_pin_d5 (dma1 channel4 stream6)
 * rx: gpio_pin_d6 (dma1 channel4 stream5)
 */
QueueHandle_t uart2_rx_queue;
SemaphoreHandle_t uart2_tx_semphr;


void uart2_init(int baudrate)
{
	uart2_tx_semphr = xSemaphoreCreateBinary();
	uart2_rx_queue = xQueueCreate(UART2_QUEUE_SIZE, sizeof(uart_c_t));

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6,
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
	USART_Init(USART2, &USART_InitStruct);
	USART_Cmd(USART2, ENABLE);
	USART_ClearFlag(USART2, USART_FLAG_TC);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = DMA1_Stream6_IRQn,
		.NVIC_IRQChannelPreemptionPriority = UART2_TX_ISR_PRIORITY,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);
	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = UART2_RX_ISR_PRIORITY;
	NVIC_Init(&NVIC_InitStruct);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void uart2_puts(char *s, int size)
{
	//uart2 tx: dma1 channel4 stream6
	DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);

	DMA_InitTypeDef DMA_InitStructure = {
		.DMA_BufferSize = (uint32_t)size,
		.DMA_FIFOMode = DMA_FIFOMode_Disable,
		.DMA_FIFOThreshold = DMA_FIFOThreshold_Full,
		.DMA_MemoryBurst = DMA_MemoryBurst_Single,
		.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
		.DMA_MemoryInc = DMA_MemoryInc_Enable,
		.DMA_Mode = DMA_Mode_Normal,
		.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR),
		.DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
		.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
		.DMA_Priority = DMA_Priority_Medium,
		.DMA_Channel = DMA_Channel_4,
		.DMA_DIR = DMA_DIR_MemoryToPeripheral,
		.DMA_Memory0BaseAddr = (uint32_t)s
	};
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);

	//send data from memory to uart data register
	DMA_Cmd(DMA1_Stream6, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

	xSemaphoreTake(uart2_tx_semphr, portMAX_DELAY);
}

void DMA1_Stream6_IRQHandler(void)
{
	/* uart2 tx dma */
	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET) {
		DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);

		BaseType_t higher_priority_task_woken = pdFALSE;
		xSemaphoreGiveFromISR(uart2_tx_semphr, &higher_priority_task_woken);
		portEND_SWITCHING_ISR(higher_priority_task_woken);
	}
}

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
		uart_c_t uart_queue_item;
		uart_queue_item.c = USART_ReceiveData(USART2);
		USART2->SR;

		BaseType_t higher_priority_task_woken = pdFALSE;
		xQueueSendToBackFromISR(uart2_rx_queue, &uart_queue_item, &higher_priority_task_woken);
		portEND_SWITCHING_ISR(higher_priority_task_woken)
	}
}

void init_GPIOE()
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType =GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_Init(GPIOE, &GPIO_InitStruct);
}


void task1(void *param){
	int flag = 0;
	char a[20] = "hello\r\n";
	while(1){
		uart2_puts(a,strlen(a));
		sprintf(a,"%d:hello\r\n",flag);
		//usart_puts(USART2,a,strlen(a));
		//blocked_delay_ms(50);		
		if (flag == 0) {
			set_rgb_led_service_motor_lock_flag(true);
		} else if (flag == 1) {
			set_rgb_led_service_navigation_on_flag(true);
		} else if (flag == 2) {
			set_rgb_led_service_motor_lock_flag(false);
		} else if (flag == 3) {
			set_rgb_led_service_navigation_on_flag(false);
		}
	//	sys_timer_blocked_delay_tick_ms(1000);
		flag ++;
		flag %= 4;
		freertos_task_delay(500); //XXX: 20Hz
	}
}
int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	Init_I2C();
	init_GPIOE();
	uart2_init(115200);
	timer12_init();
	timer3_init();
	enable_rgb_led_service();
	//sys_timer_blocked_delay_tick_ms(50);
	blocked_delay_ms(50);		
	xTaskCreate(task1, "task1", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
	
	vTaskStartScheduler();
	
	while(1) {
	}

	return 0;
}
