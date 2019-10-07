#include <stm32f4xx.h>

#include "FreeRTOS.h"
#include "task.h"
#include "freertos_config.h"
#include "delay.h"
#include "led.h"
#include "uart.h"
#include "spi.h"
#include "mpu6500.h"

int main(void)
{
	led_init();
	uart1_init(115200);
	uart3_init(115200); //telem
	uart4_init(10000);  //s-bus
	uart6_init(115200);
	uart7_init(115200); //gps
	spi1_init();

	led_on(LED_R);
	led_off(LED_G);
	led_off(LED_B);

	mpu6500_init();

	led_off(LED_R);
	led_off(LED_G);
	led_on(LED_B);

	while(1) {
		led_toggle(LED_B);
		uart_putc(USART3, 'h');
		uart_putc(USART3, 'e');
		uart_putc(USART3, 'l');
		uart_putc(USART3, 'l');
		uart_putc(USART3, 'o');
		uart_putc(USART3, '\n');
		uart_putc(USART3, '\r');
		delay_ms(25);
	}

	vTaskStartScheduler();

	return 0;
}
