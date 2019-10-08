#include <string.h>

#include "stm32f4xx.h"
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

		char *s = "test\n";
		uart6_puts(s, strlen(s));

#if 0
		uart_putc(USART1, 'h');
		uart_putc(USART1, 'e');
		uart_putc(USART1, 'l');
		uart_putc(USART1, 'l');
		uart_putc(USART1, 'o');
		uart_putc(USART1, '\n');
		uart_putc(USART1, '\r');
#endif
		delay_ms(100);
	}

	vTaskStartScheduler();

	return 0;
}
