#include <stm32f4xx.h>

#include "led.h"
#include "uart.h"
#include "spi.h"

int main(void)
{
	led_init();
	uart1_init(115200);
	uart3_init(115200); //telem
	uart4_init(10000); //s-bus
	uart6_init(115200);
	uart7_init(115200); //gps
	spi1_init();

	while(1) {
		led_on(LED_R);
	}

	return 0;
}
