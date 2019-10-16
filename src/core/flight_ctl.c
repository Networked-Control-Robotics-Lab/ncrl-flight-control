#include "delay.h"
#include "led.h"
#include "sbus_receiver.h"
#include "flight_ctl.h"

void task_flight_ctl(void *param)
{
	while(1) {
		led_toggle(LED_B);

		radio_t rc;
		read_rc_info(&rc);
		debug_print_rc_info(&rc);

		delay_ms(5);
	}
}
