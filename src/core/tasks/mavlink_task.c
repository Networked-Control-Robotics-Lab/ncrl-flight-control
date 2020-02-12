#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "mavlink.h"
#include "../mavlink/publisher.h"
#include "../mavlink/parser.h"
#include "delay.h"
#include "uart.h"

mavlink_message_t mavlink_recpt_msg;
mavlink_status_t mavlink_recpt_status;

volatile uint8_t received_mavlink_msg;

void mavlink_send_task(void *param)
{
	float update_rate = 50.0f;
	float delay_time_ms = (1.0f / update_rate) * 1000.0f;

	int prescaling_counter = 0;

	while(1) {
		if(prescaling_counter >= 10) {
			send_mavlink_heartbeat();
			//send_mavlink_system_status();
			//send_mavlink_gps();
			prescaling_counter = 0;
		}

		send_mavlink_attitude();
		//send_mavlink_current_waypoint();
		//send_mavlink_reached_waypoint();

		parse_mavlink_received_msg(&mavlink_recpt_msg);

		prescaling_counter++;
		freertos_task_delay(delay_time_ms);
	}
}

void mavlink_recpt_task(void *param)
{
	while(1) {
		volatile char c = uart3_getc();
		received_mavlink_msg = mavlink_parse_char(MAVLINK_COMM_1, (uint8_t)c, &mavlink_recpt_msg, &mavlink_recpt_status);
	}
}
