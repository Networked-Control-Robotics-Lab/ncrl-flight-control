#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "mavlink.h"
#include "../mavlink/publisher.h"
#include "delay.h"

void mavlink_handler_task(void)
{
	float update_rate = 50.0f;
	float delay_time_ms = (1.0f / update_rate) * 1000.0f;

	int prescaling_counter = 0;

	while(1) {
		if(prescaling_counter == 8) {
			send_mavlink_heartbeat();
			send_mavlink_gps();			
		}

		send_mavlink_attitude();
		//send_mavlink_current_waypoint();
		//send_mavlink_reached_waypoint();

		prescaling_counter++;
		freertos_task_delay(delay_time_ms);
	}
}
