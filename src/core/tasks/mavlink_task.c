#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"
#include "../mavlink/parser.h"
#include "../mavlink/param.h"
#include "../mavlink/mission.h"
#include "../mavlink/trajectory.h"
#include "delay.h"
#include "uart.h"

mavlink_message_t mavlink_recpt_msg;
mavlink_status_t mavlink_recpt_status;

volatile uint8_t received_mavlink_msg;

void mavlink_task(void *param)
{
	float update_rate = 1000.0f;
	float delay_time_ms = (1.0f / update_rate) * 1000.0f;

	int prescaler_div_500 = 0;
	int prescaler_div_20 = 0;

	char c;

	while(1) {
		/* send the following mavlink message with 1Hz */
		if(prescaler_div_500 == 500) {
			send_mavlink_heartbeat();
			send_mavlink_system_status();
#if (SELECT_LOCALIZATION == LOCALIZATION_USE_GPS_MAG)
			send_mavlink_gps();
#endif
			prescaler_div_500 = 0;
		}
		prescaler_div_500++;

		if(prescaler_div_20 == 20) {
			/* send the following mavlink message with 25Hz */
			//send_mavlink_attitude_quaternion();
#if (SELECT_LOCALIZATION == LOCALIZATION_USE_OPTITRACK)
			//send_mavlink_local_position_ned();
#endif
			prescaler_div_20 = 0;
		}
		prescaler_div_20++;

		//send_mavlink_attitude();
		//send_mavlink_current_waypoint();
		//send_mavlink_reached_waypoint();

		/* receive mavlink message */
		if(uart3_getc(&c, 0) == true) {
			received_mavlink_msg =
			        mavlink_parse_char(MAVLINK_COMM_1, (uint8_t)c, &mavlink_recpt_msg, &mavlink_recpt_status);
		}

		/* parse incoming mavlink message and call the message handler */
		if(received_mavlink_msg == 1) {
			parse_mavlink_received_msg(&mavlink_recpt_msg);
			received_mavlink_msg = 0;
		}

		/* microservice handlers */
		parameter_microservice_handler();
		mission_waypoint_microservice_handler();
		polynomial_trajectory_microservice_handler();

		freertos_task_delay(delay_time_ms);
	}
}
