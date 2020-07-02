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
#include "autopilot.h"

#define MAVLINK_QUEUE_SIZE 10

typedef struct {
	mavlink_message_t mav_msg;
} mavlink_queue_item_t;

QueueHandle_t mavlink_queue;

mavlink_message_t mavlink_recpt_msg;

volatile uint8_t received_mavlink_msg;

void mavlink_queue_init(void)
{
	mavlink_queue = xQueueCreate(MAVLINK_QUEUE_SIZE, sizeof(mavlink_queue_item_t));
}

void mavlink_tx_task(void *param)
{
	float update_rate = 50.0f;
	float delay_time_ms = (1.0f / update_rate) * 1000.0f;

	int prescaler_div_50 = 0;
	int prescaler_div_10 = 0;

	mavlink_queue_item_t recept_mav_queue_item;

	while(1) {
		/* send the following mavlink message with 1Hz */
		if(prescaler_div_50 == 50) {
			send_mavlink_heartbeat();
			send_mavlink_system_status();
#if (SELECT_LOCALIZATION == LOCALIZATION_USE_GPS_MAG)
			send_mavlink_gps();
#endif

			send_mavlink_attitude_quaternion();
#if (SELECT_LOCALIZATION == LOCALIZATION_USE_OPTITRACK)
			send_mavlink_local_position_ned();
#endif


			prescaler_div_50 = 0;
		}
		prescaler_div_50++;

		if(prescaler_div_10 == 10) {
			/* send the following mavlink message with 25Hz */
			send_mavlink_attitude_quaternion();
#if (SELECT_LOCALIZATION == LOCALIZATION_USE_OPTITRACK)
			if(autopilot_get_mode() != AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
				send_mavlink_local_position_ned();
			}
#endif
			prescaler_div_10 = 0;
		}
		prescaler_div_10++;

		/* send trajectory debug message if autopilot mode is set to
		 * trajectory following mode */
		if(autopilot_get_mode() == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
			send_mavlink_trajectory_position_debug();
			send_mavlink_trajectory_velocity_debug();
			send_mavlink_trajectory_acceleration_debug();
		}

		//send_mavlink_attitude();
		//send_mavlink_current_waypoint();
		//send_mavlink_reached_waypoint();

		if(xQueueReceive(mavlink_queue, &recept_mav_queue_item, 0) == pdTRUE) {
			parse_mavlink_received_msg(&recept_mav_queue_item.mav_msg);
		}

		/* microservice handlers */
		mission_waypoint_microservice_handler();
		paramater_microservice_handler();
		polynomial_trajectory_microservice_handler();

		freertos_task_delay(delay_time_ms);
	}
}

void mavlink_rx_task(void *param)
{
	char c;

	while(1) {
		mavlink_status_t mavlink_recpt_status;

		/* receive mavlink message */
		if(uart3_getc(&c, portMAX_DELAY) == true) {
			received_mavlink_msg =
			        mavlink_parse_char(MAVLINK_COMM_1, (uint8_t)c, &mavlink_recpt_msg, &mavlink_recpt_status);
		}

		/* save received mavlink to queue */
		if(received_mavlink_msg == 1) {
			mavlink_queue_item_t mavlink_queue_item;
			mavlink_queue_item.mav_msg = mavlink_recpt_msg;

			while(xQueueSendToBack(mavlink_queue,
			                       &mavlink_queue_item, portMAX_DELAY) != pdTRUE);
		}
	}
}

