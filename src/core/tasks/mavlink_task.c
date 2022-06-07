#include <stdbool.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/mav_publisher.h"
#include "../mavlink/mav_parser.h"
#include "../mavlink/mav_param.h"
#include "../mavlink/mav_mission.h"
#include "../mavlink/mav_trajectory.h"
#include "delay.h"
#include "uart.h"
#include "autopilot.h"
#include "board_porting.h"
#include "sys_time.h"
#include "mavlink_task.h"
#include "mav_command.h"

#define MAVLINK_QUEUE_SIZE 10

QueueHandle_t mavlink_queue;
QueueHandle_t mavlink_calib_status_text_queue;
QueueHandle_t mavlink_recpt_record_queue;

mavlink_message_t mavlink_recpt_msg;

volatile uint8_t received_mavlink_msg;

bool rx_debug = false;

void mavlink_rx_debug_enable(void)
{
	rx_debug = true;
}

void mavlink_rx_debug_disable(void)
{
	rx_debug = false;
}

bool get_mavlink_reception_record(int *msg_id, float *recvd_time)
{
	mavlink_recpt_record_item_t mavlink_recpt_record_item;

	if(xQueueReceive(mavlink_recpt_record_queue, &mavlink_recpt_record_item, 0) == pdTRUE) {
		*msg_id = mavlink_recpt_record_item.recvd_msg_id;
		*recvd_time = mavlink_recpt_record_item.recvd_time;
		return true;
	}
	return false;
}

void send_mavlink_calibration_status_text(char *status_text)
{
	mavlink_calib_status_text_item_t calib_queue_item;
	strcpy(calib_queue_item.status_text, status_text);

	while(xQueueSendToBack(mavlink_calib_status_text_queue, &calib_queue_item, portMAX_DELAY) != pdTRUE);
}

void mavlink_calibration_handler(void)
{
	mavlink_calib_status_text_item_t recept_calib_queue_item;
	if(xQueueReceive(mavlink_calib_status_text_queue, &recept_calib_queue_item, 0) == pdTRUE) {
		send_mavlink_status_text(recept_calib_queue_item.status_text, 6, 0, 0);
	}
}

void mavlink_tx_task(void *param)
{
	float update_rate = 50.0f;
	float delay_time_ms = (1.0f / update_rate) * 1000.0f;

	int prescaler_div_50 = 0;
	int prescaler_div_10 = 0;
	int prescaler_div_5 = 0;

	mavlink_queue_item_t recept_mav_queue_item;

	while(1) {
		/* send the following mavlink message @ 1Hz */
		if(prescaler_div_50 == 50) {
			send_mavlink_heartbeat();
			send_mavlink_system_status();

			prescaler_div_50 = 0;
		}
		prescaler_div_50++;

		/* send the following mavlink message @ 5Hz */
		if(prescaler_div_10 == 10) {
#if (SELECT_POSITION_SENSOR == POSITION_FUSION_USE_GPS)
			send_mavlink_gps();
#endif
			prescaler_div_10 = 0;
		}
		prescaler_div_10++;

		/* send the following mavlink message @ 10Hz */
		if(prescaler_div_5 == 5) {
			send_mavlink_attitude_quaternion();
			send_mavlink_rc_channels();
			send_mavlink_local_position_ned();

			prescaler_div_5 = 0;
		}
		prescaler_div_5++;

		/* send trajectory debug message if autopilot mode is set to
		 * trajectory following mode */
		if(autopilot_get_mode() == AUTOPILOT_TRAJECTORY_FOLLOWING_MODE) {
			send_mavlink_trajectory_position_debug();
			send_mavlink_trajectory_velocity_debug();
			send_mavlink_trajectory_acceleration_debug();
		}

		mavlink_calibration_handler();

		if(xQueueReceive(mavlink_queue, &recept_mav_queue_item, 0) == pdTRUE) {
			parse_mavlink_received_msg(&recept_mav_queue_item.mav_msg);
		}

		/* microservice handlers */
		mission_waypoint_microservice_handler();
		paramater_microservice_handler();
		polynomial_trajectory_microservice_handler();
		command_long_microservice_handler();

		freertos_task_delay(delay_time_ms);
	}
}

void mavlink_rx_task(void *param)
{
	char c;

	while(1) {
		mavlink_status_t mavlink_recpt_status;

		/* receive mavlink message */
		if(mavlink_getc(&c, portMAX_DELAY) == true) {
			received_mavlink_msg =
			        mavlink_parse_char(MAVLINK_COMM_1, (uint8_t)c, &mavlink_recpt_msg, &mavlink_recpt_status);
		}

		/* save received mavlink to queue */
		if(received_mavlink_msg == 1) {
			mavlink_queue_item_t mavlink_queue_item;
			mavlink_queue_item.mav_msg = mavlink_recpt_msg;

			while(xQueueSendToBack(mavlink_queue, &mavlink_queue_item, portMAX_DELAY) != pdTRUE);

			if(rx_debug == true) {
				mavlink_recpt_record_item_t mavlink_recpt_record_item = {
					.recvd_msg_id = mavlink_recpt_msg.msgid,
					.recvd_time = get_sys_time_s()
				};
				xQueueSendToBack(mavlink_recpt_record_queue, &mavlink_recpt_record_item, 0);
			}
		}
	}
}

void mavlink_tx_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                              UBaseType_t priority)
{
	mavlink_calib_status_text_queue = xQueueCreate(5, sizeof(mavlink_calib_status_text_item_t));
	xTaskCreate(mavlink_tx_task, task_name, stack_size, NULL, priority, NULL);
}

void mavlink_rx_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                              UBaseType_t priority)
{
	mavlink_queue = xQueueCreate(MAVLINK_QUEUE_SIZE, sizeof(mavlink_queue_item_t));
	mavlink_recpt_record_queue = xQueueCreate(50, sizeof(mavlink_recpt_record_item_t));
	xTaskCreate(mavlink_rx_task, task_name, stack_size, NULL, priority, NULL);
}
