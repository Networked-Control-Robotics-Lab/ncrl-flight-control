#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/mav_publisher.h"
#include "mavlink_task.h"
#include "uart.h"
#include "imu.h"
#include "compass.h"

void mavlink_compass_calibration_handler(void)
{
	freertos_task_delay(1000);

	send_mavlink_calibration_status_text("[cal] calibration started: 2 mag");
	freertos_task_delay(1000);

	send_mavlink_calibration_status_text("[cal] back orientation detected");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] back side done, rotate to a different side");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] progress <16>");
	freertos_task_delay(1000);

	send_mavlink_calibration_status_text("[cal] front orientation detected");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] front side done, rotate to a different side");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] progress <32>");
	freertos_task_delay(1000);

	send_mavlink_calibration_status_text("[cal] left orientation detected");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] left side done, rotate to a different side");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] progress <48>");
	freertos_task_delay(1000);

	send_mavlink_calibration_status_text("[cal] right orientation detected");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] right side done, rotate to a different side");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] progress <64>");
	freertos_task_delay(1000);

	send_mavlink_calibration_status_text("[cal] up orientation detected");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] up side done, rotate to a different side");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] progress <80>");
	freertos_task_delay(1000);

	send_mavlink_calibration_status_text("[cal] down orientation detected");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] down side done, rotate to a different side");
	freertos_task_delay(1000);
	send_mavlink_calibration_status_text("[cal] progress <100>");
	freertos_task_delay(1000);

	send_mavlink_calibration_status_text("[cal] calibration done: mag");
	freertos_task_delay(1000);
}

void print_compass_data_csv(void)
{
	char s[100];
	float mag[3];
	int len;

	while(1) {
		get_compass_raw(mag);

		sprintf(s, "%.3f,%.3f,%.3f\n", mag[0], mag[1], mag[2]);
		len = strlen(s);

		uart3_puts((char *)s, len);

		freertos_task_delay(50);
	}
}
