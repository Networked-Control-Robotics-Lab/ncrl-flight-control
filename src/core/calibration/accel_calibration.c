#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"

void mavlink_accel_calibration_handler(void)
{
	freertos_task_delay(1000);

	/* detected stage */
	send_mavlink_status_text("[cal] calibration started: 2 accel", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] back orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] front orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] left orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] right orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] up orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] down orientation detected", 6, 0, 0);
	freertos_task_delay(1000);

	/* measured stage */
	send_mavlink_status_text("[cal] back side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] front side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] left side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] right side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] up side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);

	send_mavlink_status_text("[cal] down side done, rotate to a different side", 6, 0, 0);
	freertos_task_delay(1000);
}
