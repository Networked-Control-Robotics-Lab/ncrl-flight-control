#ifndef __MAVLINK_TASK_H__
#define __MAVLINK_TASK_H__

void mavlink_queue_init(void);

void mavlink_tx_task(void *param);
void mavlink_rx_task(void *param);

void send_mavlink_calibration_status_text(char *status_text);

#endif

