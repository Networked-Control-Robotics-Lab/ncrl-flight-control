#ifndef __MAVLINK_TASK_H__
#define __MAVLINK_TASK_H__

void mavlink_tx_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                              UBaseType_t priority);
void mavlink_rx_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                              UBaseType_t priority);

void send_mavlink_calibration_status_text(char *status_text);

#endif

