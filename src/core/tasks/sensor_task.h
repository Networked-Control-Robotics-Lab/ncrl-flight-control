#ifndef __SENSOR_TASK_H__
#define __SENSOR_TASK_H__

void imu_semaphore_handler(void);
void baro_semaphore_handler(void);

void imu_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                     UBaseType_t priority);
void baro_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                     UBaseType_t priority);
#endif
