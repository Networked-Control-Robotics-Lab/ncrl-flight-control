#ifndef __F4_BOARD_SUPPORT_H__
#define __F4_BOARD_SUPPORT_H__

void f4_sw_i2c_driver_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                    UBaseType_t priority);
void f4_sw_i2c_driver_semaphore_handler(BaseType_t *higher_priority_task_woken);

#endif
