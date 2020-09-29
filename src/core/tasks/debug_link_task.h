#ifndef __DEBUG_LINK_TASK_H__
#define __DEBUG_LINK_TASK_H__

void debug_link_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                              UBaseType_t priority);
void debug_link_task_semaphore_handler(void);

#endif
