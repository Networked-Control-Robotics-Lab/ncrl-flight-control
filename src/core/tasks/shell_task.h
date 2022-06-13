#ifndef __SHELL_TASK_H__
#define __SHELL_TASK_H__

void shell_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                         UBaseType_t priority);

#endif
