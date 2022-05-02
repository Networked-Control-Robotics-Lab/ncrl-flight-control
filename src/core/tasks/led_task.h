#ifndef __LED_TASK_H__
#define __LED_TASK_H__

void rgb_led_semaphore_handler(void);
void task_rgb_led(void *param);
void rgb_led_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,UBaseType_t priority);

#endif
