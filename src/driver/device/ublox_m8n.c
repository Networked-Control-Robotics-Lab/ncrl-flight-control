#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#define UBLOX_M8N_QUEUE_SIZE 100

typedef struct {
	char c;
} ublox_m8n_buf_c_t;

QueueHandle_t ublox_m8n_queue;

void ublox_m8n_init(void)
{
	ublox_m8n_queue = xQueueCreate(UBLOX_M8N_QUEUE_SIZE, sizeof(ublox_m8n_buf_c_t));
}

void ublox_checksum_calc(uint8_t *result, uint8_t *payload, uint16_t len)
{
	result[0] = 0;
	result[1] = 0;

	int i;
	for(i = 0; i < len; i++) {
		result[0] += payload[i];
		result[1] += result[0];
	}
}

void ublox_m8n_isr_handler(uint8_t c)
{
	ublox_m8n_buf_c_t ublox_m8n_queue_item;
	ublox_m8n_queue_item.c = c;

	BaseType_t higher_priority_task_woken = pdFALSE;
	xQueueSendToBackFromISR(ublox_m8n_queue, &ublox_m8n_queue_item,
	                        &higher_priority_task_woken);
	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

void ublox_m8n_gps_update(void)
{
	ublox_m8n_buf_c_t recept_c;
	while(xQueueReceive(ublox_m8n_queue, &recept_c, 0) == pdTRUE) {
		//uint8_t c = recept_c.c;
	}
}
