#ifndef __USART_H__
#define __USART_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_conf.h"

typedef struct {
	char c;
} uart_c_t;

void uart1_init(int baudrate);
void uart2_init(int baudrate);
void uart3_init(int baudrate);
void uart4_init(int baudrate);
void uart6_init(int baudrate);
void uart7_init(int baudrate);

char uart_getc(USART_TypeDef *uart);
void uart_putc(USART_TypeDef *uart, char c);

void usart_puts(USART_TypeDef *uart, char *s, int size);
void uart2_puts(char *s, int size);
void uart3_puts(char *s, int size);
void uart4_puts(char *s, int size);
void uart6_puts(char *s, int size);
void uart7_puts(char *s, int size);

bool uart2_getc(char *c, long sleep_ticks);
bool uart3_getc(char *c, long sleep_ticks);

void vins_mono_puts(char *s, int size); //XXX: remove me after the porting is completed

#endif
