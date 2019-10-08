#ifndef __USART_H__
#define __USART_H__

#include <stdint.h>
#include <stdbool.h>

void uart1_init(int baudrate);
void uart3_init(int baudrate);
void uart4_init(int baudrate);
void uart6_init(int baudrate);
void uart7_init(int baudrate);

uint8_t uart_getc(USART_TypeDef *uart);
void uart_putc(USART_TypeDef *uart, char c);

void uart1_puts(char *s, int size);
void uart3_puts(char *s, int size);

#endif
