#ifndef __USART_H__
#define __USART_H__

#include <stdint.h>
#include <stdbool.h>

void uart3_init(int baudrate);
uint8_t uart3_getc();
void usart_putc(char c);

#endif
