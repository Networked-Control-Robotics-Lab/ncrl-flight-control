#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "uart.h"

#define OPTITRACK_BUF_SIZE 500

char optitrack_msg_buf[OPTITRACK_BUF_SIZE] = {0};
int curr = 0;
int delimiter_cnt = 0;

void optitrack_handler(uint8_t c)
{
#if 0   /* debug print  */
	if(c == '\n') {
		optitrack_msg_buf[curr] = '\n';
		curr++;
		optitrack_msg_buf[curr] = '\r';
		curr++;
	} else {
		optitrack_msg_buf[curr] = c;
		curr++;
	}

	if((curr + 10) == 500) {
		usart_puts(USART3, optitrack_msg_buf, curr);
		curr = 0;
	}
	return;
#endif

	if(c == '\n') {
		if(optitrack_msg_buf[0] == '7' && optitrack_msg_buf[1] == '8') {
			/* parse optitrack message */
			char s[100];
			sprintf(s, "%d\n\r", delimiter_cnt);
			usart_puts(USART3, s, strlen(s));
		}
		curr = 0;
		delimiter_cnt = 0;
	} else if(c == ' ') {
		optitrack_msg_buf[curr] = c;
		curr++;
		delimiter_cnt++;
	} else {
		optitrack_msg_buf[curr] = c;
		curr++;
	}

	if(curr == OPTITRACK_BUF_SIZE) curr = 0; //preventing boundary access
}
