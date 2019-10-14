#include <stdio.h>
#include <string.h>

#include "stm32f4xx_conf.h"
#include "uart.h"

void parse_sbus(uint8_t *raw_buff, uint16_t *rc_val);

uint16_t rc_val[15];
uint8_t c;
int sbus_cnt = 0;
uint8_t sbus_buff[25] = {0};

void sbus_rc_handler(void)
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET) {
		c = USART_ReceiveData(UART4);
		UART4->SR;

		if(c == 15) {
			sbus_cnt = 0;
		}

		sbus_buff[sbus_cnt] = c;
		sbus_cnt++;

#if 1
		if(sbus_cnt == 25) {
			parse_sbus((uint8_t *)sbus_buff, (uint16_t *)rc_val);
		}
#endif

#if 0
		/* debug message */
		if(sbus_cnt == 25) {
			int i;
			for(i = 0; i < 25; i++) {
				char s[10] = {0};
				sprintf(s, "%d,", sbus_buff[i]);
				uart3_puts(s, strlen(s));
			}
			uart_putc(USART3, '\n');
			uart_putc(USART3, '\r');
		}
#endif

		if(sbus_cnt == 25) {
			sbus_cnt = 0;
		}
	}
}

void parse_sbus(uint8_t *raw_buff, uint16_t *rc_val)
{
	/* parse channel 1 to 8 */
	rc_val[0] = ((raw_buff[1] | raw_buff[2] << 8) & 0x07ff);
	rc_val[1] = ((raw_buff[2] >> 3 | raw_buff[3] << 5) & 0x07ff);
	rc_val[2] = ((raw_buff[3] >> 6 | raw_buff[4] << 2 | raw_buff[5] << 10) & 0x07ff);
	rc_val[3] = ((raw_buff[5] >> 1 | raw_buff[6] << 7) & 0x07ff);
	rc_val[4] = ((raw_buff[6] >> 4 | raw_buff[7] << 4) & 0x07ff);
	rc_val[5] = ((raw_buff[7] >> 7 | raw_buff[8] << 1 | raw_buff[9] << 9) & 0x07ff);
	rc_val[6] = ((raw_buff[9] >> 2 | raw_buff[10] << 6) & 0x07ff);
	rc_val[7] = ((raw_buff[10] >> 5 | raw_buff[11] << 3) & 0x07ff);

	/* channel 9 ~ channel 16 haven't yet been validated, check the data before using them */
	//rc_val[8] = ((raw_buff[12] | raw_buff[13] << 8) & 0x07ff);
	//rc_val[9] = ((raw_buff[13] >> 3 | raw_buff[14] << 5) & 0x07ff);
	//rc_val[10] = ((raw_buff[14] >> 6 | raw_buff[15] << 2 | raw_buff[16] << 10) & 0x07ff);
	//rc_val[11] = ((raw_buff[16] >> 1 | raw_buff[17] << 7) & 0x07ff);
	//rc_val[12] = ((raw_buff[17] >> 4 | raw_buff[18] << 4) & 0x07ff);
	//rc_val[13] = ((raw_buff[18] >> 7 | raw_buff[19] << 1 | raw_buff[20] << 9) & 0x07ff);
	//rc_val[14] = ((raw_buff[20] >> 2 | raw_buff[21] << 6) & 0x07ff);
	//rc_val[15] = ((raw_buff[21] >> 5 | raw_buff[22] << 3) & 0x07ff);

	/* debug message */
	char s[100] = {0};
	sprintf(s, "ch1:%d, ch2:%d ch3:%d, ch4:%d, ch6:%d\n\r",
	        rc_val[0], rc_val[1], rc_val[2], rc_val[3], rc_val[5]);
	uart3_puts(s, strlen(s));
}

