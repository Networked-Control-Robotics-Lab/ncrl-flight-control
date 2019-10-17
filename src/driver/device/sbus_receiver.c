#include <stdio.h>
#include <string.h>

#include "stm32f4xx_conf.h"
#include "delay.h"
#include "bound.h"
#include "uart.h"
#include "sbus_receiver.h"
#include "sys_time.h"

void parse_sbus(uint8_t *raw_buff, uint16_t *rc_val);
void debug_print_raw_sbus(void);
void debug_print_rc_val(void);
void debug_print_rc_info(radio_t *rc);

uint8_t sbus_buff[25] = {0};
int sbus_cnt = 0;
uint16_t rc_val[15];

void sbus_rc_handler(uint8_t byte)
{
	static float curr_time_ms;
	static float last_time_ms;

	curr_time_ms = get_sys_time_ms();

	/* use reception interval time to deteminate
	   whether it is a new s-bus frame */
	if((curr_time_ms - last_time_ms) > 3.0f) {
		sbus_cnt = 0;
	}

	sbus_buff[sbus_cnt] = byte;
	sbus_cnt++;

	if((sbus_cnt == 25) && (sbus_buff[0] == 0x0f) && (sbus_buff[24] == 0x00)) {
		parse_sbus((uint8_t *)sbus_buff, (uint16_t *)rc_val);
		sbus_cnt = 0;

		//debug_print_raw_sbus();
		//debug_print_rc_val();
	}

	last_time_ms = curr_time_ms;
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
}

void read_rc_info(radio_t *rc)
{
	float throttle_raw = (float)rc_val[2]; //channel 3
	float roll_raw = (float)rc_val[0]; //channel 1
	float pitch_raw = (float)rc_val[1]; //channel 2
	float yaw_raw = (float)rc_val[3]; //channel 4
	float safety_raw = (float)rc_val[4]; //channel 5

	if(safety_raw > RC_SAFETY_THRESH) {
		rc->safety = false; //disarmed
	} else if(safety_raw < RC_SAFETY_THRESH) {
		rc->safety = true; //armed
	}

	rc->roll = (float)(roll_raw - RC_ROLL_MIN) / (RC_ROLL_MAX - RC_ROLL_MIN) *
	           (RC_ROLL_RANGE_MAX - RC_ROLL_RANGE_MIN) + RC_ROLL_RANGE_MIN;

	rc->pitch = (float)(pitch_raw - RC_PITCH_MIN) / (RC_PITCH_MAX - RC_PITCH_MIN) *
	            (RC_PITCH_RANGE_MAX - RC_PITCH_RANGE_MIN) + RC_PITCH_RANGE_MIN;

	rc->yaw = (float)(yaw_raw - RC_YAW_MIN) / (RC_YAW_MAX - RC_YAW_MIN) *
	          (RC_YAW_RANGE_MAX - RC_YAW_RANGE_MIN) + RC_YAW_RANGE_MIN;

	rc->throttle = (float)(throttle_raw - RC_THROTTLE_MIN) / (RC_THROTTLE_MAX - RC_THROTTLE_MIN) *
	               (RC_THROTTLE_RANGE_MAX - RC_THROTTLE_RANGE_MIN);

	bound_float(&rc->roll, RC_ROLL_RANGE_MAX, RC_ROLL_RANGE_MIN);
	bound_float(&rc->pitch, RC_PITCH_RANGE_MAX, RC_PITCH_RANGE_MIN);
	bound_float(&rc->yaw, RC_YAW_RANGE_MAX, RC_YAW_RANGE_MIN);
	bound_float(&rc->throttle, RC_THROTTLE_RANGE_MAX, RC_THROTTLE_RANGE_MIN);
}

int rc_safety_check(radio_t *rc)
{
	if(rc->safety == false) return 1;
	if(rc->throttle > 10.0f) return 1;
	if(rc->roll > 5.0f || rc->roll < -5.0f) return 1;
	if(rc->pitch > 5.0f || rc->pitch < -5.0f) return 1;
	if(rc->yaw > 5.0f || rc->yaw < -5.0f) return 1;

	return 0;
}

void debug_print_raw_sbus(void)
{
	int i;
	for(i = 0; i < 25; i++) {
		char s[10] = {0};
		sprintf(s, "%d,", sbus_buff[i]);
		uart3_puts(s, strlen(s));
	}

	char *s = "\n\r";
	uart3_puts(s, strlen(s));
}

void debug_print_rc_val(void)
{
	/* debug message */
	char s[100] = {0};
	sprintf(s, "ch1:%d, ch2:%d ch3:%d, ch4:%d, ch5:%d\n\r",
	        rc_val[0], rc_val[1], rc_val[2], rc_val[3], rc_val[4]);
	uart3_puts(s, strlen(s));
}

void debug_print_rc_info(radio_t *rc)
{
	char s[300] = {0};

	if(rc->safety == true) {
		sprintf(s, "[disarmed] roll:%lf,pitch:%lf,yaw:%lf,throttle:%lf\n\r",
		        rc->roll, rc->pitch, rc->yaw, rc->throttle);
	} else {
		sprintf(s, "[armed] roll:%lf,pitch:%lf,yaw:%lf,throttle:%lf\n\r",
		        rc->roll, rc->pitch, rc->yaw, rc->throttle);
	}
	uart3_puts(s, strlen(s));
}

