#include <stdio.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "delay.h"
#include "bound.h"
#include "uart.h"
#include "sbus_radio.h"
#include "sys_time.h"
#include "proj_config.h"
#include "board_porting.h"

void parse_sbus(uint8_t *raw_buff, uint16_t *rc_val);

sbus_t sbus;

void sbus_rc_isr_handler(uint8_t byte)
{
	static float curr_time_ms;
	static float last_time_ms;

	curr_time_ms = get_sys_time_ms();

	/* use reception interval time to deteminate
	   whether it is a new s-bus frame */
	if((curr_time_ms - last_time_ms) > 2.0f) {
		sbus.buf_recept_cnt = 0;
	}

	sbus.buf[sbus.buf_recept_cnt] = byte;
	sbus.buf_recept_cnt++;

	if((sbus.buf_recept_cnt == 25) && (sbus.buf[0] == 0x0f) && (sbus.buf[24] == 0x00)) {
		parse_sbus((uint8_t *)sbus.buf, (uint16_t *)sbus.rc_val);
		sbus.buf_recept_cnt = 0;
	}

	last_time_ms = curr_time_ms;
}

void parse_sbus(uint8_t *raw_buff, uint16_t *rc_val)
{
	/* parse channel 1 to 8 */
	sbus.rc_val[0] = ((raw_buff[1] | raw_buff[2] << 8) & 0x07ff);
	sbus.rc_val[1] = ((raw_buff[2] >> 3 | raw_buff[3] << 5) & 0x07ff);
	sbus.rc_val[2] = ((raw_buff[3] >> 6 | raw_buff[4] << 2 | raw_buff[5] << 10) & 0x07ff);
	sbus.rc_val[3] = ((raw_buff[5] >> 1 | raw_buff[6] << 7) & 0x07ff);
	sbus.rc_val[4] = ((raw_buff[6] >> 4 | raw_buff[7] << 4) & 0x07ff);
	sbus.rc_val[5] = ((raw_buff[7] >> 7 | raw_buff[8] << 1 | raw_buff[9] << 9) & 0x07ff);
	sbus.rc_val[6] = ((raw_buff[9] >> 2 | raw_buff[10] << 6) & 0x07ff);
	sbus.rc_val[7] = ((raw_buff[10] >> 5 | raw_buff[11] << 3) & 0x07ff);

	/* channel 9 ~ channel 16 haven't yet been validated, check the data before using them */
	//sbus.rc_val[8] = ((raw_buff[12] | raw_buff[13] << 8) & 0x07ff);
	//sbus.rc_val[9] = ((raw_buff[13] >> 3 | raw_buff[14] << 5) & 0x07ff);
	//sbus.rc_val[10] = ((raw_buff[14] >> 6 | raw_buff[15] << 2 | raw_buff[16] << 10) & 0x07ff);
	//sbus.rc_val[11] = ((raw_buff[16] >> 1 | raw_buff[17] << 7) & 0x07ff);
	//sbus.rc_val[12] = ((raw_buff[17] >> 4 | raw_buff[18] << 4) & 0x07ff);
	//sbus.rc_val[13] = ((raw_buff[18] >> 7 | raw_buff[19] << 1 | raw_buff[20] << 9) & 0x07ff);
	//sbus.rc_val[14] = ((raw_buff[20] >> 2 | raw_buff[21] << 6) & 0x07ff);
	//sbus.rc_val[15] = ((raw_buff[21] >> 5 | raw_buff[22] << 3) & 0x07ff);
}

void sbus_rc_read(radio_t *rc)
{
	float throttle_raw = (float)sbus.rc_val[2]; //channel 3
	float roll_raw = (float)sbus.rc_val[0]; //channel 1
	float pitch_raw = (float)sbus.rc_val[1]; //channel 2
	float yaw_raw = (float)sbus.rc_val[3]; //channel 4
	float safety_raw = (float)sbus.rc_val[4]; //channel 5
	float auto_flight = (float)sbus.rc_val[5]; //channel 6
	float aux1_mode_raw = (float)sbus.rc_val[6]; //channel 7

	if(safety_raw > RC_SAFETY_THRESH) {
		rc->safety = false; //disarmed
	} else if(safety_raw < RC_SAFETY_THRESH) {
		rc->safety = true; //armed
	}

	if(auto_flight < RC_AUTO_FLIGHT_THRESH) {
		rc->auto_flight = false; //manual flight
	} else if(auto_flight > RC_AUTO_FLIGHT_THRESH) {
		rc->auto_flight = true; //auto flight
	}

	rc->roll = (float)(roll_raw - RC_ROLL_MIN) / (RC_ROLL_MAX - RC_ROLL_MIN) *
	           (RC_ROLL_RANGE_MAX - RC_ROLL_RANGE_MIN) + RC_ROLL_RANGE_MIN;

	rc->pitch = (float)(pitch_raw - RC_PITCH_MIN) / (RC_PITCH_MAX - RC_PITCH_MIN) *
	            (RC_PITCH_RANGE_MAX - RC_PITCH_RANGE_MIN) + RC_PITCH_RANGE_MIN;

	rc->yaw = (float)(yaw_raw - RC_YAW_MIN) / (RC_YAW_MAX - RC_YAW_MIN) *
	          (RC_YAW_RANGE_MAX - RC_YAW_RANGE_MIN) + RC_YAW_RANGE_MIN;

	rc->throttle = (float)(throttle_raw - RC_THROTTLE_MIN) / (RC_THROTTLE_MAX - RC_THROTTLE_MIN) *
	               (RC_THROTTLE_RANGE_MAX - RC_THROTTLE_RANGE_MIN);

	/* aux-sw 1 mode */
	float aux1_mode_up_thresh = (RC_FLIGHT_MODE_MAX + RC_FLIGHT_MODE_MID) / 2.0f;
	float aux1_mode_low_thresh = (RC_FLIGHT_MODE_MID + RC_FLIGHT_MODE_MIN) / 2.0f;
	if(aux1_mode_raw < aux1_mode_low_thresh) {
		rc->aux1_mode = RC_AUX_MODE1;
	} else if (aux1_mode_raw < aux1_mode_up_thresh && aux1_mode_raw > aux1_mode_low_thresh) {
		rc->aux1_mode = RC_AUX_MODE2;
	} else if (aux1_mode_raw > aux1_mode_up_thresh) {
		rc->aux1_mode = RC_AUX_MODE3;
	} else {
		rc->aux1_mode = RC_AUX_MODE1;
	}

	bound_float(&rc->roll, RC_ROLL_RANGE_MAX, RC_ROLL_RANGE_MIN);
	bound_float(&rc->pitch, RC_PITCH_RANGE_MAX, RC_PITCH_RANGE_MIN);
	bound_float(&rc->yaw, RC_YAW_RANGE_MAX, RC_YAW_RANGE_MIN);
	bound_float(&rc->throttle, RC_THROTTLE_RANGE_MAX, RC_THROTTLE_RANGE_MIN);
}

int rc_safety_check(radio_t *rc)
{
	if(rc->safety == false) return 1;
	if(rc->auto_flight != false) return 1;
	if(rc->aux1_mode != RC_AUX_MODE1) return 1;
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
		sprintf(s, "%d,", sbus.buf[i]);
		debug_link_puts(s, strlen(s));
	}

	char *s = "\n\r";
	debug_link_puts(s, strlen(s));
}

void sbus_get_unscaled(uint16_t *rc_val)
{
	rc_val[0] = sbus.rc_val[0];
	rc_val[1] = sbus.rc_val[1];
	rc_val[2] = sbus.rc_val[2];
	rc_val[3] = sbus.rc_val[3];
	rc_val[4] = sbus.rc_val[4];
	rc_val[5] = sbus.rc_val[5];
	rc_val[6] = sbus.rc_val[6];
	rc_val[7] = sbus.rc_val[7];
	rc_val[8] = 0;
	rc_val[9] = 0;
	rc_val[10] = 0;
	rc_val[11] = 0;
	rc_val[12] = 0;
	rc_val[13] = 0;
	rc_val[14] = 0;
	rc_val[15] = 0;
	rc_val[16] = 0;
	rc_val[17] = 0;
	rc_val[18] = 0;
}

void debug_print_rc_val(void)
{
	/* debug message */
	char s[100] = {0};
	sprintf(s, "ch1:%d, ch2:%d ch3:%d, ch4:%d, ch5:%d, ch6:%d, ch7:%d\n\r",
	        sbus.rc_val[0], sbus.rc_val[1], sbus.rc_val[2], sbus.rc_val[3], sbus.rc_val[4], sbus.rc_val[5], sbus.rc_val[6]);
	debug_link_puts(s, strlen(s));
	blocked_delay_ms(100);
}

void debug_print_rc_info(void)
{
	radio_t rc;
	sbus_rc_read(&rc);

	char s[300] = {0};

	char *safety_s = 0;
	char *safety_armed_s = "[armed]";
	char *safety_disarmed_s = "[disarmed]";

	char *auto_flight_s = 0;
	char *auto_flight_enabled_s = "[auto-flight]";
	char *auto_flight_disabled_s = "[manual-flight]";

	char *aux1_mode_s = 0;
	char *aux1_mode1_s = "[aux1 mode1]";
	char *aux1_mode2_s = "[aux1 mode2]";
	char *aux1_mode3_s = "[aux1 mode3]";

	if(rc.safety == true) safety_s = safety_armed_s;
	else safety_s = safety_disarmed_s;

	if(rc.auto_flight == true) auto_flight_s = auto_flight_enabled_s;
	else auto_flight_s = auto_flight_disabled_s;

	switch(rc.aux1_mode) {
	case RC_AUX_MODE1:
		aux1_mode_s = aux1_mode1_s;
		break;
	case RC_AUX_MODE2:
		aux1_mode_s = aux1_mode2_s;
		break;
	case RC_AUX_MODE3:
		aux1_mode_s = aux1_mode3_s;
		break;
	}

	sprintf(s, "%s%s%s roll:%lf,pitch:%lf,yaw:%lf,throttle:%lf\n\r",
	        safety_s, auto_flight_s, aux1_mode_s, rc.roll, rc.pitch, rc.yaw, rc.throttle);

	debug_link_puts(s, strlen(s));
	blocked_delay_ms(100);
}
