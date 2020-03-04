#include <stdio.h>
#include <stdlib.h>
#include "quadshell.h"
#include "navigation.h"

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char *s = "supported commands:\n\r"
	          "takeoff\n\r"
	          "land\n\r"
	          "fly_enu x y z\n\r";
	shell_puts(s);
}

void shell_cmd_clear(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_cls();
}

void shell_cmd_takeoff(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char s[100] = {'\0'};
	shell_puts("confirm takeoff command [y/n]: ");
	char c = shell_getc();
	sprintf(s, "%c\n\r", c);
	shell_puts(s);
}

void shell_cmd_land(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char s[100] = {'\0'};
	shell_puts("confirm landing command [y/n] ");
	char c = shell_getc();
	sprintf(s, "%c\n\r", c);
	shell_puts(s);
}

void shell_cmd_fly_enu(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(param_cnt < 4) {
		shell_puts("wrong command format!\n\r"
		           "fly_enu x y z\n\r");
		return;
	}

	float pos[3] = {0.0f, 0.0f, 1.5f};
	//float heading = 0.0f;

	char *end_ptr[3] = {NULL};
	pos[0] = strtof(param_list[1], &end_ptr[0]);
	pos[1] = strtof(param_list[2], &end_ptr[1]);
	pos[2] = strtof(param_list[3], &end_ptr[2]);

	if(end_ptr[0] == param_list[1] || param_list[2] == end_ptr[1] || param_list[3] == end_ptr[2]) {
		shell_puts("bad command arguments!\n\r"
		           "fly_enu x y z\n\r");
		return;
	}

	char s[300] = {'\0'};
	sprintf(s, "east-north-up waypoint (x, y, z) = (%f, %f, %f)\n\r"
	        "confirm fly-to command [y/n]: ", pos[0], pos[1], pos[2]);
	shell_puts(s);
	char c = shell_getc();
	sprintf(s, "%c\n\r", c);
	shell_puts(s);

	if(c == 'Y' || c == 'y') {
		float heading = 0.0f;
		nav_goto_waypoint_now(pos, heading);
	} else {
		shell_puts("command aborted\n\r");
	}
}
