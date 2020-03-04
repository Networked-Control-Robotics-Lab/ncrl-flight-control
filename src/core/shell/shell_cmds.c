#include <stdio.h>
#include "quadshell.h"

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char *s = "supported commands:\n\r"
	          "takeoff\n\r"
	          "land\n\r"
	          "fly_enu [x y z]\n\r";
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
	char s[100] = {'\0'};
	shell_puts("confirm waypoint set (east-north-up) command [y/n]: ");
	char c = shell_getc();
	sprintf(s, "%c\n\r", c);
	shell_puts(s);
}
