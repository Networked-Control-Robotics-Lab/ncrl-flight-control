#include "quadshell.h"

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char *s = "supported commands:\n\r"
	          "takeoff\n\r"
	          "land\n\r"
	          "fly_enu [x y z]\n\r";
	shell_puts(s);
}

void shell_cmd_takeoff(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_puts("confirm takeoff command: y/n: ");
}

void shell_cmd_land(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_puts("confirm landing command: y/n: ");
}

void shell_cmd_fly_enu(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_puts("confirm waypoint set (east-north-up) command: y/n: ");
}
