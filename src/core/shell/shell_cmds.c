#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "quadshell.h"
#include "navigation.h"

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char *s = "supported commands:\n\r"
	          "takeoff\n\r"
	          "land\n\r"
	          "fly x y z\n\r"
		  "mission\n\r";
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

void shell_cmd_fly(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(param_cnt != 4) {
		shell_puts("wrong command format!\n\r"
		           "fly_enu x y z\n\r");
		return;
	}

	float pos[3] = {0.0f, 0.0f, 1.5f};
	float heading = 0.0f;

	char *end_ptr = NULL;

	errno = 0;
	pos[0] = strtof(param_list[1], &end_ptr);
	if (errno != 0 || *end_ptr != '\0') {
		shell_puts("bad command arguments!\n\r"
		           "fly_enu x y z\n\r");
		return;
	}

	errno = 0;
	pos[1] = strtof(param_list[2], &end_ptr);
	if (errno != 0 || *end_ptr != '\0') {
		shell_puts("bad command arguments!\n\r"
		           "fly_enu x y z\n\r");
		return;
	}

	errno = 0;
	pos[2] = strtof(param_list[3], &end_ptr);
	if (errno != 0 || *end_ptr != '\0') {
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
		/* convert from [m] to [cm] */
		pos[0] *= 100.0f;
		pos[1] *= 100.0f;
		pos[2] *= 100.0f;

		int ret_val = nav_goto_waypoint_now(pos, heading);
		if(ret_val == WP_SET_OUT_OF_FENCE) {
			shell_puts("failed, waypoint out of geo-fence!\n\r");
		} else {
			shell_puts("command accept.\n\r");
		}
	} else {
		shell_puts("command aborted\n\r");
	}
}

void shell_cmd_mission(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	int ret_val;

	if(param_cnt != 2) {
		shell_puts("mission add: add new waypoint\n\r"
                           "mission start: start waypoint mission\n\r"
                           "mission halt: halt the executing waypoint mission\n\r"
			   "mission resume: resume the halting waypoint mission\n\r"
			   "mission clear: clear the waypoint list\n\r");
	} else if(param_cnt == 2) {
		if(strcmp(param_list[1], "add") == 0) {
		} else if(strcmp(param_list[1], "start") == 0) {
			ret_val = nav_waypoint_mission_start();
			if(ret_val == 0) {
				shell_puts("succeeded adding new waypoint.\n\r");
			} else {
				shell_puts("failed, waypoint list is full\n\r");
			}
		} else if(strcmp(param_list[1], "halt") == 0) {
			ret_val = nav_halt_waypoint_mission();
			if(ret_val == 0) {
				shell_puts("succeeded halting the waypoint mission.\n\r");
			} else {
				shell_puts("failed, no executing mission can be halted!\n\r");
			}
		} else if(strcmp(param_list[1], "resume") == 0) {
			ret_val = nav_resume_waypoint_mission();
			if(ret_val == 0) {
				shell_puts("succeeded resuming the waypoint mission.\n\r");
			} else {
				shell_puts("failed, no halting mission can be resumed!\n\r");
			}
		} else if(strcmp(param_list[1], "clear") == 0) {
			ret_val = nav_clear_waypoint_list();
			if(ret_val == 0) {
				shell_puts("succeeded to clear the waypoint list.\n\r");
			} else {
				shell_puts("failed, waypoint list is empty!\n\r");
			}
		} else {
			shell_puts("unknown mission command!\n\r");
		}
	}
}
