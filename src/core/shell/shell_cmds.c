#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "mpu6500.h"
#include "uart.h"
#include "sbus_receiver.h"
#include "quadshell.h"
#include "navigation.h"

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char *s = "supported commands:\n\r"
	          "clear"
	          "disarm\n\r"
	          "takeoff\n\r"
	          "land\n\r"
	          "fly x y z\n\r"
	          "mission\n\r"
	          "radio\n\r"
	          "radio_raw\n\r"
	          "acc_calib\n\r"
	          "task\n\r";
	shell_puts(s);
}

void shell_cmd_clear(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_cls();
}

void shell_cmd_disarm(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm disarm command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
	} else {
		shell_puts("abort.\n\r");
	}
}

void shell_cmd_arm(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm arm command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
	} else {
		shell_puts("abort.\n\r");
	}
}

void shell_cmd_takeoff(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm takeoff command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = nav_trigger_auto_takeoff();
		if(ret_val == NAV_SET_SUCCEED) {
			shell_puts("command accept.\n\r");
		} else {
			shell_puts("failed, uav had already takeoff\n\r");
		}
	} else {
		shell_puts("abort.\n\r");
	}
}

void shell_cmd_land(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm landing command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = nav_trigger_auto_landing();
		if(ret_val == NAV_SET_SUCCEED) {
			shell_puts("command accept.\n\r");
		} else {
			shell_puts("failed, uav can only be landed while hovering at a point!\n\r");
		}
	} else {
		shell_puts("abort.\n\r");
	}
}

void shell_cmd_fly(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char s[300] = {'\0'};

	if(param_cnt != 4 && param_cnt != 3) {
		shell_puts("wrong command format!\n\r"
		           "fly_enu x y z\n\r");
		return;
	}

	float pos[3] = {0.0f, 0.0f, 1.5f};

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


	/* set z ifuser provided the third argument */
	bool change_z = false;
	if(param_cnt == 4) {
		change_z = true;
		errno = 0;
		pos[2] = strtof(param_list[3], &end_ptr);
		if (errno != 0 || *end_ptr != '\0') {
			shell_puts("bad command arguments!\n\r"
			           "fly_enu x y z\n\r");
			return;
		}

		sprintf(s, "east-north-up waypoint (x, y, z) = (%f, %f, %f)\n\r", pos[0], pos[1], pos[2]);
	} else if(param_cnt == 3) {
		sprintf(s, "east-north-up waypoint (x, y) = (%f, %f)\n\r", pos[0], pos[1]);
	}

	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm fly-to command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		/* convert from [m] to [cm] */

		pos[0] *= 100.0f;
		pos[1] *= 100.0f;
		pos[2] *= 100.0f;

		int ret_val = nav_goto_waypoint_now(pos, change_z);
		if(ret_val == NAV_WP_OUT_OF_FENCE) {
			shell_puts("failed, waypoint out of geo-fence!\n\r");
		} else {
			shell_puts("command accept.\n\r");
		}
	} else {
		shell_puts("abort\n\r");
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

void shell_cmd_radio(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_puts("press [q] to stop.\n\r");
	char c = '\0';
	while(1) {
		if(uart3_getc(&c, 0) == true) {
			if(c == 'q') break;
		}
		debug_print_rc_info();
	}
}

void shell_cmd_radio_raw(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_puts("press [q] to stop.\n\r");
	char c = '\0';
	while(1) {
		if(uart3_getc(&c, 0) == true) {
			if(c == 'q') break;
		}
		debug_print_rc_val();
	}
}

void shell_cmd_acc_calib(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_puts("press [q] to stop.\n\r");
	char c = '\0';
	while(1) {
		if(uart3_getc(&c, 0) == true) {
			if(c == 'q') break;
		}
		debug_print_mpu6500_accel();
	}
}

void shell_cmd_task(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
}
