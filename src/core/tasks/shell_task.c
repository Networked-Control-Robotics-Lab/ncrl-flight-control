#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "uart.h"
#include "quadshell.h"
#include "autopilot.h"
#include "shell_cmds.h"

struct cmd_list_entry shell_cmd_list[] = {
	DEF_SHELL_CMD(help)
	DEF_SHELL_CMD(clear)
	DEF_SHELL_CMD(arm)
	DEF_SHELL_CMD(disarm)
	DEF_SHELL_CMD(takeoff)
	DEF_SHELL_CMD(land)
	DEF_SHELL_CMD(fly)
	DEF_SHELL_CMD(mission)
	DEF_SHELL_CMD(radio)
	DEF_SHELL_CMD(radio_raw)
	DEF_SHELL_CMD(accel_calib)
	DEF_SHELL_CMD(accel)
	DEF_SHELL_CMD(perf)
	DEF_SHELL_CMD(param)
	DEF_SHELL_CMD(compass)
	DEF_SHELL_CMD(motor_calib)
	DEF_SHELL_CMD(motor_test)
	DEF_SHELL_CMD(mavlink_debug)
};

void shell_greeting(void)
{
	char *greeting = "                       _       _          _  _ \n\r"
	                 "                      | |     | |        | || |\n\r"
	                 "  __ _ _   _  __ _  __| |  ___| |__   ___| || |\n\r"
	                 " / _` | | | |/ _` |/ _` | / __| '_ \\ / _ \\ || |\n\r"
	                 "| (_| | |_| | (_| | (_| | \\__ \\ | | |  __/ || |\n\r"
	                 " \\__, |\\__,_|\\__,_|\\__,_| |___/_| |_|\\___|_||_|\n\r"
	                 "    | |\n\r"
	                 "    |_|\n\r\n\r";
	shell_puts(greeting);

	char s[150];
	sprintf(s, "firmware build time: %s %s\n\rtype `help' for help\n\r\n\r", __TIME__, __DATE__);
	shell_puts(s);
}

void shell_task(void *param)
{
	/* init shell cli */
	char ret_shell_cmd[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "uav0 > ", ret_shell_cmd);

	/* init shell parser */
	int shell_cmd_cnt = SIZE_OF_SHELL_CMD_LIST(shell_cmd_list);

	shell_cls();
	shell_greeting();

	while(1) {
		shell_cli(&shell);
		shell_cmd_exec(&shell, shell_cmd_list, shell_cmd_cnt);
		taskYIELD();
	}
}

void shell_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                         UBaseType_t priority)
{
	xTaskCreate(shell_task, task_name, stack_size, NULL, priority, NULL);
}

