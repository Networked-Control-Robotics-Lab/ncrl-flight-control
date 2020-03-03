#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "uart.h"
#include "quadshell.h"

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt);

struct cmd_list_entry shell_cmd_list[] = {
        DEF_SHELL_CMD(help)
};

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char *s = "help";
	uart3_puts(s, strlen(s));
}

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
	int len = strlen(greeting);
	uart3_puts(greeting, len);

	char s[150];
	sprintf(s, "firmware build time: %s %s\n\rtype 'help' for help\n\r\n\r", __TIME__, __DATE__);
	len = strlen(s);
	uart3_puts(s, len);
}

void shell_task(void *param)
{
	/* init shell cli */
	char ret_shell_cmd[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "shell > ", ret_shell_cmd);

	/* init shell parser */
	int shell_cmd_cnt = SIZE_OF_SHELL_CMD_LIST(shell_cmd_list);

	shell_cls();
	shell_greeting();

	while(1) {
		shell_cli(&shell);
		shell_cmd_exec(ret_shell_cmd, shell_cmd_list, shell_cmd_cnt);
		taskYIELD();
	}
}
