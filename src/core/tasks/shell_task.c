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

void shell_task(void *param)
{
	/* init shell cli */
	char ret_shell_cmd[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "shell > ", ret_shell_cmd);

	/* init shell parser */
	int shell_cmd_cnt = SIZE_OF_SHELL_CMD_LIST(shell_cmd_list);

	shell_cls();
	while(1) {
		shell_cli(&shell);
		shell_cmd_exec(ret_shell_cmd, shell_cmd_list, shell_cmd_cnt);
		taskYIELD();
	}
}
