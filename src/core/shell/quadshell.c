#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "quadshell.h"

static char shell_getc(void)
{
	char c;
	while(uart3_getc(&c, portMAX_DELAY) == false);
	return c;
}

void shell_puts(char *s)
{
	usart_puts(USART3, s, strlen(s));
}

static void shell_ctrl_c_handler(void)
{
	shell_puts("^C\n\r");
}

static void shell_unknown_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char s[200];
	sprintf(s, "unknown command: %s\n\r", param_list[0]);
	shell_puts(s);
}

void shell_cls(void)
{
	shell_puts("\x1b[H\x1b[2J");
}

void shell_init_struct(struct shell_struct *shell, char *prompt_msg, char *ret_cmd)
{
	shell->prompt_msg = prompt_msg;
	shell->prompt_len = strlen(shell->prompt_msg);

	shell->char_cnt = 0;
	shell->buf = ret_cmd;

	shell->cursor_pos = 0;
	memset(shell->buf, '\0', CMD_LEN_MAX);
}

static void shell_reset_struct(struct shell_struct *shell)
{
	shell->cursor_pos = 0;
	shell->char_cnt = 0;
}

static void shell_remove_char(struct shell_struct *shell, int remove_pos)
{
	int i;
	for(i = (remove_pos - 1); i < (shell->char_cnt); i++) {
		shell->buf[i] = shell->buf[i + 1];
	}

	shell->buf[shell->char_cnt] = '\0';
	shell->char_cnt--;
	shell->cursor_pos--;

	if(shell->cursor_pos > shell->char_cnt) {
		shell->cursor_pos = shell->char_cnt;
	}
}

static void shell_insert_char(struct shell_struct *shell, char c)
{
	int i;
	for(i = shell->char_cnt; i > (shell->cursor_pos - 1); i--) {
		shell->buf[i] = shell->buf[i - 1];
	}
	shell->char_cnt++;
	shell->buf[shell->char_cnt] = '\0';

	shell->buf[shell->cursor_pos] = c;
	shell->cursor_pos++;
}

static void shell_refresh_line(struct shell_struct *shell)
{
	char s[PROMPT_LEN_MAX * 2];
	sprintf(s, "\33[2K\r"   /* clear current line */
	        "%s%s\r"        /* show prompt */
	        "\033[%dC",     /* move cursor */
	        shell->prompt_msg, shell->buf, shell->prompt_len + shell->cursor_pos);
	shell_puts(s);
}

void shell_cli(struct shell_struct *shell)
{
	shell_puts(shell->prompt_msg);

	int c;
	char seq[2];
	while(1) {
		c = shell_getc();

		switch(c) {
		case NULL_CH:
			break;
		case CTRL_A:
			shell->cursor_pos = 0;
			shell_refresh_line(shell);
			break;
		case CTRL_C:
			shell_ctrl_c_handler();
			return;
			break;
		case CTRL_D:
			break;
		case CTRL_E:
			if(shell->char_cnt > 0) {
				shell->cursor_pos = shell->char_cnt;
				shell_refresh_line(shell);
			}
			break;
		case CTRL_F:
			break;
		case CTRL_G:
			break;
		case CTRL_H:
			break;
		case TAB:
			break;
		case CTRL_J:
			break;
		case ENTER:
			if(shell->char_cnt > 0) {
				shell_puts("\n\r");
				shell_reset_struct(shell);
				return;
			} else {
				shell_puts("\n\r");
				shell_puts(shell->prompt_msg);
			}
			break;
		case CTRL_K:
			break;
		case CTRL_L:
			break;
		case CTRL_N:
			break;
		case CTRL_O:
			break;
		case CTRL_P:
			break;
		case CTRL_Q:
			break;
		case CTRL_R:
			break;
		case CTRL_S:
			break;
		case CTRL_T:
			break;
		case CTRL_U:
			break;
		case CTRL_W:
			break;
		case CTRL_X:
			break;
		case CTRL_Y:
			break;
		case CTRL_Z:
			break;
		case ESC_SEQ1:
			seq[0] = shell_getc();
			seq[1] = shell_getc();
			if(seq[0] == ESC_SEQ2) {
				if(seq[1] == UP_ARROW) {
				} else if(seq[1] == DOWN_ARROW) {
				} else if(seq[1] == RIGHT_ARROW) {
					if(shell->cursor_pos < shell->char_cnt) {
						shell->cursor_pos++;
						shell_puts("\033[1C");
					}
				} else if(seq[1] == LEFT_ARROW) {
					if(shell->cursor_pos > 0) {
						shell_puts("\033[1D");
						shell->cursor_pos--;
					}
				} else if(seq[1] == HOME) {
					shell->cursor_pos = 0;
					shell_refresh_line(shell);
				} else if(seq[1] == END) {
					if(shell->char_cnt > 0) {
						shell->cursor_pos = shell->char_cnt;
						shell_refresh_line(shell);
					}
				} else if(seq[1] == DELETE && shell_getc() == ESC_SEQ4) {
					if(shell->char_cnt != 0 && shell->cursor_pos != shell->char_cnt) {
						shell_remove_char(shell, shell->cursor_pos + 1);
						shell->cursor_pos++;
						shell_refresh_line(shell);
					}
				}
			}
			break;
		case BACKSPACE:
			if(shell->char_cnt != 0 && shell->cursor_pos != 0) {
				shell_remove_char(shell, shell->cursor_pos);
				shell_refresh_line(shell);
			}
			break;
		case SPACE:
		default:
			if(shell->char_cnt != (CMD_LEN_MAX - 1)) {
				shell_insert_char(shell, c);
				shell_refresh_line(shell);
			}
			break;
		}
	}
}

static void shell_split_cmd_token(char *cmd, char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int *param_cnt)
{
	int param_list_index = 0;
	int i = 0, j = 0;

	int cmd_s_len = strlen(cmd);

	/* skip spaces before first parameter */
	while(i < cmd_s_len && cmd[i] == ' ') {
		i++;
	}

	for(; i < cmd_s_len; i++) {
		if(cmd[i] == ' ') {
			param_list[param_list_index][j] = '\0';
			param_list_index++;
			j = 0;

			/* exceed maximum parameter count */
			if(param_list_index == PARAM_LIST_SIZE_MAX) {
				*param_cnt = param_list_index;
				return;
			}

			/* skip spaces */
			while(cmd[i + 1] == ' ' && i < cmd_s_len) {
				i++;
			}
		} else {
			param_list[param_list_index][j] = cmd[i];
			j++;
		}
	}
	*param_cnt = param_list_index + 1;
}

void shell_cmd_exec(char *cmd, struct cmd_list_entry *cmd_list, int list_size)
{
	char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX] = {0};
	int param_cnt;
	shell_split_cmd_token(cmd, param_list, &param_cnt);

	int i;
	for(i = 0; i < list_size; i++) {
		if(strcmp(param_list[0], cmd_list[i].name) == 0) {
			cmd_list[i].handler(param_list, param_cnt);
			cmd[0] = '\0';
			return;
		}
	}

	shell_unknown_cmd_handler(param_list, param_cnt);
	cmd[0] = '\0';
}
