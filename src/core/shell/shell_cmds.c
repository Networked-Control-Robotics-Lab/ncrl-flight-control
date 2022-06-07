#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "mpu6500.h"
#include "uart.h"
#include "sbus_radio.h"
#include "quadshell.h"
#include "autopilot.h"
#include "perf.h"
#include "perf_list.h"
#include "sys_param.h"
#include "imu.h"
#include "delay.h"
#include "accel_calibration.h"
#include "esc_calibration.h"
#include "compass.h"
#include "waypoint_following.h"
#include "takeoff_landing.h"
#include "proj_config.h"
#include "board_porting.h"
#include "mavlink_task.h"

static bool parse_float_from_str(char *str, float *value)
{
	char *end_ptr = NULL;
	errno = 0;
	*value = strtof(str, &end_ptr);
	if (errno != 0 || *end_ptr != '\0') {
		return false;
	} else {
		return true;
	}
}

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char *s = "supported commands:\n\r"
	          "help\n\r"
	          "clear\n\r"
	          "arm, disarm, takeoff, land, fly, mission\n\r"
	          "radio, radio_raw\n\r"
	          "accel, compass\n\r"
	          "accel_calib\n\r"
	          "motor_calib\n\r"
	          "motor_test\n\r"
	          "perf\n\r"
	          "params\n\r"
	          "mavlink_debug\n\r";
	shell_puts(s);
}

void shell_cmd_clear(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_cls();
}

void shell_cmd_arm(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm arm command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		autopilot_set_armed();
		shell_puts("armed.\n\r");
	} else {
		shell_puts("abort.\n\r");
	}
}

void shell_cmd_disarm(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm disarm command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		autopilot_set_disarmed();
		shell_puts("disarmed.\n\r");
	} else {
		shell_puts("abort.\n\r");
	}
}

void shell_cmd_takeoff(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(autopilot_is_armed() == false) {
		shell_puts("failed, uav not armed!\n\r");
		return;
	}

	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm takeoff command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = autopilot_trigger_auto_takeoff();
		if(ret_val == AUTOPILOT_SET_SUCCEED) {
			shell_puts("command accept.\n\r");
		} else {
			shell_puts("failed, uav had already takeoff!\n\r");
		}
	} else {
		shell_puts("abort.\n\r");
	}
}

void shell_cmd_land(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(autopilot_is_armed() == false) {
		shell_puts("failed, uav not armed!\n\r");
		return;
	}

	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm landing command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = autopilot_trigger_auto_landing();
		if(ret_val == AUTOPILOT_SET_SUCCEED) {
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
	if(autopilot_is_armed() == false) {
		shell_puts("failed, uav not armed!\n\r");
		return;
	}
	char s[300] = {'\0'};

	if(param_cnt == 1) {
		shell_puts("fly x y z\n\r"
		           "fly x y\n\r");
	}
	if(param_cnt != 4 && param_cnt != 3) {
		shell_puts("abort, bad arguments!\n\r"
		           "fly x y z\n\r"
		           "fly x y\n\r");
		return;
	}

	float pos[3] = {0.0f, 0.0f, 1.5f};

	if (parse_float_from_str(param_list[1], &pos[0]) == false) {
		shell_puts("abort, bad arguments!\n\r"
		           "fly x y z\n\r"
		           "fly x y\n\r");
		return;
	}

	if (parse_float_from_str(param_list[2], &pos[1]) == false) {
		shell_puts("abort, bad arguments!\n\r"
		           "fly x y z\n\r"
		           "fly x y\n\r");
		return;
	}

	/* check if user set the height */
	bool change_z = false;
	if(param_cnt == 4) {
		change_z = true;
		if (parse_float_from_str(param_list[3], &pos[2]) == false) {
			shell_puts("abort, bad arguments!\n\r"
			           "fly x y z\n\r"
			           "fly x y\n\r");
			return;
		}

		sprintf(s, "east-north-up waypoint (x, y, z) = (%fm, %fm, %fm)\n\r",
		        pos[0], pos[1], pos[2]);
	} else if(param_cnt == 3) {
		sprintf(s, "east-north-up waypoint (x, y) = (%fm, %fm)\n\r", pos[0], pos[1]);
	}
	shell_puts(s);

	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm fly command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = autopilot_goto_waypoint_now(0, pos, change_z);
		if(ret_val == AUTOPILOT_WAYPOINT_OUT_OF_FENCE) {
			shell_puts("failed, waypoint out of geo-fence!\n\r");
		} else {
			shell_puts("command accept.\n\r");
		}
	} else {
		shell_puts("abort.\n\r");
	}
}

static void mission_add_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	float pos[3] = {0.0f, 0.0f, 0.0f};
	float heading = 0.0f;
	float stay_time_sec = 1.0f;

	if(parse_float_from_str(param_list[2], &pos[0]) == false) {
		shell_puts("abort, bad arguments\n\r");
		return;
	}
	if(parse_float_from_str(param_list[3], &pos[1]) == false) {
		shell_puts("abort, bad arguments\n\r");
		return;
	}
	if(parse_float_from_str(param_list[4], &pos[2]) == false) {
		shell_puts("abort, bad arguments\n\r");
		return;

	}
	if(parse_float_from_str(param_list[5], &heading) == false) {
		shell_puts("abort, bad arguments\n\r");
		return;
	}
	if(parse_float_from_str(param_list[6], &stay_time_sec) == false) {
		shell_puts("abort, bad arguments\n\r");
		return;
	}

	char s[200] = {'\0'};
	sprintf(s, "new waypoint: x=%.1fm, y=%.1fm, z=%.1fm, heading=%.1f, stay_time=%.1fsec\n\r",
	        pos[0], pos[1], pos[2], heading, stay_time_sec);
	shell_puts(s);

	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm mission add command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = autopilot_add_new_waypoint(pos, heading, stay_time_sec);
		if(ret_val == AUTOPILOT_SET_SUCCEED) {
			shell_puts("successfully added new waypoint.\n\r");
		} else if(ret_val == AUTOPILOT_WAYPOINT_LIST_FULL) {
			shell_puts("failed, mission is executing!\n\r");
		} else if(ret_val == AUTOPILOT_WAYPOINT_OUT_OF_FENCE) {
			shell_puts("failed, waypoint out of geo-fence!\n\r");
		}
	} else {
		shell_puts("abort.\n\r");
	}
}

static void mission_start_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	if(autopilot_is_armed() == false) {
		shell_puts("failed, uav not armed!\n\r");
		return;
	}

	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm mission start command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = autopilot_waypoint_mission_start(false);
		if(ret_val == AUTOPILOT_SET_SUCCEED) {
			shell_puts("successfully started the mission.\n\r");
		} else if(ret_val == AUTOPILOT_WAYPOINT_LIST_EMPYT) {
			shell_puts("failed, waypoint list is empyt!\n\r");
		} else if(ret_val == AUTOPILOT_NOT_IN_HOVERING_MODE) {
			shell_puts("failed, uav is not in hovering mode!\n\r");
		}
	} else {
		shell_puts("abort.\n\r");
	}
}

static void mission_loop_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	if(autopilot_is_armed() == false) {
		shell_puts("failed, uav not armed!\n\r");
		return;
	}

	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm mission start command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = autopilot_waypoint_mission_start(true);
		if(ret_val == AUTOPILOT_SET_SUCCEED) {
			shell_puts("successfully started the mission.\n\r");
		} else if(ret_val == AUTOPILOT_WAYPOINT_LIST_EMPYT) {
			shell_puts("failed, waypoint list is empty!\n\r");
		} else if(ret_val == AUTOPILOT_NOT_IN_HOVERING_MODE) {
			shell_puts("failed, uav is not in hovering mode!\n\r");
		}
	} else {
		shell_puts("abort.\n\r");
	}
}

static void mission_list_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	debug_print_waypoint_list();
}

static void mission_halt_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	if(autopilot_is_armed() == false) {
		shell_puts("failed, uav not armed!\n\r");
		return;
	}

	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm mission halt command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = autopilot_halt_waypoint_mission();
		if(ret_val == AUTOPILOT_SET_SUCCEED) {
			shell_puts("successfully halted the waypoint mission.\n\r");
		} else if(ret_val == AUTOPILOT_NOT_IN_WAYPOINT_MODE) {
			shell_puts("failed, no executing mission!\n\r");
		}
	} else {
		shell_puts("abort.\n\r");
	}
}

static void mission_resume_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	if(autopilot_is_armed() == false) {
		shell_puts("failed, uav not armed!\n\r");
		return;
	}

	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm mission resume command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = autopilot_resume_waypoint_mission();
		if(ret_val == AUTOPILOT_SET_SUCCEED) {
			shell_puts("successfully resumed the waypoint mission.\n\r");
		} else if(ret_val == AUTOPILOT_NO_HALTED_WAYPOINT_MISSION) {
			shell_puts("failed, no halted mission!\n\r");
		}
	} else {
		shell_puts("abort.\n\r");
	}
}

static void mission_clear_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm mission clear command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		int ret_val = autopilot_clear_waypoint_list();
		if(ret_val == AUTOPILOT_SET_SUCCEED) {
			shell_puts("successfully cleared the waypoint list.\n\r");
		} else if(ret_val == AUTOPILOT_WAYPOINT_FOLLOWING_BUSY) {
			shell_puts("failed, waypoint list is empty!\n\r");
		}
	} else {
		shell_puts("abort.\n\r");
	}
}

static void mission_status_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	debug_print_waypoint_status();
}

void shell_cmd_mission(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(param_cnt == 8) {
		if(strcmp(param_list[1], "add") == 0) {
			mission_add_cmd_handler(param_list);
		}
	} else if(param_cnt == 2) {
		if(strcmp(param_list[1], "start") == 0) {
			mission_start_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "loop") == 0) {
			mission_loop_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "list") == 0) {
			mission_list_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "halt") == 0) {
			mission_halt_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "resume") == 0) {
			mission_resume_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "clear") == 0) {
			mission_clear_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "status") == 0) {
			mission_status_cmd_handler(param_list);
		} else {
			shell_puts("unknown mission command!\n\r");
		}
	} else {
		shell_puts("mission add x y z heading stay_time_sec radius: add new waypoint\n\r"
		           "mission start: start waypoint mission\n\r"
		           "mission loop: loop waypoint mission\n\r"
		           "mission list: list current waypoint list\n\r"
		           "mission halt: halt current executing waypoint mission\n\r"
		           "mission resume: resume current halting waypoint mission\n\r"
		           "mission clear: clear waypoint list\n\r"
		           "mission status: show mission status\n\r");
	}
}

void shell_cmd_radio(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_puts("press [q] to stop.\n\r");
	char c = '\0';
	while(1) {

		if(debug_link_getc(&c, 0) == true) {
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
		if(debug_link_getc(&c, 0) == true) {
			if(c == 'q') break;
		}
		debug_print_rc_val();
	}
}

void shell_cmd_accel_calib(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_accel_calibration_handler();
}

void shell_cmd_accel(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_puts("press [q] to stop.\n\r");
	char c = '\0';
	while(1) {
		if(debug_link_getc(&c, 0) == true) {
			if(c == 'q') break;
		}
		debug_print_mpu6500_unscaled_lpf_accel();
	}
}

void shell_cmd_perf(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char s[100];
	shell_puts("performance analysis:\n\r---------------------\n\r");

	float flight_control_trigger_time = perf_get_time_s(PERF_FLIGHT_CONTROL_TRIGGER_TIME);
	float flight_loop_time = perf_get_time_s(PERF_FLIGHT_CONTROL_LOOP);
	float ahrs_time = perf_get_time_s(PERF_AHRS_INS);
	float controller_time = perf_get_time_s(PERF_CONTROLLER);
	float ahrs_cpu_percentage = ahrs_time / flight_control_trigger_time * 100;
	float controller_cpu_percentage = controller_time / flight_control_trigger_time * 100;
	float flight_loop_cpu_percentage = flight_loop_time / flight_control_trigger_time * 100;

	sprintf(s, "[flight controller task] executing frequency: %.0fHz\n\r",
	        1.0 / flight_control_trigger_time);
	shell_puts(s);

	sprintf(s, "* [AHRS] %.2fms (%.0f%%)\n\r",
	        ahrs_time * 1000.0f, ahrs_cpu_percentage);
	shell_puts(s);
	sprintf(s, "* [controller] %.2fms (%.0f%%)\n\r",
	        controller_time * 1000.0f, controller_cpu_percentage);
	shell_puts(s);
	sprintf(s, "* [total] %.2fms (%.0f%%)\n\r",
	        flight_loop_time * 1000.0f, flight_loop_cpu_percentage);
	shell_puts(s);
}

static void param_list_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	char s[100];

	int i;
	int list_size = get_sys_param_list_size();

	sprintf(s, "system parameters (size: %d):\n\r", list_size);
	shell_puts(s);

	char *name;
	char *type_s;
	uint8_t type;

	uint8_t u8;
	int8_t s8;
	uint16_t u16;
	int16_t s16;
	uint32_t u32;
	int32_t s32;
	float f;

	for(i = 0; i < list_size; i++) {
		get_sys_param_name(i, &name);
		get_sys_param_type(i, &type);

		switch(type) {
		case SYS_PARAM_U8:
			type_s = SYS_PARAM_TYPE_TO_STRING(SYS_PARAM_U8);
			get_sys_param_u8(i, &u8);
			sprintf(s, "[#%d][%s][%s][%u]\n\r",
			        i, name, type_s, u8);
			break;
		case SYS_PARAM_S8:
			type_s = SYS_PARAM_TYPE_TO_STRING(SYS_PARAM_S8);
			get_sys_param_s8(i, &s8);
			sprintf(s, "[#%d][%s][%s][%d]\n\r",
			        i, name, type_s, s8);
			break;
		case SYS_PARAM_U16:
			type_s = SYS_PARAM_TYPE_TO_STRING(SYS_PARAM_U16);
			get_sys_param_u16(i, &u16);
			sprintf(s, "[#%d][%s][%s][%u]\n\r",
			        i, name, type_s, u16);
			break;
		case SYS_PARAM_S16:
			type_s = SYS_PARAM_TYPE_TO_STRING(SYS_PARAM_S16);
			get_sys_param_s16(i, &s16);
			sprintf(s, "[#%d][%s][%s][%d]\n\r",
			        i, name, type_s, s16);
			break;
		case SYS_PARAM_U32:
			type_s = SYS_PARAM_TYPE_TO_STRING(SYS_PARAM_U32);
			get_sys_param_u32(i, &u32);
			sprintf(s, "[#%d][%s][%s][%lu]\n\r",
			        i, name, type_s, u32);
			break;
		case SYS_PARAM_S32:
			type_s = SYS_PARAM_TYPE_TO_STRING(SYS_PARAM_S32);
			get_sys_param_s32(i, &s32);
			sprintf(s, "[#%d][%s][%s][%ld]\n\r",
			        i, name, type_s, s32);
			break;
		case SYS_PARAM_FLOAT:
			type_s = SYS_PARAM_TYPE_TO_STRING(SYS_PARAM_FLOAT);
			get_sys_param_float(i, &f);
			sprintf(s, "[#%d][%s][%s][%.2f]\n\r",
			        i, name, type_s, f);
			break;
		default:
			sprintf(s, "[#%d][%s][unknown type %d]\n\r",
			        i, name, type);
		}

		shell_puts(s);
	}

}

static void param_save_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	int retval = save_param_list_to_flash();

	switch(retval) {
	case SYS_PARAM_FLASH_WR_SUCCEED:
		shell_puts("successfully saved parameter list to flash.\n\r");
		break;
	case SYS_PARAM_FLASH_WR_DATA_INCORRECT:
		shell_puts("failed, written data inconsistent error!\n\r");
		break;
	case SYS_PARAM_FLASH_WR_TIMEOUT:
		shell_puts("failed, flash programming timeout!\n\r");
		break;
	case SYS_PARAM_FLASH_ERASE_TIMEOUT:
		shell_puts("failed, flash erasing timeout!\n\r");
		break;
	}
}

static void param_load_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	int retval = load_param_list_from_flash();

	switch(retval) {
	case SYS_PARAM_FLASH_WR_SUCCEED:
		shell_puts("successfully loaded parameter list from flash.\n\r");
		break;
	case SYS_PARAM_FLASH_CRC_INCORRECT:
		shell_puts("failed, crc inconsistent, saved default to flash now!\n\r");
		break;
	case SYS_PARAM_FLASH_SIZE_INCORRECT:
		shell_puts("failed, list size inconsistent, saved default to flash now!\n\r");
		break;
	}
}

void shell_cmd_param(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(param_cnt == 2) {
		if(strcmp(param_list[1], "list") == 0) {
			param_list_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "save") == 0) {
			param_save_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "load") == 0) {
			param_load_cmd_handler(param_list);
		} else {
			shell_puts("unknown paramater command!\n\r");
		}
	} else {
		shell_puts("param list\n\r"
		           "param save\n\r"
		           "param load\n\r");
	}

}

void shell_cmd_compass(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(is_compass_available() == false) {
		shell_puts("compass not presents!\n\r");
	}

	float mag[3] = {0};
	volatile float update_freq = 0;

	char c;
	char s[100] = {0};

	shell_puts("press [q] to stop.\n\r");
	while(1) {
		if(debug_link_getc(&c, 0) == true) {
			if(c == 'q') break;
		}

		get_compass_raw(mag);
		update_freq = get_compass_update_rate();

		sprintf(s, "[%.0fHz] compass x:%.0f, y:%.0f, z:%.0f\n\r",
		        update_freq, mag[0], mag[1], mag[2]);
		shell_puts(s);

		freertos_task_delay(1000);
	}
}

void shell_cmd_motor_calib(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm motor calibration command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		shell_puts("entered to motor calibration mode.\n\r"
		           "restart the system after calibration.\n\r");
		trigger_esc_range_calibration();
	} else {
		shell_puts("abort.\n\r");
	}
}

void shell_cmd_motor_test(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm motor thrust testing command [y/n]: ", user_agree);
	shell_cli(&shell);

	if(strcmp(user_agree, "y") == 0 || strcmp(user_agree, "Y") == 0) {
		shell_puts("entered to motor thrust testing mode.\n\r"
		           "restart the system to end the testing.\n\r");
		trigger_motor_force_testing();
	} else {
		shell_puts("abort.\n\r");
		return;
	}

	char s[100] = {0};
	char ret_str[100] = {0};
	float curr_motor_signal = 0.0f;
	float new_motor_signal = 0.0f;

	while(1) {
		curr_motor_signal = get_motor_force_testing_percentage();

		sprintf(s, "current control signal is %.2f%%\n\r", curr_motor_signal * 100.0f);
		shell_puts(s);

		shell_init_struct(&shell, "please enter new command (0~100): ", ret_str);
		shell_cli(&shell);

		if(parse_float_from_str(ret_str, &new_motor_signal) == false) {
			shell_puts("bad argument, not a number!\n\r");
		} else {
			if(new_motor_signal < 0.0f || new_motor_signal > 100.0f) {
				shell_puts("bad argument, command out of range!\n\r");
			} else {
				set_motor_force_testing_percentage(new_motor_signal * 0.01f);
			}
		}
	}
}

void shell_cmd_mavlink_debug(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char s[150];

	mavlink_rx_debug_enable();

	shell_puts("press [q] to stop.\n\r");
	char c = '\0';
	while(1) {
		if(debug_link_getc(&c, 0) == true) {
			if(c == 'q') break;
		}

		int msg_id;
		float recvd_time;

		if(get_mavlink_reception_record(&msg_id, &recvd_time) == true) {
			sprintf(s, "[%f] received message #%d.\n\r", recvd_time, msg_id);
			shell_puts(s);
		}
	}

	mavlink_rx_debug_disable();
}
