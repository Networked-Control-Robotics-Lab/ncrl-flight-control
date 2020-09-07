#include "stm32f4xx.h"
#include "debug_link.h"
#include "delay.h"
#include "multirotor_pid_ctrl.h"
#include "motor_thrust_fitting.h"
#include "optitrack.h"
#include "multirotor_geometry_ctrl.h"
#include "free_fall.h"
#include "flight_ctrl_task.h"
#include "ms5611.h"
#include "altitude_est.h"

void task_debug_link(void *param)
{
	debug_msg_t payload;

	float update_rate = 50;
	float delay_time_ms = (1.0f / update_rate) * 1000.0f;

	/* only one kind of debug link message can be sent by one time,
	 * choose the one by uncomment it */
	while(1) {
		//send_imu_debug_message(&payload);
		//send_attitude_euler_debug_message(&payload);
		send_attitude_quaternion_debug_message(&payload);
		//send_attitude_imu_debug_message(&payload);
		//send_pid_debug_message(&payload);
		//send_motor_debug_message(&payload);
		//send_optitrack_position_debug_message(&payload);
		//send_optitrack_quaternion_debug_message(&payload);
		//send_optitrack_velocity_debug_message(&payload);
		//send_geometry_ctrl_debug(&payload);
		//send_geometry_tracking_ctrl_debug(&payload);
		//send_uav_dynamics_debug(&payload);
		//send_free_fall_debug_message(&payload);
		//send_compass_debug_message(&payload);
		//send_barometer_debug_message(&payload);
		//send_alt_est_debug_message(&payload);

		send_onboard_data(payload.s, payload.len);
		freertos_task_delay(delay_time_ms);
	}
}

void debug_link_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                              UBaseType_t priority)
{
	xTaskCreate(task_debug_link, task_name, stack_size, NULL, priority, NULL);
}
