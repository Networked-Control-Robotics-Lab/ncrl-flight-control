#include "stm32f4xx.h"
#include "debug_link.h"
#include "delay.h"
#include "multirotor_pid_ctrl.h"
#include "motor_thrust_fitting.h"
#include "optitrack.h"
#include "vins_mono.h"
#include "multirotor_geometry_ctrl.h"
#include "free_fall.h"
#include "flight_ctrl_task.h"
#include "ms5611.h"
#include "debug_msg.h"
#include "compass.h"
#include "ins_eskf.h"
#include "lidar_lite.h"

SemaphoreHandle_t debug_link_task_semphr;

void debug_link_task_semaphore_handler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(debug_link_task_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void task_debug_link(void *param)
{
	debug_msg_t payload;

	/* only one kind of debug link message can be sent by one time,
	 * choose the one by uncomment it */
	while(1) {
		//while(xSemaphoreTake(debug_link_task_semphr, portMAX_DELAY) != pdTRUE);
		//send_imu_debug_message(&payload);
		//send_compass_debug_message(&payload);
		send_attitude_euler_debug_message(&payload);
		//send_attitude_quaternion_debug_message(&payload);
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
		//send_barometer_debug_message(&payload);
		//send_alt_est_debug_message(&payload);
		//send_ins_sensor_debug_message(&payload);
		//send_ins_raw_position_debug_message(&payload);
		//send_ins_fusion_debug_message(&payload);
		//send_ahrs_compass_quality_check_debug_message(&payload);
		//send_ins_eskf1_covariance_matrix_debug_message(&payload);
		//send_vins_mono_position_debug_message(&payload);
		//send_vins_mono_quaternion_debug_message(&payload);
		//send_vins_mono_velocity_debug_message(&payload);
		//send_gps_accuracy_debug_message(&payload);
		//send_rangefinder_debug_message(&payload);
		//send_ins_eskf_correct_freq_debug_message(&payload);
		//send_optitrack_vio_debug_message(&payload);
		//send_gnss_ins_cov_norm_debug_message(&payload);

		send_onboard_data(payload.s, payload.len);
		freertos_task_delay(50); //XXX: 20Hz
	}
}

void debug_link_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                              UBaseType_t priority)
{
	debug_link_task_semphr = xSemaphoreCreateBinary();
	xTaskCreate(task_debug_link, task_name, stack_size, NULL, priority, NULL);
}
