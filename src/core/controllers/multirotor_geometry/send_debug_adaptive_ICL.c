#include "send_debug_adaptive_ICL.h"

void send_adaptive_ICL_mass_estimation_debug(debug_msg_t *payload)
{
	float theta_m_esti;
	float theta_m_dot_esti;
	float theta_m_dot_esti_adaptive;
	float theta_m_dot_esti_ICL;
	float current_time = get_sys_time_ms();
	float curr_pos[3] = {0.0f};

	get_enu_position(curr_pos);
	current_time = current_time*0.001;
	theta_m_esti = mat_data(theta_m_hat)[0];
	theta_m_dot_esti = mat_data(theta_m_hat_dot)[0];
	theta_m_dot_esti_adaptive = mat_data(theta_m_hat_dot_adaptive)[0];
	theta_m_dot_esti_ICL = mat_data(theta_m_hat_dot_ICL)[0];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_MASS_ESTIMATION);
	pack_debug_debug_message_float(&current_time, payload);
	pack_debug_debug_message_float(&curr_pos[2], payload);
	pack_debug_debug_message_float(&autopilot.wp_now.pos[2], payload);
	pack_debug_debug_message_float(&theta_m_esti, payload);
	pack_debug_debug_message_float(&theta_m_dot_esti, payload);
	pack_debug_debug_message_float(&theta_m_dot_esti_adaptive, payload);
	pack_debug_debug_message_float(&theta_m_dot_esti_ICL, payload);
}

void send_adaptive_ICL_inertia_estimation_debug(debug_msg_t *payload)
{
	float theta_diag_esti[3];
	float theta_diag_dot_esti[3];
	float theta_diag_dot_esti_adaptive[3];
	float theta_diag_dot_esti_ICL[3];
	float current_time = get_sys_time_ms();
	current_time = current_time*0.001;

	theta_diag_esti[0] = mat_data(theta_diag_hat)[0];
	theta_diag_esti[1] = mat_data(theta_diag_hat)[1];
	theta_diag_esti[2] = mat_data(theta_diag_hat)[2];
	theta_diag_dot_esti[0] = mat_data(theta_diag_hat_dot)[0];
	theta_diag_dot_esti[1] = mat_data(theta_diag_hat_dot)[1];
	theta_diag_dot_esti[2] = mat_data(theta_diag_hat_dot)[2];
	theta_diag_dot_esti_adaptive[0] = mat_data(theta_diag_hat_dot_adaptive)[0];
	theta_diag_dot_esti_adaptive[1] = mat_data(theta_diag_hat_dot_adaptive)[1];
	theta_diag_dot_esti_adaptive[2] = mat_data(theta_diag_hat_dot_adaptive)[2];
	theta_diag_dot_esti_ICL[0] = mat_data(theta_diag_hat_dot_ICL)[0];
	theta_diag_dot_esti_ICL[1] = mat_data(theta_diag_hat_dot_ICL)[1];
	theta_diag_dot_esti_ICL[2] = mat_data(theta_diag_hat_dot_ICL)[2];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_INERTIA_ESTIMATION);
	pack_debug_debug_message_float(&current_time, payload);
	pack_debug_debug_message_float(&theta_diag_esti[0], payload);
	pack_debug_debug_message_float(&theta_diag_esti[1], payload);
	pack_debug_debug_message_float(&theta_diag_esti[2], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti[0], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti[1], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti[2], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti_adaptive[0], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti_adaptive[1], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti_adaptive[2], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti_ICL[0], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti_ICL[1], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti_ICL[2], payload);
}

void send_adaptive_ICL_mass_inertia_estimation_debug(debug_msg_t *payload)
{
	float theta_m_esti;
	float theta_diag_esti[3];
	float current_time = get_sys_time_ms();
	current_time = current_time*0.001;

	theta_m_esti = mat_data(theta_m_hat)[0];
	theta_diag_esti[0] = mat_data(theta_diag_hat)[0];
	theta_diag_esti[1] = mat_data(theta_diag_hat)[1];
	theta_diag_esti[2] = mat_data(theta_diag_hat)[2];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_MASS_INERTIA_ESTIMATION);
	pack_debug_debug_message_float(&current_time, payload);
	pack_debug_debug_message_float(&theta_m_esti, payload);
	pack_debug_debug_message_float(&theta_diag_esti[0], payload);
	pack_debug_debug_message_float(&theta_diag_esti[1], payload);
	pack_debug_debug_message_float(&theta_diag_esti[2], payload);
}
