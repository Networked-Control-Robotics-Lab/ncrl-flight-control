#include "send_debug_adaptive_ICL.h"

void send_adaptive_ICL_theta_m_debug(debug_msg_t *payload)
{
	float theta_m_esti;
	theta_m_esti = mat_data(theta_m_hat)[0];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_THETA_M);
	pack_debug_debug_message_float(&theta_m_esti, payload);
}

void send_adaptive_ICL_theta_m_dot_debug(debug_msg_t *payload)
{
	float theta_m_dot_esti;
	theta_m_dot_esti = mat_data(theta_m_hat_dot)[0];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_THETA_M_DOT);
	pack_debug_debug_message_float(&theta_m_dot_esti, payload);
}

void send_adaptive_ICL_theta_m_dot_adaptive_debug(debug_msg_t *payload)
{
	float theta_m_dot_adaptive_esti;
	theta_m_dot_adaptive_esti = mat_data(theta_m_hat_dot_adaptive)[0];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_THETA_M_DOT_ADAPTIVE);
	pack_debug_debug_message_float(&theta_m_dot_adaptive_esti, payload);
}

void send_adaptive_ICL_theta_m_dot_ICL_debug(debug_msg_t *payload)
{
	float theta_m_dot_ICL_esti;
	theta_m_dot_ICL_esti = mat_data(theta_m_hat_dot_ICL)[0];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_THETA_M_DOT_ICL);
	pack_debug_debug_message_float(&theta_m_dot_ICL_esti, payload);
}

void send_adaptive_ICL_theta_diag_debug(debug_msg_t *payload)
{
	float theta_diag_esti[3];
	theta_diag_esti[0] = mat_data(theta_diag_hat)[0];
	theta_diag_esti[1] = mat_data(theta_diag_hat)[1];
	theta_diag_esti[2] = mat_data(theta_diag_hat)[2];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_THETA_DIAG);
	pack_debug_debug_message_float(&theta_diag_esti[0], payload);
	pack_debug_debug_message_float(&theta_diag_esti[1], payload);
	pack_debug_debug_message_float(&theta_diag_esti[2], payload);
}

void send_adaptive_ICL_theta_diag_dot_debug(debug_msg_t *payload)
{
	float theta_diag_dot_esti[3];
	theta_diag_dot_esti[0] = mat_data(theta_diag_hat_dot)[0];
	theta_diag_dot_esti[1] = mat_data(theta_diag_hat_dot)[1];
	theta_diag_dot_esti[2] = mat_data(theta_diag_hat_dot)[2];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_THETA_DIAG_DOT);
	pack_debug_debug_message_float(&theta_diag_dot_esti[0], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti[1], payload);
	pack_debug_debug_message_float(&theta_diag_dot_esti[2], payload);
}

void send_adaptive_ICL_theta_diag_dot_adaptive_debug(debug_msg_t *payload)
{
	float theta_diag_dot_adaptive_esti[3];
	theta_diag_dot_adaptive_esti[0] = mat_data(theta_diag_hat_dot_adaptive)[0];
	theta_diag_dot_adaptive_esti[1] = mat_data(theta_diag_hat_dot_adaptive)[1];
	theta_diag_dot_adaptive_esti[2] = mat_data(theta_diag_hat_dot_adaptive)[2];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_THETA_DIAG_DOT);
	pack_debug_debug_message_float(&theta_diag_dot_adaptive_esti[0], payload);
	pack_debug_debug_message_float(&theta_diag_dot_adaptive_esti[1], payload);
	pack_debug_debug_message_float(&theta_diag_dot_adaptive_esti[2], payload);
}

void send_adaptive_ICL_theta_diag_dot_ICL_debug(debug_msg_t *payload)
{
	float theta_diag_dot_ICL_esti[3];
	theta_diag_dot_ICL_esti[0] = mat_data(theta_diag_hat_dot_ICL)[0];
	theta_diag_dot_ICL_esti[1] = mat_data(theta_diag_hat_dot_ICL)[1];
	theta_diag_dot_ICL_esti[2] = mat_data(theta_diag_hat_dot_ICL)[2];

	pack_debug_debug_message_header(payload, MESSAGE_ID_ICL_THETA_DIAG_DOT);
	pack_debug_debug_message_float(&theta_diag_dot_ICL_esti[0], payload);
	pack_debug_debug_message_float(&theta_diag_dot_ICL_esti[1], payload);
	pack_debug_debug_message_float(&theta_diag_dot_ICL_esti[2], payload);
}
