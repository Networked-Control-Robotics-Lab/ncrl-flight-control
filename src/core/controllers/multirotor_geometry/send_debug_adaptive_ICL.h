#ifndef __SEND_DEBUG_ADAPTIVE_ICL_H__
#define __SEND_DEBUG_ADAPTIVE_ICL_H__

#include "arm_math.h"
#include "matrix.h"
#include "debug_link.h"

extern arm_matrix_instance_f32 theta_m_hat;
extern arm_matrix_instance_f32 theta_m_hat_dot;
extern arm_matrix_instance_f32 theta_m_hat_dot_adaptive;
extern arm_matrix_instance_f32 theta_m_hat_dot_ICL;
extern arm_matrix_instance_f32 theta_diag_hat;
extern arm_matrix_instance_f32 theta_diag_hat_dot;
extern arm_matrix_instance_f32 theta_diag_hat_dot_adaptive;
extern arm_matrix_instance_f32 theta_diag_hat_dot_ICL;

void send_adaptive_ICL_theta_m_debug(debug_msg_t *payload);
void send_adaptive_ICL_theta_m_dot_debug(debug_msg_t *payload);
void send_adaptive_ICL_theta_m_dot_adaptive_debug(debug_msg_t *payload);
void send_adaptive_ICL_theta_m_dot_ICL_debug(debug_msg_t *payload);
void send_adaptive_ICL_theta_diag_debug(debug_msg_t *payload);
void send_adaptive_ICL_theta_diag_dot_debug(debug_msg_t *payload);
void send_adaptive_ICL_theta_diag_dot_adaptive_debug(debug_msg_t *payload);
void send_adaptive_ICL_theta_diag_dot_ICL_debug(debug_msg_t *payload);

#endif
