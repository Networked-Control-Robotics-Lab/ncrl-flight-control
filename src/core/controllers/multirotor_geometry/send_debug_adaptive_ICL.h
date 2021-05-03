#ifndef __SEND_DEBUG_ADAPTIVE_ICL_H__
#define __SEND_DEBUG_ADAPTIVE_ICL_H__

#include "arm_math.h"
#include "matrix.h"
#include "debug_link.h"
#include "autopilot.h"
#include "position_state.h"
#include <sys_time.h>

extern arm_matrix_instance_f32 theta_m_hat;
extern arm_matrix_instance_f32 theta_m_hat_dot;
extern arm_matrix_instance_f32 theta_m_hat_dot_adaptive;
extern arm_matrix_instance_f32 theta_m_hat_dot_ICL;
extern arm_matrix_instance_f32 theta_diag_hat;
extern arm_matrix_instance_f32 theta_diag_hat_dot;
extern arm_matrix_instance_f32 theta_diag_hat_dot_adaptive;
extern arm_matrix_instance_f32 theta_diag_hat_dot_ICL;
extern arm_matrix_instance_f32 curr_moment;
extern arm_matrix_instance_f32 M_fb;
extern arm_matrix_instance_f32 M_ff;
extern arm_matrix_instance_f32 inertia_effect;
extern autopilot_t autopilot;

void send_adaptive_ICL_mass_estimation_debug(debug_msg_t *payload);
void send_adaptive_ICL_inertia_estimation_debug(debug_msg_t *payload);
void send_adaptive_ICL_mass_inertia_estimation_debug(debug_msg_t *payload);
void send_adaptive_ICL_moment_ctrl_input_debug(debug_msg_t *payload);

#endif
