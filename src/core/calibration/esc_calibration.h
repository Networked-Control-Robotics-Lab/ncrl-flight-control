#ifndef __ESC_CALIBRATION_H__
#define __ESC_CALIBRATION_H__

/* motor esc range calibration */
void trigger_esc_range_calibration(void);
bool is_esc_range_calibration_triggered(void);
void esc_range_calibration(void);

/* motor force testing */
void trigger_motor_force_testing(void);
bool is_motor_force_testing_triggered(void);
void set_motor_force_testing_percentage(float percentage);
float get_motor_force_testing_percentage(void);
void motor_force_testing(void);

#endif
