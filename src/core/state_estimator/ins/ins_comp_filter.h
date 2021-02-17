#ifndef __INS_COMP_FILTER_H__
#define __INS_COMP_FILTER_H__

void ins_comp_filter_init(float _dt);

void ins_comp_filter_predict(float *pos_enu_out, float *vel_enu_out,
                             bool gps_available, bool height_available);
void ins_comp_filter_gps_correct(float px_correct, float py_correct,
                                 float vx_correct, float vy_correct,
                                 float *pos_enu_out, float *vel_enu_out);
void ins_comp_filter_barometer_correct(float pz_correct, float vz_correct,
                                       float *pos_enu_out, float *vel_enu_out);

#endif
