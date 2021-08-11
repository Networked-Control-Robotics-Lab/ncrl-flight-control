#ifndef __INS_COMP_FILTER_H__
#define __INS_COMP_FILTER_H__

void ins_comp_filter_init(float _dt);
void set_ins_complementary_filter_state(float *pos_ned, float *vel_ned);
bool ins_complementary_filter_ready();
void ins_complementary_filter_estimate(float *pos_ned_raw, float *vel_ned_raw,
                                       float *pos_ned_fused, float *vel_ned_fused);

void ins_comp_filter_predict(float *pos_ned_out, float *vel_ned_out,
                             bool gps_available, bool height_available);
void ins_comp_filter_gps_correct(float px_correct, float py_correct,
                                 float vx_correct, float vy_correct,
                                 float *pos_ned_out, float *vel_ned_out);
void ins_comp_filter_barometer_correct(float pz_correct, float vz_correct,
                                       float *pos_ned_out, float *vel_ned_out);

#endif
