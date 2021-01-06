#include "debug_link.h"
#include "optitrack.h"
#include "barometer.h"
#include "ins.h"

void send_alt_est_debug_message(debug_msg_t *payload)
{
	float altitude = ins_get_fused_position_z();
	float optitrack_z = optitrack_read_pos_z();
	float altitude_rate = ins_get_fused_velocity_z;
	float optitrack_vz = optitrack_read_vel_z();

	pack_debug_debug_message_header(payload, MESSAGE_ID_ALT_EST);
	pack_debug_debug_message_float(&altitude, payload);
	pack_debug_debug_message_float(&optitrack_z, payload);
	pack_debug_debug_message_float(&altitude_rate, payload);
	pack_debug_debug_message_float(&optitrack_vz, payload);
}
