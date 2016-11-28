#include "file_output.h"

#ifdef DEBUG
#define DBG	printf
#else
#define DBG(...)
#endif

static bool serialize_msg_flow(file_output_t *p_fout, suseconds_t timestamp,
			       mavlink_message_t *p_msg)
{
	mavlink_optical_flow_t of;
	mavlink_msg_optical_flow_decode(p_msg, &of);

	int nwr = fprintf(p_fout->p_file,
			  "%lu;%d;%lu;%f;%f;%f;%d;%d;%d;%d;0;0;0;0\n",
			  timestamp, p_msg->msgid, of.time_usec,
			  of.flow_comp_m_x, of.flow_comp_m_y,
			  of.ground_distance, of.flow_x, of.flow_y,
			  of.sensor_id, of.quality);

	if (nwr < 0)
		return false;

	if (fflush(p_fout->p_file) != 0)
		return false;

	return true;
}

static bool serialize_msg_flow_rad(file_output_t *p_fout, suseconds_t timestamp,
				   mavlink_message_t *p_msg)
{
	mavlink_optical_flow_rad_t of;
	memset(&of, 0, sizeof(of));
	mavlink_msg_optical_flow_rad_decode(p_msg, &of);

	int nwr = fprintf(p_fout->p_file, "%lu;%d;%lu;%u;%f;%f;%f;%f;%f;",
			  timestamp, p_msg->msgid, of.time_usec,
			  of.integration_time_us, of.integrated_x,
			  of.integrated_y, of.integrated_xgyro,
			  of.integrated_ygyro, of.integrated_zgyro);
	nwr += fprintf(p_fout->p_file, "%u;%f;%d;%d;%d\n",
		       of.time_delta_distance_us, of.distance, of.temperature,
		       of.sensor_id, of.quality);

	if (nwr < 0)
		return false;

	if (fflush(p_fout->p_file) != 0)
		return false;

	return true;
}

static bool serialize_msg_flow_debug_vect(file_output_t *p_fout,
					  suseconds_t timestamp,
					  mavlink_message_t *p_msg)
{
	mavlink_debug_vect_t dv;
	mavlink_msg_debug_vect_decode(p_msg, &dv);

	int nwr = fprintf(p_fout->p_file,
			  "%lu;%d;%lu;%f;%f;%f;0;0;0;0;0;0;0;0\n", timestamp,
			  p_msg->msgid, dv.time_usec, dv.x, dv.y, dv.z);

	if (nwr < 0)
		return false;

	if (fflush(p_fout->p_file) != 0)
		return false;

	return true;
}

bool file_output_open(file_output_t *p_fout, char *p_filename)
{
	p_fout->p_file = fopen(p_filename, "w");
	if (p_fout->p_file == NULL) {
		p_fout->initialized = false;
		return false;
	}

	p_fout->initialized = true;

	return true;
}

bool file_output_serialize(file_output_t *p_fout, suseconds_t timestamp,
			   mavlink_message_t *p_msg)
{
	if (!p_fout->initialized)
		return false;

	switch (p_msg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT:
		DBG("\nfile_output_serialize(): HEARTBEAT\n");
		break;
	case MAVLINK_MSG_ID_OPTICAL_FLOW:
		DBG("\nfile_output_serialize(): OPTICAL_FLOW\n");
		return serialize_msg_flow(p_fout, timestamp, p_msg);
	case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
		DBG("\nfile_output_serialize(): OPTICAL_FLOW_RAD\n");
		return serialize_msg_flow_rad(p_fout, timestamp, p_msg);
	case MAVLINK_MSG_ID_DEBUG_VECT:
		DBG("\nfile_output_serialize(): DEBUG_VECT\n");
		return serialize_msg_flow_debug_vect(p_fout, timestamp, p_msg);
	case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
		/* Ignore */
		break;
	default:
		DBG("\nfile_output_serialize(): Unknown message ID: %d\n",
		    p_msg->msgid);
		break;
	}

	return false;
}

bool file_output_close(file_output_t *p_fout)
{
	if (p_fout->initialized && fclose(p_fout->p_file) == 0) {
		p_fout->initialized = false;
		return true;
	}

	return false;
}
