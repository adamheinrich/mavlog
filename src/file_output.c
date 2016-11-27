#include "file_output.h"
#include <stdio.h>

#ifdef DEBUG
#define DBG	printf
#else
#define DBG(...)
#endif

static FILE *m_fh;
static bool m_initialized;

static bool serialize_msg_flow(suseconds_t timestamp, mavlink_message_t *p_msg)
{
	mavlink_optical_flow_t of;
	mavlink_msg_optical_flow_decode(p_msg, &of);

	int nwr = fprintf(m_fh, "%lu;%d;%lu;%f;%f;%f;%d;%d;%d;%d;0;0\n",
			  timestamp, p_msg->msgid, of.time_usec,
			  of.flow_comp_m_x, of.flow_comp_m_y,
			  of.ground_distance, of.flow_x, of.flow_y,
			  of.sensor_id, of.quality);

	if (nwr < 0)
		return false;

	if (fflush(m_fh) != 0)
		return false;

	return true;
}

static bool serialize_msg_flow_rad(suseconds_t timestamp,
				   mavlink_message_t *p_msg)
{
	mavlink_optical_flow_rad_t of;
	mavlink_msg_optical_flow_rad_decode(p_msg, &of);

	int nwr = fprintf(m_fh, "%lu;%d;%lu;%f;%f;%f;%f;%f;%u;%f;"
			  "%d;%d\n", timestamp, p_msg->msgid, of.time_usec,
			  of.integrated_x, of.integrated_y, of.integrated_xgyro,
			  of.integrated_ygyro, of.integrated_zgyro,
			  of.time_delta_distance_us, of.distance,
			  of.temperature, of.quality);

	if (nwr < 0)
		return false;

	if (fflush(m_fh) != 0)
		return false;

	return true;
}

bool file_output_init(char *p_filename)
{
	m_fh = fopen(p_filename, "w");
	if (m_fh == NULL) {
		m_initialized = false;
		return false;
	}

	m_initialized = true;

	return true;
}

bool file_output_serialize(suseconds_t timestamp, mavlink_message_t *p_msg)
{
	if (!m_initialized)
		return false;

	switch (p_msg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT:
		DBG("\nfile_output_serialize(): HEARTBEAT\n");
		break;
	case MAVLINK_MSG_ID_OPTICAL_FLOW:
		DBG("\nfile_output_serialize(): OPTICAL_FLOW\n");
		return serialize_msg_flow(timestamp, p_msg);
	case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
		DBG("\nfile_output_serialize(): OPTICAL_FLOW_RAD\n");
		return serialize_msg_flow_rad(timestamp, p_msg);
	case MAVLINK_MSG_ID_DEBUG_VECT:
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

bool file_output_close(void)
{
	if (m_initialized && fclose(m_fh) == 0)
		return true;

	return false;
}
