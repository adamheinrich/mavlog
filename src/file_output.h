#ifndef FILE_OUTPUT_H
#define FILE_OUTPUT_H

#include <stdbool.h>
#include <sys/time.h>
#include <mavlink.h>

bool file_output_init(char *p_filename);
bool file_output_serialize(suseconds_t timestamp, mavlink_message_t *p_msg);
bool file_output_close(void);

#endif
