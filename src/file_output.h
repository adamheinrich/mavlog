#ifndef FILE_OUTPUT_H
#define FILE_OUTPUT_H

#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>
#include <mavlink.h>

typedef struct {
	FILE *p_file;
	bool initialized;
} file_output_t;

bool file_output_open(file_output_t *p_fout, char *p_filename);
bool file_output_serialize(file_output_t *p_fout, suseconds_t timestamp,
			   mavlink_message_t *p_msg);
bool file_output_close(file_output_t *p_fout);

#endif
