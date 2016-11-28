#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	int fd;
	bool initialized;
} serial_t;

bool serial_open(serial_t *p_serial, char *p_path);
int serial_write(serial_t *p_serial, uint8_t *p_buffer, int len);
int serial_read(serial_t *p_serial, uint8_t *p_buffer, int len);
bool serial_close(serial_t *p_serial);

#endif
