#include "serial.h"
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

bool serial_open(serial_t *p_serial, char *p_path)
{
	p_serial->fd = open(p_path, (O_RDWR | O_NOCTTY | O_NDELAY));

	if (p_serial->fd == -1)
		return false;

	fcntl(p_serial->fd, F_SETFL, 0);

	struct termios options;
	tcgetattr(p_serial->fd, &options);

	/* Inspired by https://github.com/mavlink/c_uart_interface_example/blob/master/serial_port.cpp: */
	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK |
			     ISTRIP | IXON);
	options.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;

	options.c_cc[VMIN] = 1; /* minimum bytes to return from read() */
	options.c_cc[VTIME] = 10; /* inter-character timer */

	/* Baudrate has no effect when using the USB/UART converter */
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	tcsetattr(p_serial->fd, TCSANOW, &options);

	return true;
}

int serial_write(serial_t *p_serial, uint8_t *p_buffer, int len)
{
	return write(p_serial->fd, p_buffer, len);
}

int serial_read(serial_t *p_serial, uint8_t *p_buffer, int len)
{
	return read(p_serial->fd, p_buffer, len);
}

bool serial_close(serial_t *p_serial)
{
	if (close(p_serial->fd) == 0)
		return true;

	return false;
}
