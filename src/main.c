#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include <mavlink.h>
#include "file_output.h"

#define SYS_ID_SELF		42
#define COMP_ID_SELF		MAV_COMP_ID_LOG
#define SYS_ID_PF4FLOW		81
#define COMP_ID_PF4FLOW		50

static volatile sig_atomic_t m_run;

static void sigint_handler(int signum)
{
	m_run = false;
}

static suseconds_t microseconds()
{
	static struct timeval tv;
	gettimeofday(&tv, NULL);

	return ((tv.tv_sec * 1000000UL) + tv.tv_usec);
}

static int open_port(char *p_path) {
	int fd = open(p_path, (O_RDWR | O_NOCTTY | O_NDELAY));

	if (fd == -1)
		return -1;

	fcntl(fd, F_SETFL, 0);

	struct termios options;
	tcgetattr(fd, &options);

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

	tcsetattr(fd, TCSANOW, &options);

	return fd;
}

static bool set_params(int fd)
{
	uint8_t buffer[64];
	mavlink_message_t msg;

	mavlink_msg_param_set_pack(SYS_ID_SELF, COMP_ID_SELF, &msg,
				   SYS_ID_PF4FLOW, COMP_ID_PF4FLOW,
				   "USB_SEND_VIDEO", 0, MAV_PARAM_TYPE_UINT8);

	int len = mavlink_msg_to_send_buffer(buffer, &msg);
	int nwr = write(fd, buffer, len);
	if (nwr != len)
		return false;

	return true;
}

int main(int argc, char **argv)
{
	char buffer[32];
	char *p_portname;
	char *p_filename;
	mavlink_message_t msg;
	mavlink_status_t status;
	bool verbose = false;

	/* Check input parameters: */
	if (argc < 3) {
		fprintf(stderr, "Usage: %s [--verbose] <serial_port> "
				"<output_file>\n", argv[0]);
		return 1;
	} else {
		if (argc > 3) {
			if (strcmp(argv[1], "-v") == 0 ||
			    strcmp(argv[1], "--verbose") == 0)
				verbose = true;

			p_portname = argv[2];
			p_filename = argv[3];
		} else {
			p_portname = argv[1];
			p_filename = argv[2];
		}
	}

	/* Open output file: */
	if (verbose)
		printf("%s: Opening file %s\n", argv[0], p_filename);

	if (!file_output_init(p_filename)) {
		fprintf(stderr, "%s: Cannot open file %s\n", argv[0],
			p_filename);
		return 1;
	}

	/* Open serial port: */
	if (verbose)
		printf("%s: Opening port %s\n", argv[0], p_portname);

	int fd = open_port(p_portname);
	if (fd == -1) {
		fprintf(stderr, "%s: Cannot open port %s\n", argv[0],
			p_portname);
		file_output_close();
		return 1;
	}

	/* Set parameters and start listening: */
	if (verbose)
		printf("%s: Setting parameters\n", argv[0]);

	if (!set_params(fd)) {
		fprintf(stderr, "%s: Unable to set parameters\n", argv[0]);
	} else {
		signal(SIGINT, sigint_handler);
		m_run = true;

		if (verbose)
			printf("%s: Listening!\n", argv[0]);
	}

	int count_recvd = 0;
	int count_saved = 0;

	while (m_run) {
		int nread = read(fd, buffer, sizeof(buffer));

		for (int i = 0; i < nread; i++) {
			if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg,
					       &status)) {
				count_recvd++;
				if (file_output_serialize(microseconds(), &msg))
					count_saved++;

				if (verbose) {
					printf("\r%s: Saved %d of %d messages",
					       argv[0], count_saved,
					       count_recvd);
					fflush(stdout);
				}
			}
		}
	}

	/* Close everything: */
	file_output_close();
	close(fd);

	if (verbose)
		printf("\n%s: Finished\n", argv[0]);

	return 0;
}
