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

static volatile sig_atomic_t m_run = true;

static void sigint_handler(int signum)
{
    m_run = false;
}

static int open_port(char *p_path) {
	int fd = open(p_path, (O_RDWR | O_NOCTTY | O_NDELAY));

	if (fd == -1)
		return -1;

	fcntl(fd, F_SETFL, 0);

	struct termios options;
	tcgetattr(fd, &options);

	/* Inspired by https://github.com/mavlink/c_uart_interface_example/blob/master/serial_port.cpp: */
	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
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

static void serialize(mavlink_message_t *p_msg)
{
	switch (p_msg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT:
		printf("MAVLINK_MSG_ID_HEARTBEAT\n");
		break;
	case MAVLINK_MSG_ID_OPTICAL_FLOW:
	{
		printf("MAVLINK_MSG_ID_OPTICAL_FLOW\n");

		mavlink_optical_flow_t msg_flow;
		mavlink_msg_optical_flow_decode(p_msg, &msg_flow);
		break;
	}
	case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
	{
		printf("MAVLINK_MSG_ID_OPTICAL_FLOW_RAD\n");

		mavlink_optical_flow_rad_t msg_flow_rad;
		mavlink_msg_optical_flow_rad_decode(p_msg, &msg_flow_rad);
		break;
	}
	case MAVLINK_MSG_ID_DEBUG_VECT:
	case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
		/* Ignore */
		break;
	default:
		printf("Unknown message ID: %d\n", p_msg->msgid);
		break;
	}
}

static bool set_params(int fd)
{
	uint8_t buffer[64];
	mavlink_message_t msg;

	mavlink_msg_param_set_pack(42, MAV_COMP_ID_LOG, &msg, 81, 50,
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

	if (argc < 2) {
		fprintf(stderr, "Usage: %s <port>\n", argv[0]);
		return 1;
	} else {
		p_portname = argv[1];
	}

	signal(SIGINT, sigint_handler);

	mavlink_message_t msg;
	mavlink_status_t status;

	int fd = open_port(p_portname);

	if (fd == -1) {
		fprintf(stderr, "%s: Cannot open port %s\n", argv[0], p_portname);
		return 1;
	}

	printf("%s: Setting parameters\n", argv[0]);

	if (!set_params(fd)) {
		fprintf(stderr, "%s: Unable to set parameters\n", argv[0]);
		m_run = false;
	} else {
		printf("%s: Listening on %s\n", argv[0], p_portname);
	}

	while (m_run) {
		int nread = read(fd, buffer, sizeof(buffer));

		for (int i = 0; i < nread; i++)
			if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
				serialize(&msg);
	}

	close(fd);

	return 0;
}
