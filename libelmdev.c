#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "libelmdev.h"

#ifdef _DEBUG
#define ERRLOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define ERRLOG(...) do {} while (0)
#endif

speed_t serialSpeedFromBaudrate(int baudrate) {
	switch (baudrate) {
		case 0:	return B0;
		case 50: return B50;
		case 75: return B75;
		case 110: return B110;
		case 134: return B134;
		case 150: return B150;
		case 200: return B200;
		case 300: return B300;
		case 600: return B600;
		case 1200: return B1200;
		case 1800: return B1800;
		case 2400: return B2400;
		case 4800: return B4800;
		case 9600: return B9600;
		case 19200: return B19200;
		case 38400: return B38400;
		case 57600: return B57600;
		case 115200: return B115200;
		case 230400: return B230400;
		default: return -1;
	}
}


bool elmdev_elm_send_text(struct elmdev *elmdev, char *text) {
	assert(elmdev && "elmdev_elm_send_text: elmdev can't be NULL");
	assert(elmdev->is_online && "elmdev_elm_send_text: elmdev must be online");
	assert(text && "elmdev_elm_send_text: text can't be NULL");

	int count = 0, ok = 0;

	ok = pselect(elmdev->fd+1, NULL, &elmdev->fdset, NULL, &(elmdev->timeout), NULL);
	if (ok > 0) {
		tcflush(elmdev->fd, TCIOFLUSH); 

		// why do we write byte per byte here?
		for (int i=0; i < strlen(text); i++)
			count += write(elmdev->fd, (const void *) &text[i], sizeof(char));

		if (count != strlen(text)) {
			fprintf(stderr,
					"could not write command to serial, written %d bytes, should be %ld\n",
					count, strlen(text));
			return false;
		}

		return true;
	} else {
		fprintf(stderr,
				"elm connection timed out while writing, after %lus, %09luns\n",
				elmdev->timeout.tv_sec, elmdev->timeout.tv_nsec);
		return false;
	}
}
bool elmdev_elm_read(struct elmdev *elmdev, char *buffer, size_t length) {
	assert(elmdev && "elmdev_elm_read: elmdev can't be NULL");
	assert(elmdev->is_online && "elmdev_elm_read: elmdev must be online");
	//assert(buffer != NULL && "elmdev_elm_read: buffer can't be NULL");

	int i = 0, ok = 0, count = 0;
	char charbuff = 0;

	while (1) {
		ok = pselect(elmdev->fd+1, &elmdev->fdset, NULL, NULL, &elmdev->timeout, NULL);

		if (ok > 0) {
			ok = read(elmdev->fd, &charbuff, sizeof(char));

			if (isprint(charbuff))	printf("%c", charbuff);
			else 					printf("\\x%02x", charbuff);

			if (charbuff == '>') {
				if (buffer) buffer[i] = '\0';
				printf("\n");
				return true;
			} else if (buffer) {
				buffer[i] = charbuff;
				i = (i+1) % length;
			}
		} else {
			printf("\n");
			fprintf(stderr,
					"ELM327 connection timed out while reading, after %lus, %09luns\n",
					elmdev->timeout.tv_sec, elmdev->timeout.tv_nsec);
			return false;
		}
	}
}
bool elmdev_elm_command(struct elmdev *elmdev, char *cmd, char *response, size_t length) {
	char *text;
	bool ok;

	assert(elmdev && "elmdev_elm_command: elmdev can't be NULL");
	assert(elmdev->is_online && "elmdev_elm_command: elmdev must be online");
	assert(cmd && "elmdev_elm_command: command can't be NULL");
	
	text = malloc(strlen(cmd)+3);
	snprintf(text, strlen(cmd)+3, "%s" ELM327_EOC, cmd);

	ok = elmdev_elm_send_text(elmdev, text);
	free(text);

	ok = elmdev_elm_read(elmdev, response, length);

	return true;
}
bool elmdev_scan_for_errors(struct elmdev *elmdev, char *buffer) {
	enum elm_errno e = ELM_ERRNO_OK;

	if (strstr(buffer, "ERROR")) {
		if (strstr(buffer, ELM327_BUS_ERROR))
			e = ELM_ERRNO_BUS_ERROR;
		else if (strstr(buffer, ELM327_CAN_ERROR))
			e = ELM_ERRNO_CAN_ERROR;
		else if (strstr(buffer, ELM327_LINE_DATA_ERROR))
			e = ELM_ERRNO_LINE_DATA_ERROR;
		else if (strstr(buffer, ELM327_DATA_ERROR))
			e = ELM_ERRNO_DATA_ERROR;
		else if (strstr(buffer, ELM327_FEEDBACK_ERROR))
			e = ELM_ERRNO_FEEDBACK_ERROR;
		else if (strstr(buffer, ELM327_LINE_RX_ERROR))
			e = ELM_ERRNO_LINE_RX_ERROR;
	} else if (strstr(buffer, ELM327_OK))
		e = ELM_ERRNO_OK;
	else if (strstr(buffer, ELM327_INVALID))
		e = ELM_ERRNO_INVALID;
	else if (strstr(buffer, ELM327_ACT_ALERT))
		e = ELM_ERRNO_ACT_ALERT;
	else if (strstr(buffer, ELM327_BUFFER_FULL))
		e = ELM_ERRNO_BUFFER_FULL;
	else if (strstr(buffer, ELM327_BUS_BUSY))
		e = ELM_ERRNO_BUS_BUSY;
	else if (strstr(buffer, ELM327_LOW_POWER_ALERT))
		e = ELM_ERRNO_LOW_POWER_ALERT;
	else if (strstr(buffer, ELM327_LOW_VOLTAGE_RESET))
		e = ELM_ERRNO_LOW_VOLTAGE_RESET;
	else if (strstr(buffer, ELM327_NO_DATA))
		e = ELM_ERRNO_NO_DATA;
	else if (strstr(buffer, ELM327_STOPPED))
		e = ELM_ERRNO_STOPPED;
	else if (strstr(buffer, ELM327_NOCONN))
		e = ELM_ERRNO_NOCONN;
	else if (strstr(buffer, ELM327_SEARCHING))
		e = ELM_ERRNO_SEARCHING;
	
	elmdev->elm_errno = e;
}
bool elmdev_query(struct elmdev *elmdev, uint8_t pid, uint32_t* response) {
	char command[5];
	char txt_response[32];
	size_t response_length = 0;
	uint8_t bytes[6] = {0};
	*response = 0;

	sprintf(command, "01%02X", pid);

	txt_response[31] = '\0';

	bool ok = elmdev_elm_command(elmdev, command, txt_response, 32);
	if (!ok) return false;

	elmdev_scan_for_errors(elmdev, txt_response);

	if (elmdev->elm_errno != ELM_ERRNO_OK) {
		fprintf(stderr, "elmdev_query: query was not successful. ELM_ERRNO: %d\n", elmdev->elm_errno);
		return false;
	}

	response_length = strlen(txt_response);
	
	// remove all trailing carriage-returns
	for (int i = response_length-1; i>=0; i--) {
		if (txt_response[i] == 0x0D) {
			txt_response[i] = 0x00;
			response_length--;
		} else {
			break;
		}
	}

	// remove all remaining carriage-returns
	int n_move = 0;
	for (int i = 0; i <= response_length; i++)
		if (txt_response[i] == 0x0D) n_move++;
		else if (n_move) 			 txt_response[i-n_move] = txt_response[i];
	response_length -= n_move;

	printf("asked for \"%s\", response: \"%s\"\n", command, txt_response);

	int res = sscanf(txt_response, "%2hhX%2hhX%2hhX%2hhX%2hhX%2hhX",
					 bytes, bytes+1, bytes+2, bytes+3, bytes+4, bytes+5);
	
	if (res == EOF) {
		fprintf(stderr, "elmdev_elm_query: string matching error ocurred\n");
		return false;
	} else if ((res >= 0) && (res <= 2)) {
		fprintf(stderr, "elmdev_elm_query: unexpected ELM327 reply\n");
		return false;
	}


	for (int i = 2; i < res; i++)
		*response = (*response << 8) | bytes[i];

	return true;
}
bool elmdev_pid_supported(struct elmdev *elmdev, uint8_t pid) {
	if (pid == 0x00) return true;
	
	uint8_t pid_bank = (pid-1) >> 5;
	uint8_t pid_index = (pid-1) & 0x1F;
			pid_index = 0x1f - pid_index;

	return (elmdev->supported_pids[pid_bank] & (1 << pid_index)) && 1;
}


bool 		   elmdev_destroy(struct elmdev *elmdev) {
	cfsetispeed(&elmdev->tty, B0);
	cfsetospeed(&elmdev->tty, B0);
	
	close(elmdev->fd);

	free(elmdev);

	return true;
}
bool 		   elmdev_init_serial(struct elmdev *elmdev, char* serial_path, int baudrate) {
	int ok;

	elmdev->fd = open(serial_path, O_RDWR | O_NOCTTY | O_NDELAY);
	if (elmdev->fd < 0) {
		fprintf(stderr, "Could not open serial device \"%s\": %s\n", serial_path, strerror(errno));
		errno = EIO;
		return false;
	}

	ok = tcgetattr(elmdev->fd, &(elmdev->tty));

	elmdev->tty.c_cflag |=  (CLOCAL|CREAD);
	elmdev->tty.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|ISIG|IEXTEN);
	elmdev->tty.c_iflag &= ~(INLCR|IGNCR|ICRNL|IGNBRK|IUCLC|PARMRK|
							 INPCK|ISTRIP|IXON|IXOFF|IXANY);
	elmdev->tty.c_oflag &= ~(OPOST);
	
	elmdev->tty.c_cc[VMIN] = 0;
	elmdev->tty.c_cc[VTIME]= 0;

	speed_t serial_speed = serialSpeedFromBaudrate(baudrate);
	if (serial_speed == -1) {
		fprintf(stderr, "Not a valid baudrate: %d\n", baudrate);
		errno = EINVAL;
		return false;
	}

	cfsetispeed(&(elmdev->tty), serial_speed);
	cfsetospeed(&(elmdev->tty), serial_speed);

	tcsetattr(elmdev->fd, TCSANOW, &(elmdev->tty));

	FD_ZERO(&elmdev->fdset);
	FD_SET(elmdev->fd, &elmdev->fdset);

	return true;
}
struct elmdev *elmdev_open(char *serial_path, int baudrate) {
	bool ok;
	struct elmdev *elmdev;
	printf("Opening ELM327 at \"%s\"\n", serial_path);

	elmdev = (struct elmdev *) malloc(sizeof(struct elmdev));
	if (!elmdev) {
		fprintf(stderr, "Could not allocate memory\n");
		return NULL;
	}

	elmdev->timeout.tv_sec = 5;
	elmdev->timeout.tv_nsec = 0;

	elmdev->is_online = false;

	printf("elmdev_init_serial\n");
	ok = elmdev_init_serial(elmdev, serial_path, baudrate);
	if (!ok) goto out_err;

	printf("elmdev ATZ\n");
	elmdev->is_online = true;
	ok = elmdev_elm_send_text(elmdev, ELM327_RESET ELM327_EOC);
	if (!ok) goto out_err;
	sleep(2);

	printf("elmdev read version\n");
	ok = elmdev_elm_read(elmdev, elmdev->version, 64);
	if (!ok) goto out_err;

	printf("elmdev AT E0\n");
	ok = elmdev_elm_command(elmdev, ELM327_ECHO_OFF, NULL, 0);
	if (!ok) goto out_err;

	printf("elmdev AT L0\n");
	ok = elmdev_elm_command(elmdev, ELM327_LINEFEEDS_OFF, NULL, 0);
	if (!ok) goto out_err;

	printf("finished elmdev setup\n");

	tcdrain(elmdev->fd);
	nanosleep(&(struct timespec) {.tv_sec=0, .tv_nsec = 100000000}, NULL);
	tcflush(elmdev->fd, TCIOFLUSH);

	printf("querying supported PIDs\n");

	ok = elmdev_query(elmdev, 0x00, elmdev->supported_pids);
	if (!ok && (elmdev->elm_errno != ELM_ERRNO_SEARCHING)) goto out_err;
	
	for (int i = 0; i < 8; i++) {
		printf("is PID 0x%02X supported? %s\n", i*0x20, elmdev_pid_supported(elmdev, i*0x20) ? "yes" : "no");
		ok = elmdev_query(elmdev, i*0x20, &elmdev->supported_pids[i]);
		if (!ok) goto out_err;
	}

	return elmdev;


	out_err:
	if (elmdev) elmdev_destroy(elmdev);

	return NULL;
}

int main(int argc, char** argv) {
	struct elmdev *elmdev;
	uint32_t rpm;
	bool ok;
	char *path = "/dev/rfcomm0";

	if (argc >= 2)
		path = argv[1];

	printf("Opening ELMdev at \"%s\"\n", path);
	elmdev = elmdev_open("/dev/rfcomm0", 9600);

	printf("Querying Engine RPM\n");
	rpm = 0;
	ok = elmdev_query(elmdev, OBDII_PID_ENGINE_RPM, &rpm);
	if (!ok) {
		fprintf(stderr, "Could not query engine RPM\n");
		return EXIT_FAILURE;
	}
	printf("engine RPM: %.2f\n", rpm/4.0);

	printf("Destroying elmdev\n");
	ok = elmdev_destroy(elmdev);
	if (!ok) {
		fprintf(stderr, "Could not destroy elmdev\n");
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
