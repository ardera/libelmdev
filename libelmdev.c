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

inline speed_t serialSpeedFromBaudrate(int baudrate) {
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
	assert(elmdev);
	assert(elmdev->is_online);
	assert(text);

	int count = 0, ok = 0;

	ok = pselect(elmdev->fd+1, NULL, &elmdev->fdset, NULL, &(elmdev->timeout), NULL);
	if (ok > 0) {
		tcflush(elmdev->fd, TCIOFLUSH);

		// why do we write byte per byte here?
		for (int i=0; i < strlen(text); i++)
			count += write(elmdev->fd, (const void *) &text[i], sizeof(char));

		if (count != strlen(text)) {
			fprintf(stderr,
					"could not write command to serial, written %d bytes, should be %d\n",
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
bool elmdev_elm_read(struct elmdev *elmdev, char* buffer, size_t length) {
	assert(elmdev);
	assert(elmdev->is_online);
	assert(buffer != NULL);

	int i = 0, ok = 0, count = 0;
	char charbuff = 0;

	while (1) {
		ok = pselect(elmdev->fd+1, &elmdev->fdset, NULL, NULL, &elmdev->timeout, NULL);

		if (ok > 0) {
			read(elmdev->fd, &charbuff, sizeof(char));
			if (isprint(charbuff))	printf("%c", charbuff);
			else 					printf("\\x%02x", charbuff);

			if (charbuff == '>') {
				buffer[i] = '\0';
				printf("\n");
				return true;
			} else {
				buffer[i] = charbuff;
				i = (i+1) % length;
			}
		} else {
			fprintf(stderr,
					"ELM327 connection timed out while reading, after %lus, %09luns\n",
					elmdev->timeout.tv_sec, elmdev->timeout.tv_nsec);
			return false;
		}
	}
}
bool elmdev_elm_command(struct elmdev *elmdev, char* cmd, char* response, size_t length) {
	assert(elmdev);
	assert(elmdev->is_online);
	assert(cmd);
	
	char* text = malloc(strlen(cmd)+3);
	snprintf(text, strlen(cmd)+3, "%s" ELM327_EOC, cmd);

	elmdev_elm_send_text(elmdev, text);
	free(text);

	elmdev_elm_read(elmdev, response, length);

	if (strncmp(&response[strlen(response) - sizeof(ELM327_NODATA)], ELM327_NODATA, sizeof(ELM327_NODATA)-1) == 0) {
		fprintf(stderr, "OBD port didn't reply. Is the ELM327 device setup correctly?\n");
		return false;
	} else if (strncmp(response, ELM327_NOCONN, sizeof(ELM327_NOCONN) - 1) == 0) {
		fprintf(stderr, "Could not communicate with ECU.\n");
		return false;
	}

	return true;
}
bool elmdev_query(struct elmdev *elmdev, uint8_t pid, uint32_t* response) {

	char command[5];
	sprintf(command, "01%02X", pid);

	char txt_response[32];
	txt_response[31] = '\0';

	bool ok = elmdev_elm_command(elmdev, command, txt_response, 32);
	if (!ok) return false;

	printf("asked for \"%s\", response: \"%s\"\n", command, txt_response);

	return true;
}


bool elmdev_init_serial(struct elmdev *elmdev, char* serial_path, int baudrate) {
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
		errno = ENOMEM;
		return NULL;
	}

	elmdev->timeout.tv_sec = 5;
	elmdev->timeout.tv_nsec = 0;

	elmdev->is_online = false;

	ok = elmdev_init_serial(elmdev, serial_path, baudrate);
	if (!ok) goto out_err;

	elmdev->is_online = true;
	ok = elmdev_elm_send_text(elmdev, ELM327_RESET ELM327_EOC);
	if (!ok) goto out_err;
	sleep(2);

	ok = elmdev_elm_read(elmdev, elmdev->version, 64);
	if (!ok) goto out_err;

	ok = elmdev_elm_command(elmdev, ELM327_ECHO_OFF, NULL, 0);
	if (!ok) goto out_err;

	ok = elmdev_elm_command(elmdev, ELM327_LINEFEEDS_OFF, NULL, 0);
	if (!ok) goto out_err;

	tcdrain(elmdev->fd);
	nanosleep(&(struct timespec) {.tv_sec=0, .tv_nsec = 100000000}, NULL);
	tcflush(elmdev->fd, TCIOFLUSH);

	return elmdev;



	out_err:

	if (elmdev) free(elmdev);

	return NULL;
}
bool elmdev_destroy(struct elmdev *elmdev) {
	cfsetispeed(&elmdev->tty, B0);
	cfsetospeed(&elmdev->tty, B0);
	
	close(elmdev->fd);

	free(elmdev);

	return true;
}

int main(int argc, char** argv) {
	struct elmdev *elmdev;
	uint32_t rpm;
	bool ok;

	elmdev = elmdev_open("/dev/rfcomm0", 9600);

	rpm = 0;
	ok = elmdev_query(elmdev, OBDII_PID_ENGINE_RPM, &rpm);
	if (!ok) {
		fprintf(stderr, "Could not query engine RPM\n");
		return EXIT_FAILURE;
	}

	ok = elmdev_destroy(elmdev);
	if (!ok) {
		fprintf(stderr, "Could not destroy elmdev\n");
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
