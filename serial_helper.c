/*
 * File name:   serial_helper.c
 * Create Date: 2014.09.01
 * Autor:       Bruce.zhu
 * Version:     0.1
 */

#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "serial_helper.h"


#define SERIAL_RECV_BUFFER_SIZE		128

static int speed_arr[] = {  B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
							B38400, B19200, B9600, B4800, B2400, B1200, B300, };
static int name_arr[]  = {	115200, 38400,  19200,  9600,  4800,  2400,  1200,  300,
							38400,  19200,  9600, 4800, 2400, 1200,  300, };

static struct termios	save_termios;


static int
tty_reset(int fd)		/* restore terminal's mode */
{
	if (tcsetattr(fd, TCSAFLUSH, &save_termios) < 0)
		return(-1);

	return 0;
}


/* put terminal into a raw mode */
static int
tty_raw(int fd, int data_bits)
{
	//int				err;
	struct termios	buf;

	if (tcgetattr(fd, &buf) < 0)
		return(-1);
	save_termios = buf;	/* structure copy */

	/*
	 * Echo off, canonical mode off, extended input
	 * processing off, signal chars off.
	 */
	buf.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

	/*
	 * No SIGINT on BREAK, CR-to-NL off, input parity
	 * check off, don't strip 8th bit on input, output
	 * flow control off.
	 */
	buf.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

	/*
	 * Clear size bits, parity checking off.
	 */
	buf.c_cflag &= ~(CSIZE | PARENB);

	/*
	 * Set 8 bits/char.
	 */
	switch (data_bits)	{
	case 7:
		buf.c_cflag |= CS7;
		break;
	case 8:
		buf.c_cflag |= CS8;
		break;
	default:
		buf.c_cflag |= CS8;
		break;
	}

	/*
	 * Output processing off.
	 */
	buf.c_oflag &= ~(OPOST);

	/*
	 * Case B: BUFFER_SIZE bytes at a time, 1 second interbyte timer.
	 */
	buf.c_cc[VMIN] = SERIAL_RECV_BUFFER_SIZE;
	buf.c_cc[VTIME] = 1;
	if (tcsetattr(fd, TCSAFLUSH, &buf) < 0)
		return(-1);

#if 0
	/*
	 * Verify that the changes stuck.  tcsetattr can return 0 on
	 * partial success.
	 */
	if (tcgetattr(fd, &buf) < 0) {
		err = errno;
		tcsetattr(fd, TCSAFLUSH, &save_termios);
		errno = err;
		return(-1);
	}

	if ((buf.c_lflag & (ECHO | ICANON | IEXTEN | ISIG)) ||
	  (buf.c_iflag & (BRKINT | ICRNL | INPCK | ISTRIP | IXON)) ||
	  (buf.c_cflag & (CSIZE | PARENB | CS8)) != CS8 ||
	  (buf.c_oflag & OPOST) || buf.c_cc[VMIN] != 1 ||
	  buf.c_cc[VTIME] != 0) {
		/*
		 * Only some of the changes were made.  Restore the
		 * original settings.
		 */
		tcsetattr(fd, TCSAFLUSH, &save_termios);
		errno = EINVAL;
		return(-1);
	}
#endif

	return(0);
}


/*
 * 0 -----> success
 * other -> fail
 */
static int
tty_set_speed(int fd, int speed)
{
	int   i;
	struct termios Opt;

	tcgetattr(fd, &Opt);

	for ( i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
		if  (speed == name_arr[i]) {	// find the correct speed
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
			if (tcsetattr(fd, TCSANOW, &Opt))
				return -1;

			break;
		}
	}

	if (i == sizeof(speed_arr) / sizeof(int))
		return -2;

	return 0;
}


static void *
serial_read_thread(void *arg)
{
	struct serial_helper_t* serial = (struct serial_helper_t*)arg;
	char buffer[SERIAL_RECV_BUFFER_SIZE];
	int len;

	for (;;) {
		len = read(serial->fd, buffer, SERIAL_RECV_BUFFER_SIZE);
		if (len <= 0) {
			printf("[%d]read error\n", serial->fd);
			pthread_exit((void*)1);
		}

		serial->serial_notification(serial->fd, buffer, len);
	}
}


/*
 * retval: 0 ------> success
 *         others -> fail
 */
int
serial_helper_register(struct serial_helper_t* serial)
{
	int fd;

	if (!serial->serial_notification)
		goto fail1;

	fd = open(serial->dev_path, O_RDWR);
	if (fd < 0)
		goto fail1;
	if (tty_raw(fd, serial->data_bits) < 0)
		goto fail2;
	if (tty_set_speed(fd, serial->baud_rate))
		goto fail3;

	serial->fd = fd;

	// create serial read thread
	if (pthread_create(&serial->serial_pid, NULL, serial_read_thread, (void *)serial))
		goto fail3;

	return 0;

fail3:
	tty_reset(fd);
fail2:
	close(fd);
fail1:
	return -1;
}


int
serial_helper_send(struct serial_helper_t* serial, char* buf, unsigned int len)
{
	if (serial->fd > 0) {
		return write(serial->fd, buf, len);
	}

	return -1;
}
