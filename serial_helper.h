

/*
 * File name:   serial_helper.h
 * Create Date: 2014.09.01
 * Autor:       Bruce.zhu
 * Version:     0.1
 */

#ifndef SERIAL_HELPER_H
#define SERIAL_HELPER_H

#include <pthread.h>

struct serial_helper_t {
	char*		dev_path;		// "dev/tty"
	int			baud_rate;		// 115200
	int			data_bits;		// 8
	char		parity;			// 'N'
	int			stop_bits;		// 1
	int	(*serial_notification)(int fd, char* buf, int len);
	int			fd;
	pthread_t	serial_pid;
};

int serial_helper_register(struct serial_helper_t* serial);
int serial_helper_send(struct serial_helper_t* serial, char* buf, unsigned int len);

#endif /* SERIAL_HELPER_H */




