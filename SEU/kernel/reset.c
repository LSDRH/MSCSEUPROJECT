#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>

//IOCTL
#define MAGIC_NUM '9'
#define SPKR_SET_MUTE_STATE _IOR(MAGIC_NUM, 1, int *) 
#define SPKR_GET_MUTE_STATE _IOR(MAGIC_NUM, 2, int *) 
#define SPKR_RESET  _IO(MAGIC_NUM, 3)

int main(int argc, char *argv[]) {
	int sd;

	if ((sd = open("/dev/intspkr", O_RDONLY)) <0) {
		perror("open");
		return 1;
	}
#ifndef SPKR_RESET
#error Debe definir el ioctl para la operación reset
#else
	if (ioctl(sd, SPKR_RESET, 0) <0) {
		perror("ioctl");
		return 1;
	}
#endif
	if (fsync(sd) <0) {
		perror("fsync");
		return 1;
        }

	return 0;
}

