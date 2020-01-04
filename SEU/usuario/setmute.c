#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>

int main(int argc, char *argv[]) {
	int sd;
	
	if (argc!=2) {
		fprintf(stderr, "Uso: %s numero (==0 mute off; <>0 mute on)\n",
			argv[0]);
		return 1;
	}
	if ((sd = open("/dev/intspkr", O_RDONLY)) <0) {
		perror("open");
		return 1;
	}
#ifndef SPKR_SET_MUTE_STATE
#error Debe definir el ioctl para la operación mute
#else
	int param = atoi(argv[1]);
	if (ioctl(sd, SPKR_SET_MUTE_STATE, &param) <0) {
		perror("ioctl");
		return 1;
	}
#endif
	return 0;
}

