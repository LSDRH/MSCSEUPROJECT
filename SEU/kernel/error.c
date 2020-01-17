#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>

int main(int argc, char *argv[]) {
	int sd;
	void *p;
	
	if ((sd = open("/dev/intspkr", O_WRONLY)) <0) {
		perror("open");
		return 1;
	}
	if ((p = mmap(NULL, getpagesize(), PROT_READ, MAP_ANONYMOUS|MAP_PRIVATE,
		-1, 0)) == MAP_FAILED) { 
		perror("mmap");
		close(sd);
		return 1;
	}
	munmap(p, getpagesize());
	if ((write(sd, p, 4)<0) && (errno == EFAULT)) 
		printf("Prueba correcta\n");
	else
		printf("Prueba incorrecta\n");
	close(sd);
	return 0;
}

