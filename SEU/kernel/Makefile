ifneq ($(KERNELRELEASE),)
	obj-m := spkr.o
	spkr-objs := spkr-main.o spkr-io.o
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
endif

clean:
	rm -fr .tmp_versions *.*o .*.o .*.o.* .*.ko.* *.mod.c modules.order Module.symvers
