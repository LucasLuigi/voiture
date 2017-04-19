###########################################################################
# Makefile qui fonctionne avec le noyau Wandboard Solo core-image-minimal #
# Kernel 3.10.17                                                          #
###########################################################################

ARCH=arm
COMPILER=$(shell pwd)/x86_64-linux/usr/bin/cortexa9hf-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-
obj-m := dlin.o
KERNELDIR := $(shell pwd)/kernel
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(COMPILER) modules
clean:
	$(MAKE) -C $(KERNELDIR) m=$(PWD) ARCH=$(ARCH) clean
