### MAKEFILE ###

ARCH=arm
COMPILER=~/Documents/CC/x86_64-linux/usr/bin/cortexa9hf-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-
obj-m := dlin.o
KERNELDIR := ~/Documents/CC/kernel
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(COMPILER) modules
clean:
$(MAKE) -C $(KERNELDIR) m=$(PWD) ARCH=$(ARCH) clean
