ARCH=arm
COMPILER=arm-poky-linux-gnueabi-
obj-m := dlin.o
KERNELDIR := /home/lukko/fsl-community-bsp/build/tmp/sysroots/wandboard-solo/usr/src/kernel
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(COMPILER) modules
clean:
	$(MAKE) -C $(KERNELDIR) m=$(PWD) ARCH=$(ARCH) clean
