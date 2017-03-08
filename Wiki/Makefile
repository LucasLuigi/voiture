ARCH=arm
COMPILER=arm-linux-gnueabihf- #NE MARCHE QUE SI LE PATH EST BIEN FAIT
obj-m += dlin.o
KERNELDIR ?= /lib/modules/$(shell uname -r)/build #METTRE LE KERNEL QU'ON VA METTRE SUR LA CARTE
PWD :=$(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(COMPILER) modules
clean:
	$(MAKE) -C $(KERNELDIR) m=$(PWD) ARCH=$(ARCH) clean
