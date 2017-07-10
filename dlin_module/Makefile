################
### MAKEFILE ###
################

# Compiler for specific target :
COMPILER=/home/pleuron/Documents/STAGE_2017/workspace/sdk/sysroots/x86_64-gadsdk-linux/usr/bin/arm-gad-linux-gnueabi/arm-gad-linux-gnueabi-

# Target Kernel sources directory (from build directory) :
KERNELDIR := /home/pleuron/workspace/yocto/TGUR_ACCESS/build-tgur-access-b1/tmp-eglibc/sysroots/tgur-access-b1/usr/src/kernel

# Target Kernel sources directory (from SDK):
#KERNELDIR :=/home/pleuron/workspace/yocto/TGUR_ACCESS/sdk/sysroots/cortexa9hf-vfp-neon-gad-linux-gnueabi/usr/src/kernel

# Local PATH
PWD := $(shell pwd)

# kbuild building
obj-m := dlin.o

# standart building
default:
	$(MAKE) -C $(KERNELDIR)  M=$(PWD) CROSS_COMPILE=$(COMPILER) modules
clean:
	$(MAKE) -C $(KERNELDIR)  m=$(PWD) clean


