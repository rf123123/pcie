
export ARCH := $(shell arch)
#export CROSS_COMPILE :=mips64-octeon-linux-gnu-

obj-m	:= Np_pcieforKt_sf_dma.o

KERNELDIR := /lib/modules/$(shell uname -r)/build
#KERNELDIR := /usr/src/kernels/linux-2.6.27
#KERNELDIR := /usr/src/kernels/2.6.32.20-16.20.201403.ky3.sparcv9
#KERNELDIR := /usr/src/linux-headers-3.16.0-4-686-pae/
#KERNELDIR := /usr/local/Cavium_Networks/OCTEON-SDK/linux/kernel/linux
PWD       := $(shell pwd)
#CC:= $(CROSS_COMPILE)gcc
#LD:= $(CROSS_COMPILE)ld
CC:= gcc
LD:= ld
all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules 

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

