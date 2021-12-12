# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
  DEBFLAGS = -O2 -g -DMHINET_DEBUG # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

ccflags-$(DEBUG):=$(DEBFLAGS)

obj-m	:= mhiuci.o mhinet.o mhitty.o
mhiuci-objs := mhi_uci.o qmap.o
mhinet-objs := mhi_netdev.o
mhitty-objs := mhi_tty.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.o.ur-safe *.symvers *.order .cache.mk

install:
	sudo cp ./mhinet.ko /lib/modules/`uname -r`/kernel/drivers/pci/mhinet.ko
	sudo cp ./mhiuci.ko /lib/modules/`uname -r`/kernel/drivers/pci/mhiuci.ko
	sudo cp ./mhiuci.ko /lib/modules/`uname -r`/kernel/drivers/pci/mhitty.ko
	


