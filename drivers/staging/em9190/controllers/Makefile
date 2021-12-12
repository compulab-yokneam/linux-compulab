# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
  DEBFLAGS = -O2 -g -DMHINET_DEBUG # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

ccflags-$(DEBUG):=$(DEBFLAGS)

obj-m	:= mhictrl.o
mhictrl-objs := mhi_qcom.o mhi_arch_qcom.o ../core/mhi_init.o ../core/mhi_main.o ../core/mhi_pm.o ../core/mhi_boot.o ../core/mhi_dtr.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

	cp ./Module.symvers ../devices/Module.symvers

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.o.ur-safe *.symvers *.order .cache.mk

install:
	sudo cp ./mhictrl.ko /lib/modules/`uname -r`/kernel/drivers/pci/mhictrl.ko

