# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
  DEBFLAGS = -O2 -g -DMHINET_DEBUG # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

ccflags-$(DEBUG):=$(DEBFLAGS)

obj-m	:= mhicore.o
mhicore-objs := mhi_init.o mhi_main.o mhi_pm.o mhi_boot.o mhi_dtr.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.o.ur-safe *.symvers *.order .cache.mk


