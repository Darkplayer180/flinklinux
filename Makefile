# Makefile for the flink linux kernel modules
# 2014-01-14 original version
# 2023-11-20 support for AXI bus added

ifeq ($(KERNELRELEASE),)

ifeq ($(CHROOT),)
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
else
	$(info we are in chroot)
	KERNEL_SRC ?= /usr/src/linux
	CHROOT_CMD ?= schroot -c $(CHROOT) --
endif
	PWD := $(shell pwd)

modules: flink_ioctl.h flink_fmi.h flink_funcid.h
	$(CHROOT_CMD) $(MAKE) -C $(KERNELDIR) M=$(PWD) modules
	
modules_install:
	$(CHROOT_CMD) $(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

else
#	EXTRA_CFLAGS += -DDEBUG
	ccflags-y := -std=gnu99
	obj-m := flink.o

# Default
CONFIG_FLINK_BRIDGES ?= y
ifeq ($(CONFIG_FLINK_BRIDGES),y)
	ccflags-y += -DCONFIG_FLINK_BRIDGES
	obj-m += subsystem_bridges/flink_bridge_gpio.o
	obj-m += subsystem_bridges/flink_bridge_wd.o
endif

ifeq ($(CONFIG_PCI),y) 
#$(info +pci)
	obj-m += flink_pci.o 
endif

ifeq ($(CONFIG_SPI),y) 
#$(info +spi)
	obj-m += flink_spi.o
endif

ifeq ($(CONFIG_PPC_MPC5200_SIMPLE),y)
#$(info +lpb)
	obj-m += mpc5200/flink_lpb.o
endif

ifneq ($(CONFIG_IMX_WEIM),)
#$(info +eim)
	obj-m += imx6/flink_eim.o
endif
	
ifeq ($(CONFIG_ARM_AMBA),y)
#$(info +axi)
	obj-m += zynq/flink_axi.o
endif
	
#$(info +core)
	flink-objs := flink_core.o
endif

clean:
	rm -rf *.o .*.d *~ core .depend .*.cmd *.ko *.mod.c *.mod .tmp_versions modules.order Module.symvers
	rm -rf mpc5200/*.ko mpc5200/*.mod.c mpc5200/*.o
	rm -rf imx6/*.ko imx6/*.mod.c imx6/*.o
	rm -rf zynq/*.ko zynq/*.mod.c zynq/*.o zynq/*.cmd zynq/*.mod
	rm -rf subsystem_bridges/*.ko subsystem_bridges/*.mod.c subsystem_bridges/*.o
	rm -rf subsystem_bridges/.*.cmd subsystem_bridges/*.mod
	rm -f flink_ioctl.h
	rm -f flink_fmi.h
	rm -f flink_funcid.h

flink_ioctl.h: flinkinterface/ioctl/create_flink_ioctl.h.sh flinkinterface/func_id/func_id_definitions.sh
	flinkinterface/ioctl/create_flink_ioctl.h.sh
	
flink_fmi.h: flinkinterface/func_id/create_flink_fmi.h.sh flinkinterface/func_id/func_id_definitions.sh
	flinkinterface/func_id/create_flink_fmi.h.sh

flink_funcid.h: flinkinterface/func_id/create_flink_funcid.h.sh flinkinterface/func_id/func_id_definitions.sh 
	flinkinterface/func_id/create_flink_funcid.h.sh

.PHONY: modules clean

