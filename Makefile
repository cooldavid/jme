MODNAME := jme
obj-m := $(MODNAME).o

ifneq ($(KERNELRELEASE),)
#########################
# kbuild part of makefile
#########################
EXTRA_CFLAGS += -Wall -O3
#EXTRA_CFLAGS += -DTX_DEBUG
#EXTRA_CFLAGS += -DREG_DEBUG

else
#########################
# Normal Makefile
#########################
TEMPFILES := $(MODNAME).o $(MODNAME).mod.c $(MODNAME).mod.o Module.symvers .$(MODNAME).*.cmd .tmp_versions modules.order Module.markers Modules.symvers

ifeq (,$(BUILD_KERNEL))
BUILD_KERNEL=$(shell uname -r)
endif
KSRC ?= /lib/modules/$(BUILD_KERNEL)/build

all: modules
	@rm -rf $(TEMPFILES)
modules:
	@$(MAKE) -C $(KSRC) M=$(shell pwd) modules

checkstack: modules
	objdump -d $(obj-m) | perl $(KSRC)/scripts/checkstack.pl $(shell uname -m)
	@rm -rf $(TEMPFILES)

namespacecheck: modules
	perl $(KSRC)/scripts/namespace.pl
	@rm -rf $(TEMPFILES)

install: modules
	$(MAKE) -C $(KSRC) M=`pwd` modules_install

patch:
	@/usr/bin/diff -uar -X dontdiff ../../trunc ./ > bc.patch || echo > /dev/null

clean:
	@rm -rf $(MODNAME).ko $(TEMPFILES)

%::
	$(MAKE) -C $(KSRC) M=`pwd` $@

endif
