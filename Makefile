
MODNAME := jme
TEMPFILES := $(MODNAME).o $(MODNAME).mod.c $(MODNAME).mod.o Module.symvers .$(MODNAME).*.cmd .tmp_versions modules.order

EXTRA_CFLAGS += -Wall -O3
#EXTRA_CFLAGS += -DTX_DEBUG
#EXTRA_CFLAGS += -DREG_DEBUG

obj-m := $(MODNAME).o

ifeq (,$(BUILD_KERNEL))
BUILD_KERNEL=$(shell uname -r)
endif

KSRC ?= /lib/modules/$(BUILD_KERNEL)/build

all:
	@$(MAKE) -C $(KSRC) SUBDIRS=$(shell pwd) modules
	@rm -rf $(TEMPFILES)

checkstack:
	$(MAKE) -C $(KSRC) SUBDIRS=$(shell pwd) modules
	objdump -d $(obj-m) | perl $(KSRC)/scripts/checkstack.pl i386
	@rm -rf $(TEMPFILES)

namespacecheck:
	$(MAKE) -C $(KSRC) SUBDIRS=$(shell pwd) modules
	perl $(KSRC)/scripts/namespace.pl
	@rm -rf $(TEMPFILES)

patch:
	@/usr/bin/diff -uarN -X dontdiff ../mod ./ > bc.patch || echo > /dev/null

clean:
	@rm -rf $(MODNAME).ko $(TEMPFILES)

