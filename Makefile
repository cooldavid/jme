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

ifeq (,$(KVER))
KVER=$(shell uname -r)
endif
KSRC ?= /lib/modules/$(KVER)/build
MINSTDIR ?= /lib/modules/$(KVER)/kernel/drivers/net

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
	install -m 644 $(MODNAME).ko $(MINSTDIR)
	depmod -a $(KVER)

patch:
	@/usr/bin/diff -uar -X dontdiff ../../trunc ./ > bc.patch || echo > /dev/null

buildtest:
	SRCDIRS=`find ~/linux-src -mindepth 1 -maxdepth 1 -type d -name 'linux-*' | sort -r -n`; \
	SRCDIRS="$${SRCDIRS} `find ~/linux-src/centos -mindepth 2 -maxdepth 2 -type d -name 'linux-*' | sort -r -n`"; \
	SRCDIRS="$${SRCDIRS} `find ~/linux-src/fedora -mindepth 2 -maxdepth 2 -type d -name 'linux-*' | sort -r -n`"; \
	for d in $${SRCDIRS}; do \
		$(MAKE) clean && $(MAKE) -C . KSRC=$${d} modules; \
		if [ $$? != 0 ]; then \
			exit $$?; \
		fi; \
	done
	$(MAKE) clean

clean:
	@rm -rf $(MODNAME).ko $(TEMPFILES)

%::
	$(MAKE) -C $(KSRC) M=`pwd` $@

endif
