
MODNAME := jme
TEMPFILES := $(MODNAME).o $(MODNAME).mod.c $(MODNAME).mod.o Module.symvers .$(MODNAME).*.cmd .tmp_versions
#EXTRA_CFLAGS += -Wall -DDEBUG -DTX_DEBUG -DRX_DEBUG
#EXTRA_CFLAGS += -Wall -DDEBUG -DRX_DEBUG
#EXTRA_CFLAGS += -Wall -DDEBUG
EXTRA_CFLAGS += -Wall

obj-m := $(MODNAME).o

ifeq (,$(BUILD_KERNEL))
BUILD_KERNEL=$(shell uname -r)
endif

KSRC := /lib/modules/$(BUILD_KERNEL)/build

all:
	@$(MAKE) -C $(KSRC) SUBDIRS=$(shell pwd) modules
	@rm -rf $(TEMPFILES)

clean:
	@rm -rf $(MODNAME).ko $(TEMPFILES)

