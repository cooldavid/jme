
MODNAME := jme
TEMPFILES := $(MODNAME).o $(MODNAME).mod.c $(MODNAME).mod.o Module.symvers .$(MODNAME).*.cmd .tmp_versions modules.order

DEBUG_FLAGS += -DDEBUG
#DEBUG_FLAGS += -Wpointer-arith -Wbad-function-cast -Wsign-compare
#DEBUG_FLAGS += -DCSUM_DEBUG
#DEBUG_FLAGS += -DTX_DEBUG
#DEBUG_FLAGS += -DRX_DEBUG
#DEBUG_FLAGS += -DQUEUE_DEBUG

EXTRA_CFLAGS += -Wall -O3
#EXTRA_CFLAGS += $(DEBUG_FLAGS)

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

