KERN_ROOT ?= $(abspath ../CactKernel-x86_32)
LOCAL_REPO ?= $(abspath ../LocalRepoCactOS)

_ACTIVE := $(filter-out clean,$(or $(MAKECMDGOALS),all))
ifneq ($(_ACTIVE),)
ifndef KERN_ROOT
$(error KERN_ROOT is required — path to kernel sources with Cact/ headers)
endif
ifndef LOCAL_REPO
$(error LOCAL_REPO is required — directory whose lib/ receives *.cctk)
endif
endif
INSTALL_DIR := $(LOCAL_REPO)/lib

MOD_CFLAGS := -m32 -ffreestanding -fno-pie -fno-stack-protector -nostdlib \
	-I$(KERN_ROOT)/Cact/kernel/sync \
	-I$(KERN_ROOT)/Cact/kernel/core \
	-I$(KERN_ROOT)/Cact/drivers/pci/enum \
	-I$(KERN_ROOT)/Cact/drivers/pci \
	-I$(KERN_ROOT)/Cact/fs/vfs \
	-I$(KERN_ROOT)/Cact/fs/vfs/devfs \
	-I. \
	-Wall -O2

.PHONY: all install clean
all: nvme.cctk

nvme.cctk: nvme_mod.o
	cp -f $< $@

nvme_mod.o: nvme_mod.c nvme.h
	gcc $(MOD_CFLAGS) -c nvme_mod.c -o $@

install: nvme.cctk
	@mkdir -p $(INSTALL_DIR)
	cp -f nvme.cctk $(INSTALL_DIR)/nvme.cctk
	@echo "installed: $(INSTALL_DIR)/nvme.cctk"

clean:
	rm -f nvme_mod.o nvme.cctk
