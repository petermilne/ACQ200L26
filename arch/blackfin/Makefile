#
# arch/blackfin/Makefile
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#


CROSS_COMPILE    ?= bfin-uclinux-
LDFLAGS_vmlinux  := -X
OBJCOPYFLAGS     := -O binary -R .note -R .comment -S
GZFLAGS          := -9

CFLAGS_MODULE    += -mlong-calls
KALLSYMS         += --symbol-prefix=_


# setup the machine name and the machine dependent settings
machine-$(CONFIG_BF531) := bf533
machine-$(CONFIG_BF532) := bf533
machine-$(CONFIG_BF533) := bf533
machine-$(CONFIG_BF534) := bf537
machine-$(CONFIG_BF536) := bf537
machine-$(CONFIG_BF537) := bf537
machine-$(CONFIG_BF561) := bf561
MACHINE := $(machine-y)
export MACHINE


head-y   := arch/$(ARCH)/mach-$(MACHINE)/head.o arch/$(ARCH)/kernel/init_task.o

core-y   += arch/$(ARCH)/kernel/ arch/$(ARCH)/mm/ arch/$(ARCH)/mach-common/

# If we have a machine-specific directory, then include it in the build.
ifneq ($(machine-y),)
core-y   += arch/$(ARCH)/mach-$(MACHINE)/
core-y   += arch/$(ARCH)/mach-$(MACHINE)/boards/
endif

libs-y   += arch/$(ARCH)/lib/

drivers-$(CONFIG_OPROFILE) += arch/$(ARCH)/oprofile/



#	Update machine arch symlinks if something which affects
#	them changed.  We use .mach to indicate when they were updated
#	last, otherwise make uses the target directory mtime.

include/asm-blackfin/.mach: $(wildcard include/config/arch/*.h) include/config/auto.conf
	@echo '  SYMLINK include/asm-$(ARCH)/mach-$(MACHINE) -> include/asm-$(ARCH)/mach'
ifneq ($(KBUILD_SRC),)
	$(Q)mkdir -p include/asm-$(ARCH)
	$(Q)ln -fsn $(srctree)/include/asm-$(ARCH)/mach-$(MACHINE) include/asm-$(ARCH)/mach
else
	$(Q)ln -fsn mach-$(MACHINE) include/asm-$(ARCH)/mach
endif
	@touch $@

CLEAN_FILES += \
	include/asm-$(ARCH)/asm-offsets.h \
	arch/$(ARCH)/kernel/asm-offsets.s \
	include/asm-$(ARCH)/mach \
	include/asm-$(ARCH)/.mach

archprepare: include/asm-blackfin/.mach
archclean:
	$(Q)$(MAKE) $(clean)=$(boot)


all: vmImage
boot := arch/$(ARCH)/boot
BOOT_TARGETS = vmImage
.PHONY: $(BOOT_TARGETS)
$(BOOT_TARGETS): vmlinux
	$(Q)$(MAKE) $(build)=$(boot) $(boot)/$@
define archhelp
  echo  '* vmImage         - Kernel-only image for U-Boot (arch/$(ARCH)/boot/vmImage)'
endef
