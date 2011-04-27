#
# Makefile for the linux kernel. (arch/arm/mach-iop3xx/acq200
#

# Object file lists.

CONFIG_ARCH_DTACQ_CORE=m
CONFIG_ARCH_DTACQ_GTMR=m
CONFIG_ARCH_DTACQ_ACQ216=m
CONFIG_ARCH_DTACQ_WAV232=m
CONFIG_ARCH_DTACQ_ACQ196=m
CONFIG_ARCH_DTACQ_ACQ196m=m
#CONFIG_ARCH_DTACQ_ACQ196f=m
CONFIG_ARCH_DTACQ_ACQ196t=m
CONFIG_ARCH_DTACQ_ACQ196c=m
CONFIG_ARCH_DTACQ_ACQ196h=m
CONFIG_ARCH_DTACQ_ACQ132=m
CONFIG_ARCH_DTACQ_ACQ164=m
CONFIG_ARCH_DTACQ_BRIDGE=m
#CONFIG_ARCH_DTACQ_DMAC=m
CONFIG_ARCH_DTACQ_MU=m
CONFIG_ARCH_MMUD=n
CONFIG_ARCH_DTACQ_ACQ100_LLC=m
CONFIG_ARCH_DTACQ_ACQ100_RTM_T_LLC=m
CONFIG_ARCH_DTACQ_ACQ200_LLC=m
CONFIG_ARCH_DTACQ_ACQ100_SCATTER=m
CONFIG_ARCH_DTACQ_ACQ100_SKEL=m
CONFIG_ARCH_DTACQ_RTM1=m
CONFIG_ARCH_DTACQ_RTM2=m
CONFIG_ARCH_DTACQ_RTM_DDS=m
CONFIG_ARCH_DTACQ_MAC=m
CONFIG_ARCH_DTACQ_PPCUSTOM=m
CONFIG_ARCH_DTACQ_EB_PPCUSTOM=m
CONFIG_ARCH_DTACQ_MA_FILTER=m
CONFIG_ARCH_DTACQ_PREP=m
CONFIG_ARCH_DTACQ_TBLOCKFS=m
CONFIG_ARCH_DTACQ_MEAN=m
CONFIG_ARCH_DTACQ_THRESHOLD=m
CONFIG_ARCH_DTACQ_PWM=m
CONFIG_ARCH_DTACQ_REGFS=m
CONFIG_ARCH_DTACQ_HWPULSEGEN=m
CONFIG_ARCH_DTACQ_DEBUGFS2=m
CONFIG_ARCH_DTACQ_CPS_SIMUL=m
CONFIG_ARCH_DTACQ_REGDBG=m
CONFIG_ARCH_DTACQ_RTMCPS=m
CONFIG_ARCH_DTACQ_CPS_TRANSFORM=m
CONFIG_ARCH_DTACQ_PENTA_LOCKIN=m
CONFIG_ARCH_DTACQ_BTT=m
CONFIG_ARCH_DTACQ_DEBUG_IOP=m
CONFIG_ARCH_DTACQ_DEBUG_ACQ1xx=m
CONFIG_ARCH_DTACQ_ACQ132_LFP=m
CONFIG_ARCH_DTACQ_ACQ100_RTM_T=m
CONFIG_ARCH_DTACQ_ACQ100_SPIFLASH=m
CONFIG_ACQ200_HOSTDRV=m
CONFIG_ARCH_DTACQ_ACQ100_GATHER=m
CONFIG_ARCH_DTACQ_ACQ100_RTMCLK=m


obj-y			:= \
	acq200-time.o \
	acqX00-setup.o   \
	acq200-sys.o \
	acq100.o 	acq200.o \
	acq200-irq.o 	acq200-mm.o \
	acq100-pci.o 	acq200-pci.o \
	acq200-leds.o	acq200-core.o \
	iop321-pci.o \
	iop321-ppmu.o \
	iop321-auxtimer.o \
	acq200-dmac.o 



obj-m			:=
obj-n			:=
obj-			:=


obj-$(CONFIG_ARCH_DTACQ_CORE) += acq200-core.o

obj-$(CONFIG_ARCH_DTACQ_BRIDGE) += acq200-bridge.o

obj-$(CONFIG_ARCH_DTACQ_ACQ216) += acq216.o

obj-$(CONFIG_ARCH_DTACQ_WAV232) += wav232.o

obj-$(CONFIG_ARCH_DTACQ_ACQ196) += acq196.o

obj-$(CONFIG_ARCH_DTACQ_ACQ196m) += acq196m.o

obj-$(CONFIG_ARCH_DTACQ_ACQ196f) += acq196f.o

obj-$(CONFIG_ARCH_DTACQ_ACQ196t) += acq196t.o

obj-$(CONFIG_ARCH_DTACQ_ACQ196c) += acq196c.o

obj-$(CONFIG_ARCH_DTACQ_ACQ196h) += acq196h.o

obj-$(CONFIG_ARCH_DTACQ_ACQ132) += acq132.o

obj-$(CONFIG_ARCH_DTACQ_ACQ164) += acq164.o

obj-$(CONFIG_ARCH_DTACQ_MU) += acq200-mu.o

obj-$(CONFIG_ARCH_MMUD) += mmud.o

obj-$(CONFIG_ARCH_DTACQ_DMAC) += acq200-dmac.o

# .ko has _, source file has - is this confusing? yes, but
# the reason is, loaded module hav _ and it's handy to have .ko match modname

obj-$(CONFIG_ARCH_DTACQ_ACQ100_LLC) += acq100_llc.o

obj-$(CONFIG_ARCH_DTACQ_ACQ100_LLC) += acq100-llc-rtm-t.o

obj-$(CONFIG_ARCH_DTACQ_ACQ200_LLC) += acq200_llc.o

obj-$(CONFIG_ARCH_DTACQ_ACQ100_SCATTER) += acq100_scatter.o

obj-$(CONFIG_ARCH_DTACQ_ACQ100_SKEL) += acq100_skel.o

obj-$(CONFIG_ARCH_DTACQ_RTM1) += acq100-rtm.o

obj-$(CONFIG_ARCH_DTACQ_RTM2) += acq200-rtm.o

obj-$(CONFIG_ARCH_DTACQ_RTM_DDS) += acqX00-rtm-dds.o

obj-$(CONFIG_ARCH_DTACQ_MAC) += acq196_mac.o

obj-$(CONFIG_ARCH_DTACQ_PPCUSTOM) += acq216_ppcustom.o

obj-$(CONFIG_ARCH_DTACQ_EB_PPCUSTOM) += eb_ppcustom.o

obj-$(CONFIG_ARCH_DTACQ_MA_FILTER) += ma-filter.o

obj-$(CONFIG_ARCH_DTACQ_PREP) += acq200-prep.o

obj-$(CONFIG_ARCH_DTACQ_TBLOCKFS) += acq200-tblockfs.o


obj-$(CONFIG_ARCH_DTACQ_THRESHOLD) += acq200_threshold.o

obj-$(CONFIG_ARCH_DTACQ_PWM) += acq2xx-pwm.o

obj-$(CONFIG_ARCH_DTACQ_REGFS) += acq200-regfs.o

obj-$(CONFIG_ARCH_DTACQ_HWPULSEGEN) += acq200_hwpulse.o

obj-$(CONFIG_ARCH_DTACQ_DEBUGFS2) += debugfs2.o

obj-$(CONFIG_ARCH_DTACQ_CPS_SIMUL) += cps_simul.o

obj-$(CONFIG_ARCH_DTACQ_REGDBG) += acq200regdbg.o

obj-$(CONFIG_ARCH_DTACQ_RTMCPS) += acq200-rtmcps.o

obj-$(CONFIG_ARCH_DTACQ_CPS_TRANSFORM) += cps_transform.o

obj-$(CONFIG_ARCH_DTACQ_PENTA_LOCKIN) += penta_lockin.o

obj-$(CONFIG_ARCH_DTACQ_BTT) += btt.o

obj-$(CONFIG_ARCH_DTACQ_DEBUG_IOP) += debug_iop.o

obj-$(CONFIG_ARCH_DTACQ_DEBUG_IOP) += debug_acq1xx.o

obj-$(CONFIG_ARCH_DTACQ_ACQ132_LFP) += acq132_lfp.o

obj-$(CONFIG_ARCH_DTACQ_ACQ100_RTM_T) += acq100_rtm_t.o

obj-$(CONFIG_ARCH_DTACQ_ACQ100_SPIFLASH) += acq100_spiflash.o

obj-$(CONFIG_ARCH_DTACQ_MEAN) += acq200_mean.o

obj-$(CONFIG_ACQ200_HOSTDRV) += acq200_hostdrv.o
obj-$(CONFIG_ARCH_DTACQ_ACQ100_GATHER) += acq100_gather.o
obj-$(CONFIG_ARCH_DTACQ_ACQ100_RTMCLK) += acq100_rtmclk.o
obj-$(CONFIG_ARCH_DTACQ_GTMR) += gtmr.o

ACQ2XX_FIFO_COMMONA=\
	acq200-fifo-tblock.o \
	acq200-transform.o \
	acq200-rb.o \
	acq200-fifo-bigbuf-fops.o \
	acq200-hook.o \
	acq200-cdog.o \
	refill-client.o 

ACQ2XX_FIFO_COMMON=\
	$(ACQ2XX_FIFO_COMMONA) \
	acq200-signal.o \

acq216-objs := acq216-fifo.o acq216-fiq.o \
	acq216-fifo-procfs.o acq2xx-fifo.o acq216-offset.o \
	$(ACQ2XX_FIFO_COMMON) \
	prebuiltChainUtils.o

acq196-objs := acq196-fifo.o acq196-fiq.o \
	acq196-fifo-procfs.o acq196-offset.o acq196-AO.o \
	$(ACQ2XX_FIFO_COMMON)

acq164-objs := acq164-fifo.o acq164-fiq.o \
	acq164-fifo-procfs.o \
	acq164-offset.o acq196-AO.o \
	acq200-clk_counter.o \
	$(ACQ2XX_FIFO_COMMONA) \
	acq200-signal2.o

acq196m-objs := acq196-fifo-m.o acq196-fiq-m.o \
	acq196-fifo-procfs-m.o acq196-offset.o acq196-AO.o \
	$(ACQ2XX_FIFO_COMMON)

acq196c-objs := acq196-fifo-c.o acq196-fiq-c.o \
	acq196-fifo-procfs.o acq196-offset.o acq196-AO.o \
	$(ACQ2XX_FIFO_COMMON)

acq196f-objs := acq196-fifo-f.o acq196-fiq-f.o \
	acq196-fifo-procfs-f.o acq196-offset.o acq196-AO.o \
	$(ACQ2XX_FIFO_COMMON)

acq196t-objs := acq196-fifo-t.o acq196-fiq-t.o \
	acq196-fifo-procfs-t.o acq196-offset.o acq196-AO.o \
	$(ACQ2XX_FIFO_COMMON) \
	prebuiltChainUtils.o

acq196h-objs := acq196-fifo-h.o acq196-fiq-h.o \
	acq196-fifo-procfs.o acq196-offset.o acq196-AO.o \
	$(ACQ2XX_FIFO_COMMON)

acq132-objs := acq132-fifo.o acq132-fiq.o \
	acq132-fifo-procfs.o acq132-offset.o acq196-AO.o \
	acq200-clk_counter.o \
	acq132-osam.o \
	acq132-gated.o acq132-transform.o \
	$(ACQ2XX_FIFO_COMMON)

wav232-objs := wav232-fifo.o wav232-fiq.o \
	wav232-fifo-procfs.o acq2xx-fifo.o \
	$(ACQ2XX_FIFO_COMMON)

acq200_mean-objs := acq200-mean.o refill-client.o

obj-$(CONFIG_MODULES)	+= acq200syms.o


acq100_llc-objs := acq100-llc.o

acq200_llc-objs := acq200-llc.o

acq100_scatter-objs := acq100-scatter.o

acq100_skel-objs := acq100-skel.o

acq196_mac-objs := acq196-mac.o

acq100_rtm_t-objs := acq100_rtm_t_main.o acq100_rtm_t_uart.o acq100_rtm_t_spi.o

#acq216-fiq.o: acq216-fiq.S
#wav232-fiq.o: wav232-fiq.S

.links: 
	ln -s /home/pgm/PROJECTS/ACQ200/project/BASE/DTACQ/SRC/include/acq32busprot.h acq32busprot.h
	echo links made >.links
