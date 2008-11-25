#ifndef __ACQ200_FIFO_LOCAL_H__
#define __ACQ200_FIFO_LOCAL_H__


#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm-arm/fiq.h>
#include <linux/proc_fs.h>

#include <asm-arm/arch-iop32x/iop321.h>
#include <asm-arm/arch-iop32x/iop321-dma.h>
#include "acq200-dmac.h"
#include "acq200_debug.h"

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif


/** @@todo - #includes a bit of a mixup */

#ifdef ACQ196
#include "acq196.h"
#endif

#ifdef ACQ216
#include "acq216.h"
#define FIFSTAT ACQ200_FIFCON
#define FIFCON ACQ200_FIFCON
#define FIFCON_COLDUNDER ACQ200_FIFCON_COLDUNDER
#define SYSCON ACQ200_SYSCON

#include "acq200.h"
#include "acq200-fifo.h"
#include "acq200_hml.h"
#endif

#ifdef WAV232
#include "wav232.h"
#define FIFSTAT ACQ200_FIFCON
#define FIFCON  ACQ200_FIFCON
#define FIFCON_COLDUNDER ACQ200_FIFCON_COLDUNDER
#define SYSCON WAV232_SYSCON
#include "acq200.h"
#include "acq200-fifo.h"
#include "acq200_hml.h"
#endif


#include "acq200.h"
#include "acq200-fifo.h"


#include "acq200_debug.h"

int find_event_diag(char *buf, int maxbuf);
struct file_operations *acq200_fifo_get_bigbuf_datafops(int iminor);
int acq200_fifo_create_AIfs(struct device* dev, int nchannels);
int acq200_fifo_destroy_AIfs(void);


extern int acq200_lookup_pchan(int lchannel);


/**
 *   EVENT_MAGIC - bit pattern to ident ES
 *   EVENT_MAGIC_MASK - discard these bits when making ident
 */

#ifdef ACQ216
#define EVENT_MAGIC      0xaa550000
#define EVENT_MAGIC_MASK 0x0000ffff      

/** NB FIXED ES for ACQ216 */
#define SIZE_12C	(12*sizeof(short))
#define SIZE_16C	(16*sizeof(short))

/* acq216 uses double sample ES to cope with double read cycle 123321 */
#define ES_SIZE_12C	(2*SIZE_12C)
#define ES_SIZE_16C	(2*SIZE_16C)
#define ES_SIZE_MAX	(ES_SIZE_16C)
#define ES_SIZE  (sample_size()==SIZE_12C? ES_SIZE_12C: ES_SIZE_16C)

#define EVENT_MAGIC_EXEMPT(ipair) (ipair>=6)

#elif defined(ACQ132)
#define EVENT_MAGIC      0xaa55aa55
#define EVENT_MAGIC_MASK 0x00000000

#define ES_SIZE  (8*sizeof(short))
#define EVENT_MAGIC_EXEMPT(ipair) ((ipair&1) == 1)
#else
/* ACQ196 event appears to be 0xaa55fBxx B {123} */
#define EVENT_MAGIC      0xaa55f000
#define EVENT_MAGIC_MASK 0x0000037f

#define ES_SIZE  (sample_size())
#define EVENT_MAGIC_EXEMPT(ipair) (0)
#endif

#define EVENT_SOFT   0x40
#define EVENT_PT     0x20
#define EVENT_TRIG   0x10
#define EVENT_EVENT  0x0f

#define ES_LONGS	(ES_SIZE/sizeof(unsigned))
#define SAMPLE_LONGS	(sample_size()/sizeof(unsigned))

#define IS_EVENT_MAGIC(x) (((x)&~EVENT_MAGIC_MASK) == EVENT_MAGIC)

int acq200_rounding(int khz, int precision);

/* tweak precision to coax into two digit range, most of the time */
#define PRECISION(div) ((div)+8)

#define DTACQ_MACH_EVENT_ADJUST(phase, isearch, first, last)

int acq200_dumpregs_diag(char* buf, int len);


#ifdef ACQ216
u32 acq216_pci2bus(u32 pci_addr);
#endif



extern int acq200_check_entire_es(unsigned *pes);
extern int acq200_lookup_pchan(int lchannel);

#endif /* __ACQ200_FIFO_LOCAL_H__ */
