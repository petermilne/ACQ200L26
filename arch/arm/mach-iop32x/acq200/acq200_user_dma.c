/* ------------------------------------------------------------------------- */
/* file acq200_user_dma.c                                                                 */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2011 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>
 *  Created on: Jun 2, 2013
 *      Author: pgm

    http://www.d-tacq.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of Version 2 of the GNU General Public License
    as published by the Free Software Foundation;

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

/** @file acq200_user_dma.c DESCR 
 *
 * User controller dma.
 * write : write(u32[4])
 * PADR, LADDR, BC, DC
 * DC&0x80000000 :=> chain the entry.
 * write MUST be a multiple of 4x4 = 16b
 * blocks until last non-chain entry complete, then returns.
 */



#include <asm/uaccess.h>

#ifndef CONFIG_ARCH_ACQ200
#error ERROR: this stuffvalid for acq200 only, sorry
#endif

#include <linux/init.h>


#include <linux/kernel.h>   /* printk() */
#include <linux/fs.h>       /* everything... */
#include <linux/errno.h>    /* error codes */
#include <linux/mm.h>       /* VMA */
#include <linux/types.h>    /* size_t */
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>    /* O_ACCMODE */

#include <linux/pci.h>

#include <asm/system.h>     /* cli(), *_flags */

#include <asm/hardware.h>
#include <asm/mach/pci.h>

#include <linux/kdev_t.h>   /* MINOR */

#include <asm-arm/mach-types.h>

#include <asm/arch/iop321.h>


#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif
#include <linux/module.h>

#define MAXCHAIN 16
#include "acq200-inline-dma.h"

#include "acq200.h"
#include "acq200-common.h"

#include <asm-arm/arch-iop32x/iop321.h>
#include <asm-arm/arch-iop32x/iop321-dma.h>
#include "acq200-dmac.h"
#include "acq200_debug.h"

#include "acq200_hostdrv.h"

#include "acq200_user_dma.h"

#define acq200_user_dma_driver_name "acq200_user_dma"
#define acq200_user_dma_version "B1000"
#define acq200_user_dma_copyright "Copyright (c) 2013 D-TACQ Solutions Ltd"

int acq200_user_dma_major = 0;
module_param(acq200_user_dma_major, int, 0444);

int debug = 1;
module_param(debug, int, 0644);

int dry_run = 1;
module_param(dry_run, int, 0644);

#define DMACHAN(filp)  ((struct DmaChannel*)(filp)->private_data)

static int print_chain(struct DmaChannel* channel, char* buf)
{
	int ic;
	int len = 0;
	struct iop321_dma_desc* desc = channel->dmad[0];

	len += sprintf(buf+len, "[  ] %8s %8s %8s %8s %8s %8s\n",
		       "NDA", "PDA/MMSRC", "PUAD", "LAD/MMDST", "BC", "DC");

	for (ic = 0; ic < channel->nchain; ++ic, ++desc){
		len += sprintf(buf+len,
			       "[%2d] %08x %08x %08x %08x %08x %08x %s\n",
			       ic,
			       desc->NDA, desc->PDA, desc->PUAD,
			       desc->LAD, desc->BC, desc->DC,
			       channel->description[ic]);
	}
	return len;
}

static void actuallyFireDma(struct DmaChannel* channel)
{
	u32 stat;
	int pollcat = 0;
	DMA_ARM(*channel);
	DMA_FIRE(*channel);

	while(!DMA_DONE(*channel, stat)){
		yield();
		++pollcat;
	}
	if (debug){
		T("pollcat:%d", pollcat);
	}
}
static void maybeFireDma(struct DmaChannel* channel)
{
	char buf[256];
	if (debug){
		print_chain(channel, buf);
		printk(buf);
	}
	if (!dry_run){
		actuallyFireDma(channel);
	}
}
static int acq200_user_dma_open(struct inode *inode, struct file *file)
{
	file->private_data = dma_allocate_fill_channel();
	return 0;
}

ssize_t acq200_user_dma_write(
	struct file *file, const char *buf, size_t count, loff_t *f_pos)
{
	struct DmaChannel* channel = DMACHAN(file);
	u32 tmp[4];
	int cursor = 0;

	if (*f_pos == 0){
		channel->nchain = 0;
	}

	for (cursor = 0; count >= cursor + ACQ200_USER_DMA_WRITE_LEN;
			cursor += ACQ200_USER_DMA_WRITE_LEN) {
		if (copy_from_user(tmp, buf+cursor, ACQ200_USER_DMA_WRITE_LEN)){
			return -EFAULT;
		}else{
			struct iop321_dma_desc* dmad = channel->dmad[channel->nchain];
			int make_chain = tmp[ACQ200_USER_DMA_DC]&ACQ200_USER_DMA_DC_CHAIN;

			dmad->PDA = tmp[ACQ200_USER_DMA_PDA];
			dmad->LAD = tmp[ACQ200_USER_DMA_LAD];
			dmad->BC = tmp[ACQ200_USER_DMA_BC];
			dmad->DC = tmp[ACQ200_USER_DMA_DC]&ACQ200_USER_DMA_DC_MASK;
			dma_append_chain_recycle(channel, "UDMA");
			if (!make_chain || channel->nchain > MAXCHAIN -1){
				maybeFireDma(channel);
				channel->nchain = 0;
			}
		}
		*f_pos += ACQ200_USER_DMA_WRITE_LEN;
	}

	return cursor;
}

static int acq200_user_dma_release(struct inode *inode, struct file *file)
{
	dma_free_channel(DMACHAN(file));
	return 0;
}

static int acq200_user_dma_probe(struct device * dev)
{
	static struct file_operations acq200_user_dma_fops = {
		.open	 = acq200_user_dma_open,
		.release = acq200_user_dma_release,
		.write   = acq200_user_dma_write
	};

	int rc;

	T("probe 01");
	rc = register_chrdev( acq200_user_dma_major,
				acq200_user_dma_driver_name,
				&acq200_user_dma_fops );

	if ( rc < 0 ){
		E( "can't get major %d\n", acq200_user_dma_major );
		return rc;
	}else if ( acq200_user_dma_major == 0 ){
		acq200_user_dma_major = rc;
	}
	T("probe 99 major %d", acq200_user_dma_major);
	return 0;
}

static int acq200_user_dma_remove(struct device * dev)
{
	unregister_chrdev(acq200_user_dma_major, acq200_user_dma_driver_name);
	return 0;
}
static struct device_driver acq200_user_dma_device_driver = {
	.name  = acq200_user_dma_driver_name,
	.bus   = &platform_bus_type,
	.probe = acq200_user_dma_probe,
	.remove= acq200_user_dma_remove
};

static struct platform_device acq200_user_dma_device = {
	.name  = "acq200_user_dma",
	.id    = 0,
	.dev = {
		.kobj.name = acq200_user_dma_driver_name,
	},
};

int __init acq200_user_dma_init_module(void)
{
	int rc;

	info("%s %s %s\n%s",
			acq200_user_dma_driver_name,
			acq200_user_dma_version,
			__DATE__,
			acq200_user_dma_copyright);
	rc = driver_register(&acq200_user_dma_device_driver);
	if (rc != 0){
		E("Driver registration failed %d", rc);
		return rc;
	}else{
		return platform_device_register(&acq200_user_dma_device);
	}
}

void acq200_user_dma_exit_module(void)
{
	platform_device_unregister(&acq200_user_dma_device);
	driver_unregister(&acq200_user_dma_device_driver);
}


module_init(acq200_user_dma_init_module);
module_exit(acq200_user_dma_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("User Dma device driver");
