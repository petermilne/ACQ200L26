/* ------------------------------------------------------------------------- */
/* cps_transform.c transformer for ACQ216 CPS firmware                       */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2003 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>
                                                                               
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


#define ACQ216


#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm-arm/fiq.h>
#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

/* keep debug local to this module */
#define acq200_debug cps_transform_debug   

#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

int cps_transform_debug;
module_param(cps_transform_debug, int, 0664);


#define VERID "$Revision: 1.0 $ build B1013 "


char cps_transform_driver_name[] = "cps_transform";
char cps_transform_driver_string[] = "D-TACQ CPS Transform data demangler";
char cps_transform_driver_version[] = VERID __DATE__;
char cps_transform_copyright[] = "Copyright (c) 2008 D-TACQ Solutions Ltd";

#define NCHANNELS	12

/* one ES ever SIGSTRIDE samples, but inserted only as 
 *	SIGSTRIDE*SUPERSTRIDE 
 * samples.
 */
#define SIGSTRIDE	6
#define SUPERSTRIDE	3

#define ROWLEN (NCHANNELS*sizeof(short))

int sigstride = SIGSTRIDE;
module_param(sigstride, int, 0664);

int nsigtblocks = 10;
module_param(nsigtblocks, int, 0444);

static struct TblockListElement ** captives;
int last_stride;
int sig_cursor;
int block_cursor = -1;
int delta_sam_total;

int cps_major = 0;

#define SIG(block) (captives[block])

/** NB SIG_TBLOCK usage:
 * SIG(block)->phase_sample_start is in BYTES
 * SIG(block)->sample_count is in BYTES
 */

static void adjust_this_tblock(
	struct Phase * phase, struct TblockListElement *tble, 
	int delta_sam,
	int phase_only)
{
	if (!phase_only){
		tble->sample_count -= delta_sam;
		tble->phase_sample_start -= delta_sam_total;
	}

	phase->actual_len -= delta_sam;
	phase->actual_samples -= delta_sam;


#ifdef DODGYISGOOD
	if (tble->phase_sample_start+tble->sample_count ==
	    phase->actual_samples - 1){
		struct Phase *next_phase;

		dbg(1, "correct rounding error");
		phase->actual_samples -= 1;
		phase->actual_len -= sample_size();

		next_phase = list_entry(phase->list.next, struct Phase, list);

		if (next_phase != 0){
			next_phase -> start_sample -= 1;
		}
	}
#endif
}

#define ADJUSTED 1

static int adjust_tblock_in_phase(
	struct Phase * phase, 
	void *cursor, 
	int delta_sam,
	int phase_only)
/* the data in this tblock just got shortened, so we have to find all
 * the corresponding tble's and adjust them...
 * Also, make a once-only adjustment to phase global actual_samples ..
 * there is an implicit assumption that all the phase gets transformed,
 * not just this tblock, but that is reasonable, we don't do partials.
 */
{
	struct TblockListElement *tble;
	int this_index = TBLOCK_INDEX(cursor - va_buf(DG));

	if (phase == 0){
		return 0;
	}

	list_for_each_entry(tble, &phase->tblocks, list){
		if (TBLOCK_INDEX(tble->tblock->offset) == this_index){
			dbg(1, "looking for %d got tblock:%d %s", 
			    this_index,	tble->tblock->iblock, "Adjusting");
			adjust_this_tblock(phase, tble, delta_sam, phase_only);
			return ADJUSTED;
		}
	}	

	return 0;
}

static void stash_sig(short *from) 
{
	if (block_cursor == -1){
		++block_cursor;
		SIG(block_cursor)->tblock->touched = 1;
	}
	if (sig_cursor + ROWLEN > TBLOCK_LEN(DG)){
		if (++block_cursor > nsigtblocks){
			err("not enough captive tblocks to store all sigs");
			return;
		}else{
			SIG(block_cursor)->phase_sample_start = 
				block_cursor*TBLOCK_LEN(DG);
			SIG(block_cursor)->tblock->touched = 1;
			sig_cursor = 0;
		}
	}else{
		SIG(block_cursor)->sample_count += ROWLEN;
	}

	memcpy(VA_TBLOCK(SIG(block_cursor)->tblock)+sig_cursor, from, ROWLEN);
	sig_cursor += ROWLEN;
}

static void cps_transform(short *to, short *from, int nwords, int stride)
/**< transform data:
 * in:                out:
 *      s10s00             s00s01
 *      s11s01             s10s11
 *
 * in sigstride*3 times:
 *	ch01.... ch12
 *	sig1 sig2 sig3
 *
 * out:
 * ch01 ... ch12 in same tblock
 * sig1 sig2 sig3 > reserved tblock
 */
{
	int nsamples = nwords/stride;
	int isample, ichannel;
	const int max_stride = sigstride * SUPERSTRIDE;
	int delta_sam = 0;
	int po = 0;
	int tosample = 0;

	dbg(1, "to:%p from:%p nwords:%d stride:%d max_stride:%d last_stride:%d",
	    to, from, nwords, stride, max_stride, last_stride);

	for (isample = 0; isample != nsamples; ++isample){
		int fromrow = isample * stride;
		if (last_stride++ == max_stride){
			stash_sig(from+fromrow);

			++delta_sam;

			dbg(2, "stash:%d delta_sam:%d isample:%d", 
			    last_stride, delta_sam, isample);

			last_stride = 0;
		}else{
			for (ichannel = 0; ichannel != stride; ++ichannel){
				to[ichannel*nsamples + tosample] =
					from[fromrow + ichannel];
			}
			tosample++;
		}
	}

	po = adjust_tblock_in_phase(DMC_WO->pre, from, delta_sam, po)==ADJUSTED;
	adjust_tblock_in_phase(DMC_WO->post, from, delta_sam, po);
	delta_sam_total += delta_sam;
}


static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr, 
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       cps_transform_driver_name,
		       cps_transform_driver_string,
		       cps_transform_driver_version,
		       cps_transform_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);


static ssize_t show_sigtblocks(
	struct device *dev, 
	struct device_attribute *attr, 
	char * buf)
{
	int iblock;

	strcpy(buf, "");

	for (iblock = 0; iblock < nsigtblocks; ++iblock){
		char blockid[5];

		sprintf(blockid, "%03d", 
			captives[iblock]->tblock->iblock);

		if (iblock){
			strcat(buf, " ");
		}
		strcat(buf, blockid);
	}
	strcat(buf, "\n");
	return strlen(buf);
}

static DEVICE_ATTR(sigtblocks, S_IRUGO, show_sigtblocks, 0);

static int mk_ppcustom_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	DEVICE_CREATE_FILE(dev, &dev_attr_sigtblocks);
	return 0;
}


static void rm_ppcustom_sysfs(struct device *dev) 
{
	device_remove_file(dev, &dev_attr_version);
	device_remove_file(dev, &dev_attr_sigtblocks);
}



static void reserve_tblocks(void)
{
	int it;

	if (captives){
		for (it = 0; it < nsigtblocks; ++it){
			if (captives[it] == 0){
				info("captives[%d] was null (ignore)", it);
				continue;
			}
			acq200_replaceFreeTblock(captives[it]);
		}
		kfree(captives);
	}
	captives = kzalloc(nsigtblocks*sizeof(struct TblockListElement *), 
			   GFP_KERNEL);


	for (it = 0; it < nsigtblocks; ++it){
		SIG(it) = acq200_reserveFreeTblock();
		if (SIG(it) == 0){
			info("SIG(%d) was null (ignore)", it);
			break;
		}
		atomic_inc(&SIG(it)->tblock->in_phase);
		SIG(it)->tblock->touched = 0;
		SIG(it)->phase_sample_start = 0;
		SIG(it)->sample_count = 0;
		

		dbg(1, "%d: tblock:%03d in_phase:%d",
		    it, captives[it]->tblock->iblock,
		    atomic_read(&captives[it]->tblock->in_phase));
	}	
}

static void onStart(void *notused)
{
	reserve_tblocks();
	block_cursor = -1;
	sig_cursor = 0;
	last_stride = 0;
	delta_sam_total = 0;
	dbg(1, "01 cursor:%d", sig_cursor);
}

static int cps_open(struct inode *inode, struct file *filp)
{
	dbg(1, "return 0");
	return 0;	/* WORKTODO */
}
static int cps_release(struct inode *inode, struct file *filp)
{
	dbg(1, "return 0");
        return 0;
}
ssize_t cps_read(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
	int iblock;
	loff_t _pos = *f_pos;

	dbg(1, "01");

	for (iblock = 0; iblock <= block_cursor; ++iblock){
		loff_t p1 = (loff_t)SIG(iblock)->phase_sample_start;
		loff_t p2 = p1 + SIG(iblock)->sample_count;
		if (_pos >= p1 && _pos < p2){
			size_t headroom = (size_t)(p2 - _pos);
			int nbytes = min(count, headroom);
			void *src = VA_TBLOCK(SIG(iblock)->tblock) + (_pos-p1);

			if (copy_to_user(buf, src, nbytes)){
				dbg(1, "return %d", -EFAULT);
				return -EFAULT;
			}else{
				*f_pos += nbytes;
				dbg(1, "return %d", nbytes);
				return nbytes;
			}			
		}
	}

	dbg(1, "return 0");
	return 0;
}


static int cps_cdev_create(void)
{
	static struct file_operations cps_fops = {
		.owner = THIS_MODULE,
		.open = cps_open,
		.read = cps_read,
		.release = cps_release
	};
	int rc = register_chrdev(0, "cps_transform", &cps_fops);

	if (rc >= 0){
		cps_major = rc;
		return 0;
	}else{
		err("can't get major %d", rc);
		return rc;
	}
}

static void cps_cdev_remove(void)
{
	unregister_chrdev(cps_major, "cps_transform");
}


static struct Hookup startHook = {
	.the_hook = onStart,
	.clidata = 0,
};



static struct device_driver cps_transform_driver;

static 	struct Transformer transformer = {
	.name = "cps",
	.transform = cps_transform
};


static int cps_transform_probe(struct device *dev)
{
	int it;
	info("");

	it = acq200_registerTransformer(&transformer);
	acq200_add_start_of_shot_hook(&startHook);

	info("transformer registered %d", it);

	if (it >= 0){
		acq200_setTransformer(it);
	}else{
		err("transformer NOT registered");
	}

	cps_cdev_create();
	mk_ppcustom_sysfs(dev);
	dbg(1, "99");
	return 0;
}

static int cps_transform_remove(struct device *dev)
{
	if (captives){
		kfree(captives);
	}
	rm_ppcustom_sysfs(dev);
	cps_cdev_remove();
	acq200_unregisterTransformer(&transformer);
	acq200_del_start_of_shot_hook(&startHook);
	return 0;
}


static struct device_driver cps_transform_driver = {
	.name     = "cps_transform",
	.probe    = cps_transform_probe,
	.remove   = cps_transform_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static void cps_transform_dev_release(struct device * dev)
{
	info("");
}


static struct platform_device cps_transform_device = {
	.name = "cps_transform",
	.id   = 0,
	.dev = {
		.release    = cps_transform_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init cps_transform_init( void )
{
	int rc = driver_register(&cps_transform_driver);
	if (rc){
		return rc;
	}

	return platform_device_register(&cps_transform_device);
}


static void __exit
cps_transform_exit_module(void)
{
	info("");
	platform_device_unregister(&cps_transform_device);
	driver_unregister(&cps_transform_driver);
}

module_init(cps_transform_init);
module_exit(cps_transform_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("CPS data transformer");


