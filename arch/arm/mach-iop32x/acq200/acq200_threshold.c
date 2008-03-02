/* ------------------------------------------------------------------------- */
/* acq200_threshold.c - threshold values for acq200                          */
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

/*
 * From example at http://lwn.net/Articles/57373/
 * Copyright 2002, 2003 Jonathan Corbet <corbet-AT-lwn.net>
 * This file may be redistributed under the terms of the GNU GPL.
 */

/*
 * VFS From example at http://lwn.net/Articles/57373/
 * Copyright 2002, 2003 Jonathan Corbet <corbet-AT-lwn.net>
 * This file may be redistributed under the terms of the GNU GPL.
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/moduleparam.h>
#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <linux/fs.h>     	/* This is where libfs stuff is declared */
#include <asm/uaccess.h>	/* copy_to_user */

/* keep debug local to this module */
#define acq200_debug acq200_threshold_debug   

#include "acq200_debug.h"
#include "acq200-fifo-local.h"     /* DG */


int acq200_threshold_debug;
module_param(acq200_threshold_debug, int, 0664);

int cskip = 0;
module_param(cskip, int, 0444);

int nskips = 0;
module_param(nskips, int, 0444);

int stub = 0;
module_param(stub, int, 0664);

int trigger_count = 0;
module_param(trigger_count, int, 0444);

#define VERID "$Revision: 1.6 $ build B1002 "

#define TD_SZ (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((1+(numchan)+1+1)*TD_SZ)

#define THRESHOLDFS_MAGIC 0xbeb02005
#define TMPSIZE 80

char acq200_threshold_driver_name[] = "acq200-threshold";
char acq200_threshold_driver_string[] = "rolling threshold of captured data";
char acq200_threshold_driver_version[] = VERID __DATE__;
char acq200_threshold_copyright[] = "Copyright (c) 2005 D-TACQ Solutions Ltd";


static void on_arm(void);


static struct tree_descr *my_files;
struct CHNAME {
	char name[4];
};
static struct CHNAME* my_names;


/** internal data is all in physchan order */

struct Threshold {
	int th_min;
	int th_max;
};

static struct Threshold* TH;      /* one threshold per channel */
static int TH_NCHAN;              /* number of channels        */
static int th_state;

/** experiment: use kthread to collect data */

#include <linux/kthread.h>


typedef int (*threshold_calculator)(void* data);







/** output function */

#define DIO_REG_TYPE (volatile u16*)
#include "acqX00-rtm.h"


#define OUTPUT_IDLE 0
#define OUTPUT_TRIG 1

static void set_output(int state){
	dbg(1, "state %s", state==OUTPUT_TRIG? "OUTPUT_TRIG": "OUTPUT_IDLE");

	/** NB: must set direction as outputs - set ALL the bits! */
	setDO32(state==OUTPUT_TRIG? 0xffffffff: 0);
}


/** threshold_calculators - required for 3 possible input word sizes */

static int _threshold_calculator16(void* data)
{
	short* d16 = (short*)data;
	int ch;

	for (ch = 0; ch != TH_NCHAN; ch++ ){
		short xx = *d16++;

		if (TH[ch].th_max == TH[ch].th_min){
			continue;
		}else if (xx > TH[ch].th_max){
			dbg(2, "%02d %d > %d trigger", 
			    ch, xx, TH[ch].th_max);
			return 1;
		}else if (xx < TH[ch].th_min){
			dbg(2, "%02d %d < %d trigger",
			    ch, xx, TH[ch].th_min);
			return 1;
		}
	}
	return 0;
}

static int _threshold_calculator24(void* data)
{
	int ch;
	union{
		int ival;
		unsigned char bval[4];
        } xnew;
	char* d8 =(char*)data;
	int offset = 0;

#define GETS24(x, buf, off) do {			\
		x.bval[0] = (d8)[off+0];		\
		x.bval[1] = (d8)[off+1];		\
		x.bval[2] = (d8)[off+2];		\
		x.bval[3] = (x.bval[2]&0x80)? 0xff: 0;	\
	} while(0)

	for (ch = 0; ch < TH_NCHAN; ++ch, offset += 3){
		GETS24(xnew, new, offset);

		if (TH[ch].th_max == TH[ch].th_min){
			continue;
		}else if (xnew.ival > TH[ch].th_max){
			return 1;
		}else if (xnew.ival < TH[ch].th_min){
			return 1;
		}
	}

	return 0;
}
static int _threshold_calculator32(void* data)
{
	int* d32 = (int*)data;
	int ch;

	for (ch = 0; ch != TH_NCHAN; ch++ ){
		short xx = *d32++;

		if (TH[ch].th_max == TH[ch].th_min){
			continue;
		}else if (xx > TH[ch].th_max){
			return 1;
		}else if (xx < TH[ch].th_min){
			return 1;
		}
	}
	return 0;
}



static int do_threshold(
	threshold_calculator calculator,
	struct DataConsumerBuffer *dcb,
	int sample_sz)
{
	while(dcb->last_start < dcb->last_finish){
		void *new_data = (void*)BB_PTR(dcb->last_start);

		if (calculator(new_data)){
			dcb->last_start = dcb->last_finish;
			return 1;
		}else{
			dcb->last_start += sample_sz;
		}
	}

	return 0;
}

static int threshold_work(void *clidata) 
{
	int iter = 0;
	threshold_calculator my_calculator;
	       
	struct DataConsumerBuffer *dcb = acq200_createDCB();
	int sample_sz = sample_size();

	acq200_addDataConsumer(dcb);
	
	switch(capdef_get_word_size()){
	case 4:
		my_calculator = _threshold_calculator32; break;
	case 3:
		my_calculator = _threshold_calculator24; break;
	case 2:
	default:
		my_calculator = _threshold_calculator16;
	}
		
	while(1){
		unsigned new_start;
		int myskips = 0;
		if (kthread_should_stop()){
			break;
		}
		wait_event_interruptible(
			dcb->waitq, !u32rb_is_empty(&dcb->rb));
		
		while (!u32rb_is_empty(&dcb->rb)){
			u32rb_get(&dcb->rb, &dcb->last_finish);
		}

		new_start = TBLOCK_START(TBLOCK_NUM(dcb->last_finish));
		      
		if (TBLOCK_NUM(dcb->last_start) != TBLOCK_NUM(new_start)){
			dbg(1, "new TBLOCK %08x %08x %08x",
			    dcb->last_start, new_start, dcb->last_finish);

			dcb->last_start = new_start;
			myskips++;
		}

		dma_sync_single(DG->dev,
				dcb->handle + dcb->last_start, 
				dcb->last_finish - dcb->last_start, 
				DMA_FROM_DEVICE);       

		if (stub < 2){
			if (do_threshold(my_calculator, dcb, sample_sz)){
				if (th_state != OUTPUT_TRIG){
					set_output(th_state = OUTPUT_TRIG);
					trigger_count++;
				}
			}else{
				if (th_state != OUTPUT_IDLE){
					set_output(th_state = OUTPUT_IDLE);
				}
			}
		}

		dbg(3,"hello from threshold_work %d offset:%06x", 
		    iter, dcb->last_finish);

		iter++;
		cskip = myskips;
		nskips += myskips;
	}

	acq200_removeDataConsumer(dcb);
	acq200_deleteDCB(dcb);
	
	return 0;
}

static struct task_struct *the_worker;

static void start_work(void) 
{
	on_arm();
	the_worker = kthread_run(threshold_work, NULL, "threshold_work");
}
static void stop_work(void)
{
	kthread_stop(the_worker);
	/** WARNING: may not be enough to allow worker to stop */
	schedule();              
}


static struct Threshold* getThreshold(int lchan) 
{
	return &TH[acq200_lookup_pchan(lchan)];
}

static void setThreshold(int lchan, int th_min, int th_max)
{
	if (lchan >= 1 && lchan <= TH_NCHAN){
		struct Threshold* th = &TH[acq200_lookup_pchan(lchan)];
		th->th_min = th_min;
		th->th_max = th_max;
	}
}


static int ch_open(struct inode *inode, struct file *filp) 
{
	filp->private_data = (void*)inode->i_ino;
	return 0;
}

static ssize_t ch_read(
	struct file *filp, char *buf, size_t count, loff_t *offset)
{
	char str[80];
	struct Threshold* th = getThreshold((int)filp->private_data);
	sprintf(str, "%d,%d\n", th->th_min, th->th_max);

	if (count > strlen(str) && *offset == 0){
		if (copy_to_user(buf, str, strlen(str))){
			return -EFAULT;
		}
		*offset = strlen(str);
		return strlen(str);
	}else{
		return 0;
	}
}

static ssize_t ch_write(
	struct file* filp, const char *buf, size_t count, loff_t *offset)
{
	char tmp[80];
	int th_min, th_max;

	if (*offset != 0){
		return -EINVAL;
	}
	if (count > TMPSIZE){
		return -EINVAL;
	}
	memset(tmp, 0, TMPSIZE);
	if (copy_from_user(tmp, buf, count)){
		return -EFAULT;
	}
	if ( sscanf(tmp, "%d,%d", &th_min, &th_max) == 2 ||
	     sscanf(tmp, "0x%x,0x%x", &th_min, &th_max) == 2 ){
		setThreshold((int)filp->private_data, th_min, th_max);
	}
	return count;
}


static int ch_release(struct inode *inode, struct file *filp) 
{
	return 0;
}




static int thresholdfs_fill_super(struct super_block *sb, void *data, int silent)
{
	static struct file_operations chops = {
		.open = ch_open,
		.write = ch_write,
		.read = ch_read,
		.release = ch_release
	};
	static struct tree_descr front = {
		NULL, NULL, 0
	};
	static struct tree_descr backstop = {
		"", NULL, 0
	};

	int nchan = CAPDEF_get_nchan();
	int ichan;


	memcpy(&my_files[0], &front, TD_SZ);

	for (ichan = 1; ichan <= nchan; ++ichan){
		sprintf(my_names[ichan].name, "%02d", ichan);
		my_files[ichan].name = my_names[ichan].name;
		my_files[ichan].ops = &chops;
		my_files[ichan].mode = S_IRUGO|S_IWUGO;
	}
	memcpy(&my_files[ichan], &backstop, TD_SZ);

	return simple_fill_super(sb, THRESHOLDFS_MAGIC, my_files);
}


static int thresholdfs_get_super(
	struct file_system_type *fst,
	int flags, const char *devname, void *data,
	struct vfsmount* mnt)
{
	return get_sb_single(fst, flags, data, thresholdfs_fill_super, mnt);
}

static void on_arm(void)
{
	trigger_count = 0;
	set_output(th_state = OUTPUT_IDLE);
}

static void init_thresholds(void)
/**< set thresholds that will "never" trigger */
{
	int nchan = CAPDEF_get_nchan();
	int ichan;
	int th_min;
	int th_max;

	switch(capdef_get_word_size()){
	case 4:
		th_max = 0x7fffffff;
		th_min = -th_max;
		break;
	case 3:
		th_max = 0x007fffff;
		th_min = -th_max;
		break;
	case 2:
	default:
		th_max = 0x00007fff;
		th_min = -th_max;
	}

	for (ichan = 0; ichan != nchan; ++ichan){
		TH[ichan].th_min = th_min;
		TH[ichan].th_max = th_max;
	}	
	TH_NCHAN = nchan;
}

static void init_buffers(void)
{
	int nchan = CAPDEF_get_nchan();

	my_files = kmalloc(MY_FILES_SZ(nchan), GFP_KERNEL);
	my_names = kmalloc((1+nchan)*sizeof(struct CHNAME), GFP_KERNEL);

	TH = kmalloc(sizeof(struct Threshold) * nchan, GFP_KERNEL);
	init_thresholds();
}
static void release_buffers(void)
{
	if (my_files) kfree(my_files);
	if (my_names) kfree(my_names);
	if (TH) kfree(TH);
}

static struct file_system_type thresholdfs_type = {
	.owner 		= THIS_MODULE,
	.name		= "thresholdfs",
	.get_sb		= thresholdfs_get_super,
	.kill_sb	= kill_litter_super,
};

static int __init acq200_threshold_init( void )
{
	acq200_debug = acq200_threshold_debug;

	info("%s %s %s",
	     acq200_threshold_driver_name,
	     acq200_threshold_driver_version, acq200_threshold_copyright);

	init_buffers();	
	start_work();
	return register_filesystem(&thresholdfs_type);
}


static void __exit
acq200_threshold_exit_module(void)
{
	stop_work();                 
	unregister_filesystem(&thresholdfs_type);
	release_buffers();
}

module_init(acq200_threshold_init);
module_exit(acq200_threshold_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Module for 2G Threshold acq data");

