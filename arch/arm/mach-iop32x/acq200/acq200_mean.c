/* ------------------------------------------------------------------------- */
/* acq200_mean.c - rolling mean values for acq200                            */
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

#define REVID "$Revision: 1.5 $ B102\n"


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
#define acq200_debug acq200_mean_debug   

#include "acq200_debug.h"
#include "acq200-fifo-local.h"     /* DG */


int acq200_mean_debug;
module_param(acq200_mean_debug, int, 0664);

int nmean = 64;
module_param(nmean, int, 0664);

int cskip = 0;
module_param(cskip, int, 0444);

int nskips = 0;
module_param(nskips, int, 0444);

int stub = 0;
module_param(stub, int, 0664);

#define VERID "$Revision: 1.5 $ build B1000 "

#define TD_SZ (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((1+(numchan)+1+1+1)*TD_SZ)

#define MEANFS_MAGIC 0xbea02005
#define TMPSIZE 20

char acq200_mean_driver_name[] = "acq200-mean";
char acq200_mean_driver_string[] = "rolling mean of captured data";
char acq200_mean_driver_version[] = VERID __DATE__;
char acq200_mean_copyright[] = "Copyright (c) 2005 D-TACQ Solutions Ltd";


static void on_arm(void);


static struct tree_descr *my_files;
struct CHNAME {
	char name[4];
};
static struct CHNAME* my_names;

static int enable;


/** experiment: use kthread to collect data */

#include <linux/kthread.h>


struct DataEntry {
	struct list_head list;
	void* data;
};

static struct MeanDataStore {
	struct list_head busy_list;
	struct list_head free_list;
	struct list_head x_list;
	int samples_in_list;
	int sample_size;
	int nchannels;
	int* the_sums;
	struct DataEntry* entries;
} mds;


#define PTR(offset)  (va_buf(DG) + offset)


static void _boxcar16(int chmax, int *sums, void *old, void *new)
{
	int ch;

	for (ch = chmax; ch--; ){
		int delta = ((short*)new)[ch];
		if (old){
			delta -= ((short*)old)[ch];
		}
		sums[ch] += delta;
	}
}

static void _boxcar32(int chmax, int *sums, void *old, void *new)
{
	int ch;

	for (ch = chmax; ch--; ){
		int delta = ((int*)new)[ch];
		if (old){
			delta -= ((int*)old)[ch];
		}
		sums[ch] += delta;
	}
}
static void _boxcar24(int chmax, int *sums, void *old, void *new)
{
	int ch;
	union{
		int ival;
		unsigned char bval[4];
        } xnew, xold;

	xold.ival = 0;
#define GETS24(x, buf, off) do {				\
		x.bval[0] = ((unsigned char*)buf)[off+0];	\
		x.bval[1] = ((unsigned char*)buf)[off+1];	\
		x.bval[2] = ((unsigned char*)buf)[off+2];	\
		x.bval[3] = (x.bval[2]&0x80)? 0xff: 0;		\
	} while(0)

	for (ch = chmax; ch--; ){
		int offset = 3 * ch;
		
		GETS24(xnew, new, offset);
		
		if (old){
			GETS24(xold, old, offset);
		}
		sums[ch] += xnew.ival - xold.ival;
	}
}


static void boxcar(
	void (*_boxcar)(int chmax, int *sums, void *old, void *new),
	struct DataConsumerBuffer* dcb
) 
/** boxcar average over nmeans points (lazy average on read). */
{
	struct DataEntry* x;
	int headroom = (dcb->last_finish - dcb->last_start)/mds.sample_size;
	int chmax = mds.nchannels;

	if (stub) chmax = 1;

	if (headroom > nmean){
		dcb->last_start += (headroom - nmean)*mds.sample_size;
	}
	
	while (dcb->last_start < dcb->last_finish){
		void *new_data = (short *)PTR(dcb->last_start);

		if (likely(mds.samples_in_list == nmean)){
			/** boxcar rolling along */
			list_move(mds.busy_list.prev, &mds.x_list);
			x = list_entry(mds.x_list.next, 
				       struct DataEntry, list);
			_boxcar(chmax, mds.the_sums, x->data, new_data);

			x->data = new_data;
			list_move(mds.x_list.next, &mds.busy_list);
			dcb->last_start += mds.sample_size;
		}else if (mds.samples_in_list > nmean){
			/** someone changed nmean on us! */
			while(mds.samples_in_list > nmean){
				list_move(mds.busy_list.prev, &mds.free_list);
				--mds.samples_in_list;
			}
			continue;
		}else if (list_empty(&mds.free_list)){
			/** out of samples - fake a result */
			mds.samples_in_list = nmean;
			err("free_list empty");
			continue;
		}else{
			/** fill the boxcar */
			list_move(mds.free_list.next, &mds.x_list);
			x = list_entry(mds.x_list.next, 
				       struct DataEntry, list);
		        x->data = new_data;

			_boxcar(chmax, mds.the_sums, 0, new_data);

			list_move(mds.x_list.next, &mds.busy_list);
			mds.samples_in_list++;
			dcb->last_start += mds.sample_size;
		}
	}
}


static int mean_work(void *clidata) 
{
	int iter = 0;
	void (*my_boxcar_action)(int chmax, int *sums, void *old, void *new);
	       
	struct DataConsumerBuffer *dcb = acq200_createDCB();

	acq200_addDataConsumer(dcb);
	
	switch(capdef_get_word_size()){
	case 4:
		my_boxcar_action = _boxcar32; break;
	case 3:
		my_boxcar_action = _boxcar24; break;
	case 2:
	default:
		my_boxcar_action = _boxcar16;
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
		      
		       

		if (iter == 0){
			dcb->last_start = new_start;
		}else if (TBLOCK_NUM(dcb->last_start) != 
			  TBLOCK_NUM(new_start)){
			dcb->last_start = new_start;
			myskips++;
		}
		dma_sync_single(DG->dev,
				dcb->handle + dcb->last_start, 
				dcb->last_finish - dcb->last_start, 
				DMA_FROM_DEVICE);       

		if (stub < 2){
			boxcar(my_boxcar_action, dcb);
		}

		dbg(1,"hello from mean_work %d offset:%06x", 
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
	the_worker = kthread_run(mean_work, NULL, "mean_work");
}
static void stop_work(void)
{
	kthread_stop(the_worker);
	/** WARNING: may not be enough to allow worker to stop */
	schedule();              
}

static int getMean(int lchan) 
{
	int pchan = acq200_lookup_pchan(lchan);

	if (mds.samples_in_list){
		return mds.the_sums[pchan]/mds.samples_in_list;
	}else{
		return 0;
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
        if (count > sizeof(int) && *offset == 0){
		int my_mean = getMean((int)filp->private_data);
		COPY_TO_USER(buf, &my_mean, sizeof(int));
		*offset = 4;
		return sizeof(int);
	}
	return 0;
}

static int ch_release(struct inode *inode, struct file *filp) 
{
	return 0;
}

static ssize_t enable_read(
	struct file *filp, char *buf, size_t count, loff_t *offset)
{
	char *value = enable? "1": "0";
	COPY_TO_USER(buf, value, 1);
	return 1;
}

static ssize_t enable_write(
	struct file *filp, const char *buf, size_t count, loff_t *offset)
{
	char lbuf[4];
	
	COPY_FROM_USER(lbuf, buf, 1);
	if (lbuf[0] == '1'){
		enable = 1;
		start_work();
	}else if (lbuf[0] == '0'){
		enable = 0;
		stop_work();
	}
	return count;
}



static int meanfs_fill_super(struct super_block *sb, void *data, int silent)
{
	static struct file_operations chops = {
		.open = ch_open,
		.read = ch_read,
		.release = ch_release
	};
	static struct file_operations enops = {
		.write = enable_write,
		.read = enable_read
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
		my_files[ichan].mode = S_IRUGO;
	}

	my_files[ichan].name = "enable";
	my_files[ichan].ops = &enops;
	my_files[ichan].mode = S_IRUGO|S_IWUGO;
	ichan++;

	memcpy(&my_files[ichan], &backstop, TD_SZ);

	return simple_fill_super(sb, MEANFS_MAGIC, my_files);
}


static int meanfs_get_super(
	struct file_system_type *fst,
	int flags, const char *devname, void *data,
	struct vfsmount* mnt)
{
	return get_sb_single(fst, flags, data, meanfs_fill_super, mnt);
}

static void on_arm(void)
{
	int ch;


	while(!list_empty(&mds.busy_list)){
		list_move(&mds.busy_list, &mds.free_list);
	}
	mds.samples_in_list = 0;
	mds.sample_size = sample_size();
	mds.nchannels = get_nchan();

	for (ch = 0; ch < mds.nchannels; ++ch){
		mds.the_sums[ch] = 0;
	}
}

static void init_buffers(void)
{
	int nchan = CAPDEF_get_nchan();
	int entry;

	my_files = kmalloc(MY_FILES_SZ(nchan), GFP_KERNEL);
	my_names = kmalloc((1+nchan)*sizeof(struct CHNAME), GFP_KERNEL);
	mds.the_sums = kmalloc((nchan)*sizeof(int), GFP_KERNEL);
	mds.entries = 
		kmalloc(max(nmean,64)*sizeof(struct DataEntry), GFP_KERNEL);

	INIT_LIST_HEAD(&mds.busy_list);
	INIT_LIST_HEAD(&mds.free_list);
	INIT_LIST_HEAD(&mds.x_list);

	for (entry = max(nmean,64); entry--; ){
		list_add(&(mds.entries[entry].list), &mds.free_list);
	}
}
static void release_buffers(void)
{
	if (my_files) kfree(my_files);
	if (my_names) kfree(my_names);
	if (mds.the_sums) kfree(mds.the_sums);
	if (mds.entries) kfree(mds.entries);
}

static struct file_system_type meanfs_type = {
	.owner 		= THIS_MODULE,
	.name		= "meanfs",
	.get_sb		= meanfs_get_super,
	.kill_sb	= kill_litter_super,
};

static int __init acq200_mean_init( void )
{
	acq200_debug = acq200_mean_debug;

	info("%s %s %s",
	     acq200_mean_driver_name,
	     acq200_mean_driver_version, acq200_mean_copyright);

	init_buffers();	
	start_work();
	return register_filesystem(&meanfs_type);
}


static void __exit
acq200_mean_exit_module(void)
{
	stop_work();                 
	unregister_filesystem(&meanfs_type);
	release_buffers();
}

module_init(acq200_mean_init);
module_exit(acq200_mean_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Module for 2G Mean acq data");

