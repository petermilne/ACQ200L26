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

#define VERID "$Revision: 1.5 $ build B1021 "

/*
 * VFS From example at http://lwn.net/Articles/57373/
 * Copyright 2002, 2003 Jonathan Corbet <corbet-AT-lwn.net>
 * This file may be redistributed under the terms of the GNU GPL.
*/

#include <linux/jiffies.h>
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

#include "acqX00-port.h"
/* keep debug local to this module */
#define acq200_debug acq200_mean_debug   

#include "acq200_debug.h"
#include "acq200-fifo-local.h"     /* DG */


/* example: 20kHz capture, 10Hz output, nmean=16 (2 bit SNR improvement)
 *
 * nmean = 16
 * cskip = 20000 / 10 / 16 = 125
 */

int acq200_mean_debug;
module_param(acq200_mean_debug, int, 0664);

int log2_mean = 4;
module_param(log2_mean, int, 0664);

#define nmean (1<<log2_mean)

int skip = 125;
module_param(skip, int, 0664);

int iter = 0;
module_param(iter, int, 0444);

int nice_mean = 10;
module_param(nice_mean, int, 0644);

char* verid = VERID;
module_param(verid, charp, 0444);


int update_interval_ms;
module_param(update_interval_ms, int, 0444);

#define TD_SZ (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((1+(numchan)+1+2+1)*TD_SZ)

#define MEANFS_MAGIC 0xbea02005
#define TMPSIZE 20

char acq200_mean_driver_name[] = "acq200-mean";
char acq200_mean_driver_string[] = "rolling mean of captured data";
char acq200_mean_driver_version[] = VERID __DATE__;
char acq200_mean_copyright[] = "Copyright (c) 2005 D-TACQ Solutions Ltd";


/* convert ino to channel - ch01 is at ino 2 */
#define INO2CH(ch) (int)((ch) - 1)  

static void on_arm(void);


static struct tree_descr *my_files;
struct CHNAME {
	char name[4];
};
static struct CHNAME* my_names;

static int enable;



static struct MeanDataStore {
	int samples_in_list;
	int sample_size;
	int nchannels;
	int* the_sums;
	spinlock_t lock;
} app_state;


#define SUMSLEN (app_state.nchannels * sizeof(int))

static struct MEAN_WORK_STATE {
	int iskip;
	int imean;
	int *the_sums;
} work_state;

static void sum_up(void *data)
{
	int ic;
	short *channels = (short*)data;

	dma_map_single(DG->dev, data, app_state.sample_size, DMA_FROM_DEVICE);

	for (ic = 0; ic != app_state.nchannels; ++ic){
		work_state.the_sums[ic] += channels[ic];
	}
}

#define LHIST 3
#define NHIST (1<<LHIST)
#define HINC(ix)	(((ix)+1)&(NHIST-1))

static void calculate_interval(void)
/* store delta jiffies in history, do boxcar avg */
{
	static unsigned long history[NHIST];
	static unsigned long previous;
	static unsigned long boxcar;
	static int ix;

	unsigned long dj;

	if (previous == 0){
		previous = jiffies;
		return;
	}
	/* jiffies is negative for the first 5 mins ... */
	if (likely(time_after(jiffies, previous))){
		dj = jiffies - previous;
	}else{
		dj = previous - jiffies;
	}

	boxcar = boxcar - history[ix] + dj;
	history[ix] = dj;
	update_interval_ms = ((boxcar*1000) >> LHIST) / HZ;	

	previous = jiffies;
	ix = HINC(ix);
}

static void mean_work(void *data) 
{
	if (++work_state.iskip >= skip){
		sum_up(data);
		if (++work_state.imean >= nmean){

			dbg(1, "summing up iter:%d ch01:0x%04x sum:0x%08x",
				iter, *(short*)data,
				work_state.the_sums[0]);

			spin_lock(&app_state.lock);
			memcpy(app_state.the_sums, work_state.the_sums,SUMSLEN);
			spin_unlock(&app_state.lock);
			memset(work_state.the_sums, 0, SUMSLEN);
			work_state.imean = 0;
			iter++;
			calculate_interval();
		}
		work_state.iskip = 0;
	}
}

static struct task_struct *the_worker;

void acq200_setRefillClient(void (*client)(void *pbuf))
{
	spin_lock(&DG->refillClient.lock);
	DG->refillClient.client = mean_work;
	spin_unlock(&DG->refillClient.lock);
}

static void start_work(void) 
{
	dbg(1, "01");
	work_state.iskip = work_state.imean = 0;
	acq200_setRefillClient(mean_work);
	on_arm();

	dbg(1, "99 the worker %p", the_worker);
}
static void stop_work(void)
{
	dbg(1, "01");
	acq200_setRefillClient(0);
	dbg(1, "99");            
}

static int getMean(int lchan) 
{
	int pchan = acq200_lookup_pchan(lchan);
	int mean;

	spin_lock(&app_state.lock);
	mean = app_state.the_sums[pchan] >> log2_mean;
	spin_unlock(&app_state.lock);
	return mean;
}

static int ch_open(struct inode *inode, struct file *filp) 
{
	int ch = INO2CH(inode->i_ino);

	dbg(3, "ch %d", ch);
	filp->private_data = (void*)ch;
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

static ssize_t xx_read(	
	struct file *filp, char *buf, size_t count, loff_t *offset)
/** read a single sample and return */
/** WARNING: NO EOF, so it only makes sense to read on timer */
{
	int nread = 0;
	int ichan;

	if (count > app_state.sample_size*sizeof(int)){
		count = app_state.sample_size*sizeof(int);
	}

	for (ichan = 1; nread < count; nread += sizeof(int), ++ichan){
		int my_mean = getMean(ichan);
		COPY_TO_USER(buf+nread, &my_mean, sizeof(int));
	}
	*offset += count;

	return count;
}


static int ch_release(struct inode *inode, struct file *filp) 
{
	return 0;
}

static ssize_t enable_read(
	struct file *filp, char *buf, size_t count, loff_t *offset)
{
	if (*offset != 0){
		return 0;
	}else{
		char *value = enable? "1": "0";
		COPY_TO_USER(buf, value, 1);
		*offset	+= 1;
		return 1;
	}
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
	static struct file_operations xxops = {
		.open = ch_open,
		.read = xx_read,
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
	int fn = 0;


	memcpy(&my_files[fn++], &front, TD_SZ);
	memcpy(&my_files[fn++], &front, TD_SZ);

	for (ichan = 1; ichan <= nchan; ++ichan, ++fn){
		sprintf(my_names[ichan].name, "%02d", ichan);
		my_files[fn].name = my_names[ichan].name;
		my_files[fn].ops = &chops;
		my_files[fn].mode = S_IRUGO;
	}

	my_files[fn].name = "XX";
	my_files[fn].ops = &xxops;
	my_files[fn].mode = S_IRUGO;
	fn++;

	my_files[fn].name = "enable";
	my_files[fn].ops = &enops;
	my_files[fn].mode = S_IRUGO|S_IWUGO;
	fn++;

	memcpy(&my_files[fn], &backstop, TD_SZ);

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
	dbg(1, "on_arm 01");
	app_state.sample_size = sample_size();
	app_state.nchannels = CAPDEF_get_nchan();
	memset(app_state.the_sums, 0, SUMSLEN);

	dbg(1, "on_arm 99");
}

static void init_buffers(void)
{
	int nchan = CAPDEF_get_nchan();

	my_files = kmalloc(MY_FILES_SZ(nchan), GFP_KERNEL);
	my_names = kmalloc((2+nchan)*sizeof(struct CHNAME), GFP_KERNEL);
	app_state.nchannels = CAPDEF_get_nchan();
	app_state.the_sums = kmalloc(SUMSLEN, GFP_KERNEL);
	work_state.the_sums = kmalloc(SUMSLEN, GFP_KERNEL);
}

static void release_buffers(void)
{
	if (my_files) kfree(my_files);
	if (my_names) kfree(my_names);
	if (app_state.the_sums) kfree(app_state.the_sums);
	if (work_state.the_sums) kfree(work_state.the_sums);
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
	enable = 1;
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

