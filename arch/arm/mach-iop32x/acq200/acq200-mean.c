/* ------------------------------------------------------------------------- */
/* acq200-mean.c - rolling mean values for acq200                            */
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

#define VERID "$Revision: 1.5 $ build B1035 "

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

#include "acq32busprot.h"

/* example: 20kHz capture, 10Hz output, nmean=16 (2 bit SNR improvement)
 *
 * nmean = 16
 * cskip = 20000 / 10 / 16 = 125
 */

#define LOG2_MEAN_DEF	4

int acq200_mean_debug;
module_param(acq200_mean_debug, int, 0664);

int log2_mean = LOG2_MEAN_DEF;
module_param(log2_mean, int, 0444);

#define nmean (1<<log2_mean)

int skip = 125;
module_param(skip, int, 0444);

int iter = 0;
module_param(iter, int, 0444);

char* verid = VERID;
module_param(verid, charp, 0444);


int update_interval_ms;
module_param(update_interval_ms, int, 0444);

int overruns;
module_param(overruns, int, 0644);

int xxs_waiting;
module_param(xxs_waiting, int, 0644);

int xxs_nowaiting;
module_param(xxs_nowaiting, int, 0644);

int acq164_shr = 8;
module_param(acq164_shr, int, 0444);
/**< shift right (shr) for ACQ164 data - module_param makes it visible. */

int out_shr = LOG2_MEAN_DEF;
module_param(out_shr, int, 0444);
/*** shift right (shr) for computing output value - nominal log2_mean */

#define B24_IDEAL_SHR	8
#define B24_MAX_SHR	16

#define TD_SZ (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((1+(numchan)+1+5+1)*TD_SZ)

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

/** MC - Mean Consumer - task context data consumer */

#define DLEN	512
#define NCB	4
#define MCP	struct MC*

/** histogram of number of samples per read, XXS */
int xxs_readsam[NCB];
module_param_array(xxs_readsam, int, NULL, 0644);

/** state store for D-TACQ streaming frame protocol. */
struct TagState {
	int line;
	unsigned s0, s1;
	unsigned extra;
};

struct MC {
	union {
		struct MC_HEADER {
			struct StateListener listener;
			int (*filler)(MCP self, const void* from, int nbytes);
			struct u32_ringbuffer empties;
			struct TagState tagState;	/* streaming only */
		}
			header;			
		char _pad[DLEN];
	} u;
	char buffers[NCB][DLEN];
};


static struct MEAN_CONSUMERS {
	struct list_head clients;
	spinlock_t lock;
} mc_list;


static int maybe_es16(u16 data1)
{
	return data1 == 0xaa55;
}

static void sum_up_s16(void *data)
{
	short *channels = (short*)data;
	int ic;


	for (ic = 0; ic != app_state.nchannels; ++ic){
		if (maybe_es16((u16)channels[ic])){
			break;
		}
		work_state.the_sums[ic] += channels[ic];
	}
}


static int maybe_es32(u32 data2) 
{
	u16 lh = data2 >> 16;
	u16 rh = data2 & 0x0000ffff;

	return lh == 0xaa55 || rh == 0xaa55;
}

static void sum_up_acq164(void *data)
/* acq164 data is 24 bit wide in a 32 bit field. right shift to 
 * eliminate effect of overflow (for NACC < 256)
 * eliminate ES - even if it's not ES, no harm to leave it out .. 
 */
{
	int *channels = (int *)data;
	int ic;
	int shr = acq164_shr;

	for (ic = 0; ic != app_state.nchannels; ++ic){
		if (maybe_es32((u32)channels[ic])){
			break;
		}		
		work_state.the_sums[ic] += channels[ic] >> shr;
	}	
}

static void (*sum_up)(void *data) = sum_up_s16;


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

static void run_consumers(u32 *sums, int sumslen)
{
	struct MC *mc;

	spin_lock(&mc_list.lock);
	
	/* now alert any listeners */
	list_for_each_entry(mc, &mc_list.clients, u.header.listener.list){
		dbg(2, "filler %p", mc->u.header.filler);
		mc->u.header.filler(mc, sums, sumslen);
	}		

	spin_unlock(&mc_list.lock);
}


static void _mean_work(void *data)
{
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
		run_consumers(app_state.the_sums, SUMSLEN);	
	}
}



static void mean_work(void *data, int nbytes) 
{
	int nsamples = nbytes / app_state.sample_size;

	dma_map_single(DG->dev, data, nbytes, DMA_FROM_DEVICE);

	while(nsamples--){
		if (++work_state.iskip >= skip){
			_mean_work(data);
			data += app_state.sample_size;
			work_state.iskip = 0;
		}
	}
}


static struct task_struct *the_worker;


void add_mc(struct MC *mc)
{
	spin_lock(&mc_list.lock);
	list_add_tail(&mc_list.clients, &mc->u.header.listener.list);
	spin_unlock(&mc_list.lock);
}

void del_mc(struct MC *mc)
{
	spin_lock(&mc_list.lock);
	list_del(&mc->u.header.listener.list);
	spin_unlock(&mc_list.lock);
}

static void start_work(void *clidata) 
{
	int _enable = *(int*)clidata;

	if (!_enable) return;

	dbg(1, "01");
	work_state.iskip = work_state.imean = 0;

	out_shr = log2_mean;

	if (DG->btype == BTYPE_ACQ164){
		sum_up = sum_up_acq164;
		/** prevent overflow by limiting log2_mean, and discarding
		 * LSB's. Do not discard too many LSB's ! 
		 */
		if (log2_mean > B24_IDEAL_SHR){
			acq164_shr = min(log2_mean, B24_MAX_SHR);
			log2_mean = acq164_shr;
			out_shr = log2_mean - (acq164_shr - B24_IDEAL_SHR);
		}else{
			acq164_shr = B24_IDEAL_SHR;
		}
	}else{
		sum_up = sum_up_s16;
	}
	acq200_addRefillClient(mean_work);
	on_arm();

	dbg(1, "99 the worker %p", the_worker);
}
static void stop_work(void)
{
	dbg(1, "01");
	acq200_delRefillClient(mean_work);
	dbg(1, "99");            
}

static int getMean(int lchan) 
{
	int pchan = acq200_lookup_pchan(lchan);
	int mean;

	spin_lock(&app_state.lock);
	mean = app_state.the_sums[pchan] >> out_shr;
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



static int ch_release(struct inode *inode, struct file *filp) 
{
	return 0;
}

static int mean_filler(MCP self, const void *from, int nbytes)
{
	u32 ibuf;
	dbg(1, "self :%p from:%p nbytes:%d", self, from, nbytes);

	if (u32rb_get(&self->u.header.empties, &ibuf) == 0){
		++overruns;
		return -1;
	}else{
		int *pbuf = (int*)self->buffers[ibuf];
		int ichan;
		int cursor = 0;
		int nchan = nbytes/sizeof(int);

		for (ichan = 1; cursor < nchan; ++cursor, ++ichan){
			int my_mean = getMean(ichan);
			pbuf[cursor] = my_mean;
		}
		u32rb_put(&self->u.header.listener.rb, ibuf);
		wake_up_interruptible(&self->u.header.listener.waitq);
	}
       
	return 0;
}

static void initFrame(struct TagState *tag_state)
{
	const unsigned long long scc = DMC_WO->scc.scc;
	tag_state->s0 = scc&0xffffffffUL;
	tag_state->s1 = scc>>32;
	tag_state->extra = update_interval_ms;
}

static unsigned short tag_getDIO32(int ibyte)
{
	union {
		unsigned dio32;
		unsigned char bytes[4];
	} u;

	u.dio32 = acq200_getDIO32();
	return u.bytes[ibyte];
}

static unsigned short tag_getDIO6(void)
{
	return acq200_getDI6();
}

static unsigned short getTrigger(void)
{
	return DMC_WO->pit_count != 0;
}
static unsigned short buildTag(struct TagState *tag_state)
{
	const int iline = tag_state->line;
	unsigned short the_tag;
	unsigned nX;

	switch (iline){
	case 0:
		initFrame(tag_state);
		the_tag = 0xfe;
		break;
	case 1:
		the_tag = 0xed;
		break;
	default:
		if (iline&1){
			the_tag = tag_getDIO32((iline>>1)&0x3);
		}else{
			the_tag = tag_getDIO6();
		}
	}

	the_tag |= (iline << s0_bit) | (getTrigger() << T_bit);

	if (iline < 32){
		nX = (tag_state->s0 >> iline) & 0x1U;
	}else if (iline < NID_BITS){
		nX = (tag_state->s1 >> (iline-32)) & 0x1U;
	}else{
		nX = (tag_state->extra >> (iline-NID_BITS)) & 0x1U;       
	}
	the_tag |= nX << nX_bit;

	tag_state->line = (iline+1)&0x3f;

	return the_tag;
}

static int stream_filler(struct MC *self, const void *from, int nbytes)
{
	u32 ibuf;
	dbg(1, "self :%p from:%p nbytes:%d", self, from, nbytes);

	if (u32rb_get(&self->u.header.empties, &ibuf) == 0){
		++overruns;
		return -1;
	}else{
		short *pbuf = (short*)self->buffers[ibuf];
		int ichan;
		int cursor = 0;
		int nchan = nbytes/sizeof(int);

		*(unsigned short*)pbuf = buildTag(&self->u.header.tagState);
		pbuf++;

		for (ichan = 1; cursor < nchan; ++cursor, ++ichan){
			short my_mean = (short)getMean(ichan);
			pbuf[cursor] = my_mean;
		}
		u32rb_put(&self->u.header.listener.rb, ibuf);
		wake_up_interruptible(&self->u.header.listener.waitq);
	}
       
	return 0;
}

#define FMC(filp) (struct MC*)((filp)->private_data)

static void create_mc(
	struct file* filp, 
	int (*filler)(struct MC *self, const void *from, int nbytes)
	)
{
	struct MC *mc = (struct MC*)__get_free_page(GFP_KERNEL);	
	int ibuf;

	clear_page(mc);

	sl_init(&mc->u.header.listener, NCB);
	u32rb_init(&mc->u.header.empties, NCB);
	for (ibuf = 0; ibuf < NCB; ++ibuf){
		u32rb_put(&mc->u.header.empties, ibuf);
	}	
	mc->u.header.filler = filler;
	add_mc(mc);	
	filp->private_data = mc;
}

static void delete_mc(struct file* filp)
{
	struct MC* mc = FMC(filp);
	assert(FMC(filp) != 0);
	del_mc(mc);
	free_page((unsigned)mc);
}
static int xx_open(struct inode *inode, struct file *filp) 
{
	create_mc(filp, mean_filler);
	return 0;
}

static int stream_open(struct inode *inode, struct file *filp) 
{
	create_mc(filp, stream_filler);
	
	return 0;
}  
static ssize_t xx_read(	
	struct file *filp, char *buf, size_t count, loff_t *offset)
/** read a single sample and return */
{
	struct MC *mc = FMC(filp);
	u32 ibuf = 0;

	if (count > app_state.sample_size*sizeof(int)){
		count = app_state.sample_size*sizeof(int);
	}

	if ((filp->f_flags & O_NONBLOCK) == 0){
		while(u32rb_get(&mc->u.header.listener.rb, &ibuf) == 0){
			int rc = wait_event_interruptible(
				mc->u.header.listener.waitq,
				!u32rb_is_empty(&mc->u.header.listener.rb));
			if (rc != 0){
				return rc;
			}
		}
		COPY_TO_USER(buf, mc->buffers[ibuf], count);
		u32rb_put(&mc->u.header.empties, ibuf);
	}else{
		int ichan;
		int nread = 0;

		for (ichan = 1; nread < count; nread += sizeof(int), ++ichan){
			int my_mean = getMean(ichan);
			COPY_TO_USER(buf+nread, &my_mean, sizeof(int));
		}
	}

	*offset += count;

	return count;
}

static int xx_release(struct inode *inode, struct file *filp) 
{
	delete_mc(filp);
	return 0;
}

static ssize_t stream_read(	
	struct file *filp, char *buf, size_t count, loff_t *offset)
/** read a single sample and return */
/** WARNING: NO EOF, so it only makes sense to read on timer */
{
	struct MC *mc = FMC(filp);
	u32 ibuf = 0;
	int line_sz = app_state.sample_size + 2;
	int nbytes = 0;
	int waited = 0;
	int readsam = 0;

	/* (file->f_flags & O_NONBLOCK) */

	if (count < line_sz){
		return -1;
	}

	if ((filp->f_flags & O_NONBLOCK) == 0){
		while(u32rb_get(&mc->u.header.listener.rb, &ibuf) == 0){
			int rc = wait_event_interruptible(
				mc->u.header.listener.waitq,
				!u32rb_is_empty(&mc->u.header.listener.rb));
			if (rc != 0){
				return rc;
			}
			waited = 1;
		}
		
		do {		
			COPY_TO_USER(buf+nbytes, mc->buffers[ibuf], line_sz);
			u32rb_put(&mc->u.header.empties, ibuf);
			nbytes += line_sz;
			++readsam;
		} while( nbytes+line_sz <= count && 
			 u32rb_get(&mc->u.header.listener.rb, &ibuf) != 0);

		xxs_readsam[min(readsam, NCB)-1]++;
		if (waited){
			xxs_waiting++;
		}else{
			xxs_nowaiting++;
		}
	}else{
		int ichan;
		int nread = 0;
		
		for (ichan = 1; nread < line_sz; nread += sizeof(short)){
			short my_mean = (short)getMean(ichan++);
			COPY_TO_USER(buf+nread, &my_mean, sizeof(short));
		}
		nbytes = line_sz;		
	}


	*offset += nbytes;

	return nbytes;
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
		start_work((void*)&enable);
	}else if (lbuf[0] == '0'){
		enable = 0;
		stop_work();
	}
	return count;
}




static ssize_t parameter_mean_read(
	struct file *filp, char *buf, size_t count, loff_t *offset,
	int pram)
{
	if (*offset != 0){
		return 0;
	}else{
		char str[16];
		int nchars = snprintf(str, sizeof(str)-1, "%d\n", pram);
		COPY_TO_USER(buf, str, nchars);
		*offset	+= nchars;
		return nchars;
	}
}

static ssize_t parameter_mean_write(
	struct file *filp, const char *buf, size_t count, loff_t *offset,
	int* pram, int pmin, int pmax)
{
	char lbuf[16];
	int _pram;
	
	COPY_FROM_USER(lbuf, buf, min(count, sizeof(lbuf)));
	if (sscanf(lbuf, "%d", &_pram) == 1 && _pram >= pmin && _pram < pmax){
		if (enable){
			stop_work();
		}
		*pram = _pram;
		if (enable){
			start_work((void*)&enable);
		}
	}
	return count;
}


static ssize_t log2_mean_read(
	struct file *filp, char *buf, size_t count, loff_t *offset)
{
	return parameter_mean_read(filp, buf, count, offset, log2_mean);
}
static ssize_t log2_mean_write(
	struct file *filp, const char *buf, size_t count, loff_t *offset)
{
	return parameter_mean_write(filp, buf, count, offset, 
				    &log2_mean, 0, 15);
}

static ssize_t skip_mean_read(
	struct file *filp, char *buf, size_t count, loff_t *offset)
{
	return parameter_mean_read(filp, buf, count, offset, skip);
}

static ssize_t skip_mean_write(
	struct file *filp, const char *buf, size_t count, loff_t *offset)
{
	return parameter_mean_write(filp, buf, count, offset, 
				    &skip, 0, 32000);
}




static int meanfs_fill_super(struct super_block *sb, void *data, int silent)
{
	static struct file_operations chops = {
		.open = ch_open,
		.read = ch_read,
		.release = ch_release
	};
	static struct file_operations xxops = {
		.open = xx_open,
		.read = xx_read,
		.release = xx_release
	};
	static struct file_operations stream_ops = {
		.open = stream_open,
		.read = stream_read,
		.release = xx_release
	};
	static struct file_operations enops = {
		.write = enable_write,
		.read = enable_read
	};
	static struct file_operations log2_mean_ops = {
		.write = log2_mean_write,
		.read = log2_mean_read
	};	
	static struct file_operations skip_ops = {
		.write = skip_mean_write,
		.read = skip_mean_read
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

#define ADDFILE(_name, _ops, _mode) do {	\
	my_files[fn].name = _name;		\
	my_files[fn].ops = _ops;		\
	my_files[fn++].mode = _mode;		\
} while(0)

	memcpy(&my_files[fn++], &front, TD_SZ);
	memcpy(&my_files[fn++], &front, TD_SZ);

	for (ichan = 1; ichan <= nchan; ++ichan){
		sprintf(my_names[ichan].name, "%02d", ichan);
		ADDFILE(my_names[ichan].name, &chops, S_IRUGO);
	}

	ADDFILE("XX",		&xxops,		S_IRUGO);
	ADDFILE("XXS",		&stream_ops,	S_IRUGO);
	ADDFILE("enable",	&enops,		S_IRUGO|S_IWUGO);
	ADDFILE("log2_mean",    &log2_mean_ops, S_IRUGO|S_IWUGO);
	ADDFILE("skip",		&skip_ops,      S_IRUGO|S_IWUGO);

	memcpy(&my_files[fn], &backstop, TD_SZ);

	dbg(1, "mean_filler:   %p", mean_filler);
	dbg(1, "stream_filler: %p", stream_filler);

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

	INIT_LIST_HEAD(&mc_list.clients);
	spin_lock_init(&mc_list.lock);
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

static struct Hookup start_action = {
	.the_hook = start_work,
	.clidata = &enable
};
static int __init acq200_mean_init( void )
{
	acq200_debug = acq200_mean_debug;

	info("%s %s %s",
	     acq200_mean_driver_name,
	     acq200_mean_driver_version, acq200_mean_copyright);

	init_buffers();	
	enable = 1;
	start_work((void*)&enable);
	acq200_add_start_of_shot_hook(&start_action);
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

