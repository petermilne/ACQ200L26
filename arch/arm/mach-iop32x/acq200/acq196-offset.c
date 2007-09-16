/* ------------------------------------------------------------------------- */
/* acq196-offset.c offset control for acq196                                 */
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

#define REVID "$Revision: 1.4 $ B101\n"

#define DTACQ_MACH 2
#define ACQ196
#define ACQ_IS_INPUT 1

#define FPGA_INT   IRQ_ACQ100_FPGA
#define FPGA_INT_MASK (1<<FPGA_INT)

#include "acq200-fifo-top.h"

#include "acq200_debug.h"
#include "acq200-fifo-local.h"
#include "acq200-fifo.h"
#include "acq196.h"


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>     	/* This is where libfs stuff is declared */
#include <asm/atomic.h>
#include <asm/uaccess.h>	/* copy_to_user */


#define LFS_MAGIC 0xa196a196

#define TD_SZ  (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((2+(numchan)+1+1)*TD_SZ)


#define MAXCHAN 96

#define DACX 0
#define DACY 1

#define MAXBLOCKS  3
#define NCHANNELSBLOCK 32
#define NDACSBLOCK 4
#define NDACSCHIP  8

/* convert ino to channel - ch01 is at ino 2 */
#define INO2CH(ch) (int)((ch) - 1)  
/*
 * map chip, dac to channel in block
 */
static const int MAP[NDACSBLOCK+1][NDACSCHIP+1] = {
/* MAP[chip][dac] ... unfortunately, these numbers are pchan order :-( */
	[ 1][DACA] = 1,  [ 1][DACB] = 2,
	[ 1][DACC] = 17, [ 1][DACD] = 18,
	[ 1][DACE] = 9,  [ 1][DACF] = 10,
	[ 1][DACG] = 25, [ 1][DACH] = 26,
	[ 2][DACA] = 3,  [ 2][DACB] = 4,
	[ 2][DACC] = 19, [ 2][DACD] = 20,
	[ 2][DACE] = 11, [ 2][DACF] = 12,
	[ 2][DACG] = 27, [ 2][DACH] = 28,
	[ 3][DACA] = 5,  [ 3][DACB] = 6,
	[ 3][DACC] = 21, [ 3][DACD] = 22,
	[ 3][DACE] = 13, [ 3][DACF] = 14,
	[ 3][DACG] = 29, [ 3][DACH] = 30,
	[ 4][DACA] = 7,  [ 4][DACB] = 8,
	[ 4][DACC] = 23, [ 4][DACD] = 24,
	[ 4][DACE] = 15, [ 4][DACF] = 16,
	[ 4][DACG] = 31, [ 4][DACH] = 32,
};


static	int plut[] = {
		[ 1] =  1, 
		[ 2] = 17, 
		[ 3] =  2, 
		[ 4] = 18, 
		[ 5] =  9,	
		[ 6] = 25,
		[ 7] = 10,
		[ 8] = 26,

		[ 9] =  3,
		[10] = 19,
		[11] =  4,
		[12] = 20,
		[13] = 11,
		[14] = 27,
		[15] = 12,
		[16] = 28,

		[17] =  5,
		[18] = 21,
		[19] =  6,
		[20] = 22,
		[21] = 13,
		[22] = 29,
		[23] = 14,
		[24] = 30,

		[25] =  7,
		[26] = 23,
		[27] =  8,
		[28] = 24,
		[29] = 15,
		[30] = 31,
		[31] = 16,
		[32] = 32 
};

int acq200_lookup_pchan(int lchannel)
{
	int block = ((lchannel-1)/32);
	int index = lchannel - block*32;

	return block*32 + plut[index] - 1;
}

int acq200_lookup_lchan(int pchan)
{
	int block = ((pchan-1)/32);
	int match = ((pchan-1)%32) + 1;
	int index = 1;

	for (; index <= 32; ++index){
		if (plut[index] == match){
			return block * 32 + index;
		}
	}
	return -1;
}

/*
 * iterators
 */
static int each_dac(int dac)
{
	int ndac;

	switch(dac){
	case 0: 		ndac = DACA; break;
	case DACA:		ndac = DACB; break;
	case DACB:              ndac = DACC; break;
	case DACC:		ndac = DACD; break;
	case DACD:		ndac = DACE; break;
	case DACE:		ndac = DACF; break;
	case DACF:		ndac = DACG; break;
	case DACG:		ndac = DACH; break;
	case DACH:		ndac = 0;    break;
	default:                ndac = 0;    break;
	}

	dbg(1, "each_dac( %c ) returns %c\n", dac-1+'A', ndac-1+'A');

	return ndac;
}

static int each_block(int block){
	if (block == 0){
		return NCHAN/NCHANNELSBLOCK;
	}else{
		return --block;
	}
}


static int each_chip(int chip){
	if (chip < 4){
		return chip + 1;
	}else{
		return 0;
	}
}

static unsigned short offsets[MAXBLOCKS+1][NCHANNELSBLOCK+1];


static inline unsigned short *key2offset(int ikey)
/* ikey 1..96 */
{
	int block = (ikey-1)/NCHANNELSBLOCK + 1;
	int lchan = (ikey-1)%NCHANNELSBLOCK + 1;
	int pchan = acq200_lookup_pchan(lchan);

	dbg(1, "key2offset key %d block %d lchan %d pchan %d",
	    ikey, block, lchan, pchan);

	return &offsets[block][pchan+1];
}
static void set_offset(int ikey, unsigned short offset)
{
	*key2offset(ikey) = offset;
}


static unsigned short get_offset(int ikey)
{
	return *key2offset(ikey);
}

static unsigned short lookup_offset(int block, int chip, int dac)
{
	dbg(1, "block: %d chip:%d dac:%c pchan %02d lchan %02d value:%d",
	    block, chip, dac-1+'A', 
	    (block-1)*32+MAP[chip][dac], 
	    acq200_lookup_lchan((block-1)*32+MAP[chip][dac]),
	    offsets[block][MAP[chip][dac]] );

	if (MAP[chip][dac] == 0){
		err("BAD MAP[%d][%d]", chip, dac);
	}
	return offsets[block][MAP[chip][dac]];
}

static inline void set_chipsel(void)
{
	*ACQ196_OFFSET_DACS = ACQ196_OFFSET_DACS_CHIPSEL;

	dbg(1, "write %08x readback %08x",
	    ACQ196_OFFSET_DACS_CHIPSEL, *ACQ196_OFFSET_DACS);
}


static inline void write_data(int dac, unsigned short opair[])
{
	u32 addr = dac << ACQ196_OFFSET_DACS_ASHIFT;
	u32 control = 
		((opair[DACX] | addr) << ACQ196_OFFSET_DACS_XSHIFT) |
		((opair[DACY] | addr) << ACQ196_OFFSET_DACS_YSHIFT);
	u32 status;

	*ACQ196_OFFSET_DACS = control;
	
	while((status = *ACQ196_OFFSET_DACS)&ACQ196_OFFSET_DACS_HSHAKE){
		dbg(3, "polling HSHAKE %08x", status);
	}

	dbg(1, "wrote:%08x final:%08x", control, status);
}
static void write_all(void)
{
	int dac;
	int block;
	int chip;
	int dacx = DACX;
	unsigned short opair[2];

	for (dac = 0; (dac = each_dac(dac)); ){
		for (block = 0; (block = each_block(block)); ){
			for (chip = 0; (chip = each_chip(chip)); ){
				opair[dacx] = lookup_offset(block, chip, dac);
				
				switch(dacx){
				case DACX:
					dacx = DACY;
					break;
				case DACY:
					write_data( dac, opair);
					dacx = DACX;
					break;
				}
			}
		}
		set_chipsel();
	}
}

#define TMPSIZE 32

static int access_open(struct inode *inode, struct file *filp)
{
	int ch = INO2CH(inode->i_ino);

	if (ch < 1 || ch > MAXCHAN){
		return -ENODEV;
	}
	filp->private_data = (void*)ch;
	return 0;
}


static int hw_fudge(int lchan)
{
	switch(lchan){
	case 61:
		return 63;
	case 63:
		return 61;
	default:
		return lchan;
	}
}

static ssize_t access_read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	char tmp[TMPSIZE];
	int len;
	unsigned yy = get_offset(hw_fudge((int)filp->private_data));

	len = snprintf(tmp, TMPSIZE, "%d\n", yy);
	if (*offset > len){
		return 0;
	}
	if (count > len - *offset){
		count = len - *offset;
	}
	if (copy_to_user(buf, tmp + *offset, count)){
		return -EFAULT;
	}else{
		*offset += count;
		return count;
	}
}


static ssize_t access_write(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	char tmp[TMPSIZE];
	long yy;

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
	yy = simple_strtol(tmp, NULL, 0);
	if (yy < 0 ) yy = 0;
	if (yy > 1023 ) yy = 1023;

	set_offset(hw_fudge((int)filp->private_data), (unsigned short)yy);
	return count;
}


static int commit_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t commit_write(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	if (*offset != 0){
		return -EINVAL;
	}else{
		write_all();
	}

	return count;
}

static ssize_t commit_read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	char tmp[80];
	int len;

	len = snprintf(tmp, 80, REVID);
	if (*offset > len){
		return 0;
	}
	if (count > len - *offset){
		count = len - *offset;
	}
	if (copy_to_user(buf, tmp + *offset, count)){
		return -EFAULT;
	}else{
		*offset += count;
		return count;
	}
	return 0;
}


static struct tree_descr *my_files;



static int offsetfs_fill_super (struct super_block *sb, void *data, int silent)
{
	static struct file_operations access_ops = {
		.open = access_open,
		.read = access_read,
		.write = access_write
	};
	static struct file_operations commit_ops = {
		.open = commit_open,
		.write = commit_write,
		.read = commit_read,
	};
	static struct tree_descr front = {
		NULL, NULL, 0
	};
	static struct tree_descr commit = {
		.name = "commit",
		.ops = &commit_ops,
		.mode = S_IWUSR|S_IRUGO
	};
	static struct tree_descr backstop = {
		"", NULL, 0
	};
	static char names[MAXCHAN+1][4];

	int ichan;
	int fn = 0;	/* file number in fs */

	for (ichan = 1; ichan <= MAXCHAN; ++ichan){
		sprintf(names[ichan], "%02d", ichan);
	}

	my_files = kmalloc(MY_FILES_SZ(NCHAN), GFP_KERNEL);

	memcpy(&my_files[fn++], &front, TD_SZ);
	memcpy(&my_files[fn++], &front, TD_SZ);
	
	for (ichan = 1; ichan <= NCHAN; ++ichan, fn++){
		my_files[fn].name = names[ichan];
		my_files[fn].ops  = &access_ops;
		my_files[fn].mode = S_IWUSR|S_IRUGO;
	}

	memcpy(&my_files[fn++], &commit, TD_SZ);
	memcpy(&my_files[fn++], &backstop, TD_SZ);
	
	return simple_fill_super(sb, LFS_MAGIC, my_files);
}



static int offsetfs_get_super(
	struct file_system_type *fst,
	int flags, 
	const char *devname, 
	void *data,
	struct vfsmount* mnt
	)
{
	return get_sb_single(
		fst, flags, data, offsetfs_fill_super, mnt);
}



static struct file_system_type offset_fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "adcoffsetfs",
	.get_sb		= offsetfs_get_super,
	.kill_sb	= kill_litter_super,
};






int acq196_offset_fs_create(struct device* device)
{
	info("about to register fs");
	return register_filesystem(&offset_fs_type);
}
void acq196_offset_fs_remove(void)
{
	unregister_filesystem(&offset_fs_type);
	kfree(my_files);
}

EXPORT_SYMBOL_GPL(acq200_lookup_pchan);
