/* ------------------------------------------------------------------------- */
/* acq100-offset-inc.c acq100 common offset defs NB INCLUDED FILE            */
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

#define REVID "$Revision: 1.4 $ B102\n"
/*
 * From example at http://lwn.net/Articles/57373/
 * Copyright 2002, 2003 Jonathan Corbet <corbet-AT-lwn.net>
 * This file may be redistributed under the terms of the GNU GPL.
 */


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
	if (chip < NCHIPSBLOCK){
		return chip + 1;
	}else{
		return 0;
	}
}


static inline void set_chipsel(void)
{
	*ACQ100_OFFSET_DACS = ACQ100_OFFSET_DACS_CHIPSEL;

	dbg(1, "write %08x readback %08x",
	    ACQ100_OFFSET_DACS_CHIPSEL, *ACQ100_OFFSET_DACS);
}


static inline void write_data(int dac, unsigned short opair[])
{
	u32 addr = dac << ACQ100_OFFSET_DACS_ASHIFT;
	u32 control = 
		((opair[DACX] | addr) << ACQ100_OFFSET_DACS_XSHIFT) |
		((opair[DACY] | addr) << ACQ100_OFFSET_DACS_YSHIFT);
	u32 status;

	*ACQ100_OFFSET_DACS = control;
	
	while((status = *ACQ100_OFFSET_DACS)&ACQ100_OFFSET_DACS_HSHAKE){
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

	*ACQ100_OFFSET_DACS = 0;
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







int acq100_offset_fs_create(struct device* device)
{
	info("about to register fs");
	return register_filesystem(&offset_fs_type);
}
void acq100_offset_fs_remove(void)
{
	unregister_filesystem(&offset_fs_type);
	kfree(my_files);
}

void acq200_setChannelLut(const int *lut, int _nlut)
{
	if (lut){
		plut = lut;
		nlut = _nlut;
	}else{
		plut = __plut;
		nlut = __PLUT_ELEMS;
	}
}

EXPORT_SYMBOL_GPL(acq200_lookup_pchan);
EXPORT_SYMBOL_GPL(acq200_setChannelLut);
