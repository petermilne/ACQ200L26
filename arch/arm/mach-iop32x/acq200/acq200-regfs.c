/* ------------------------------------------------------------------------- */
/* acq200-regfs.c file per reg access to DSP regs area in BAR[2]	     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2004 Peter Milne, D-TACQ Solutions Ltd
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
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>

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
#define acq200_debug acq200_regfs_debug   

#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

int acq200_regfs_debug;
module_param(acq200_regfs_debug, int, 0664);

#define EBAD_FORMAT 1234

#define VERID "$Revision: 1.1 $ build B1000 "



char acq200_regfs_driver_name[] = "acq200-regfs";
char acq200_regfs_driver_string[] = "D-TACQ Intelligent ACQ device";
char acq200_regfs_driver_version[] = VERID __DATE__;
char acq200_regfs_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";


/** @file acq200-regfs.c vfs mapping for DSP Extra register block.
 */

/** number of 32 bit registers modelled */
#define MAX_REG 512

#define PRINT_DECIMAL 1


#define REG_VA(private) \
	((u32*)((unsigned long)(my_pointers[(int)private]) &~ PRINT_DECIMAL))
#define IS_DECIMAL(private) \
	((unsigned long)(my_pointers[(int)private]) & PRINT_DECIMAL)

#define SET_DECIMAL(private)						\
do {									\
	unsigned long tmp = (unsigned long)my_pointers[(int)private];	\
	tmp |= PRINT_DECIMAL;						\
	my_pointers[(int)private]	= (void*)tmp;			\
} while (0)

#define CLR_DECIMAL(private)						\
do {									\
	unsigned long tmp = (unsigned long)my_pointers[(int)private];	\
	tmp &= ~PRINT_DECIMAL;						\
	my_pointers[(int)private]	= (void*)tmp;			\
} while (0)


static u32** my_pointers;

static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       acq200_regfs_driver_name,
		       acq200_regfs_driver_string,
		       acq200_regfs_driver_version,
		       acq200_regfs_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);


static int mk_reg_sysfs(struct device *dev)
{
	device_create_file(dev, &dev_attr_version);
	return 0;
}


u32 nowhere = 0xdeadda1a;

static int reg_data_open(struct inode *inode, struct file *filp)
{
	if (inode->i_ino == 0 || inode->i_ino > MAX_REG){
		return -ENODEV;
	}else{
		filp->private_data = (void*)inode->i_ino;
		return 0;
	}
}


static ssize_t reg_data_read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	int len = sizeof(u32);
	u32 *reg_va = REG_VA(filp->private_data);
	int is_decimal = IS_DECIMAL(filp->private_data);

	char lbuf[16];




	if (*offset > 0){
		dbg(1, "*offset > 0 return 0");
		return 0;
	}

	len = sprintf(lbuf, is_decimal? "%d\n": "0x%08x\n", *reg_va);

	if (count < len){
		dbg(1, "len < count %d < %d return 0", len, count);
		return 0;
	}

	count = len;

	if (copy_to_user(buf, lbuf, count)){
		dbg(1, "FAULT");
		return -EFAULT;
	}else{
		*offset += count;
		dbg(1, "SUCCESS: return %d", count);
		return count;
	}
}


static ssize_t reg_data_write(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	u32 *reg_va = REG_VA(filp->private_data);
	char lbuf[16] = {};
	u32 value;

	if (*offset > 0){
		return 0;
	}
	if (count >= 15){
		count = 15;
	}

	copy_from_user(lbuf, buf, count);

	if (sscanf(lbuf, "0x%x", &value) == 1){
		CLR_DECIMAL(filp->private_data);
	}else if (sscanf(lbuf, "%d", &value) == 1){
		SET_DECIMAL(filp->private_data);
	}else{
		return -EBAD_FORMAT;
	}
		
	*reg_va = value;
	*offset += count;
	return count;
}










#define REGFS_MAGIC 0xa2111112

#define TD_SZ  (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((1+(numchan)+2+1)*TD_SZ)
static struct tree_descr *my_files;

typedef char Name[4];

static Name* my_names;



static void init_inodes(struct super_block *sb)
{
	struct inode *inode;

	list_for_each_entry(inode, &sb->s_inodes, i_sb_list){
		if (inode->i_ino > 0 && inode->i_ino < MAX_REG){
			inode->i_size = sizeof(u32);
		}
	}
}

static int acq200_regfs_fill_super (
	struct super_block *sb, void *data, int silent)
{
	static struct file_operations access_ops = {
		.open = reg_data_open,
		.read = reg_data_read,
		.write = reg_data_write
	};

	static struct tree_descr front = {
		NULL, NULL, 0
	};
	static struct tree_descr backstop = {
		"", NULL, 0
	};

	int iblock;
	int rc;

	info("create tree for %d regs", MAX_REG);
	my_files = kmalloc(MY_FILES_SZ(MAX_REG), GFP_KERNEL);
	if (!my_files){
		err("failed to allocate memory for my_files %d",
		    MY_FILES_SZ(MAX_REG));
		return -ENOMEM;
	}	
	my_names = kmalloc((MAX_REG+1)*sizeof(Name), GFP_KERNEL);


	if (!my_names){
		err("failed to allocate memory for my_names %d",
		    (MAX_REG+1)*sizeof(Name));
		kfree(my_files);
		return -ENOMEM;
	}


	my_pointers = kmalloc((MAX_REG+1)*sizeof(u32), GFP_KERNEL);
	if (!my_pointers){
		err("failed to allocate backing memory");
		kfree(my_names);
		kfree(my_files);
		return -ENOMEM;
	}

	memcpy(&my_files[0], &front, TD_SZ);
	
	for (iblock = 1; iblock <= MAX_REG; ++iblock){

		int offset = 2*(iblock - 1); /* 64 bit boundary */

		if (DG->fpga.extra.va != 0){
			my_pointers[iblock] = (u32*)DG->fpga.extra.va + offset;
		}else{
			my_pointers[iblock] = &nowhere;
		}


		sprintf(my_names[iblock], "%03d", iblock-1);
		my_files[iblock].name = my_names[iblock];
		my_files[iblock].ops  = &access_ops;
		my_files[iblock].mode = S_IRUGO|S_IWUGO;
	}

	memcpy(&my_files[iblock++], &backstop, TD_SZ);

	info("call simple_fill_super");
	rc = simple_fill_super(sb, REGFS_MAGIC, my_files);

	if (rc == 0){
		init_inodes(sb);
	}

	return rc;
}




static int acq200_regfs_get_super(
	struct file_system_type *fst,
	int flags, const char *devname, void *data,
	struct vfsmount* mnt)
{
	return get_sb_single(
		fst, flags, data, acq200_regfs_fill_super, mnt);
}

static struct file_system_type custom_fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "regfs",
	.get_sb		= acq200_regfs_get_super,
	.kill_sb	= kill_litter_super,
};

static void mk_regfs(void)
/* store results as nodes in a custom file system */
{
	register_filesystem(&custom_fs_type);
}

static void rm_regfs(void)
{
	unregister_filesystem(&custom_fs_type);
	kfree(my_files);
	kfree(my_names);
	kfree(my_pointers);
}

static void acq200_regfs_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver acq200_regfs_driver;

static int acq200_regfs_probe(struct device *dev)
{
	info("");
	mk_regfs();
	mk_reg_sysfs(dev);

	dbg(1, "99");
	return 0;
}

static int acq200_regfs_remove(struct device *dev)
{
	rm_regfs();

	return 0;
}


static struct device_driver acq200_regfs_driver = {
	.name     = "acq200_regfs",
	.probe    = acq200_regfs_probe,
	.remove   = acq200_regfs_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq200_regfs_device = {
	.name = "acq200_regfs",
	.id   = 0,
	.dev = {
		.release    = acq200_regfs_dev_release,
		.dma_mask   = &dma_mask
	}

};




static int __init acq200_regfs_init( void )
{
	acq200_debug = acq200_regfs_debug;

	driver_register(&acq200_regfs_driver);
	return platform_device_register(&acq200_regfs_device);
}


static void __exit
acq200_regfs_exit_module(void)
{
	info("");
	platform_device_unregister(&acq200_regfs_device);
	driver_unregister(&acq200_regfs_driver);
}

module_init(acq200_regfs_init);
module_exit(acq200_regfs_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Filesystem for DSP regs area");


