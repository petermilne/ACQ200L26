/* ------------------------------------------------------------------------- */
/* acq200-tblockfs.c custom post allows raw access to entire bigbuf for diags*/
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

/** WARNING: NON FUNCTIONAL AT THIS TIME */

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
#define acq200_debug acq200_tblockfs_debug   

#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

int acq200_tblockfs_debug;
module_param(acq200_tblockfs_debug, int, 0666);


#define VERID "$Revision: 1.3 $ build B1000 "


char acq200_tblockfs_driver_name[] = "acq200-rawtbraw";
char acq200_tblockfs_driver_string[] = "D-TACQ Intelligent ACQ device";
char acq200_tblockfs_driver_version[] = VERID __DATE__;
char acq200_tblockfs_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";






static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       acq200_tblockfs_driver_name,
		       acq200_tblockfs_driver_string,
		       acq200_tblockfs_driver_version,
		       acq200_tblockfs_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);


static int mk_tblock_sysfs(struct device *dev)
{
	device_create_file(dev, &dev_attr_version);
	return 0;
}




static int tblock_data_open(struct inode *inode, struct file *filp)
{
	if (inode->i_ino == 0 || inode->i_ino > MAX_TBLOCK){
		return -ENODEV;
	}else{
		int blocknum = inode->i_ino + 1;
		filp->private_data = &DG->bigbuf.tblocks.the_tblocks[blocknum];
		return 0;
	}
}


static ssize_t tblock_data_read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
{
	struct TBLOCK* tblock = (struct TBLOCK*)filp->private_data;
	int len = tblock->length;
	void *tblock_va = tblock->offset + va_buf(DG);

	if (*offset > len){
		return 0;
	}
	if (count > len - *offset){
		count = len - *offset;
	}
	if (copy_to_user(buf, tblock_va + *offset, count)){
		return -EFAULT;
	}else{
		*offset += count;
		return count;
	}
}


static int tblock_data_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct TBLOCK* tblock = (struct TBLOCK*)filp->private_data;

	return io_remap_pfn_range( 
		vma, vma->vm_start, 
		__phys_to_pfn(pa_buf(DG)+tblock->offset), 
		vma->vm_end - vma->vm_start, 
		vma->vm_page_prot 
	);
}








#define TBFS_MAGIC 0xa2111111

#define TD_SZ  (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((1+(numchan)+2+1)*TD_SZ)
static struct tree_descr *my_files;

typedef char Name[4];

static Name* my_names;

static int acq200_tblockfs_fill_super (
	struct super_block *sb, void *data, int silent)
{
	static struct file_operations access_ops = {
		.open = tblock_data_open,
		.read = tblock_data_read,
		.mmap = tblock_data_mmap
	};

	static struct tree_descr front = {
		NULL, NULL, 0
	};
	static struct tree_descr backstop = {
		"", NULL, 0
	};

	int iblock;

	info("create tree for %d tblocks", MAX_TBLOCK);
	my_files = kmalloc(MY_FILES_SZ(MAX_TBLOCK), GFP_KERNEL);
	if (!my_files){
		err("failed to allocate memory for my_files %d",
		    MY_FILES_SZ(MAX_TBLOCK));
		return -ENOMEM;
	}	
	my_names = kmalloc((MAX_TBLOCK+1)*sizeof(Name), GFP_KERNEL);
	if (!my_names){
		err("failed to allocate memory for my_names %d",
		    (MAX_TBLOCK+1)*sizeof(Name));

		kfree(my_files);
		return -ENOMEM;
	}

	memcpy(&my_files[0], &front, TD_SZ);
	
	for (iblock = 1; iblock <= MAX_TBLOCK; ++iblock){
		sprintf(my_names[iblock], "%02d", iblock);
		my_files[iblock].name = my_names[iblock];
		my_files[iblock].ops  = &access_ops;
		my_files[iblock].mode = S_IRUGO;
	}

	memcpy(&my_files[iblock++], &backstop, TD_SZ);

	info("call simple_fill_super");
	return simple_fill_super(sb, TBFS_MAGIC, my_files);
}




static int acq200_tblockfs_get_super(
	struct file_system_type *fst,
	int flags, const char *devname, void *data,
	struct vfsmount* mnt)
{
	return get_sb_single(
		fst, flags, data, acq200_tblockfs_fill_super, mnt);
}

static struct file_system_type custom_fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "tbfs",
	.get_sb		= acq200_tblockfs_get_super,
	.kill_sb	= kill_litter_super,
};

static void mk_tblockfs(void)
/* store results as nodes in a custom file system */
{
	register_filesystem(&custom_fs_type);
}

static void rm_tblockfs(void)
{
	unregister_filesystem(&custom_fs_type);
	kfree(my_files);
	kfree(my_names);
}

static void acq200_tblockfs_dev_release(struct device * dev)
{
	info("");
}


static struct device_driver acq200_tblockfs_driver;


static int acq200_tblockfs_probe(struct device *dev)
{
	info("");
	mk_tblockfs();
	mk_tblock_sysfs(dev);
	dbg(1, "99");
	return 0;
}

static int acq200_tblockfs_remove(struct device *dev)
{
	rm_tblockfs();

	return 0;
}


static struct device_driver acq200_tblockfs_driver = {
	.name     = "acq200_tblockfs",
	.probe    = acq200_tblockfs_probe,
	.remove   = acq200_tblockfs_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq200_tblockfs_device = {
	.name = "acq200_tblockfs",
	.id   = 0,
	.dev = {
		.release    = acq200_tblockfs_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq200_tblockfs_init( void )
{
	acq200_debug = acq200_tblockfs_debug;

	driver_register(&acq200_tblockfs_driver);
	return platform_device_register(&acq200_tblockfs_device);
}


static void __exit
acq200_tblockfs_exit_module(void)
{
	info("");
	platform_device_unregister(&acq200_tblockfs_device);
	driver_unregister(&acq200_tblockfs_driver);
}

module_init(acq200_tblockfs_init);
module_exit(acq200_tblockfs_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Filesystem for Raw TBLOCK");


