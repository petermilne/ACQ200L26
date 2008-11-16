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
#include <linux/poll.h>

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

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

int acq200_tblockfs_debug;
module_param(acq200_tblockfs_debug, int, 0664);


#define VERID "$Revision: 1.3 $ build B1000 "


char acq200_tblockfs_driver_name[] = "acq200-rawtbraw";
char acq200_tblockfs_driver_string[] = "D-TACQ Intelligent ACQ device";
char acq200_tblockfs_driver_version[] = VERID __DATE__;
char acq200_tblockfs_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";


#define INO2TBLOCK(ino) ((ino)-2)



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
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	return 0;
}


static ssize_t tblock_data_extractPages(
	struct TBLOCK *this,
	int maxbytes,
	int offset,
	read_descriptor_t * desc,
	read_actor_t actor	
	)
{
	char* tblock_base = (va_buf(DG) + this->offset);
	char* cursor = (char*)(tblock_base + offset);

	unsigned long cplen = 0;

	while (cplen < maxbytes){
		struct page *page = virt_to_page(cursor);
		unsigned poff = (unsigned)(cursor)&(PAGE_SIZE-1);
		unsigned long len = min(PAGE_SIZE, maxbytes-cplen);

		actor(desc, page, poff, len);
		
		cplen += len;
		cursor += len;
	}

	return cplen;
}

static ssize_t tblock_data_mapping_read ( 
	struct file * file, loff_t *offset,
	read_descriptor_t * desc,
	read_actor_t actor
)
/** read a linear buffer. len, offset in bytes pass the data a page at a time
 *  to actor.
 */
/*
 * BEWARE: this is a nightmare of mixed units
 *
 * contract with caller: read bytes, return bytes
 * contract with extract: units are words
 *
 */
{
	struct TBLOCK* tb = (struct TBLOCK*)file->private_data;
	int maxbytes = min(TBLOCK_LEN, (int)desc->count);

	int rc = tblock_data_extractPages(
		tb, 
		maxbytes,
		*offset,
		desc,
		actor);

	if (rc > 0){
		*offset += rc;
	}
	return rc;
}




static int tblock_data_open(struct inode *inode, struct file *filp)
{
	int blocknum = INO2TBLOCK(inode->i_ino);

	if (!IN_RANGE(blocknum, 0, MAX_TBLOCK-1)){
		return -ENODEV;
	}else{
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

static ssize_t tblock_data_write(struct file *filp, const char *buf,
		size_t count, loff_t *offset)
{
	struct TBLOCK* tblock = (struct TBLOCK*)filp->private_data;
	void *tblock_va = tblock->offset + va_buf(DG);
	size_t headroom = tblock->length - *offset;

	count = min(count, headroom);

	if (count == 0){
		return -EFBIG;
	}
	if (copy_from_user(tblock_va + *offset, buf, count)){
		return -EFAULT;
	}

	*offset += count;
	return count;
}

/*
 * StateListener
 */
/* store per path StateListener object */
#define SL(filp) ((struct StateListener*)(filp)->private_data)
/* stash previous state value in second StateListener element */
#define _SL_STATE(filp) (u32*)(&SL(filp)[1])

#define SL_PUT_STATE(filp, value) (*_SL_STATE(filp) = (value))
#define SL_GET_STATE(filp)        (*_SL_STATE(filp))

static int state_open(struct inode *inode, struct file *filp)
{
	filp->private_data = kzalloc(sizeof(struct StateListener)*2,GFP_KERNEL);
	sl_init(SL(filp), 16);
	SL_PUT_STATE(filp, 12345);
	spin_lock(&DMC_WO->stateListeners.lock);
	list_add_tail(&SL(filp)->list, &DMC_WO->stateListeners.list);
	spin_unlock(&DMC_WO->stateListeners.lock);
	return 0;
}

static char *stateString(u32 state)
{
	switch(state){
	case ST_STOP:
		return "ST_STOP";
	case ST_ARM:
		return "ST_ARM"; 
	case ST_RUN:
		return "ST_RUN"; 
	case ST_TRIGGER:
		return "ST_TRIGGER";
	case ST_POSTPROCESS:
		return "ST_POSTPROCESS";
	case ST_CAPDONE:
		return "ST_CAPDONE";
	default:
		return "ST_UNKNOWN";
	}
}
static int state_read(struct file *filp, char *buf,
		size_t count, loff_t *offset)
/** report changes of state. If the state is the same, don't report */
{
	char sbuf[32];
	u32 scode;
	u32 state = SL_GET_STATE(filp);	
	u32 tcode;
	u32 sec;
	int len;

	do {
		wait_event_interruptible(SL(filp)->waitq, 
				!u32rb_is_empty(&SL(filp)->rb));

		if (u32rb_is_empty(&SL(filp)->rb)){
			return -EINTR;
		}
		u32rb_get(&SL(filp)->rb, &scode);
	} while(SL_TO_STATE(scode) == state);
	
	state = SL_TO_STATE(scode);
	tcode = SL_TO_TCODE(scode);
	sec = tcode / 100;
	tcode = tcode - sec*100;
	
	len = snprintf(sbuf, sizeof(sbuf), "%05d.%02d %d %s\n", 
		       sec, tcode, state, stateString(state));

	if (count < len){
		return -ENOMEM;
	}else{
		COPY_TO_USER(buf, sbuf, len);
		if (offset){
			*offset += len;
		}
		SL_PUT_STATE(filp, state);
		return len;
	}
}
static int state_release(struct inode *inode, struct file *filp)
{
	spin_lock(&DMC_WO->stateListeners.lock);
	list_del(&SL(filp)->list);
	spin_unlock(&DMC_WO->stateListeners.lock);	

	u32rb_destroy(&SL(filp)->rb);
	kfree(filp->private_data);
	return 0;
}

static unsigned int state_poll(
	struct file *file, struct poll_table_struct *poll_table)
{
	if (u32rb_is_empty(&SL(file)->rb)){
		poll_wait(file, &SL(file)->waitq, poll_table);
	}
	if (!u32rb_is_empty(&SL(file)->rb)){
		return POLLIN | POLLRDNORM;
	}else{
		return 0;
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

static ssize_t tblock_data_sendfile(struct file *in_file, loff_t *ppos,
			 size_t count, read_actor_t actor, void __user *target)
{
	read_descriptor_t desc = {
		.written = 0,
		.count = count,
		.arg.data = target,
		.error = 0
	};
	ssize_t rc;

	if (!count)
		return 0;

	dbg(1, "STEP:A1 count %d", count);

	while (desc.written < count){
		rc = tblock_data_mapping_read(
			in_file, ppos, &desc, actor);

		if (rc == 0){
			break;
		}else if (rc < 0){
			dbg(1, "ERROR returning %d", rc);
			return rc;
		}

		dbg(1, "desc.written %d count %d rc %d desc.count %d", 
		    desc.written, count, rc, desc.count);
	}
		
	dbg(1, "returning %d", desc.written);

	if (desc.written > 0){
		DG->stats.sendfile_bytes += desc.written;
	}
	return desc.written;
}



#define TBFS_MAGIC 0xa2111111

#define TD_SZ  (sizeof(struct tree_descr))
#define MY_FILES_SZ(numchan) ((1+(numchan)+2+1+1)*TD_SZ)
static struct tree_descr *my_files;

typedef char Name[4];

static Name* my_names;

static int acq200_tblockfs_fill_super (
	struct super_block *sb, void *data, int silent)
{
	static struct file_operations access_ops = {
		.open = tblock_data_open,
		.read = tblock_data_read,
		.write = tblock_data_write,
		.mmap = tblock_data_mmap,
		.sendfile = tblock_data_sendfile
	};

	static struct file_operations state_ops = {
		.open = state_open,
		.read = state_read,
		.release = state_release,
		.poll = state_poll
	};
	static struct tree_descr front = {
		NULL, NULL, 0
	};
	static struct tree_descr statemon = {
		.name = "acqstate",
		.ops = &state_ops,
		.mode = S_IRUGO
	};
	static struct tree_descr backstop = {
		"", NULL, 0
	};
	
	int ino = 0;
	int iblock;
	int rc;

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

	memcpy(&my_files[ino++], &front, TD_SZ);
	memcpy(&my_files[ino++], &front, TD_SZ);
	
	for (iblock = 0; iblock < MAX_TBLOCK; ++iblock, ++ino){
		sprintf(my_names[iblock], "%03d", iblock);
		my_files[ino].name = my_names[iblock];
		my_files[ino].ops  = &access_ops;
		my_files[ino].mode = S_IRUGO;
	}

	memcpy(&my_files[ino++], &statemon, TD_SZ);
	memcpy(&my_files[ino++], &backstop, TD_SZ);

	rc = simple_fill_super(sb, TBFS_MAGIC, my_files);


	if (rc == 0){
		struct inode *inode;

		list_for_each_entry(inode, &sb->s_inodes, i_sb_list){
			if (IN_RANGE(INO2TBLOCK(inode->i_ino), 0, MAX_TBLOCK)){
				inode->i_size = TBLOCK_LEN;
			}
		}		
	}
	return rc;
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
	int rc;
	acq200_debug = acq200_tblockfs_debug;

	rc = driver_register(&acq200_tblockfs_driver);
	if (rc){
		return rc;
	}
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


