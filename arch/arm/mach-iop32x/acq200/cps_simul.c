/* ------------------------------------------------------------------------- */
/* cps_simul.c                                                               */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2007 Peter Milne, D-TACQ Solutions Ltd
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


#include <linux/kernel.h>
#include <linux/time.h>




#include <asm/hardware.h>
#include <asm/io.h>

#include <asm/types.h>

#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include "debugfs2.h"

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include "acqX00-port.h"
#include "acq200.h"
#include "acq200_debug.h"

#include <linux/mm.h>           /* everything */
#include <linux/errno.h>        /* error codes */
#include <asm/pgtable.h>

#define CPS_MEMORY_SIZE	0x1000

static struct dentry *top;
static struct dentry *create_hook;

static u32* cps_memory;
static struct DebugFs2NodeInfo cps_simul_base_info;

static ssize_t cps_simul_write(struct file *file, 
				   const char __user *user_buf, 
					size_t count, loff_t *ppos)
{
	char myline[80];
	ssize_t rc = debugfs2_write_line(file, user_buf, 
					 min(count, sizeof(myline)-1),
					 ppos, myline);

	myline[79] = '\0';

	if (rc > 0 && myline[0] != '#' && strlen(myline) > 10){
		struct dentry* newfile;
		struct DebugFs2NodeInfo* nodeInfo = 
			kmalloc(sizeof(struct DebugFs2NodeInfo), GFP_KERNEL);
		memcpy(nodeInfo, &cps_simul_base_info, 
		       sizeof(struct DebugFs2NodeInfo));

		newfile = debugfs2_create_file_def(top, nodeInfo, myline);

		/* @@todo: newfile -> list */			
	}
	
	if (rc > 0){
		ppos += rc;
	}
	return rc;
}

void cps_simul_vma_open(struct vm_area_struct *vma)
{

}

void cps_simul_vma_close(struct vm_area_struct *vma)
{

}

struct page *cps_simul_vma_nopage(struct vm_area_struct *vma,
                                unsigned long address, int *type)
{
        unsigned long offset;
        struct page *page = NOPAGE_SIGBUS;
        void *pageptr = cps_memory;

        offset = (address - vma->vm_start) + (vma->vm_pgoff << PAGE_SHIFT);
        if (offset >= CPS_MEMORY_SIZE) goto out; /* out of range */

        /*
         * Now retrieve the scullp device from the list,then the page.
         * If the device has holes, the process receives a SIGBUS when
         * accessing the hole.
         */
        offset >>= PAGE_SHIFT; /* offset is a number of pages */
        page = virt_to_page(pageptr);

        /* got it, now increment the count */
        get_page(page);
        if (type)
                *type = VM_FAULT_MINOR;
  out:
        return page;
}


struct vm_operations_struct cps_simul_vm_ops = {
        .open =     cps_simul_vma_open,
        .close =    cps_simul_vma_close,
        .nopage =   cps_simul_vma_nopage,
};


static int cps_simul_mmap(struct file *filp, struct vm_area_struct *vma)
{
        /* don't do anything here: "nopage" will set up page table entries */
        vma->vm_ops = &cps_simul_vm_ops;
        vma->vm_flags |= VM_RESERVED;
        cps_simul_vma_open(vma);
        return 0;
}


static int cps_simul_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;

	return 0;
}

const struct file_operations cps_simul_fops = {
	.write =        cps_simul_write,
	.open =		cps_simul_open,	
	.mmap =		cps_simul_mmap
};


static int __init cps_simul_init(void)
{


	info("");	
	cps_memory = (u32*)__get_free_page(GFP_KERNEL);
	memset(cps_memory, 0, PAGE_SIZE);
	cps_simul_base_info.pwrite = cps_memory;
	cps_simul_base_info.pread  = cps_memory;
	cps_simul_base_info.pcache = cps_memory;

	top = debugfs_create_dir("CPS", NULL);
	create_hook = debugfs_create_file(".create", S_IWUGO,
					  top, 0, &cps_simul_fops);
	return 0;
}

static void __exit
cps_simul_exit_module(void)
{
	info("");	
	debugfs_remove(create_hook);
	/* big leak here ... need to rm all nodes, inc Info's */
	debugfs_remove(top);
	free_page((unsigned)cps_memory);
}

module_init(cps_simul_init);
module_exit(cps_simul_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Debug FS for CPS_SIMUL");


