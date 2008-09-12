/* ------------------------------------------------------------------------- */
/* debugfs2.c                                                                */
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

#include <asm/arch-iop32x/iop321.h>
#include "acqX00-port.h"
#include "acq200.h"
#include "acq200_debug.h"

static struct dentry *top;
static struct dentry *create_hook;

static struct DebugFs2NodeInfo debug_iop_base_info;

static ssize_t debug_iop_write(
	struct file *file,
	const char __user *user_buf, 
	size_t count, loff_t *ppos)
{
	char myline[80];
	ssize_t rc = debugfs2_write_line(file, user_buf, 
					 min(count, sizeof(myline)-1),
					 ppos, myline);
	static struct DebugFs2NodeInfo *defaultNodeInfo = &debug_iop_base_info;

	myline[79] = '\0';

	if (rc > 0 && myline[0] != '#' && strlen(myline) > 5){
		struct dentry* newfile;
		struct DebugFs2NodeInfo* nodeInfo = 
			kmalloc(DBGFS2_NI_SZ, GFP_KERNEL);
		memcpy(nodeInfo, defaultNodeInfo, DBGFS2_NI_SZ);

		newfile = debugfs2_create_file_def(
			top, nodeInfo, myline, (int)*ppos);			
	}
	return rc;
}

const struct file_operations debug_iop_fops = {
	.write = debug_iop_write,
};

static int __init debug_iop_init(void)
{
	info("");	
	top = debugfs_create_dir("iop321", NULL);
	create_hook = debugfs_create_file(".create", S_IWUGO,
					   top, 0, &debug_iop_fops);

	debug_iop_base_info.pwrite =
	debug_iop_base_info.pread = 
		debug_iop_base_info.pcache = (void*)IOP321_VIRT_MEM_BASE;
	debug_iop_base_info.read_fmt = "0x%08x\n";
	return 0;
}

static void __exit
debug_iop_exit_module(void)
{
	info("");
	debugfs_remove(top);
}

module_init(debug_iop_init);
module_exit(debug_iop_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Debug FS for IOP");


