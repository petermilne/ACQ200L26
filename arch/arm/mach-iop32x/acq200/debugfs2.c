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

#define acq200_debug debugfs2_debug

#include "acqX00-port.h"
#include "acq200.h"
#include "acq200_debug.h"

int debugfs2_debug;
module_param(debugfs2_debug, int, 0600);


static struct dentry *top;
static struct dentry *example;

static u8 my_u8;

static int __init debugfs2_init(void)
{
	info("");	
	top = debugfs_create_dir("pgm", NULL);
	example = debugfs_create_u8( "my_u8", S_IWUGO|S_IRUGO, top, &my_u8);
	return 0;
}

static void __exit
debugfs2_exit_module(void)
{
	info("");
	debugfs_remove(example);
	debugfs_remove(top);
}

module_init(debugfs2_init);
module_exit(debugfs2_exit_module);

static u32 to_mask(u32 mask, u32 value)
{
	int shl;

	if (mask == 0){
		return 0;
	}

	for (shl = 0; ((1<<shl)&mask) == 0; ++shl){
		;
	}
	return (value << shl) & mask;
}

static u32 from_mask(u32 mask, u32 value)
{
	int shr;

	if (mask == 0){
		return 0;
	}

	for (shr = 0; ((1<<shr)&mask) == 0; ++shr){
		;
	}
	return (value & mask) >> shr;
}

static ssize_t debugfs2_read(struct file *file, char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	char buf[32];
	struct DebugFs2NodeInfo *nodeInfo = file->private_data;

	u32 value = *nodeInfo->pread;
	
	value = from_mask(nodeInfo->mask, value);
	sprintf(buf, "%u", value);

	dbg(2, "call simple_read_from_buffer  \"%s\"", buf);

	return simple_read_from_buffer(
		user_buf, count, ppos, buf, strlen(buf));
}

static ssize_t debugfs2_write(struct file *file, 
			      const char __user *user_buf, 
			      size_t count, loff_t *ppos)
{
	char buf[32];
	struct DebugFs2NodeInfo *nodeInfo = file->private_data;
	int buf_size = min(count, (sizeof(buf)-1));
	u32 value;
	u32 tmp;

	dbg(2, "copy_from_user %d", buf_size);

	if (copy_from_user(buf, user_buf, buf_size)){
		return -EFAULT;
	}
	buf[buf_size] = '\0';

	switch (strlen(buf) == 1? buf[0]: 0) {
	case 'y':
	case 'Y':
	case '1':
		value = 1;
		break;
	case 'n':
	case 'N':
	case '0':
		value = 0;
		break;
	default:
		value = simple_strtoul(buf, 0, 0);
	}

	tmp = *nodeInfo->pwrite;
	tmp &= ~nodeInfo->mask;
	tmp |= to_mask(nodeInfo->mask, value);
	*nodeInfo->pwrite = tmp;
	return count;
}

static int debugfs2_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;

	return 0;
}

const struct file_operations debugfs2_fops = {
	.read =		debugfs2_read,
	.write =	debugfs2_write,
	.open =		debugfs2_open,	

};


struct dentry* debugfs2_create_file_def(
	struct dentry* parent,
	struct DebugFs2NodeInfo *nodeInfo,
	const char *def,
	int nline)
{
	char name[21];
	char s_mask[21];
	char mode[6];
	unsigned offset;
	int fields;

	if ((fields = sscanf(def, "%20s 0x%x %20s %5s", 
			     name, &offset, s_mask, mode)) == 4){
		if (strncmp(s_mask, "0x", 2) == 0){
			nodeInfo->mask = simple_strtoul(s_mask, 0, 0);
		}else{
			nodeInfo->mask = 1<<simple_strtoul(s_mask, 0, 0);
		}
		/* mode ignored for now */
		
		offset >>= 2;
		nodeInfo->pwrite += offset;
		nodeInfo->pread += offset;
		nodeInfo->pcache += offset;

		dbg(1, "accept line %d \"%s\"", nline, def);
		
		return debugfs_create_file(
			name, 
			((strstr(mode, "r") == 0? 0: S_IRUGO) |
			 (strstr(mode, "w") == 0? 0: S_IWUGO)   ),
			parent,
			nodeInfo,
			&debugfs2_fops);
	}else{
		info("reject line %d \"%s\" at field %d", nline, def, fields);
		return 0;
	}
}

ssize_t debugfs2_write_line(struct file *file, 
		       const char __user *user_buf, size_t count, loff_t *ppos,
		       char* myline)
{
	char *cp;


	if (copy_from_user(myline, user_buf, count)){
		return -EFAULT;
	}
	myline[count-1] = '\0';

	dbg(2, "count %d myline \"%s\"", count, myline);

	for (cp = myline; cp - myline < count; ++cp){
		if (*cp == '\n'){
			*cp++ = '\0';
			*ppos += 1;
			break;
		}
	}

	dbg(2, "returning %d myline \"%s\"", cp-myline, myline);

	return cp - myline;
}

EXPORT_SYMBOL_GPL(debugfs2_create_file_def);
EXPORT_SYMBOL_GPL(debugfs2_write_line);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ2xx RTM DIO32");


