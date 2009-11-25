/* ------------------------------------------------------------------------- */
/* lockin.c driver for acq196 lockin  FPGA                                   */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2008 Peter Milne, D-TACQ Solutions Ltd
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

#define ACQ196


#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>     	/* This is where libfs stuff is declared */
#include <linux/gfp.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>	/* copy_to_user */

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
#define acq200_debug plockin_debug   

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

#include "acq196.h"


#define LFS_MAGIC	0xa19610c1

int plockin_debug;
module_param(plockin_debug, int, 0664);

int plockin_word_size = sizeof(u32);
module_param(plockin_word_size, int , 0664);

int test_dummy;
module_param(test_dummy, int, 0644);

#define VERID "$Revision: 1.3 $ build B1012 "

char acq196_lockin_driver_name[] = "acq196-lockin";
char acq196_lockin_driver_string[] = "D-TACQ Low Latency Control Device";
char acq196_lockin_driver_version[] = VERID __DATE__;
char acq196_lockin_copyright[] = "Copyright (c) 2004 D-TACQ Solutions Ltd";

#define REFLEN	512

#define MAXREF	16
#define MAXBU32	(MAXREF*sizeof(u16)/sizeof(u32))

#define TD_SZ  (sizeof(struct tree_descr))
#define MY_FILES_SZ ((2+MAXREF+1)*TD_SZ)

#define INO2IX(ino) ((ino)-2)

struct FunctionBuf {
	u16 *p_start;
	u16 *p_cur;
};

#define ADDR_R110_R100 0x00300000
#define ADDR_R120_R111 0x00300800
#define ADDR_R200_R121 0x00301000
#define ADDR_R211_R210 0x00301800
#define ADDR_R221_R220 0x00302000
#define ADDR_R310_R300 0x00302800
#define ADDR_R320_R311 0x00303000
#define ADDR_R000_R321 0x00303800


/*
 * same regs as original ACQ196 MAC, different bits
 * LocK Register LKR
 */
#define LKR_MACCON	ACQ196_MACCON
#define LKR_SUBCON	ACQ196_MACSUB

#define LKR_MACCON_ADC3_REF_SEL_SHL	20
#define LKR_MACCON_ADC2_REF_SEL_SHL	18
#define LKR_MACCON_ADC1_REF_SEL_SHL	16

#define LKR_MACCON_OVFL	0x7fff
#define LKR_MACCON_LEN	0x07ff

#define LKR_SUBCON_SUBTRACT	0x0000ffff
#define LKR_SUBCON_ENABLE	0xffff0000



struct Globs {
	struct FunctionBuf fb[MAXREF];
	u32 * buffers[MAXBU32];
	u32 fpga_buffers[MAXBU32];
} LG = {
	.fpga_buffers[0] = ADDR_R110_R100,
	.fpga_buffers[1] = ADDR_R120_R111,
	.fpga_buffers[2] = ADDR_R200_R121,
	.fpga_buffers[3] = ADDR_R211_R210,
	.fpga_buffers[4] = ADDR_R221_R220,
	.fpga_buffers[5] = ADDR_R310_R300,
	.fpga_buffers[6] = ADDR_R320_R311,
	.fpga_buffers[7] = ADDR_R000_R321
};

static struct tree_descr *my_files;

#define FB(filp) ((struct FunctionBuf *)(filp)->private_data)
#define SET_FB(filp, fb) ((filp)->private_data = (fb))

#define IFB2IBU32(ifb)	((ifb)/2)

static int new_buffers(void)
{
	int ib32;	/* index the 32 bit buffers */
	int ifb = 0;	/* index the channel buffers */

	for (ib32 = 0; ib32 < MAXBU32; ++ib32){
		u32 *bu32 = kzalloc(REFLEN*sizeof(32), GFP_KERNEL);

		LG.buffers[ib32] = bu32;
		LG.fb[ifb].p_start = (u16*)bu32;
		++ifb;
		LG.fb[ifb].p_start = (u16*)bu32 + 1;
		++ifb;
	}
	return 0;
}

static void delete_buffers(void)
{
	int ib32;	/* index the 32 bit buffers */

	for (ib32 = 0; ib32 < MAXBU32; ++ib32){
		kfree(LG.buffers[ib32]);
	}
	kfree(my_files);
}
static int data_open(struct inode *inode, struct file *filp)
{	
	int iref = INO2IX(inode->i_ino);

	if (!IN_RANGE(iref, 0, MAXREF)){
		return -ENODEV;
	}
	dbg(1, "inode:%d iref:%d", inode->i_ino, iref);

	SET_FB(filp, &LG.fb[iref]);
	if ((filp->f_mode & FMODE_WRITE) != 0){
		/* truncate on write */
		FB(filp)->p_cur = FB(filp)->p_start; 

		filp->f_dentry->d_inode->i_size = 0;
	}	
	return 0;
}

static ssize_t data_write(
	struct file *filp, const char *buf, size_t count, loff_t *offset)
{
	u16 tmp;
	u16 *ubuf = (u16*)buf;
	u16 ucount = count/sizeof(u16);
	int nwords = 0;
	

	if (*offset >= MAXREF){
		return -EINVAL;
	}
		
	/* stride thru array[u16] double spaced */
	while (ucount-- > 0 && *offset + nwords < MAXREF){
		get_user(tmp, ubuf);
		*FB(filp)->p_cur = tmp;
		++ubuf;
		++nwords;
		FB(filp)->p_cur += 2;
	}
	*offset += nwords;
	filp->f_dentry->d_inode->i_size += count;
	
	dbg(1, "count %d return %d", count, nwords*sizeof(u16));

	return nwords*sizeof(u16) + count&1;
}

static ssize_t data_read(
	struct file *filp, char *buf, size_t count, loff_t *offset)
{
	u16 tmp;
	u16 *ubuf = (u16*)buf;
	u16 ucount = count/sizeof(u16);
	int nwords = 0;

	if (*offset >= FB(filp)->p_cur - FB(filp)->p_start){
		return 0;
	}
		

	while (ucount-- > 0 && *offset + nwords < MAXREF){
		tmp = FB(filp)->p_start[2*nwords];
		put_user(tmp, ubuf);
		++ubuf;
		++nwords;
	}
		
	*offset += nwords;
	
	return nwords*sizeof(u16);
}

static int data_mmap(
	struct file *filp, struct vm_area_struct *vma)
{
	return io_remap_pfn_range( 
		vma, vma->vm_start, 
		__phys_to_pfn(virt_to_phys(FB(filp)->p_start)), 
		vma->vm_end - vma->vm_start, 
		vma->vm_page_prot 
	);
}



static void flush(int ibuf)
{
		
	void *to = (test_dummy? BB_PTR(0): DG->fpga.fifo.va)+
			LG.fpga_buffers[ibuf];
	void *from = LG.buffers[ibuf];
	int nbytes = MAXREF*sizeof(u32);

	if (test_dummy <= 1){
		dbg(1, "[%d] memcpy(%p, %p, %d)", 
			ibuf, to, from, nbytes);

		memcpy(to, from, nbytes);
	}else{
		dbg(1, "[%d] memcpy(%p, %p, %d) STUBBED", 
			ibuf, to, from, nbytes);
	}
}

static int data_release(
        struct inode *inode, struct file *file)
{
	if ((file->f_mode & FMODE_WRITE) != 0){
		int iref = INO2IX(inode->i_ino);
		/* flush on close. Wasteful - may have to write twice
		 *  who cares - simple is good
		 */
		dbg(1, "inode:%d iref:%d", inode->i_ino, iref);

		flush(IFB2IBU32(iref));
	}	
	return 0;
}

static int lockin_fs_fill_super (struct super_block *sb, void *data, int silent)
{
	static struct file_operations data_ops = {
		.open = data_open,
		.read = data_read,
		.write = data_write,
		.release = data_release,
		.mmap = data_mmap
	};
	static char* ref_names[MAXREF] = {
		"R100", "R110", "R111", "R120", "R121",
		"R200", "R210", "R211", "R220", "R221",
		"R300", "R310", "R311", "R320", "R321",
		"XNC"
	};
	static struct tree_descr front = {
		NULL, NULL, 0
	};
	static struct tree_descr backstop = {
		"", NULL, 0
	};
	
	int iref;
	int myfino = 0;
	my_files = kzalloc(MY_FILES_SZ, GFP_KERNEL);

	memcpy(&my_files[myfino++], &front, TD_SZ);
	/* linux 2.6.21 ... ino starts at 2 */
	memcpy(&my_files[myfino++], &front, TD_SZ); 

	for (iref = 0; iref < MAXREF; ++iref, ++myfino){
		my_files[myfino].name = ref_names[iref];
		my_files[myfino].ops = &data_ops;
		my_files[myfino].mode = S_IWUSR|S_IRUGO;
	}	

	memcpy(&my_files[myfino++], &backstop, TD_SZ);
	return simple_fill_super(sb, LFS_MAGIC, my_files);
}


static int lockin_fs_get_super(
		struct file_system_type *fst,
		int flags, const char *devname, void *data,
		struct vfsmount* mnt)
{
	return get_sb_single(
		fst, flags, data, lockin_fs_fill_super, mnt);
}

static struct file_system_type lockin_fs_type = {
	.owner 		= THIS_MODULE,
	.name		= "acq196_lockinfs",
	.get_sb		= lockin_fs_get_super,
	.kill_sb	= kill_litter_super,
};

int lockin_fs_create(struct device *dev)
{
	info("about to register fs");
	return register_filesystem(&lockin_fs_type);
}
/**
 * todo LOCKIN transform - 32 bit data ignore shorts and shift ints
 * piece of cake
 */


void do_transform32(u32* to, u32* from, int nitems, int stride)
{
	int nsamples = nitems/stride;
	int isample, ichannel;

	for (isample = 0; isample != nsamples; ++isample){
		for (ichannel = 0; ichannel != stride; ++ichannel){
			to[ichannel*nsamples + isample] =
				from[isample*stride + ichannel];
		}
	}
}

void transform32(short *to, short *from, int nwords, int stride)
{
	do_transform32((u32*)to, (u32*)from, nwords/2, stride);
}





static ssize_t show_version(
	struct device *dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       acq196_lockin_driver_name,
		       acq196_lockin_driver_string,
		       acq196_lockin_driver_version,
		       acq196_lockin_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);

static unsigned lockin_get_maccon_ref(int bank)
{
	unsigned shl;

	switch(bank){
	case 1: 
		shl = LKR_MACCON_ADC1_REF_SEL_SHL; break;
	case 2:
		shl = LKR_MACCON_ADC2_REF_SEL_SHL; break;
	case 3:
		shl = LKR_MACCON_ADC3_REF_SEL_SHL; break;
	default:
		return 0;		/* ERROR */
	}
	return (*LKR_MACCON & (ACQ196_MACCON_REF_SEL_MASK << shl)) >> shl;
}

static void lockin_set_maccon_ref(int bank, unsigned ref)
{
	unsigned shl;

	switch(bank){
	case 1: 
		shl = LKR_MACCON_ADC1_REF_SEL_SHL; break;
	case 2:
		shl = LKR_MACCON_ADC2_REF_SEL_SHL; break;
	case 3:
		shl = LKR_MACCON_ADC3_REF_SEL_SHL; break;
	default:
		return;		/* ERROR */
	}
	*LKR_MACCON &= ~(ACQ196_MACCON_REF_SEL_MASK << shl);
	*LKR_MACCON |= ref << shl;
}


static ssize_t show_K(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf, int bank)
{
	unsigned ref = lockin_get_maccon_ref(bank);

	return sprintf(buf, "%s\n", 
		       ref==ACQ196_MACCON_REF_SEL_ONE? "ONE":
		       ref==ACQ196_MACCON_REF_SEL_DAC? "REF":
		       ref==ACQ196_MACCON_REF_SEL_ADC? "ADC": "ERROR");
}

static ssize_t store_K(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count, int bank)
{
	unsigned ref;

	if (strncmp(buf, "ADC", 3) == 0){
		ref = ACQ196_MACCON_REF_SEL_ADC;
	}else if (strncmp(buf, "DAC", 3) == 0 || strncmp(buf, "REF", 3) == 0){
		ref = ACQ196_MACCON_REF_SEL_DAC;
	}else if (strncmp(buf, "ONE", 3) == 0){
		ref = ACQ196_MACCON_REF_SEL_ONE;
	}else{
		return strlen(buf);
	}
	
	lockin_set_maccon_ref(bank, ref);
	return strlen(buf);
}


#define K_KNOB(BANK)					\
static ssize_t show_K ## BANK(				\
	struct device * dev,				\
	struct device_attribute *attr,			\
	char * buf)					\
{							\
	return show_K(dev, attr, buf, BANK);		\
}							\
							\
static ssize_t store_K ## BANK (			\
	struct device * dev,				\
	struct device_attribute *attr,			\
	const char * buf, size_t count)			\
{							\
	return store_K(dev, attr, buf, count, BANK);	\
}							\
							\
static DEVICE_ATTR(K ## BANK, S_IRUGO|S_IWUGO,		\
		   show_K ## BANK, store_K ## BANK);

K_KNOB(1);
K_KNOB(2);
K_KNOB(3);

static ssize_t show_status(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int status = (*LKR_MACCON & LKR_MACCON_OVFL);

	
        return sprintf(buf,"0x%04x\n", status);
}

static ssize_t clear_status(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	*LKR_MACCON |= LKR_MACCON_OVFL;
	return strlen(buf);
}

static DEVICE_ATTR(status, S_IRUGO|S_IWUGO, show_status, clear_status);

static ssize_t show_subcon(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int subcon = *LKR_SUBCON & LKR_SUBCON_SUBTRACT;

        return sprintf(buf,"%d\n", subcon);
}

static ssize_t store_subcon(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int subcon = 0;

	if (sscanf(buf, "0x%x", &subcon) == 1 || 
	    sscanf(buf, "%d", &subcon) == 1       ){
		*LKR_SUBCON &= ~LKR_SUBCON_SUBTRACT;
		*LKR_SUBCON |= subcon;
	}
	
	return strlen(buf);
}

static DEVICE_ATTR(subcon, S_IRUGO|S_IWUGO, show_subcon, store_subcon);

#define COUNTS_FROM_ZERO	1

/* @@TODO */
static ssize_t show_depth(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int depth = *LKR_MACCON & ACQ196_MACCON_LENGTH;

        return sprintf(buf,"%d\n", depth + COUNTS_FROM_ZERO);
}

static ssize_t store_depth(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int depth = 0;

	if (sscanf(buf, "%d", &depth) == 1 &&
	    depth >= 1 && depth <= 0xfff){
		*ACQ196_MACCON &= ~ACQ196_MACCON_LENGTH;
		*ACQ196_MACCON |= depth - COUNTS_FROM_ZERO;
	}
	
	return strlen(buf);
}

static DEVICE_ATTR(depth, S_IRUGO|S_IWUGO, show_depth, store_depth);

static ssize_t show_enable(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%08x\n", 
		       (*LKR_SUBCON & LKR_SUBCON_ENABLE) >> 16);
}

static ssize_t store_enable(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int enables = 0;

	if (sscanf(buf, "0x%x", &enables) || sscanf(buf, "%x", &enables)){
			*LKR_SUBCON &= ~LKR_SUBCON_ENABLE;
			*LKR_SUBCON |= enables << 16;
	}
	return strlen(buf);
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUGO, show_enable, store_enable);




static int mk_lockin_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_version);

	DEVICE_CREATE_FILE(dev, &dev_attr_K1);
	DEVICE_CREATE_FILE(dev, &dev_attr_K2);
	DEVICE_CREATE_FILE(dev, &dev_attr_K3);
	DEVICE_CREATE_FILE(dev, &dev_attr_status);
	DEVICE_CREATE_FILE(dev, &dev_attr_depth);
	DEVICE_CREATE_FILE(dev, &dev_attr_subcon);	
	DEVICE_CREATE_FILE(dev, &dev_attr_enable);
	return 0;
}


static void acq196_lockin_dev_release(struct device * dev)
{
	info("");
}


static 	struct Transformer transformer = {
	.name = "lock-in",
	.transform = transform32
};

static void register_custom_transformer(void)
{
	int it;
	it = acq200_registerTransformer(&transformer);
	if (it >= 0){
		acq200_setTransformer(it);
	}else{
		err("transformer %s NOT registered", transformer.name);
	}
}
static struct device_driver acq196_lockin_driver;

static int acq196_lockin_probe(struct device *dev)
{
	info("");
	register_custom_transformer();
	mk_lockin_sysfs(dev);
	new_buffers();
	return lockin_fs_create(dev);
}

static int acq196_lockin_remove(struct device *dev)
{
	delete_buffers();
	unregister_filesystem(&lockin_fs_type);
	return 0;
}


static struct device_driver acq196_lockin_driver = {
	.name     = "acq196_lockin",
	.probe    = acq196_lockin_probe,
	.remove   = acq196_lockin_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq196_lockin_device = {
	.name = "acq196_lockin",
	.id   = 0,
	.dev = {
		.release    = acq196_lockin_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init acq196_lockin_init( void )
{
	int rc;
	acq200_debug = plockin_debug;

	CAPDEF_set_word_size(plockin_word_size);
	rc = driver_register(&acq196_lockin_driver);
	if (rc){
		return rc;
	}

	return platform_device_register(&acq196_lockin_device);
}


static void __exit
acq196_lockin_exit_module(void)
{
	info("");
	platform_device_unregister(&acq196_lockin_device);
	driver_unregister(&acq196_lockin_driver);
}

module_init(acq196_lockin_init);
module_exit(acq196_lockin_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ196 LOCKIN");


