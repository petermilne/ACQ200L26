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
#define acq200_debug pl_debug   

#include "acqX00-port.h"
#include "acq200_debug.h"
#include "mask_iterator.h"

#include "acq200-fifo-top.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq200-mu.h"


#include "acq32busprot.h"          /* soft link to orig file */

#include "acq196.h"


#define LFS_MAGIC	0xa19610c1

int pl_debug;
module_param(pl_debug, int, 0664);

int pl_word_size = sizeof(u32);
module_param(pl_word_size, int , 0664);

int test_dummy;
module_param(test_dummy, int, 0644);

#define VERID "B1003"

char penta_lockin_driver_name[] = "penta_lockin";
char penta_lockin_driver_string[] = "Penta Lockin device";
char penta_lockin_driver_version[] = VERID ":" __DATE__;
char penta_lockin_copyright[] = "Copyright (c) 2009 D-TACQ Solutions Ltd";

#define REFLEN_SAMPLES	512
#define REFLEN		REFLEN_SAMPLES
#define REFLEN_U32	(REFLEN_SAMPLES/2)
#define REFLEN_BYTES	(REFLEN_SAMPLES*2)

#define MAXREF	16


#define TD_SZ  (sizeof(struct tree_descr))
#define MY_FILES_SZ ((2+MAXREF+1)*TD_SZ)

#define INO2IX(ino) ((ino)-2)

struct FunctionBuf {
	u16 *p_start;
	u16 *p_cur;
};

#define ADDR_R100	0x00300000
#define ADDR_RDELTA	0x00000800	/* 1k words spacing */


/*
 * same regs as original ACQ196 MAC, different bits
 * LocK Register LKR
 */
#define LKR_MACCON	FPGA_REG(0x30)
#define LKR_SUBCON	FPGA_REG(0x34)
#define LKR_MACSTA	FPGA_REG(0x38)

#define LKR_MACCON_ADC3_REF_SEL_SHL	20
#define LKR_MACCON_ADC2_REF_SEL_SHL	18
#define LKR_MACCON_ADC1_REF_SEL_SHL	16
#define LKR_MACCON_LENGTH	0x07ff

#define LKR_SUBCON_SUBTRACT	0x0000ffff
#define LKR_SUBCON_ENABLE	0xffff0000

#define LKR_MACSTA_OVFL	0x7fff
struct Globs {
	struct FunctionBuf fb[MAXREF];
	u16 *buffers[MAXREF];
	int fpga_buffers[MAXREF];
} LG;

static struct tree_descr *my_files;


static void to_bit_array(char ba[], int nbits, unsigned pattern)
{
	int ibit;
	u32 mask = 0x1;
	int cursor;

	for (ibit = cursor = 0; ibit < nbits; mask <<= 1){
		ba[cursor++] = pattern&mask? '1': '0';
		ba[cursor++] = ++ibit < nbits? ',': '\0';
	}
}

static int from_bit_array(const char ba[], int nbits, unsigned *ppattern)
{
	int ibit = 0;
	int cursor = 0;
	u32 mask = 0x1;
	u32 pattern = 0;
	
	while(ibit < nbits){
		switch(ba[cursor]){
		case ',':
			++cursor;
			continue;
		case '1':
			pattern |= mask;	/* fall through */
		case '0':
			++cursor;
			++ibit;
			mask <<= 1;
			break;
		case '\n':
		case '\0':
		default:
			goto _done;
		}
	}
_done:
	*ppattern = pattern;
	return 0;
}

#define FB(filp) ((struct FunctionBuf *)(filp)->private_data)
#define SET_FB(filp, fb) ((filp)->private_data = (fb))

static int new_buffers(void)
{
	int iref = 0;	/* index the channel buffers */

	for (iref = 0; iref < MAXREF; ++iref){
		u16 *bp = kzalloc(REFLEN*sizeof(u16), GFP_KERNEL);
		LG.fb[iref].p_start = LG.fb[iref].p_cur = LG.buffers[iref] = bp;
		LG.fpga_buffers[iref] = ADDR_R100 + iref * ADDR_RDELTA;
	}
	return 0;
}

static void delete_buffers(void)
{
	int iref;      

	for (iref = 0; iref < MAXREF; ++iref){
		kfree(LG.buffers[iref]);
	}
	kfree(my_files);
}
static int data_open(struct inode *inode, struct file *filp)
{	
	int iref = INO2IX(inode->i_ino);

	if (!IN_RANGE(iref, 0, MAXREF)){
		return -ENODEV;
	}
	SET_FB(filp, &LG.fb[iref]);
	if ((filp->f_mode & FMODE_WRITE) != 0){
		/* truncate on write */
		FB(filp)->p_cur = FB(filp)->p_start;
		memset(FB(filp)->p_start, 0, REFLEN_BYTES);

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
	

	if (*offset >= REFLEN){
		return -EFBIG;
	}
		
	/* stride thru array[u16] double spaced */
	while (ucount-- > 0 && *offset + nwords < REFLEN){
		get_user(tmp, ubuf+nwords);
		*FB(filp)->p_cur = tmp;
		FB(filp)->p_cur += 1;
		++nwords;
	}
	*offset += nwords;
	filp->f_dentry->d_inode->i_size += nwords*sizeof(u16);
	
	dbg(1, "count %d return %d", count, nwords*sizeof(u16));

	/* round up to allow apps with odd char count to terminate */
	return nwords*sizeof(u16) + (count&1);
}

static ssize_t data_read(
	struct file *filp, char *buf, size_t count, loff_t *offset)
{
	u16 tmp;
	u16 *ubuf = (u16*)buf;
	u16 ucount = count/sizeof(u16);
	int nwords = 0;
	int lenwords = (FB(filp)->p_cur - FB(filp)->p_start);

	if (*offset >= lenwords){
		return 0;
	}
		

	while (ucount-- > 0 && *offset + nwords < lenwords){
		tmp = FB(filp)->p_start[nwords];
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
	void *bp = test_dummy? BB_PTR(0): DG->fpga.regs.va;
	u32 *to = (u32*)(bp + LG.fpga_buffers[ibuf]);
	u32 *from = (u32*)LG.buffers[ibuf];
	int nwrites = REFLEN_U32;
	int stubbed = test_dummy > 1;

	dbg(1, "[%d] %06x (%p = %p, %d*4) %s", 
	    ibuf, LG.fpga_buffers[ibuf], to, from, nwrites,
	    stubbed? "STUBBED": test_dummy? "TEST_DUMMY": "");

	if (!stubbed){
		while(nwrites--){
			*to++ = *from++;
		}		       
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
		flush(iref);
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
	.name		= "penta_lockinfs",
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
		       penta_lockin_driver_name,
		       penta_lockin_driver_string,
		       penta_lockin_driver_version,
		       penta_lockin_copyright
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
	int status = (*LKR_MACSTA & LKR_MACSTA_OVFL);
	char ba[40];
	to_bit_array(ba, MAXREF, status);
	
        return sprintf(buf,"0x%04x %s\n", status, ba);
}

static ssize_t clear_status(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	*LKR_MACSTA |= LKR_MACSTA_OVFL;
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
		u32 lkr_subcon = *LKR_SUBCON;
		lkr_subcon &= ~LKR_SUBCON_SUBTRACT;
		lkr_subcon |= subcon;
		*LKR_SUBCON = lkr_subcon;
	}
	
	return strlen(buf);
}

static DEVICE_ATTR(subcon, S_IRUGO|S_IWUGO, show_subcon, store_subcon);

#define COUNTS_FROM_ZERO	1

/* @@TODO */
static ssize_t show_mac_length(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int mac_length = *LKR_MACCON & LKR_MACCON_LENGTH;

        return sprintf(buf,"%d\n", mac_length + COUNTS_FROM_ZERO);
}

static ssize_t store_mac_length(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int mac_length = 0;

	if (sscanf(buf, "%d", &mac_length) == 1 &&
	    mac_length >= 1 && mac_length <= 0xfff){
		u32 lkr_maccon = *LKR_MACCON;
		lkr_maccon &= ~LKR_MACCON_LENGTH;
		if (mac_length < 2){
			mac_length = 2;
		}
		lkr_maccon |= mac_length - COUNTS_FROM_ZERO;
		*LKR_MACCON = lkr_maccon;
	}
	
	return strlen(buf);
}

static DEVICE_ATTR(mac_length, S_IRUGO|S_IWUGO, 
	show_mac_length, store_mac_length);


static u32 pl_get_enable(void) {
	return (*LKR_SUBCON & LKR_SUBCON_ENABLE) >> 16;
}


static unsigned pl_getNumChanEquivalent(void)
{
	u32 enable = pl_get_enable();
	u32 mask = 0x8000;
	unsigned nblocks;

	for (nblocks = 0; mask; mask >>= 1){
		if ((enable&mask) != 0){
			++nblocks;
		}
	}
	return nblocks * 32;
}


static void pl_set_enable(u32 enables)
{
	u32 lkr_subcon = *LKR_SUBCON;
	lkr_subcon &= ~LKR_SUBCON_ENABLE;
	lkr_subcon |= enables << 16;
	*LKR_SUBCON = lkr_subcon;
	CAPDEF_set_nchan(pl_getNumChanEquivalent());
}


static ssize_t show_enable(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	char ba[40];
	u32 enable = pl_get_enable();
	to_bit_array(ba, MAXREF, enable); 
	return sprintf(buf, "0x%04x %s\n", enable, ba);
}

static ssize_t store_enable(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	unsigned enables = 0;

	if (sscanf(buf, "0x%x", &enables)){
		/* NB MUST work on mask reg... good practise anyway */
		pl_set_enable(enables);
	}else if (strstr(buf, ",")){
		
		if (from_bit_array(buf, MAXREF, &enables) == 0){
			pl_set_enable(enables);
		}
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
	DEVICE_CREATE_FILE(dev, &dev_attr_mac_length);
	DEVICE_CREATE_FILE(dev, &dev_attr_subcon);	
	DEVICE_CREATE_FILE(dev, &dev_attr_enable);
	return 0;
}


static void penta_lockin_dev_release(struct device * dev)
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
static struct device_driver penta_lockin_driver;

static int penta_lockin_probe(struct device *dev)
{
	info("");
	register_custom_transformer();
	mk_lockin_sysfs(dev);
	new_buffers();
	return lockin_fs_create(dev);
}

static int penta_lockin_remove(struct device *dev)
{
	delete_buffers();
	unregister_filesystem(&lockin_fs_type);
	return 0;
}


static struct device_driver penta_lockin_driver = {
	.name     = penta_lockin_driver_name,
	.probe    = penta_lockin_probe,
	.remove   = penta_lockin_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device penta_lockin_device = {
	.name = "penta_lockin",
	.id   = 0,
	.dev = {
		.release    = penta_lockin_dev_release,
		.dma_mask   = &dma_mask
	}

};



static int __init penta_lockin_init( void )
{
	int rc;
	acq200_debug = pl_debug;

	CAPDEF_set_word_size(pl_word_size);
	CAPDEF_set_nchan(pl_getNumChanEquivalent());
	rc = driver_register(&penta_lockin_driver);
	if (rc){
		return rc;
	}

	return platform_device_register(&penta_lockin_device);
}


static void __exit
penta_lockin_exit_module(void)
{
	info("");
	platform_device_unregister(&penta_lockin_device);
	driver_unregister(&penta_lockin_driver);
}

module_init(penta_lockin_init);
module_exit(penta_lockin_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Driver for ACQ196 PENTA_LOCKIN");
