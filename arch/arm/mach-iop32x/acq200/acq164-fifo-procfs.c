/* ------------------------------------------------------------------------- */
/* acq164-fifo-procfs.c                                                      */
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

#define DTACQ_MACH 2
#define ACQ164
#include <linux/seq_file.h>
#include "acq200-signal2.h"
#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"
#include "acq200-fifo.h"
#include "acq164.h"

#include "iop321-auxtimer.h"

#include "linux/ctype.h"

/*
 * forward local defs 
 */

struct device_driver;
struct device;

static ssize_t show_daq_enable(struct device_driver * driver, char * buf);
static ssize_t set_daq_enable(
	struct device_driver * driver, const char * buf, size_t count);

static int acq200_proc_dumpregs(
	char *buf, char **start, off_t offset, int len,
	int* eof, void* data );

static void acq164_mk_dev_sysfs(struct device *dev);
static void acq164_create_proc_entries(struct proc_dir_entry* root);

#define DEVICE_MK_DEV_SYSFS(dev) acq164_mk_dev_sysfs(dev)
#define DEVICE_CREATE_PROC_ENTRIES(root) acq164_create_proc_entries(root)


extern int ads1278_group_delay;

#define GROUP_DELAY	ads1278_group_delay	


#include "acq200-fifo-procfs.c"


static ssize_t show_coding(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%s\n",
		       acq164_getSyscon()&ACQ164_SYSCON_CODE_OB?
		       "unsigned":
		       "signed" );
}

#define S_SIGNED "signed"
#define S_UNSIGNED "unsigned"

static ssize_t store_coding(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (strncmp(buf, S_SIGNED, strlen(S_SIGNED)) == 0){
		*ACQ164_SYSCON &= ~ACQ164_SYSCON_CODE_OB;
	}else if (strncmp(buf, S_UNSIGNED, strlen(S_UNSIGNED)) == 0){
		*ACQ164_SYSCON |= ACQ164_SYSCON_CODE_OB;
	}
	return strlen(buf);
}

static DEVICE_ATTR(coding, S_IRUGO|S_IWUGO, show_coding, store_coding);


struct OB_CLOCK_DEF ob_clock_def;


static ssize_t store_ob_clock(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct OB_CLOCK_DEF def;
	
	if (sscanf(buf, "%d %d %d %d %d %d",
		   &def.demand, &def.actual,
		   &def.FDW, &def.RDW, &def.R, &def.Sx) == 6){
		ob_clock_def = def;

		dbg(1, "demand:%d actual:%d", def.demand, def.actual);
  
		acq164_set_obclock(def.FDW, def.RDW, def.R, def.Sx);
	}	

	return count;
}

static ssize_t show_ob_clock(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
#define def ob_clock_def
	return sprintf(buf, "%d %d %d %d %d %d",
		   def.demand, def.actual,
		       def.FDW, def.RDW, def.R, def.Sx);
#undef def
}

static DEVICE_ATTR(ob_clock, S_IRUGO|S_IWUGO, show_ob_clock, store_ob_clock);




static ssize_t show_slow_clock(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf,"%d\n", DG->slow_clock);
}



/** ACQ196 has a 12 bit external clock counter.
 *  Normally this is serviced on EOC.
 *  But at low sample rates, EOC can be too low, so then we need a dedicated
 *  interrupt, which is, unfortunately the AuxTimer
 */

#define COUNT_SVC_RATE 2000   /* service a 4096 counter @ 1MHz */

extern void acq200_service_clock_counters(unsigned long);

static ssize_t store_slow_clock(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int slow_clock;
	static struct AuxTimerClient atc = {
		.func = acq200_service_clock_counters
	};
	if (sscanf(buf, "%d", &slow_clock) == 1){
		DG->slow_clock = slow_clock != 0;

		iop321_hookAuxTimer(&atc, DG->slow_clock? COUNT_SVC_RATE: 0);
	} 
	return strlen(buf);
}

static DEVICE_ATTR(
	slow_clock, S_IRUGO|S_IWUGO, show_slow_clock, store_slow_clock);


static char role_text[32];

#define ROLE_SOLO	"SOLO"
#define ROLE_MASTER	"MASTER"
#define ROLE_SLAVE	"SLAVE"
#define ROLE_EXT_MAS    "EXT_MASTER"
#define SLEN(str)	strlen(str)

#define CLKCON_
static ssize_t show_clock_role(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{	
	return sprintf(buf, "%s\n", role_text);
}

static ssize_t store_clock_role(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	u32 set = 0;
	int len = 0;
	if (strncmp(buf, ROLE_SOLO, (len = SLEN(ROLE_SOLO))) == 0){
		set = 0;
	}else if (strncmp(buf, ROLE_EXT_MAS, (len = SLEN(ROLE_EXT_MAS))) == 0){
		u32 ics = ACQ164_CLKCON_CS_MASK|ACQ164_CLKCON_CS_RISING;
		ics &= *ACQ164_CLKCON;
		set =	(ACQ164_CLKCON_D1 << ACQ164_CLKCON_OCS_SHIFT)|
			(ACQ164_CLKCON_D2 << ACQ164_CLKCON_OIND_SHIFT)|
			ACQ164_CLKCON_CS_EXT_MASTER | ics;
		dbg(1, "ics:%08x set:%08x", ics, set);
	}else if (strncmp(buf, ROLE_MASTER, (len = SLEN(ROLE_MASTER))) == 0){
		set =	(ACQ164_CLKCON_D1 << ACQ164_CLKCON_OCS_SHIFT)|
			(ACQ164_CLKCON_D2 << ACQ164_CLKCON_IND_SHIFT)|
			(ACQ164_CLKCON_D2 << ACQ164_CLKCON_OIND_SHIFT);
	}else if (strncmp(buf, ROLE_SLAVE, (len = SLEN(ROLE_SLAVE))) == 0){
		set =	(ACQ164_CLKCON_D1 << ACQ164_CLKCON_CS_SHIFT)|
			(ACQ164_CLKCON_D2 << ACQ164_CLKCON_IND_SHIFT);
	}else{
		return -1;
	}

	acq164_clkcon_clr(ACQ164_CLKCON_ALL);
	acq164_clkcon_set(set);

	strncpy(role_text, buf, len);
	role_text[len] = '\0';
	dbg(1, "role %s CLKCON set %08x", role_text, set);

	return count;
}

static DEVICE_ATTR(clock_role, S_IRUGO|S_IWUGO, 
			show_clock_role, store_clock_role);

DEFINE_EVENT_ATTR(1);


int acq200_lookup_pchan(int lchannel);


static int build_channel_mapping(int start, char* buf)
{
	int lchan;
	int len = 0;

	for (lchan = start + 1; lchan <= start + 32; ++lchan){
		len += sprintf(buf + len,
			       "%2d %2d\n", 
			       lchan, acq200_lookup_pchan(lchan));
	}
	return len;
}

static ssize_t show_channel_mapping(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int len = 0;
	if ((CAPDEF->channel_mask&1)){
		len += build_channel_mapping( 0, buf+len);
	}
	if ((CAPDEF->channel_mask&2)){
		len += build_channel_mapping(32, buf+len);
	}
	return len;
}
static DEVICE_ATTR(channel_mapping, S_IRUGO, show_channel_mapping, 0);



static ssize_t show_channel_mapping_bin(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int len = 0;
	if ((CAPDEF->channel_mask&1)){
		len += build_channel_mapping( 0, buf+len);
	}
	if ((CAPDEF->channel_mask&2)){
		len += build_channel_mapping(32, buf+len);
	}
	return len;
}
static DEVICE_ATTR(channel_mapping_bin, S_IRUGO, show_channel_mapping_bin, 0);

static ssize_t set_nacc(
	struct device *dev,
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int nacc;

	if (sscanf(buf, "%d", &nacc) == 1){
		acq164_set_nacc(nacc);
		return strlen(buf);		
	}else{
		dbg(1, "90 FAIL");
		return -1;
	}
}

static int show_nacc(
	struct device *dev,
	struct device_attribute *attr,
	char *buf
	)
{
	return sprintf(buf, "%d\n", acq164_get_nacc());
}

static DEVICE_ATTR(nacc, S_IRUGO|S_IWUGO, show_nacc, set_nacc);


static int sfpga_get_rev(void)
{
	u32 reg;
	u32 rev;
	u32 test;
	int pass;

	*ACQ164_SYSCON |= ACQ164_SYSCON_REV_RESET;
	*ACQ164_SYSCON &= ~ACQ164_SYSCON_REV_RESET;

	reg = *ACQ164_BDR;
	dbg(1, "BDR read #%d value 0x%08x %s", 1, reg, 
				reg==ACQ100_BDR_MAGIC? "GOOD": "BAD");

	if (reg != ACQ100_BDR_MAGIC){
		return - __LINE__;
	}

	*ACQ164_BDR = 0;
	reg = *ACQ164_BDR;

	dbg(1, "BDR read #%d value 0x%08x %s", 2, reg, 
	    reg>=0x300? "HAS ADC comms": "NO ADC_COMMS");
	if (reg == 0){
		return 0;
	}
       
	rev = reg;

	/* now make a bus test ... simple crossed bit is OK */

	reg = 0xaa55aa55;
	*ACQ164_BDR = reg;


	test = *ACQ164_BDR;
	pass = test == reg;

	dbg(1, "BDR read #%d value 0x%08x %s", 3, test, 
	    pass? "PASS": "FAIL");       

	if (!pass){
		return - __LINE__;
	}
	reg = 0x55aa55aa;
	*ACQ164_BDR = reg;
	test = *ACQ164_BDR;
	pass = test == reg;

	dbg(1, "BDR read #%d value 0x%08x %s", 4, test, 
	    pass? "PASS": "FAIL");       
	
	*ACQ164_SYSCON |= ACQ164_SYSCON_REV_RESET;
	*ACQ164_SYSCON &= ~ACQ164_SYSCON_REV_RESET;

	test = *ACQ164_BDR;
	pass = test == ACQ100_BDR_MAGIC;

	dbg(1, "BDR read #%d value 0x%08x %s", 5, test, 
	    pass? "PASS": "FAIL");       
	
	return pass? rev: - __LINE__;	
}

int acq164_sfpga_get_rev(void)
{
	static int rev;
	static int rev_valid;

	if (!rev_valid){
		rev = sfpga_get_rev();
		rev_valid = 1;
	}
	return rev;
}
static ssize_t show_fpga_state(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int len = 0;
	u32 debug_old;
	u32 t1, t2;
	int s_ok = 0;
	int rev = sfpga_get_rev();

	if (rev == 0){
		debug_old = *ACQ164_BDR;
		*ACQ164_BDR = ACQ100_BDR_MAGIC;
		t1 = *ACQ164_BDR;
		*ACQ164_BDR = ~ACQ100_BDR_MAGIC;
		t2 = *ACQ164_BDR;
		*ACQ164_BDR = debug_old;

		s_ok =	t1 == ACQ100_BDR_MAGIC && 
			t2 == ~t1 && *ACQ164_BDR == debug_old;
	}else if (rev > 0){
		s_ok = 1;
	}

	len = sprintf(buf, "S_FPGA=%s rev=%04x\n", 
		      s_ok? "GOOD": "ERR", rev);
	return len;
}

static DEVICE_ATTR(fpga_state, S_IRUGO, show_fpga_state, 0);



DEFINE_SIGNAL_ATTR(sync_trig_mas);
DEFINE_SIGNAL_ATTR(clk_counter_src);

DEFINE_SIGNAL_ATTR(index_src);
DEFINE_SIGNAL_ATTR(index_mas);
DEFINE_SIGNAL_ATTR(adc_mode);
extern int acq132_showClkCounter(char *buf);

static ssize_t show_ClkCounter(
	struct device *dev,
	struct device_attribute *attr,
	char* buf)
{
	return acq132_showClkCounter(buf);
}

static DEVICE_ATTR(ClkCounter, S_IRUGO, show_ClkCounter, 0);

static ssize_t show_RGM(
	struct device * dev,
	struct device_attribute *attr,
	char *buf)
{
	unsigned rgm_word = *ACQ164_RGM;
	unsigned dio = (rgm_word&ACQ164_RGM_GATE_DIO)>> ACQ164_RGM_GATE_DIO_SHL;

	if (rgm_word & ACQ164_RGM_ENABLE){
		return sprintf(buf, "%d DI%d\n", 1, dio);
	}else{
		return sprintf(buf, "0\n");
	}
}

static ssize_t store_RGM(
	struct device * dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned mode;
	unsigned dio;
	unsigned rgm_word = *ACQ164_RGM;

	rgm_word &= ~(ACQ164_RGM_GATE_DIO|ACQ164_RGM_ENABLE);

	switch(sscanf(buf, "%u DI%u", &mode, &dio)){
	case 2:
		 rgm_word |= dio << ACQ164_RGM_GATE_DIO_SHL;
		 if (mode != 0){
			 rgm_word |= ACQ164_RGM_ENABLE;
		 }	/* fall thru */
	case 1:
		*ACQ164_RGM = rgm_word;
		return count;
	default:
		return -EPERM;
	}
}

static DEVICE_ATTR(RepeatingGateMode, S_IRUGO|S_IWUGO, show_RGM, store_RGM);


static void acq164_mk_dev_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_coding);
	DEVICE_CREATE_FILE(dev, &dev_attr_slow_clock);
	DEVICE_CREATE_FILE(dev, &dev_attr_clock_role);
	DEVICE_CREATE_FILE(dev, &dev_attr_event1);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mapping);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mapping_bin);
	DEVICE_CREATE_FILE(dev, &dev_attr_sync_trig_mas);
	DEVICE_CREATE_FILE(dev, &dev_attr_clk_counter_src);
	DEVICE_CREATE_FILE(dev, &dev_attr_ob_clock);
	DEVICE_CREATE_FILE(dev, &dev_attr_fpga_state);
	DEVICE_CREATE_FILE(dev, &dev_attr_ClkCounter);
	DEVICE_CREATE_FILE(dev, &dev_attr_index_src);
	DEVICE_CREATE_FILE(dev, &dev_attr_index_mas);
	DEVICE_CREATE_FILE(dev, &dev_attr_adc_mode);
	DEVICE_CREATE_FILE(dev, &dev_attr_nacc);
	DEVICE_CREATE_FILE(dev, &dev_attr_RepeatingGateMode);
}



void acq200_setChannelMask(unsigned mask)
{
	int lchan;

	CAPDEF->channel_mask = mask&0x3;
	CAPDEF_set_nchan(((mask&0x1) != 0)*32 + ((mask&0x2) != 0)*32);

	for (lchan = 1; lchan <= NCHAN; ++lchan){
		int enable =  ((1 << (lchan-1)/32) & mask) != 0;
		acq200_setChannelEnabled(acq200_lookup_pchan(lchan), enable);
	}
}



static ssize_t show_daq_enable(
	struct device_driver * driver, char * buf)
{
	int enable = (acq164_getSyscon()&ACQ164_SYSCON_ACQEN) != 0;

        return sprintf(buf,"%d\n", enable);
}


static ssize_t set_daq_enable(
	struct device_driver * driver, const char * buf, size_t count)
{
	int enable;
	sscanf(buf, "%d", &enable);

	if (enable){
		enable_acq();
	}else{
		disable_acq();
	}
	return strlen(buf);
}

#define BANK_ENTRY(dev, cmask)	  { "BANK" #dev " " #cmask "\n", 0 }
#define SPACER_ENTRY	  { "\n", 0 }
#define REGS_LUT_ENTRY(n) { #n, n }


static struct REGS_LUT {
	const char *name;
	volatile u32* preg;
}
	regs_lut[] = {
		REGS_LUT_ENTRY(ACQ164_BDR),
		REGS_LUT_ENTRY(ACQ164_FIFCON),
		REGS_LUT_ENTRY(ACQ164_FIFSTAT),
		REGS_LUT_ENTRY(ACQ164_SYSCON),
		REGS_LUT_ENTRY(ACQ164_OSR),
		REGS_LUT_ENTRY(ACQ164_ICS527),
		REGS_LUT_ENTRY(ACQ164_SYSCONDAC),
		REGS_LUT_ENTRY(ACQ164_CLKCON),
		REGS_LUT_ENTRY(ACQ164_DIOCON),
		REGS_LUT_ENTRY(ACQ164_OFFSET_DACS),
		REGS_LUT_ENTRY(ACQ164_WAVLIMIT),
		REGS_LUT_ENTRY(ACQ164_TCR_IMMEDIATE),
		REGS_LUT_ENTRY(ACQ164_TCR_LATCH),
		REGS_LUT_ENTRY(ACQ164_RGATE),
		REGS_LUT_ENTRY(ACQ164_CLK_COUNTER),
		REGS_LUT_ENTRY(ACQ164_GPG)
	};

#define REGS_LUT_ENTRIES (sizeof(regs_lut)/sizeof (struct REGS_LUT))


int acq200_dumpregs_diag(char* buf, int len)
/** @todo still needed elsewhere, unfortunately */
{
	char *bp = buf;
#define APPEND(reg) \
        bp += snprintf(bp, len - (bp-buf), "%20s:[%02x] 0x%08X\n", \
                   #reg, ((unsigned)reg- (unsigned)ACQ200_FPGA), *reg)

	APPEND(ACQ164_BDR);
	APPEND(ACQ164_FIFCON);
	APPEND(ACQ164_FIFSTAT);
	APPEND(ACQ164_SYSCON);
	APPEND(ACQ164_OSR);
	APPEND(ACQ164_ICS527);
	APPEND(ACQ164_SYSCONDAC);
	APPEND(ACQ164_CLKCON);
	APPEND(ACQ164_DIOCON);
	APPEND(ACQ164_OFFSET_DACS);
	APPEND(ACQ164_WAVLIMIT);
	APPEND(ACQ164_TCR_IMMEDIATE);
	APPEND(ACQ164_TCR_LATCH);
	APPEND(ACQ164_RGATE);
	APPEND(ACQ164_CLK_COUNTER);
	APPEND(ACQ164_GPG);

	return bp-buf;
}


static void *dump_regs_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos < REGS_LUT_ENTRIES){
		return regs_lut + *pos;
	}else{
		return NULL;
	}
}

static void *dump_regs_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;

	if (*pos < REGS_LUT_ENTRIES){
		return regs_lut + *pos;
	}else{
		return NULL;
	}
}

static int dump_regs_seq_show(struct seq_file *s, void *v)
{
	const struct REGS_LUT *plut = (struct REGS_LUT*)v;

	const char* name = plut->name;
	volatile u32 *reg = plut->preg;

	if (reg == 0){
		seq_printf(s, "%s", name);
	}else{
		unsigned offset = ((unsigned)reg - (unsigned)ACQ200_FPGA);
		seq_printf(s, "%20s:[%03x] 0x%08X\n", name, offset, *reg);
	}
	return 0;
}	

static void dump_regs_seq_stop(struct seq_file* s, void *v)
{

}

static int dump_regs_proc_open(struct inode *inode, struct file *file)
{
	static struct seq_operations seq_ops = {
		.start = dump_regs_seq_start,
		.next = dump_regs_seq_next,
		.stop = dump_regs_seq_stop,
		.show = dump_regs_seq_show
	};
	return seq_open(file, &seq_ops);
}

static struct file_operations dump_regs_proc_fops = {
	.owner = THIS_MODULE,
	.open = dump_regs_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release	
};


static int acq200_proc_dumpregs(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	return acq200_dumpregs_diag(buf, len);
}


static void acq164_create_proc_entries(struct proc_dir_entry* root)
{

	
        struct proc_dir_entry *dump_regs_entry =
		create_proc_entry("dump_regs", S_IRUGO, root);

	if (dump_regs_entry){
	        dump_regs_entry->proc_fops = &dump_regs_proc_fops;
	}
}



static int acq164_bits(void)
{
	if (acq164_sfpga_get_rev() >= ACQ164_RJ_REV){
		int ibit = 0;
		int nacc = acq164_get_nacc();

		for (ibit = 0; 1<<ibit < nacc; ++ibit)
			;
		return ibit + 24;
	}else{
		return 32;
	}
}

#define MAX_24	((1 << (24-1)) - 1)
#define MIN_24  (-(1 << (24 - 1)))

#define MAX_32 0x7FFFFFFF
#define MIN_32 (-MAX_32 - 1)

int acq164_code_min(void)
{
	if (acq164_sfpga_get_rev() >= ACQ164_RJ_REV){
		return acq164_get_nacc() * MIN_24;
	}else{
		return MIN_32;
	}
}

int acq164_code_max(void)
{
	if (acq164_sfpga_get_rev() >= ACQ164_RJ_REV){
		return acq164_get_nacc() * MAX_24;
	}else{
		return MAX_32;
	}
}

int acq200_bits(void)
{
	return acq164_bits();
}
EXPORT_SYMBOL_GPL(acq200_bits);
