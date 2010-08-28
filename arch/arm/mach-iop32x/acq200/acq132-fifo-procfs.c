/* ------------------------------------------------------------------------- */
/* acq132-fifo-procfs.c                                                      */
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

#define DTACQ_MACH 1
#define ACQ132
#include <linux/seq_file.h>

#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"
#include "acq200-fifo.h"
#include "acq132.h"

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


static void acq132_mk_dev_sysfs(struct device *dev);
static void acq132_create_proc_entries(struct proc_dir_entry* root);

#define DEVICE_MK_DEV_SYSFS(dev) acq132_mk_dev_sysfs(dev)
#define DEVICE_CREATE_PROC_ENTRIES(root) acq132_create_proc_entries(root)



#include "acq200-fifo-procfs.c"


static ssize_t show_coding(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%s\n",
		       acq196_getSyscon()&ACQ196_SYSCON_CODE_OB?
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
		*ACQ196_SYSCON_ADC &= ~ACQ196_SYSCON_CODE_OB;
	}else if (strncmp(buf, S_UNSIGNED, strlen(S_UNSIGNED)) == 0){
		*ACQ196_SYSCON_ADC |= ACQ196_SYSCON_CODE_OB;
	}
	return strlen(buf);
}

static DEVICE_ATTR(coding, S_IRUGO|S_IWUGO, show_coding, store_coding);


/* testmode id's :

2, 3
4, 5
6, 7
8, 9

*/

static void acq132_set_testmode(int enable)
{
	int dev;

	for (dev = BANK_A; dev <= BANK_D; ++dev){
		u32 bankid = (dev+1)*2;		
		u32 control = enable? (bankid<<16) | (bankid|1) : 0;

		info( "%p = 0x%08x", ACQ132_ADC_TESTM(dev), control);
		*ACQ132_ADC_TESTM(dev) = control;
	}	
}



static int test_mode;

static ssize_t show_testmode(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", test_mode);
}

#define S_SIGNED "signed"
#define S_UNSIGNED "unsigned"

static ssize_t store_testmode(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (sscanf(buf, "%d", &test_mode)){
		acq132_set_testmode(test_mode != 0);
	}	
	return strlen(buf);
}

static DEVICE_ATTR(testmode, S_IRUGO|S_IWUGO, show_testmode, store_testmode);


/* @todo - single bit all channel control atm.
 * but simulate multi channel control
 */
static ssize_t store_adc_range(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	u32 range_sel = 0;
#ifdef WORKTODO
	int ibit;
	int max_bit = count < 32? count: 32;
#endif
	if (sscanf(buf, "%x", &range_sel) == 1){
		acq132_set_adc_range(range_sel);
	}

	return count;
}

static ssize_t show_adc_range(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	u32 range_sel = acq132_get_adc_range();
	return sprintf(buf, "%08x\n", range_sel);
}

static DEVICE_ATTR(ADC_RANGE, S_IRUGO|S_IWUGO, show_adc_range, store_adc_range);


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
  
		acq132_set_obclock(def.FDW, def.RDW, def.R, def.Sx);
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




static ssize_t show_FAWG_div(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf,"%d\n", getFAWG_DIV());
}

static ssize_t store_FAWG_div(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int div;

	if (sscanf(buf, "%d", &div) == 1 && (div >= 1 && div <= 256)){
		setFAWG_DIV(div);
	} 
	return strlen(buf);
}

static DEVICE_ATTR(
	FAWG_div, S_IRUGO|S_IWUGO, show_FAWG_div, store_FAWG_div);


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


static ssize_t show_AO_coding(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%s\n",
		       *ACQ196_SYSCON_DAC&ACQ196_SYSCON_CODE_OB?
		       "unsigned":
		       "signed" );
}

static ssize_t store_AO_coding(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "0x%x", &DG->FIFERR);

	if (strncmp(buf, S_SIGNED, strlen(S_SIGNED)) == 0){
		*ACQ196_SYSCON_DAC &= ~ACQ196_SYSCON_CODE_OB;
	}else if (strncmp(buf, S_UNSIGNED, strlen(S_UNSIGNED)) == 0){
		*ACQ196_SYSCON_DAC |= ACQ196_SYSCON_CODE_OB;
	}
	return strlen(buf);
}

static DEVICE_ATTR(
	AO_coding, S_IRUGO|S_IWUGO, show_AO_coding, store_AO_coding);



#define S_SOFT "soft"
#define S_HARD "clocked"

static ssize_t show_AO_clock_mode(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%s\n",
		       *ACQ196_SYSCON_DAC&ACQ196_SYSCON_LOWLAT?
		       S_SOFT " (lowlatency)":
		       S_HARD );
}

static ssize_t store_AO_clock_mode(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (strncmp(buf, S_SOFT, strlen(S_SOFT)) == 0){
		*ACQ196_SYSCON_DAC |= ACQ196_SYSCON_LOWLAT;
	}else if (strncmp(buf, S_HARD, strlen(S_HARD)) == 0){
		*ACQ196_SYSCON_DAC &= ~ACQ196_SYSCON_LOWLAT;
	}
	return strlen(buf);
}


static DEVICE_ATTR(
	AO_clock_mode, S_IRUGO|S_IWUGO, 
	show_AO_clock_mode, store_AO_clock_mode);

static ssize_t show_event_timer_prescale(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%d\n", acq132_get_prescale());	
}

static ssize_t store_event_timer_prescale(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int prescale;

	if (sscanf(buf, "%d", &prescale) == 1){
		acq132_set_prescale(prescale);
		return count;
	}else{
		return -EINVAL;
	}
}

static DEVICE_ATTR(
	event_timer_prescale,  S_IRUGO|S_IWUGO,
	show_event_timer_prescale, store_event_timer_prescale);


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
	len += build_channel_mapping( 0, buf+len);
	return len;
}
static DEVICE_ATTR(channel_mapping, S_IRUGO, show_channel_mapping, 0);



static int build_channel_mapping_bin(int start, char* buf)
{
	int lchan;
	short *sb = (short*)buf;

	for (lchan = start + 1; lchan <= start + 32; ++lchan){
		*sb ++ = (short)acq200_lookup_pchan(lchan);
	}
	return 32*sizeof(short);
}

static ssize_t show_channel_mapping_bin(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int len = 0;
	len += build_channel_mapping_bin( 0, buf+len);
	return len;
}
static DEVICE_ATTR(channel_mapping_bin, S_IRUGO, show_channel_mapping_bin, 0);





static int belongs(int key, const int lut[], int nlut)
{
	int ikey;

	for (ikey = 0; ikey < nlut; ++ikey){
		if (key == lut[ikey]){
			return 1;
		}
	}
	return 0;
}

static int store_osam(
	int block, 
	int lr,
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf)
{
	static const int good_shift[] = { -2, -1, 0, 1, 2, 3, 4, 5 };
#define GOOD_SHIFT (sizeof(good_shift)/sizeof(int))

	int nacc;
	int shift;
	int decimate = strstr(buf, "decimate") != 0;
	int n4 = 0;
	const char *cursor;

	if ((cursor = strstr(buf, "nacc4="))){
		sscanf(cursor, "nacc4=%d", &n4);
	}
	if (sscanf(buf, "nacc=%d shift=%d", &nacc, &shift) == 2 ||
	    sscanf(buf, "%d %d", &nacc, &shift) == 2){
		if (!IN_RANGE(nacc, 1, 16)){
			err("bad nacc %d", nacc);		      
		}else if (!belongs(shift, good_shift, GOOD_SHIFT)){
			err("bad shift %d", shift);
		}else{
			acq132_set_osam_nacc(
				block, OSAMLR(lr), nacc, shift, decimate, n4);
			return strlen(buf);
		}
	}else{
		err("failed to scan \"nacc=N shift=S\"");
	}

	return -EINVAL;
}
static ssize_t show_osam(
	int block, int lr,
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int nacc, shift;
	unsigned shift_code;
	unsigned n4 = 0;

	const u32 osam = *ACQ132_ADC_OSAM(block) >> OSAMLR(lr);

	nacc = ((osam>>ACQ132_ADC_OSAM_R_NACC) & 0x0f) + 1;
	shift_code = (osam>>ACQ132_ADC_OSAM_R_SHIFT) & 0x0f;
	switch(shift_code){
	case SHIFT_M1:
		shift = -1; break;
	case SHIFT_M2:
		shift = -2; break;
	case SHIFT_0:
		shift = 0; break;
	case SHIFT_P1:
		shift = 1; break;
	case SHIFT_P2:
		shift = 2; break;
	default:
		return sprintf(buf, "illegal shift field 0x%x", shift_code);
	}
	n4 = (osam & (1<<ACQ132_ADC_OSAM_R_NACC4)) != 0;

	return sprintf(buf, "nacc=%d shift=%d %s nacc4=%d\n", nacc, shift, 
		       osam&(1<<ACQ132_ADC_OSAM_R_ACCEN)? 
				       "accumulate": "decimate", n4);
}

#define OSAM(BLK, LR)							\
static int store_osam_##BLK##_##LR(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf, size_t count)					\
{									\
	return store_osam(*#BLK-'A', *#LR, dev, attr, buf);		\
}									\
static ssize_t show_osam_##BLK##_##LR(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_osam(*#BLK-'A', *#LR, dev, attr, buf);		\
}									\
static DEVICE_ATTR(oversample_##BLK##_##LR, S_IRUGO|S_IWUGO,			\
		   show_osam_##BLK##_##LR, store_osam_##BLK##_##LR) 


#define OSAM_PAIR(BLK) 	OSAM(BLK, L); OSAM(BLK, R)

OSAM_PAIR(A);
OSAM_PAIR(B);
OSAM_PAIR(C);
OSAM_PAIR(D);


#define DEVICE_CREATE_OSAM_PAIR(dev, BLK)			 \
	DEVICE_CREATE_FILE(dev, &dev_attr_oversample_##BLK##_L); \
	DEVICE_CREATE_FILE(dev, &dev_attr_oversample_##BLK##_R);

#define DEVICE_CREATE_OSAM_GROUP(dev) do {	\
	DEVICE_CREATE_OSAM_PAIR(dev, A);	\
	DEVICE_CREATE_OSAM_PAIR(dev, B);	\
	DEVICE_CREATE_OSAM_PAIR(dev, C);	\
	DEVICE_CREATE_OSAM_PAIR(dev, D);	\
	} while(0)


#define TO_SX(c) ((c)-'A')
#define FROM_SX(sx) (((sx)&0x3)+'A')
#define IS_SX(c) ((c) >= 'A' && (c) <= 'D')

static ssize_t show_scanlist(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	u32 scan_def = *ACQ132_SCAN_LIST_DEF;
	int nscan = acq132_getScanlistLen();
	int iscan;

	for (iscan = 0; iscan < nscan; ++iscan){
		buf[iscan] = FROM_SX(scan_def >> (iscan*2));
	}

	buf[iscan++] = '\n';
	buf[iscan++] = '\0';
	return strlen(buf);
}


static ssize_t store_scanlist(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int iscan;
	int maxscan = min((int)strlen(buf), ACQ132_SCAN_MAX);
	u32 scan_def = 0;
	int ok = 0;

	while(isspace(buf[maxscan-1])){
		--maxscan;
	}
	for (iscan = 0; iscan < maxscan; ++iscan){
		if (IS_SX(buf[iscan])){
       			scan_def |= TO_SX(buf[iscan]) << 2*iscan;
			ok = 1;
		}else{
			err("\"%s\" char %d valid scan codes: A,B,C,D",
			    buf, iscan);
			ok = 0;
			break;
		}		
	}
	if (ok){
		acq132_setScanList(iscan, scan_def);
		return strlen(buf);
	}else{
		return -EINVAL;
	}
}
static DEVICE_ATTR(scanlist, S_IRUGO|S_IWUGO, show_scanlist, store_scanlist);


static ssize_t show_gpg_mas(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	u32 syscon = *ACQ132_SYSCON;

	if ((syscon&ACQ132_SYSCON_GPG_MAS) == 0){
		return sprintf(buf, "none\n");
	}else{
		return sprintf(buf, "%s%s%s%s\n",
		       syscon&ACQ132_SYSCON_GPG_MASD7 ? "d7 ": "",
		       syscon&ACQ132_SYSCON_GPG_MASD6 ? "d6 ": "",
		       syscon&ACQ132_SYSCON_GPG_MASD5 ? "d5 ": "",
		       syscon&ACQ132_SYSCON_GPG_MASD4 ? "d4 ": "" );
	}
}

static ssize_t store_gpg_mas(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
/* set mas bit to be output as well. NB - doesn't set input if omitted! */
{
	
	unsigned diocon = *ACQ200_DIOCON;
	u32 syscon = *ACQ132_SYSCON;
	char w[4][16] = {};
	int nw;
	int iw;
	int ibit;
	int init_high = 0;

	syscon &= ~ACQ132_SYSCON_GPG_MAS;

	if ((nw = sscanf(buf, "%15s %15s %15s %15s", 
			 w[0], w[1], w[2], w[3])) == 0){
		err("failed to scan command \"%s\"", buf);
		return -EINVAL;
	}
	if (strcmp(w[0], "none") == 0){
		*ACQ132_SYSCON = syscon;
		return count;
	}

	for (iw = 0; iw != nw; ++iw){
		if (sscanf(w[iw], "d%1d,%1d", &ibit, &init_high) >= 1){
			switch(ibit){
			default:
				err("dx must be in range 7654");
				return -EINVAL;
			case 7:
				syscon |= ACQ132_SYSCON_GPG_MASD7;
				break;
			case 6:
				syscon |= ACQ132_SYSCON_GPG_MASD6;
				break;
			case 5:
				syscon |= ACQ132_SYSCON_GPG_MASD5;
				break;
			case 4:
				syscon |= ACQ132_SYSCON_GPG_MASD4;
				break;
			}
			if (init_high){
				diocon = DIO_SET_OUTPUT1(diocon, ibit);
			}else{
				diocon = DIO_SET_OUTPUT0(diocon, ibit);
			}
		}else{
			err("invalid w[%d] \"%s\"", iw, w[iw]);
			return -EINVAL;
		}
	}

	acq200_setDiocon(diocon);
	*ACQ132_SYSCON = syscon;
	return count;			
}

static DEVICE_ATTR(gpg_mas, S_IRUGO|S_IWUGO, show_gpg_mas, store_gpg_mas);



static int sfpga_get_rev(void)
{
	u32 reg;
	u32 rev;
	u32 test;
	int pass;

	*ACQ132_SYSCON |= ACQ132_SYSCON_REV_RESET;
	*ACQ132_SYSCON &= ~ACQ132_SYSCON_REV_RESET;

	reg = *ACQ132_BDR;
	dbg(1, "BDR read #%d value 0x%08x %s", 1, reg, 
				reg==ACQ100_BDR_MAGIC? "GOOD": "BAD");

	if (reg != ACQ100_BDR_MAGIC){
		return - __LINE__;
	}

	*ACQ132_BDR = 0;
	reg = *ACQ132_BDR;

	dbg(1, "BDR read #%d value 0x%08x %s", 2, reg, 
	    reg>=0x300? "HAS ADC comms": "NO ADC_COMMS");
	if (reg == 0){
		return 0;
	}
       
	rev = reg;

	/* now make a bus test ... simple crossed bit is OK */

	reg = 0xaa55aa55;
	*ACQ132_BDR = reg;


	test = *ACQ132_BDR;
	pass = test == reg;

	dbg(1, "BDR read #%d value 0x%08x %s", 3, test, 
	    pass? "PASS": "FAIL");       

	if (!pass){
		return - __LINE__;
	}
	reg = 0x55aa55aa;
	*ACQ132_BDR = reg;
	test = *ACQ132_BDR;
	pass = test == reg;

	dbg(1, "BDR read #%d value 0x%08x %s", 4, test, 
	    pass? "PASS": "FAIL");       
	
	*ACQ132_SYSCON |= ACQ132_SYSCON_REV_RESET;
	*ACQ132_SYSCON &= ~ACQ132_SYSCON_REV_RESET;

	test = *ACQ132_BDR;
	pass = test == ACQ100_BDR_MAGIC;

	dbg(1, "BDR read #%d value 0x%08x %s", 5, test, 
	    pass? "PASS": "FAIL");       
	
	if (rev == 0xffffffff){
		info("WARNING: FPGA is a development release");
	}
	return pass? rev: - __LINE__;	
}

int acq132_sfpga_get_rev(void)
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
	int s_ok = 0, a_ok = 0;
	u32 a_done = 0;
	int rev = sfpga_get_rev();

	if (rev == 0){
		debug_old = *ACQ196_BDR;
		*ACQ196_BDR = 0xdeadbeef;
		t1 = *ACQ196_BDR;
		*ACQ196_BDR = ~0xdeadbeef;
		t2 = *ACQ196_BDR;
		*ACQ196_BDR = debug_old;

		s_ok =	t1 == 0xdeadbeef && 
			t2 == ~t1 && *ACQ196_BDR == debug_old;
	}else if (rev > 0){
		s_ok = 1;
	}

	if (s_ok){
		a_done = ((*ACQ132_SFPGA_CONF) >> ACQ132_SFPGA_CONF_DONE_8) &
				ACQ132_SFPGA_CONF_8MASK;
		a_ok = a_done == ACQ132_SFPGA_CONF_8MASK;
		if (!a_ok){
			err("A_FPGA not OK: %08x", *ACQ132_SFPGA_CONF);
		}
	}
	len = sprintf(buf, "%s S_FPGA=%s A_FPGA=%02x rev:%08x\n",
		      s_ok && a_ok? "GOOD": "ERR", s_ok? "OK": "ERR", a_done,
			rev);
	return len;
}

static DEVICE_ATTR(fpga_state, S_IRUGO, show_fpga_state, 0);



DEFINE_SIGNAL_ATTR(ao_trig);
DEFINE_SIGNAL_ATTR(ao_clk);

DEFINE_SIGNAL_ATTR(sync_trig_src);
DEFINE_SIGNAL_ATTR(sync_trig_mas);
DEFINE_SIGNAL_ATTR(gate_src);
DEFINE_SIGNAL_ATTR(clk_counter_src);


static ssize_t show_RGM(
	struct device * dev,
	struct device_attribute *attr,
	char *buf)
{
	static const char* modes[] = {
		[0] = "OFF",
		[1] = "GATE",
		[2] = "GATE",
	};
	int mode = acq132_getRGM() != 0;

	return sprintf(buf, "%d %s\n", mode, modes[mode]);		
}

static ssize_t store_RGM(
	struct device * dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned mode;

	if (sscanf(buf, "%u", &mode) > 0){
		acq132_setRGM(mode);
		return count;
	}else{
		return -EPERM;
	}
}

static DEVICE_ATTR(RepeatingGateMode, S_IRUGO|S_IWUGO, show_RGM, store_RGM);

char channelSpeedMask[MAXCHAN+1];

const char* acq132_getChannelSpeedMask(void)
{
	return channelSpeedMask;
}

static ssize_t show_ChannelSpeedMask(
	struct device *dev,
	struct device_attribute *attr,
	char* buf)
{
	strcpy(buf, channelSpeedMask);
	return strlen(channelSpeedMask);
}

static ssize_t store_ChannelSpeedMask(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	strncpy(channelSpeedMask, buf, sizeof(channelSpeedMask));
	return count;
}

static DEVICE_ATTR(ChannelSpeedMask, S_IRUGO|S_IWUGO, 
		   show_ChannelSpeedMask, store_ChannelSpeedMask);


static ssize_t show_clksel(
	struct device *dev,
	struct device_attribute *attr,
	char* buf)
{
	u32 clkdat = *ACQ132_CLKDAT & ~ACQ196_CLKDAT_CLKDIV;
	const char *ukey;

	switch(clkdat){
	case LO_ICS:	 
		ukey = "LO_ICS";       break;
	case INTCLK_ICS: 
		ukey = "INTCLK_ICS";   break;
	case INTCLK_NOICS: 
		ukey = "INTCLK_NOICS"; break;
	default:
		ukey = "";
	}
	return sprintf(buf, "0x%02x %s\n", clkdat>>24, ukey);
}

static ssize_t store_clksel(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	u32 clksel;
	if ((clksel = simple_strtoul(buf, 0, 0) << 24) != 0){
		acq132_initClkDatFields(clksel);
	}else{
		if (strncmp(buf, "LO_ICS", strlen("LO_ICS")) == 0){
			acq132_initClkDatFields(LO_ICS);	
		}else if (
			strncmp(buf, "INTCLK_ICS", strlen("INTCLK_ICS")) == 0){
			acq132_initClkDatFields(INTCLK_ICS);
		}else if (
			strncmp(buf,"INTCLK_NOICS",strlen("INTCLK_NOICS"))==0){
			acq132_initClkDatFields(INTCLK_NOICS);
		}else{
			return -1;
		}
	}

	return count;
}

static DEVICE_ATTR(clksel, S_IRUGO|S_IWUGO, 
		   show_clksel, store_clksel);



extern int acq132_showClkCounter(char *buf);

static ssize_t show_ClkCounter(
	struct device *dev,
	struct device_attribute *attr,
	char* buf)
{
	return acq132_showClkCounter(buf);
}

static DEVICE_ATTR(ClkCounter, S_IRUGO, show_ClkCounter, 0);

extern int acq132_set_best_decimation(int khz, int *khz_clock, int *decim);

static int khz_clock;
static int decim;

static ssize_t show_best_decimation(
	struct device *dev,
	struct device_attribute *attr,
	char* buf)
{
	return sprintf(buf, "%d %d\n", khz_clock, decim);
}

static ssize_t store_best_decimation(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int khz;

	khz_clock = -1;	

	if (sscanf(buf, "%d", &khz) == 1){
		if (acq132_set_best_decimation(khz, &khz_clock, &decim) == 0){
			return count;						
		}
	}
	return -1;
}

static DEVICE_ATTR(best_decimation, S_IRUGO|S_IWUGO, 
		   show_best_decimation, store_best_decimation);


static ssize_t show_DR(
	struct device *dev,
	struct device_attribute *attr,
	char* buf)
{
	int nacc, shift;
	int enable = acq132_get_DR(&nacc, &shift);
	if (enable){
		return sprintf(buf, "1 %d %d\n", nacc, shift);
	}else{
		return sprintf(buf, "0\n");
	}
}

static ssize_t store_DR(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int enable, nacc = 8, shift = -2;

	if (sscanf(buf, "%d %d %d", &enable, &nacc, &shift) > 0){
		acq132_set_DR(enable, nacc, shift);
	}
	return count;
}

static DEVICE_ATTR(DualRate, S_IRUGO|S_IWUGO, show_DR, store_DR);


static void acq132_mk_dev_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_coding);
	DEVICE_CREATE_FILE(dev, &dev_attr_testmode);
	DEVICE_CREATE_FILE(dev, &dev_attr_FAWG_div);
	DEVICE_CREATE_FILE(dev, &dev_attr_slow_clock);
	DEVICE_CREATE_FILE(dev, &dev_attr_AO_coding);
	DEVICE_CREATE_FILE(dev, &dev_attr_AO_clock_mode);
	DEVICE_CREATE_FILE(dev, &dev_attr_event_timer_prescale);
	DEVICE_CREATE_FILE(dev, &dev_attr_event1);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mapping);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mapping_bin);
	DEVICE_CREATE_FILE(dev, &dev_attr_ao_trig);
	DEVICE_CREATE_FILE(dev, &dev_attr_ao_clk);
	DEVICE_CREATE_FILE(dev, &dev_attr_sync_trig_src);
	DEVICE_CREATE_FILE(dev, &dev_attr_sync_trig_mas);
	DEVICE_CREATE_FILE(dev, &dev_attr_gate_src);
	DEVICE_CREATE_FILE(dev, &dev_attr_clk_counter_src);
	DEVICE_CREATE_OSAM_GROUP(dev);
	DEVICE_CREATE_FILE(dev, &dev_attr_scanlist);
	DEVICE_CREATE_FILE(dev, &dev_attr_gpg_mas);
	DEVICE_CREATE_FILE(dev, &dev_attr_ADC_RANGE);
	DEVICE_CREATE_FILE(dev, &dev_attr_ob_clock);
	DEVICE_CREATE_FILE(dev, &dev_attr_fpga_state);
	DEVICE_CREATE_FILE(dev, &dev_attr_RepeatingGateMode);
	DEVICE_CREATE_FILE(dev, &dev_attr_ChannelSpeedMask);
	DEVICE_CREATE_FILE(dev, &dev_attr_ClkCounter);
	DEVICE_CREATE_FILE(dev, &dev_attr_best_decimation);
	DEVICE_CREATE_FILE(dev, &dev_attr_clksel);
	DEVICE_CREATE_FILE(dev, &dev_attr_DualRate);
}



int count_bits(unsigned mask) {
	int ibit;
	int count = 0;

	for (ibit = 0; ibit < 32; ++ibit){
		if (mask & (1<<ibit)){
			++count;
		}
	}
	return count;
}
static int isValidMask(unsigned mm, int shr)
{
	unsigned left = mm >> (shr + 16);
	unsigned right = (mm >> shr) & 0x0000ffff;

	if (left != right){
		dbg(1, "%x ! = %x\n", left<<16, right);
		return 0;
	}
	switch(left){
	case MASK_1:
	case MASK_2:
	case MASK_4:
		return 1;
	default:
		dbg(1, "%x not valid\n", left);
		return 0;
	}
}


/*
Valid Channel Masks

11111111111111111111111111111111
11110000000000001111000000000000
10100000000000001010000000000000
00010000000000000001000000000000
00001111000000000000111100000000
00001010000000000000101000000000
00000001000000000000000100000000
00000000111100000000000011110000
00000000101000000000000010100000
00000000000100000000000000010000
00000000000011110000000000001111
00000000000010100000000000001010
00000000000000010000000000000001

... and of course any combo between columns 
eg

10100000000000001010000000000000
00001010000000000000101000000000

10101010000000001010101000000000
*/


void acq200_setChannelMask(unsigned mask)
{
	unsigned mm;
	int lchan;
	unsigned vmask = 0;

	if ((mm = mask&MASK_D) != 0 && isValidMask(mm, 12)){
		vmask |= mm;
	}
	if ((mm = mask&MASK_C) != 0 && isValidMask(mm, 8)){
		vmask |= mm;
	}       
	if ((mm = mask&MASK_B) != 0 && isValidMask(mm, 4)){
		vmask |= mm;
	}
	if ((mm = mask&MASK_A) != 0 && isValidMask(mm, 0)){
		vmask |= mm;
	}

	dbg(1, "mask %08x vmask %08x %s", mask, vmask, mask==vmask?"OK":"ERR");

	if (vmask == mask){
		for (lchan = mm = 1; mm; mm<<=1, lchan++){
			acq200_setChannelEnabled(
				acq200_lookup_pchan(lchan), (mm&vmask) != 0);
		}

		CAPDEF_set_nchan(count_bits(vmask));
		CAPDEF->channel_mask = vmask;
		acq132_set_channel_mask(vmask);
	}else{
		err("mask not valid: 0x%08x good=0x%08x", mask, vmask);
	}
}



static ssize_t show_daq_enable(
	struct device_driver * driver, char * buf)
{
	int enable = (acq196_getSyscon()&ACQ196_SYSCON_ACQEN) != 0;

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
#define SPACER_ENTRY	  { "|\n", 0 }
#define REGS_LUT_ENTRY(n) { #n, n }

#define ADC_REGS_ENTRY(dev, cmask)		\
	BANK_ENTRY(dev, cmask),			\
	REGS_LUT_ENTRY(ACQ132_ADC_VERID(dev)),	\
	REGS_LUT_ENTRY(ACQ132_ADC_CTRL(dev)),	\
	REGS_LUT_ENTRY(ACQ132_ADC_RANGE(dev)),	\
	REGS_LUT_ENTRY(ACQ132_ADC_OSAM(dev)),   \
	REGS_LUT_ENTRY(ACQ132_ADC_FIFSTA(dev)),	\
	REGS_LUT_ENTRY(ACQ132_ADC_DR(dev)),     \
	REGS_LUT_ENTRY(ACQ132_ADC_TESTM(dev)),	\
	SPACER_ENTRY

static struct REGS_LUT {
	const char *name;
	volatile u32* preg;
}
	regs_lut[] = {
		REGS_LUT_ENTRY(ACQ132_BDR),
		REGS_LUT_ENTRY(ACQ196_FIFCON),
		REGS_LUT_ENTRY(ACQ132_FIFSTAT),
		REGS_LUT_ENTRY(ACQ132_SYSCON),
		REGS_LUT_ENTRY(ACQ132_SFPGA_CONF),
		REGS_LUT_ENTRY(ACQ132_ICS527),
		REGS_LUT_ENTRY(ACQ132_CLKDAT),
		REGS_LUT_ENTRY(ACQ132_CLK_COUNTER),
		REGS_LUT_ENTRY(ACQ132_SCAN_LIST_DEF),
		REGS_LUT_ENTRY(ACQ132_SCAN_LIST_LEN),
		SPACER_ENTRY,
		ADC_REGS_ENTRY(BANK_A, MASK_A),
		ADC_REGS_ENTRY(BANK_B, MASK_B),
		ADC_REGS_ENTRY(BANK_C, MASK_C),
		ADC_REGS_ENTRY(BANK_D, MASK_D),
		REGS_LUT_ENTRY(ACQ196_CLKCON),
		REGS_LUT_ENTRY(ACQ200_CLKDAT),
		REGS_LUT_ENTRY(ACQ200_DIOCON),
		REGS_LUT_ENTRY(ACQ196_OFFSET_DACS),
		REGS_LUT_ENTRY(ACQ196_WAVLIMIT),
		REGS_LUT_ENTRY(ACQ196_TCR_IMMEDIATE),
		REGS_LUT_ENTRY(ACQ196_TCR_LATCH)
	};

#define REGS_LUT_ENTRIES (sizeof(regs_lut)/sizeof (struct REGS_LUT))


int acq200_dumpregs_diag(char* buf, int len)
/** @todo still needed elsewhere, unfortunately */
{
	char *bp = buf;
#define APPEND(reg) \
        bp += snprintf(bp, len - (bp-buf), "%20s:[%02x] 0x%08X\n", \
                   #reg, ((unsigned)reg- (unsigned)ACQ200_FPGA), *reg)


#define APPEND_ADC(dev)				\
	APPEND(ACQ132_ADC_VERID(dev));		\
	APPEND(ACQ132_ADC_CTRL(dev));		\
	APPEND(ACQ132_ADC_RANGE(dev));		\
	APPEND(ACQ132_ADC_OSAM(dev));		\
	APPEND(ACQ132_ADC_FIFSTA(dev));		\
	APPEND(ACQ132_ADC_DR(dev));		\
	APPEND(ACQ132_ADC_TESTM(dev))		\

	APPEND(ACQ132_BDR);
	APPEND(ACQ196_FIFCON);
	APPEND(ACQ132_FIFSTAT);
	APPEND(ACQ132_SFPGA_CONF);
	APPEND(ACQ132_ICS527);
	APPEND(ACQ132_CLKDAT),
	APPEND(ACQ132_CLK_COUNTER),
	APPEND(ACQ132_SCAN_LIST_DEF);
	APPEND(ACQ132_SCAN_LIST_LEN);
	APPEND_ADC(BANK_A);
	APPEND_ADC(BANK_B);
	APPEND_ADC(BANK_C);
	APPEND_ADC(BANK_D);

	APPEND(ACQ196_CLKCON);
	APPEND(ACQ200_CLKDAT);
	APPEND(ACQ200_DIOCON);
	APPEND(ACQ196_OFFSET_DACS);
	APPEND(ACQ196_WAVLIMIT);
	APPEND(ACQ196_TCR_IMMEDIATE);
	APPEND(ACQ196_TCR_LATCH);

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

static void acq132_create_proc_entries(struct proc_dir_entry* root)
{	
        struct proc_dir_entry *dump_regs_entry =
		create_proc_entry("dump_regs", S_IRUGO, root);

	if (dump_regs_entry){
	        dump_regs_entry->proc_fops = &dump_regs_proc_fops;
	}
}
