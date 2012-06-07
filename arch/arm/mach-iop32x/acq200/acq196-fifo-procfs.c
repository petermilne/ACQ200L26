/* ------------------------------------------------------------------------- */
/* acq196-fifo-procfs.c                                                      */
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
#define ACQ196

#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"
#include "acq200-fifo.h"
#include "acq196.h"

#include "iop321-auxtimer.h"

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

static void acq196_mk_dev_sysfs(struct device *dev);

#define DEVICE_MK_DEV_SYSFS(dev) acq196_mk_dev_sysfs(dev)
#ifdef ACQ196T
#define DEVICE_CREATE_PROC_ENTRIES(root) pbc_create_proc_entries(root)
#else
#define DEVICE_CREATE_PROC_ENTRIES(root) 
#endif


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



static ssize_t show_RGM(
	struct device * dev,
	struct device_attribute *attr,
	char *buf)
{
	static const char* modes[] = {
		[ACQ196_RGATE_MODE_OFF] = "OFF",
		[ACQ196_RGATE_MODE_TRAN] = "TRAN",
		[ACQ196_RGATE_MODE_GATE] = "GATE",
		[ACQ196_RGATE_MODE_COMB] = "TRAN+GATE"
	};
	u32 rgm = *ACQ196_RGATE;
	int tlen = rgm&ACQ196_RGATE_TLEN;
	int mode = (rgm&ACQ196_RGATE_MODE)>>ACQ196_RGATE_MODE_SHL;

	switch (mode){
	case ACQ196_RGATE_MODE_TRAN:
	case ACQ196_RGATE_MODE_COMB:
		break;
	case ACQ196_RGATE_MODE_OFF:
	case ACQ196_RGATE_MODE_GATE:
	default:
		tlen = 0;
		break;
	}
	return sprintf(buf, "%d %d %s\n", mode, tlen, modes[mode]);		
}

static ssize_t store_RGM(
	struct device * dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned tlen = 0;
	unsigned mode;

	if (sscanf(buf, "%u %u", &mode, &tlen) > 0 &&
	    mode <= ACQ196_RGATE_MODE_COMB &&
	    tlen <= ACQ196_RGATE_TLEN){
		*ACQ196_RGATE = (mode << ACQ196_RGATE_MODE_SHL)|tlen;
		return count;
	}else{
		return -EPERM;
	}
}

static DEVICE_ATTR(RepeatingGateMode, S_IRUGO|S_IWUGO, show_RGM, store_RGM);

static ssize_t show_FSF(
	struct device * dev,
	struct device_attribute *attr,
	char *buf)
{
	u32 syscon = *ACQ196_SYSCON_ADC;
	int nacc = (syscon&ACQ196_SYSCON_FSF_ACC) >> ACQ196_SYSCON_FSF_ACC_BIT;
	int shr = (syscon&ACQ196_SYSCON_FSF_SHR);

	if (nacc != 0){
		nacc += 1;
		switch(shr){
		case ACQ196_SYSCON_FSF_SHR_2:	shr = 2; break;
		case ACQ196_SYSCON_FSF_SHR_3:	shr = 3; break;
		case ACQ196_SYSCON_FSF_SHR_4:	shr = 4; break;
		case ACQ196_SYSCON_FSF_SHR_5:	shr = 5; break;
		}
	}else{
		shr = 0;
	}


	return sprintf(buf, "%u %u\n", nacc, shr);
}

#define AUTO_SHR	-37

int acq196_set_fsf(unsigned nacc, unsigned shr)
{
	u32 syscon = *ACQ196_SYSCON_ADC;
	syscon &= ~ (ACQ196_SYSCON_FSF_ACC|ACQ196_SYSCON_FSF_SHR);

	if (nacc != 0){
		u32 nacc_bits;
		u32 shr_bits;

		if (nacc < 1 || nacc > 32){
			err("nacc %u out of range 1..32", nacc);
			return -1;
		}
		nacc_bits = (nacc-1) << ACQ196_SYSCON_FSF_ACC_BIT;
		if (shr == AUTO_SHR){
			if (nacc > 16){
				shr = 5;
			}else if (nacc > 8){
				shr = 4;
			}else if (nacc > 4){
				shr = 3;
			}else{
				shr = 2;
			}
		}
		switch(shr){
		case 5:	shr_bits = ACQ196_SYSCON_FSF_SHR_5; break;
		case 4: shr_bits = ACQ196_SYSCON_FSF_SHR_4; break;
		case 3: shr_bits = ACQ196_SYSCON_FSF_SHR_3; break;
		case 2: shr_bits = ACQ196_SYSCON_FSF_SHR_2; break;
		default:
			err("shr %u out of range 2..5", shr);
			return -2;
		}
		syscon |= nacc_bits | shr_bits;
	}

	*ACQ196_SYSCON_ADC = syscon;
	return 0;
}
static ssize_t store_FSF(
	struct device * dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned nacc;
	unsigned shr = AUTO_SHR;
	int rc = strlen(buf);

	switch (sscanf(buf, "%u %u", &nacc, &shr)){
	case 1:
	case 2:
		if (nacc == 0 || nacc == 1){
			acq196_set_fsf(0, 0);
		}else{
			if (acq196_set_fsf(nacc, shr)){
				rc = -EPERM;
			}
		}
		break;
	default:
		rc = -EPERM;
	}

	return rc;
}

static DEVICE_ATTR(FS_Filter, S_IRUGO|S_IWUGO, show_FSF, store_FSF);



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

#if defined (ACQ196F) || defined(ACQ196M)
static ssize_t show_FIRK(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf, int bank)
{
	u32 firk = *ACQ196_FIRK;

        return sprintf(buf,"%d\n", (firk >> (bank*8)) & 0x7);
}

static ssize_t store_FIRK(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count, int bank)
{
	u32 kk;

	if (sscanf(buf, "%d", &kk)){
		u32 firk = *ACQ196_FIRK;

		firk &= ~ (0x7 << (bank*8));
		firk |= (kk&0x7) << (bank*8);

		*ACQ196_FIRK = firk;
	}
	return strlen(buf);
}

#define DECL_FIRQ(bank)							\
static ssize_t show_FIRK##bank(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf) {							\
	return show_FIRK(dev, attr, buf, bank-1);				\
}									\
static ssize_t store_FIRK##bank(					\
	struct device* dev,						\
	struct device_attribute *attr,					\
	const char * buf, size_t count) {				\
	return store_FIRK(dev, attr, buf, count, bank-1);			\
}									\
static DEVICE_ATTR(							\
	FIR_MODE_##bank, S_IRUGO|S_IWUGO, show_FIRK##bank, store_FIRK##bank \
);

#define DECL_FIRK_GROUP DECL_FIRQ(1); DECL_FIRQ(2); DECL_FIRQ(3)

#define DEVICE_CREATE_FIRK(dev, bank) \
        DEVICE_CREATE_FILE(dev, &dev_attr_FIR_MODE_##bank)

#define DEVICE_CREATE_FIRK_GROUP(dev) do {	\
	DEVICE_CREATE_FIRK(dev, 1);		\
	DEVICE_CREATE_FIRK(dev, 2);		\
	DEVICE_CREATE_FIRK(dev, 3);		\
} while(0)

DECL_FIRK_GROUP;
#endif

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
	if ((CAPDEF->channel_mask&4)){
		len += build_channel_mapping(64, buf+len);
	}
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
	if ((CAPDEF->channel_mask&1)){
		len += build_channel_mapping_bin( 0, buf+len);
	}
	if ((CAPDEF->channel_mask&2)){
		len += build_channel_mapping_bin(32, buf+len);
	}
	if ((CAPDEF->channel_mask&4)){
		len += build_channel_mapping_bin(64, buf+len);
	}
	return len;
}
static DEVICE_ATTR(channel_mapping_bin, S_IRUGO, show_channel_mapping_bin, 0);



static ssize_t show_soft_fifo(
	struct device * dev,
	struct device_attribute *attr,
	char *buf)
{
	u32 fifcon = *ACQ196_FIFCON;

	sprintf(buf, "%d\n", (fifcon&ACQ196_FIFCON_SOFT_OFLOW) != 0);
	return strlen(buf);

}

extern void init_arbiter(void);

static void soft_fifo_tweak_arbiter(int enable)
{
	if (enable){
		*IOP321_IACR  = 0x00000082;/* CORE=HI DMA0=HI DMA1=LO ATU=LO */
		*IOP321_MTTR1 = 0x40;      /* give FIQ a chance */
		*IOP321_MTTR2 = 0x20;	   /* want low lat on DMA slice */
	}else{
		init_arbiter();
	}
}
static ssize_t store_soft_fifo(
	struct device * dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int enable = 0;

	if (sscanf(buf, "%d", &enable) == 1){
		u32 fifcon = *ACQ196_FIFCON;

		if (enable){
			fifcon |= ACQ196_FIFCON_SOFT_OFLOW;
		}else{
			fifcon &= ~ACQ196_FIFCON_SOFT_OFLOW;
		}
		*ACQ196_FIFCON = fifcon;

		soft_fifo_tweak_arbiter(enable);
		return count;
	}
	return -1;
}

static DEVICE_ATTR(soft_fifo, S_IRUGO|S_IWUGO, show_soft_fifo, store_soft_fifo);


/* hack delta counting - this should really be a separate device
 * with a per-path variable (how many folks are polling)
 * This is quick 'n dirty, chances are only one poller anyway
 */
static ssize_t show_soft_fifo_count(
	struct device * dev,
	struct device_attribute *attr,
	char *buf)
{
	struct timeval ts;
	static int count0;
	static long last_nz;
	int count = acq196_get_soft_fifo_count();
	int dc;
	long secs;

	if (likely(count >= count0)){
		dc = count - count0;
	}else{
		dc = 0x10000 - count0 + count;
	}

	DG->stats.total_soft_fifo_count += dc;
	
	do_gettimeofday(&ts);
	secs = ts.tv_sec % (3600*24);
	sprintf(buf, "%6ld %10ld %6d\n", 
		secs, DG->stats.total_soft_fifo_count, dc);

	last_nz = secs;
	count0 = count;

	return strlen(buf);
}

static DEVICE_ATTR(soft_fifo_count, S_IRUGO, show_soft_fifo_count, 0);





DEFINE_SIGNAL_ATTR(ao_trig);
DEFINE_SIGNAL_ATTR(ao_clk);

DEFINE_SIGNAL_ATTR(sync_trig_src);
DEFINE_SIGNAL_ATTR(sync_trig_mas);

#if defined(ACQ196M)
#warning this is ACQ196M
#endif

static void acq196_mk_dev_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_coding);
	DEVICE_CREATE_FILE(dev, &dev_attr_FAWG_div);
	DEVICE_CREATE_FILE(dev, &dev_attr_slow_clock);
	DEVICE_CREATE_FILE(dev, &dev_attr_AO_coding);
	DEVICE_CREATE_FILE(dev, &dev_attr_AO_clock_mode);
	DEVICE_CREATE_FILE(dev, &dev_attr_event1);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mapping);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mapping_bin);
	DEVICE_CREATE_FILE(dev, &dev_attr_soft_fifo);
	DEVICE_CREATE_FILE(dev, &dev_attr_soft_fifo_count);
	DEVICE_CREATE_FILE(dev, &dev_attr_ao_trig);
	DEVICE_CREATE_FILE(dev, &dev_attr_ao_clk);
	DEVICE_CREATE_FILE(dev, &dev_attr_sync_trig_src);
	DEVICE_CREATE_FILE(dev, &dev_attr_sync_trig_mas);
#if defined (ACQ196F) || defined (ACQ196M)
#warning FIRK enabled
	DEVICE_CREATE_FIRK_GROUP(dev);
#endif
	DEVICE_CREATE_FILE(dev, &dev_attr_RepeatingGateMode);
	DEVICE_CREATE_FILE(dev, &dev_attr_FS_Filter);
}

void acq200_setChannelMask(unsigned mask)
{
	int nblocks;
	unsigned block;

	mask = mask&0x7;      /* actually a bank mask */
	CAPDEF->channel_mask = mask;


	for (block = 0; block < 3; block++){
		int ic1 = 1 + block*32;
		int ic2 = ic1 + 32;
		int icc;
		for (icc = ic1; icc < ic2; ++icc){
			acq200_setChannelEnabled(
				acq200_lookup_pchan(icc), 
					(mask&(1<<block)) != 0);
		}
	}

	for (nblocks = 0; mask; mask >>= 1){
		if (mask&1){
			++nblocks;
		}
	}
	CAPDEF_set_nchan(nblocks * 32);
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

int acq200_dumpregs_diag(char* buf, int len)
{
	char *bp = buf;
#define APPEND(reg) \
        bp += snprintf(bp, len - (bp-buf), "%20s:[%02x] 0x%08X\n", \
                   #reg, ((unsigned)reg- (unsigned)ACQ200_FPGA), *reg)
	
	APPEND(ACQ196_BDR);
	APPEND(ACQ196_FIFCON);
	APPEND(ACQ196_FIFSTAT);
	APPEND(ACQ196_SYSCON_ADC);
	APPEND(ACQ196_SYSCON_DAC);

	APPEND(ACQ196_CLKCON);
	APPEND(ACQ200_CLKDAT);
	APPEND(ACQ200_DIOCON);
	APPEND(ACQ196_OFFSET_DACS);
	APPEND(ACQ196_WAVLIMIT);
	APPEND(ACQ196_PGCSS);
	APPEND(ACQ196_PGTIMER);
	APPEND(ACQ196_TCR_IMMEDIATE);
	APPEND(ACQ196_TCR_LATCH);
	APPEND(ACQ200_ICR);

	return bp-buf;
}



static int acq200_proc_dumpregs(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	return acq200_dumpregs_diag(buf, len);
}
