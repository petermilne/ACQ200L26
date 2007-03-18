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




#define DTACQ_MACH 1
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
#define DEVICE_CREATE_PROC_ENTRIES(root) 



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

#ifdef ACQ196F
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
	return show_FIRK(dev, attr, buf, bank);				\
}									\
static ssize_t store_FIRK##bank(					\
	struct device* dev,						\
	struct device_attribute *attr,					\
	const char * buf, size_t count) {				\
	return store_FIRK(dev, attr, buf, count, bank);			\
}									\
static DEVICE_ATTR(							\
	FIRK##bank, S_IRUGO|S_IWUGO, show_FIRK##bank, store_FIRK##bank	\
);

#define DECL_FIRK_GROUP DECL_FIRQ(0); DECL_FIRQ(1); DECL_FIRQ(2)

#define DEVICE_CREATE_FIRK(dev, bank) \
        device_create_file(dev, &dev_attr_FIRK##bank)

#define DEVICE_CREATE_FIRK_GROUP(dev) do {	\
	DEVICE_CREATE_FIRK(dev, 0);		\
	DEVICE_CREATE_FIRK(dev, 1);		\
	DEVICE_CREATE_FIRK(dev, 2);		\
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




DEFINE_SIGNAL_ATTR(ao_trig);
DEFINE_SIGNAL_ATTR(ao_clk);

DEFINE_SIGNAL_ATTR(sync_trig_src);
DEFINE_SIGNAL_ATTR(sync_trig_mas);


static void acq196_mk_dev_sysfs(struct device *dev)
{
	device_create_file(dev, &dev_attr_coding);
	device_create_file(dev, &dev_attr_FAWG_div);
	device_create_file(dev, &dev_attr_slow_clock);
	device_create_file(dev, &dev_attr_AO_coding);
	device_create_file(dev, &dev_attr_AO_clock_mode);
	device_create_file(dev, &dev_attr_event1);
	device_create_file(dev, &dev_attr_channel_mapping);
	device_create_file(dev, &dev_attr_channel_mapping_bin);
	device_create_file(dev, &dev_attr_ao_trig);
	device_create_file(dev, &dev_attr_ao_clk);
	device_create_file(dev, &dev_attr_sync_trig_src);
	device_create_file(dev, &dev_attr_sync_trig_mas);
#ifdef ACQ196F
	DEVICE_CREATE_FIRK_GROUP(dev);
#endif
}

void acq200_setChannelMask(unsigned mask)
{
	int nblocks;

	mask = mask&0x7;      /* actually a bank mask */
	CAPDEF->channel_mask = mask;

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
	APPEND(ACQ196_TCR_IMMEDIATE);
	APPEND(ACQ196_TCR_LATCH);

	return bp-buf;
}



static int acq200_proc_dumpregs(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	return acq200_dumpregs_diag(buf, len);
}
