/* ------------------------------------------------------------------------- */
/* acq200-fifo-procfs.c driver for acq200 fifo streaming                     */
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


/*
 * this unit ACQ200 generic.
 * only FPGA REG is ACQ200_DIOCON which is generic.
 */
#ifndef DTACQ_MACH
#error DTACQ_MACH NOT DEFINED. PLease do not compile directly
#endif
#define VERID \
"$Id: acq200-fifo-procfs.c,v 1.27 2006/10/04 09:07:43 pgm Exp $\n"\
"Build 1079 " __DATE__ " "__TIME__

#include <linux/seq_file.h>

#include "acq200-fifo-tblock.h"
#include "dio_defs.h"

#include <asm-arm/arch-iop32x/acq200.h>


#define GTSR_COUNT_US  50
#define GTSR_TCOUNT_US ((0x1000000/GTSR_COUNT_US)*0x100)

static int calc_process_us(void)
{
	int start_us = DG->stats.start_gtsr/GTSR_COUNT_US;
	int end_us   = DG->stats.end_gtsr/GTSR_COUNT_US;
	int proc_us = end_us - start_us;

	if ( proc_us < 0 ){
		proc_us += GTSR_TCOUNT_US;
	}
	return proc_us;
}



#ifndef ARCH_FIFERR_DESCRIPTION
#define ARCH_FIFERR_DESCRIPTION(flags) "..."
#endif


static inline unsigned long long elapsed_samples(void) 
{
	unsigned long spb = DMA_BLOCK_LEN/sample_size();
	return DG->stats.refill_blocks * spb;
}

#define ELAPSED_SAMPLES elapsed_samples()

static ssize_t show_signal(
	struct Signal* signal, struct device * dev, char * buf)
{
	if (!signal){
		err("ERROR: null signal");
		return -ENODEV;
	}
	switch(signal->DIx){
	case DIX_NONE:
		return sprintf(buf, "%s none\n", signal->name);
	case DIX_INTERNAL:	
		return sprintf(buf, "%s internal %s\n",
			       signal->name,
			       signal->is_active?"ACTIVE":"inactive");
	default:
		return sprintf(buf, "%s D%c%d %s %s\n",
			       signal->name, 
			       signal->is_output? 'O': 'I',
/* frig DIx for case of phys bits already set (ob_clk_src) gimme C++ */
			       signal->DIx <= 16? signal->DIx: 16,
			       signal->is_output? "":
			               signal->rising? "rising": "falling",
			       signal->is_active? "ACTIVE":"inactive");
	}
}


static ssize_t store_signal(
	struct Signal* signal,
	struct device * dev, const char * _buf, size_t count)
{
	char buf[80];
	char name[64];
	char io;
	int xx;
	char edge[64];
	int ok = __LINE__;
	int nc;
/**
 * debug parsing - sets ok to line# on fail, second cond ALWAYS fails
 * resulting in drop out.
 *
 */
#define OKL(cond) ((cond) || !(ok = __LINE__))
#define OKOK (ok = 0)

	if (!signal){
		err("ERROR: null signal");
		return -ENODEV;
	}

	buf[79] = '\0';
	strncpy(buf, _buf, min((int)count,78));
	if (strchr(buf, '\n')){
		*strchr(buf, '\n') = '\0';
	}

	if (signal->has_internal_option &&
	    OKL(strcmp(name, signal->name) == 0) && 
	    OKL(strcmp(edge, "internal") == 0)){
		if (OKL(setSignal(signal, DIX_INTERNAL, 1) == 1)){
		    activateSignal(signal);
		    OKOK;
		}
	}else if (OKL(sscanf(buf, "%s %s", name, edge) == 2) && 
	    OKL(strcmp(name, signal->name) == 0) && 
	    OKL(strcmp(edge, "none") == 0)){
		deactivateSignal(signal);
		setSignal(signal, DIX_NONE, 0);
		OKOK;
	}else if (OKL((nc=sscanf(buf,"%s D%c%d %s",name,&io,&xx,edge)) >= 3) &&
		  OKL(strcmp(name, signal->name) == 0) &&
		  OKL((signal->is_output == 1 && io == 'O') || 
		      (signal->is_output == 0 && io == 'I')   ) ){

		if ((nc == 3 || OKL(strcmp(edge, "falling") == 0)) &&
		     OKL(setSignal(signal, xx, 0) == 0)                ){
			activateSignal(signal);
			OKOK;
		}else if (OKL(strcmp(edge, "rising") == 0)  &&
  		    OKL(setSignal(signal, xx, 1) == 0)   ){
			activateSignal(signal);
			OKOK;
		}

		dbg(1, "nc:%d name \"%s\" D%c%d edge \"%s\"", 
		    nc, name, io, xx, edge);
	}

	if (ok != 0){
		err("signal %s input validation failed %d \"%s\"", 
		    signal->name, ok, buf);
	}
	return count;	  
}

#ifndef WAV232

#define DEFINE_EVENT_ATTR(evnum)					\
static ssize_t show_event##evnum(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_signal(CAPDEF->ev[evnum], dev, buf);		\
}									\
									\
static ssize_t store_event##evnum(					\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf, size_t count)					\
{									\
	return store_signal(CAPDEF->ev[evnum], dev, buf, count);	\
}									\
static DEVICE_ATTR(event##evnum, S_IRUGO|S_IWUGO,			\
		   show_event##evnum, store_event##evnum)

DEFINE_EVENT_ATTR(0);

#endif



#define DEFINE_SIGNAL_ATTR(signal)					\
static ssize_t show_##signal(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return show_signal(CAPDEF->signal, dev, buf);			\
}									\
									\
static ssize_t store_##signal(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf, size_t count)					\
{									\
	return store_signal(CAPDEF->signal, dev, buf, count);		\
}									\
static DEVICE_ATTR(signal, S_IRUGO|S_IWUGO, show_##signal, store_##signal)



DEFINE_SIGNAL_ATTR(trig);
DEFINE_SIGNAL_ATTR(ext_clk);
#ifndef WAV232
DEFINE_SIGNAL_ATTR(int_clk_src);
DEFINE_SIGNAL_ATTR(counter_src);

static ssize_t show_counter_update(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"counter_update %d %s\n", 
		       CAPDEF->counter_update,
		       CAPDEF->counter_update==1? "event0":
		       CAPDEF->counter_update==2? "event1": "sample");
}

#if defined(ACQ196) || defined (ACQ132)
static ssize_t store_counter_update(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (sscanf(buf, "%u", &CAPDEF->counter_update) ||
	    sscanf(buf, "counter_update %u", &CAPDEF->counter_update)){
		u32 tcr = *ACQ216_TCR_IMM;
		tcr &= ~ACQ216_TCR_UPDCTRL;
		tcr |= (CAPDEF->counter_update << ACQ216_TCR_UPDCTRL_SHL) & 
			ACQ216_TCR_UPDCTRL;
		*ACQ216_TCR_IMM = tcr;
	}
	return strlen(buf);
}
#endif
#if defined(ACQ216)
static ssize_t store_counter_update(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (sscanf(buf, "%u", &CAPDEF->counter_update) ||
	    sscanf(buf, "counter_update %u", &CAPDEF->counter_update)){
		u32 tcr = *ACQ216_CCT_CON;
		tcr &= ~ACQ216_CCT_UPDCTRL;
		tcr |= (CAPDEF->counter_update << ACQ216_CCT_UPDCTRL_SHL) & 
			ACQ216_CCT_UPDCTRL;
		*ACQ216_CCT_CON = tcr;
	}
	return strlen(buf);
}
#endif
static DEVICE_ATTR(counter_update, S_IRUGO|S_IWUGO,
		   show_counter_update, store_counter_update);


#endif

DEFINE_SIGNAL_ATTR(mas_clk);




#define DEFINE_COUNT_ATTR(ctr)						\
static ssize_t show_##ctr(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	char * buf)							\
{									\
	return sprintf(buf, "%u\n", DMC_WO->ctr);			\
}									\
static ssize_t clear_##ctr(						\
	struct device * dev,						\
	struct device_attribute *attr,					\
	const char * buf, size_t count)					\
{									\
	DMC_WO->ctr = 0;						\
	return count;							\
}									\
static DEVICE_ATTR(ctr, S_IRUGO|S_IWUGO, show_##ctr, clear_##ctr)

DEFINE_COUNT_ATTR(clock_count_immediate);
DEFINE_COUNT_ATTR(clock_count_latched);

static int rb_entries(struct acq200_dma_ring_buffer *rb)
/* WARNING: SLOW! */
{
	int entries = 0;
	int itemp;

	for (itemp = rb->iget; itemp != rb->iput; ++entries ){
		itemp = RB_INCR(itemp);
	}

	return entries;
}






static void blt_null(short *to, short *from, int nshorts)
/* block transfer from to, assume DMAC does this out of line */
{

}


static void on_personality_change(void)
{
	acq200_init_tblock_list();
}


static unsigned getChannelMask(void)
{
	return CAPDEF->channel_mask;
}




static ssize_t show_activate_event_on_arm(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", DG->activate_event_on_arm);
}

static ssize_t store_activate_event_on_arm(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &DG->activate_event_on_arm);
	return strlen(buf);
}
static DEVICE_ATTR(activate_event_on_arm, S_IRUGO|S_IWUGO,
		   show_activate_event_on_arm, 
		   store_activate_event_on_arm);

static ssize_t show_stub_live_copy_to_user(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", DG->stub_live_copy_to_user);
}

static ssize_t store_stub_live_copy_to_user(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &DG->stub_live_copy_to_user);
	return strlen(buf);
}
static DEVICE_ATTR(stub_live_copy_to_user, S_IRUGO|S_IWUGO,
		   show_stub_live_copy_to_user, 
		   store_stub_live_copy_to_user);

static ssize_t show_sample_read_start(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", DG->sample_read_start);
}

static ssize_t store_sample_read_start(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%u", &DG->sample_read_start);
	return strlen(buf);
}
static DEVICE_ATTR(sample_read_start, S_IRUGO|S_IWUGO,
		   show_sample_read_start, store_sample_read_start);


static ssize_t show_sample_read_length(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", DG->sample_read_length);
}

static ssize_t store_sample_read_length(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%u", &DG->sample_read_length);
	return strlen(buf);
}
static DEVICE_ATTR(sample_read_length, S_IRUGO|S_IWUGO,
		   show_sample_read_length, store_sample_read_length);


static ssize_t show_sample_read_ssl(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d %d %d\n", 
		       DG->sample_read_start,
		       DG->sample_read_stride,
		       DG->sample_read_length);
}

static ssize_t store_sample_read_ssl(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (strstr(buf, "reset") != 0){
		DG->sample_read_start = 0;
		DG->sample_read_stride = 1;
		DG->sample_read_length = 0;
	}else{
		unsigned start, stride, length;

		if (sscanf(buf, "%u %u %u", &start, &stride, &length) == 3){
			DG->sample_read_start = start;
			DG->sample_read_stride = stride;
			DG->sample_read_length = length;
		}
	}
	return strlen(buf);
}
static DEVICE_ATTR(sample_read_ssl, S_IRUGO|S_IWUGO,
		   show_sample_read_ssl, store_sample_read_ssl);


static ssize_t show_show_event(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", DG->show_event);
}

static ssize_t store_show_event(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%u", &DG->show_event);
	return strlen(buf);
}
static DEVICE_ATTR(show_event, S_IRUGO|S_IWUGO,
		   show_show_event, store_show_event);




static ssize_t show_burst_delay(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", DG->burst.delay);
}

static ssize_t store_burst_delay(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int adelay;

	if (sscanf(buf, "m%d", &adelay)){
		adelay = DG->burst.delay - adelay;
	}else if (sscanf(buf, "p%d", &adelay)){
		adelay = DG->burst.delay + adelay;
	}else if (sscanf(buf, "%d", &adelay)){
		;
	}else{
		return strlen(buf);
	}

	adelay = max(adelay, -1);
	adelay = min(adelay, 8192);			
	DG->burst.delay = adelay;

	return strlen(buf);
}
static DEVICE_ATTR(burst_delay, S_IRUGO|S_IWUGO,
		   show_burst_delay, store_burst_delay);

#ifndef WAV232
static ssize_t show_dcb_max_backlog(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%d\n", DG->dcb.dcb_max_backlog);
}

static ssize_t store_dcb_max_backlog(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int blog;

	if (sscanf(buf, "%d", &blog) == 1){
		if (blog > 0 && blog <= DG->dcb.dcb_max){
			DG->dcb.dcb_max_backlog = blog;
		}
	}
	return strlen(buf);
}

static DEVICE_ATTR(dcb_max_backlog, S_IRUGO|S_IWUGO,
		   show_dcb_max_backlog, store_dcb_max_backlog);
#endif

static ssize_t show_burst_es_channel(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", DG->burst.es_channel);
}

static ssize_t store_burst_es_channel(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int achannel;

	if (sscanf(buf, "%d", &achannel)){
		achannel = max(achannel, 0);
		achannel = min(achannel, 96);
			
		DG->burst.es_channel = achannel;
	}
	return strlen(buf);
}
static DEVICE_ATTR(burst_es_channel, S_IRUGO|S_IWUGO,
		   show_burst_es_channel, store_burst_es_channel);


static ssize_t show_burst_len(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", DG->burst.len);
}

static ssize_t store_burst_len(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
/* ENSURE MULTIPLE of 64 */
{
	unsigned bl;

	if (sscanf(buf, "%u", &bl)){
		unsigned bl64 = bl/64;
		bl64 = bl64 * 64;
		if (bl64 < bl){
			bl64 += 64;
		}

		DG->burst.len = bl64;
	}

	return strlen(buf);
}
static DEVICE_ATTR(burst_len, S_IRUGO|S_IWUGO,
		   show_burst_len, store_burst_len);







static ssize_t show_pit_stop(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", CAPDEF->pit_stop);	
}

static ssize_t store_pit_stop(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int pit_stop;

	if (sscanf(buf, "%d", &pit_stop) == 1){
		if (pit_stop < 0){
			pit_stop = 0;
		}else if (pit_stop > DG->pit_store.max_pits){
			pit_stop = DG->pit_store.max_pits;
		}
		CAPDEF->pit_stop = pit_stop;
	}
	return strlen(buf);
}

static DEVICE_ATTR(pit_stop, S_IRUGO|S_IWUGO, show_pit_stop, store_pit_stop);

static ssize_t show_sample_read_stride(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%u\n", DG->sample_read_stride);
}

static ssize_t store_sample_read_stride(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%u", &DG->sample_read_stride);
	if (DG->sample_read_stride < 1){
		DG->sample_read_stride = DG->sample_read_stride;
	}
	return strlen(buf);
}
static DEVICE_ATTR(sample_read_stride, S_IRUGO|S_IWUGO,
		   show_sample_read_stride, store_sample_read_stride);





static ssize_t show_pulse_def(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"c=%-2d ms=%-4d bit=%d active_high=%d delay=%d\n", 
		       DG->pulse.pulse_count, DG->pulse.delay_ms,
		       DG->pulse.ibit, DG->pulse.active_high,
		       DG->pulse.start_delay);
}

static ssize_t store_pulse_def(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	struct PulseDef pd = DG->pulse;
	int nfields = sscanf(buf, "c=%d ms=%d bit=%d active_high=%d delay=%d", 
			     &pd.pulse_count,
			     &pd.delay_ms, &pd.ibit, &pd.active_high,
			     &pd.start_delay);

	if (nfields >= 1){
		DG->pulse = pd;
	}
	return strlen(buf);
}
static DEVICE_ATTR(pulse_def, S_IRUGO|S_IWUGO,
		   show_pulse_def, store_pulse_def);

/*
#include <asm/div64.h>
unsigned long long x, y, result;
unsigned long mod;
mod = do_div(x, y);
result = x;
*/

static int _show_measured_sample_rate(
	char *buf, 
	int maxbuf)
{
	int process_us = calc_process_us();

	if (process_us > 1000){
		int decimals = 
			process_us > 100000? 1000:
			process_us > 10000? 100: 10;
		unsigned long long tot_samples = ELAPSED_SAMPLES;
		unsigned long long xx = tot_samples * decimals;
		unsigned srate;

		do_div(xx, process_us);
		srate = xx;

		if (srate > decimals){
			return snprintf(buf, maxbuf, "%2d.%d MHz\n", 
						srate/decimals, srate%decimals);
		}else{
			process_us = calc_process_us();
			if (process_us > 1000000){
				process_us /= 1000;
				xx = tot_samples;
				do_div(xx, process_us);
				srate = xx;
			}else{
				xx = tot_samples * 1000;       /* result kHz */
				do_div(xx, process_us);
				srate = xx;
			}
			if (srate < 1000){
				return snprintf(buf, maxbuf, "%3d kHz\n", 
									srate);
			}
		}
	}

	return snprintf(buf, maxbuf, "---\n");
}
	

static ssize_t show_measured_sample_rate(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return _show_measured_sample_rate(buf, 128);
}

static DEVICE_ATTR(measured_sample_rate, S_IRUGO,
			 show_measured_sample_rate, 0);

void finish_with_engines(int ifinish); /* call when end detected */

static ssize_t show_user_abort(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"user_abort\n");
}

#define ABORT_MESSAGE "USER ABORT"
static ssize_t store_user_abort(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (strstr(buf, "abort")){
		DMC_WO->error = ABORT_MESSAGE;
		finish_with_engines(-__LINE__);
	}

	return strlen(buf);
}
static DEVICE_ATTR(user_abort, S_IRUGO|S_IWUGO,
		   show_user_abort, store_user_abort);

static ssize_t show_cap_status(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%s\n", 
		DMC_WO->error == 0? "OK": DMC_WO->error);
}

static DEVICE_ATTR(cap_status, S_IRUGO, show_cap_status, 0);

static int decodeBinaryMask(
	const char * buf, 
	unsigned* mask)
{
	int imask = 1;
	const char *bp;

	*mask = 0;

	for (bp = buf; *bp; ++bp, imask <<= 1){
		switch(bp[0]){
		case '1':
			*mask |= imask;
			break;
		case'0':
			break;
		default:
			return -1;
		}
	}

	return 0;
}
static ssize_t show_channel_mask(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"0x%08x\n", getChannelMask());
}


static ssize_t store_channel_mask(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	unsigned mask;

	if (sscanf(buf, "%x", &mask) || 
	    sscanf(buf, "0x%x", &mask) ||
		   decodeBinaryMask(buf, &mask) == 0 ){
		acq200_setChannelMask(mask);
	}
	return strlen(buf);
}
static DEVICE_ATTR(channel_mask, S_IRUGO|S_IWUGO,
		   show_channel_mask, store_channel_mask);


ssize_t show_event_magic(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"0x%08x\n", EVENT_MAGIC);
}

static DEVICE_ATTR(event_magic, S_IRUGO, show_event_magic, 0);


ssize_t show_event_magic_mask(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"0x%08x\n", EVENT_MAGIC_MASK);
}

static DEVICE_ATTR(event_magic_mask, S_IRUGO, show_event_magic_mask, 0);


static ssize_t acq200_store_state(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	unsigned state;

	if (sscanf(buf, "%d", &state)){
		switch(DMC_WO_getState()){
		case ST_STOP:
		case ST_CAPDONE:
			if (state == ST_POSTPROCESS || state == ST_STOP){
				DMC_WO_setState(state);
			}
			break;
		case ST_POSTPROCESS:
			if (state == ST_STOP){
				DMC_WO_setState(ST_STOP);
			}
			break;
		default:
			;
		}
	}
	return strlen(buf);
}

static ssize_t acq200_show_state(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", DMC_WO_getState());
}

static DEVICE_ATTR(state, S_IRUGO|S_IWUGO,
		   acq200_show_state, acq200_store_state);

#ifndef WAV232
static ssize_t acq200_show_scc(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%Ld\n", DMC_WO->scc.scc);
}

static DEVICE_ATTR(scc, S_IRUGO,  acq200_show_scc, 0);

static ssize_t acq200_show_ecc(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%Ld\n", DMC_WO->ecc.scc);
}

static DEVICE_ATTR(ecc, S_IRUGO,  acq200_show_ecc, 0);
#endif



static ssize_t show_mode(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
/* quick diagnostic hack */
#define MLUT_ENTRY(mode) #mode
	static char *MLUT[] = {
		MLUT_ENTRY(M_GATED_TRANSIENT),
		MLUT_ENTRY(M_GATED_CONTINUOUS),
		MLUT_ENTRY(M_SOFT_TRANSIENT),
		MLUT_ENTRY(M_SOFT_CONTINUOUS),
		MLUT_ENTRY(M_TRIGGERED_CONTINUOUS)
	};
	switch(CAPDEF->mode){
	case M_TRIGGERED_CONTINUOUS:
		return sprintf(buf, "%d %s %d %d\n",
			       CAPDEF->mode, MLUT[CAPDEF->mode],
			       CAPDEF->demand_prelen, CAPDEF->demand_postlen);
	case M_SOFT_CONTINUOUS:
		return sprintf(buf,"%d %s %d\n", 
			       CAPDEF->mode, MLUT[CAPDEF->mode],
				CAPDEF->demand_prelen);
	default:
		return sprintf(buf,"%d %s %d\n", 
			       CAPDEF->mode, MLUT[CAPDEF->mode],
				CAPDEF->demand_postlen);
	}

}



static ssize_t store_mode(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	unsigned mode;
	unsigned pre, post;
	int nconverted = sscanf(buf, "%u %u %u", &mode, &pre, &post);

	if (nconverted == 3 &&  mode <= M_LAST){

		unsigned prelen = samplesToBytes(pre);
		unsigned postlen = samplesToBytes(post);
		unsigned usable_len = len_buf(DG) - TBLOCK_LEN;
		

		postlen = min(postlen, usable_len);
		prelen = min(prelen, (usable_len - postlen));

		CAPDEF->mode = mode;
		CAPDEF->demand_prelen = prelen;
		CAPDEF->demand_postlen = postlen;

		/** only used for TRANSIENT see init_phases() */
		LEN = postlen;
	}
	return strlen(buf);
}
static DEVICE_ATTR(mode, S_IRUGO|S_IWUGO, show_mode, store_mode);



static inline unsigned DIO_SET_OUTPUT1(unsigned cc, int ib) 
{
	cc |= 1 <<(ib+ACQ200_DIOCON_SETOUT_SHL);
	cc |= 1 <<(ib+ACQ200_DIOCON_OUTDAT_SHL);

	return cc;
}
static inline unsigned DIO_SET_OUTPUT0(unsigned cc, int ib) 
{
	cc |= 1 <<(ib+ACQ200_DIOCON_SETOUT_SHL);
        cc &= ~(1 <<(ib+ACQ200_DIOCON_OUTDAT_SHL));

	return cc;
}
static inline unsigned DIO_SET_INPUT(unsigned cc, int ib)
{
	return cc &= ~(1 << (ib+ACQ200_DIOCON_SETOUT_SHL));
}
static inline int DIO_IS_OUTPUT(unsigned cc, int ib)
{
	return (cc & (1 << (ib+ACQ200_DIOCON_SETOUT_SHL))) != 0;
}
static inline int DIO_IS_OUTPUT1(unsigned cc, int ib)
{
	return (cc & (1 << (ib+ACQ200_DIOCON_OUTDAT_SHL))) != 0;
}
static inline int DIO_IS_INPUTH(unsigned cc, int ib)
{
	return (cc & (1 << (ib+ACQ200_DIOCON_INPDAT_SHL))) != 0;
}

#define MAXDIOBIT 6

/** @@todo this is a hack to circumvent duff dio readback */
static u32 control_mirror;

static char last_store[32];

static ssize_t store_dio_bit(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int ibit;
	char value;
	unsigned control = *ACQ200_DIOCON;

	if (sscanf(buf, "%d %c", &ibit, &value) == 2 ||
            sscanf(buf, "%d=%c", &ibit, &value) == 2    ){
		if (ibit >= 0 && ibit < MAXDIOBIT){
			switch(value){
			case DIO_MASK_OUTPUT1:
				control = DIO_SET_OUTPUT1(control, ibit);
				break;
			case DIO_MASK_OUTPUT0:
				control = DIO_SET_OUTPUT0(control, ibit);
				break;
			case DIO_MASK_OUTPUT_PP:
				control = DIO_SET_OUTPUT0(control, ibit);
				*ACQ200_DIOCON = control;
				control = DIO_SET_OUTPUT1(control, ibit);
				*ACQ200_DIOCON = control;
				control = DIO_SET_OUTPUT0(control, ibit);
				break;
			case DIO_MASK_OUTPUT_NP:
				control = DIO_SET_OUTPUT1(control, ibit);
				*ACQ200_DIOCON = control;
				control = DIO_SET_OUTPUT0(control, ibit);
				*ACQ200_DIOCON = control;
				control = DIO_SET_OUTPUT1(control, ibit);
				break;
			case DIO_MASK_INPUT:
			default:
				/* set as input. record standard value */
				control = DIO_SET_INPUT(control, ibit);
				sprintf(last_store, "%d -", ibit);
				goto write_reg;
			}
			strncpy(last_store, buf, sizeof(last_store)-1);
		write_reg:
			*ACQ200_DIOCON = control;
			control_mirror = control;
		}
	}

        return strlen(buf);
}

static ssize_t show_dio_bit(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	strcpy(buf, last_store);
	return strlen(last_store);
}

static DEVICE_ATTR(dio_bit, S_IRUGO|S_IWUGO, show_dio_bit, store_dio_bit);


static ssize_t store_dio(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int ibit;
	unsigned control = 0;

	for (ibit = 0; ibit != MAXDIOBIT; ++ibit){
		switch(buf[ibit]){
		case DIO_MASK_OUTPUT1:
			control = DIO_SET_OUTPUT1(control, ibit);
			break;
		case DIO_MASK_OUTPUT0:
			control = DIO_SET_OUTPUT0(control, ibit);
			break;
		case DIO_MASK_INPUT:
		default:
			control = DIO_SET_INPUT(control, ibit);
		}
	}
	*ACQ200_DIOCON = control_mirror = control;
        return strlen(buf);
}

static ssize_t show_dio(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int ibit;
	unsigned control = *ACQ200_DIOCON;

	for (ibit = 0; ibit != MAXDIOBIT; ++ibit){
		if (DIO_IS_OUTPUT(control, ibit)){
			buf[ibit] = DIO_IS_OUTPUT1(control, ibit)?
				DIO_MASK_OUTPUT1: DIO_MASK_OUTPUT0;
		}else{
			buf[ibit] = DIO_IS_INPUTH(control, ibit)?
				DIO_MASK_INPUT1: DIO_MASK_INPUT0;
		} 
	}
	buf[ibit++] = '\n';
	buf[ibit] = '\0';
	return strlen(buf);
}

static DEVICE_ATTR(dio, S_IRUGO|S_IWUGO, show_dio, store_dio);


static ssize_t show_dio_raw(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	unsigned dio = *ACQ200_DIOCON;
	buf[0] = dio&0x00ff;
	return 1;
}
static DEVICE_ATTR(dio_raw, S_IRUGO, show_dio_raw, 0);


static unsigned findTogglingBits(void)
{
	unsigned toggling = 0;
	unsigned c1 = *ACQ200_DIOCON;
	unsigned c2;
	unsigned long j1 = jiffies;
	int npolls = 0;
	int duffer_report = 0;
       
	while (ABS(jiffies - j1) < 5){
		c2 = *ACQ200_DIOCON;
		if ((c2 & ~ACQ200_DIOCON_INPDAT) != 
		    (control_mirror & ~ACQ200_DIOCON_INPDAT)){
			if (!duffer_report++){
				err("duff diocon ctrl %08x c1 %08x c2 %08x", 
				    control_mirror, c1, c2);
			}
			continue;
		}
		if (c2 ^ c1){
			toggling |= (c2 ^ c1)&((1<<MAXDIOBIT)-1);
		}
		yield();
		++npolls;
	}

	if (duffer_report > 1){
		err("duff diocon occurred %d times out of %d",
		      duffer_report, npolls);
	}
	return toggling;

}

static ssize_t poll_dio(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int ibit;
	unsigned toggling = findTogglingBits();
	unsigned control = *ACQ200_DIOCON;

	for (ibit = 0; ibit != MAXDIOBIT; ++ibit){
		if (DIO_IS_OUTPUT(control, ibit)){
			buf[ibit] = DIO_IS_OUTPUT1(control, ibit)?
				DIO_MASK_OUTPUT1: DIO_MASK_OUTPUT0;
		}else{
			buf[ibit] = 
				DIO_IS_INPUTH(toggling, ibit)?
				DIO_MASK_INPUT_TOGGLE:
			        DIO_IS_INPUTH(control, ibit)?
				DIO_MASK_INPUT1: DIO_MASK_INPUT0;
		} 
	}
	buf[ibit++] = '\n';
	buf[ibit] = '\0';
	return strlen(buf);
}

static DEVICE_ATTR(poll_dio, S_IRUGO, poll_dio, 0);

static ssize_t store_simulate(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d", &DG->simulate);
        return strlen(buf);
}

static ssize_t show_simulate(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf,"%d\n", DG->simulate);
}

static DEVICE_ATTR(simulate, S_IRUGO|S_IWUGO, show_simulate, store_simulate);

static ssize_t store_free_tblocks(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (DMC_WO_getState() == ST_STOP){
		int set_free;

		if (sscanf(buf, "%d", &set_free) != 0 && set_free == 1){
			acq200_release_phases();
			acq200_empties_release_tblocks();
			acq200_sort_free_tblocks();
		}
	}

        return strlen(buf);
}

int free_block_count(void)
{
	struct TblockListElement* tle;
	int iblock = 0;

	list_for_each_entry(tle, &DG->bigbuf.free_tblocks, list){
		++iblock;
	}
	return iblock;
}
static ssize_t show_free_tblocks(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{

	return sprintf(buf,"%d\n", free_block_count());
}

static DEVICE_ATTR(free_tblocks, S_IRUGO|S_IWUGO, show_free_tblocks, store_free_tblocks);



static ssize_t store_oneshot(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value)){
		DG->is_oneshot = value != 0;
	}
        return strlen(buf);
}

static ssize_t show_oneshot(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf,"%d\n", DG->is_oneshot);
}

static DEVICE_ATTR(oneshot, S_IRUGO|S_IWUGO, show_oneshot, store_oneshot);

static ssize_t show_sample_size(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%d\n", sample_size());
}

static DEVICE_ATTR(sample_size, S_IRUGO, show_sample_size, 0);


static ssize_t show_word_size(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%d\n", capdef_get_word_size());
}

static DEVICE_ATTR(word_size, S_IRUGO, show_word_size, 0);




#ifndef WAV232
static ssize_t store_bda(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)	
{
	struct BDA bda;
	char scaler[3];
#define SCALE(ss) ((ss) == 'M'? 1000000: (ss) == 'k'? 1000: 1)

	if (sscanf(buf, "%u%c %u%c %u%c", 
		   &bda.before, scaler,
		   &bda.during, scaler+1,
		   &bda.after,  scaler+2) == 6){

		bda.before *= SCALE(scaler[0]);
		bda.during *= SCALE(scaler[1]);
		bda.after  *= SCALE(scaler[2]);

		if (bda.before == 0){
			DMC_WO->getNextEmpty = GET_NEXT_EMPTY;
			memset(&bda, 0, sizeof(bda));
			DG->bda_samples = bda;
		}else{
			if (bda.before <= bda.during &&
			    bda.during <= bda.after     ){
				DG->bda_samples = bda;
				DMC_WO->getNextEmpty = bda_getNextEmpty;
			}
		}
	}
	return(strlen(buf));

#undef SCALE
}
static ssize_t show_bda(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%u %u %u\n", 
		       DG->bda_samples.before, 
		       DG->bda_samples.during,
		       DG->bda_samples.after);
}

static DEVICE_ATTR(BeforeDuringAfter, S_IRUGO|S_IWUGO, show_bda, store_bda);

static ssize_t store_ct(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	struct CONTROL_TARGET ct = { 0, };

	if (sscanf(buf, "%x %u %x %u %u %u", 
		   &ct.pa_data, &ct.data_blocks, 
		   &ct.pa_status, &ct.stride, &ct.subsample,
		   &ct.iodd) >= 4){
		DMC_WO->control_target = ct;
	}
	return strlen(buf);
}
static ssize_t show_ct(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	struct CONTROL_TARGET ct = DMC_WO->control_target;

	return sprintf(buf, "0x%08x %u 0x%08x %u %u %d\n",
		        ct.pa_data, ct.data_blocks, ct.pa_status, 
		       ct.stride, ct.subsample,
		       ct.iodd
		);
		       
}
static DEVICE_ATTR(ControlTarget, S_IRUGO|S_IWUGO, show_ct, store_ct);

#endif /* WAV232 */

static ssize_t store_pre_arm_hook(
	struct device * dev,
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	return store_hook(&DG->pre_arm_hook, buf, count);
}

static ssize_t show_pre_arm_hook(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return show_hook(&DG->pre_arm_hook, buf, PAGE_SIZE);
}
static DEVICE_ATTR(pre_arm_hook, S_IRUGO|S_IWUGO,
		   show_pre_arm_hook,store_pre_arm_hook);

static ssize_t store_post_arm_hook(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	return store_hook(&DG->post_arm_hook, buf, count);
}

static ssize_t show_post_arm_hook(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return show_hook(&DG->post_arm_hook, buf, PAGE_SIZE);
}
static DEVICE_ATTR(post_arm_hook, S_IRUGO|S_IWUGO,
		   show_post_arm_hook,store_post_arm_hook);

 
static ssize_t store_post_shot_hook(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	return store_hook(&DG->post_shot_hook, buf, count);
}

static ssize_t show_post_shot_hook(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return show_hook(&DG->post_shot_hook, buf, PAGE_SIZE);
}
static DEVICE_ATTR(post_shot_hook, S_IRUGO|S_IWUGO,
		   show_post_shot_hook,store_post_shot_hook);

 

void mk_dev_sysfs(struct device* dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_sample_read_start);
	DEVICE_CREATE_FILE(dev, &dev_attr_sample_read_stride);
	DEVICE_CREATE_FILE(dev, &dev_attr_sample_read_length);
	DEVICE_CREATE_FILE(dev, &dev_attr_sample_read_ssl);
	DEVICE_CREATE_FILE(dev, &dev_attr_show_event);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mask);
	DEVICE_CREATE_FILE(dev, &dev_attr_event_magic);
	DEVICE_CREATE_FILE(dev, &dev_attr_event_magic_mask);
	DEVICE_CREATE_FILE(dev, &dev_attr_state);
	DEVICE_CREATE_FILE(dev, &dev_attr_measured_sample_rate);
	DEVICE_CREATE_FILE(dev, &dev_attr_mode);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_bit);
	DEVICE_CREATE_FILE(dev, &dev_attr_dio_raw);
	DEVICE_CREATE_FILE(dev, &dev_attr_poll_dio);
#ifndef WAV232
	DEVICE_CREATE_FILE(dev, &dev_attr_event0);
	DEVICE_CREATE_FILE(dev, &dev_attr_int_clk_src);

	DEVICE_CREATE_FILE(dev, &dev_attr_scc);
	DEVICE_CREATE_FILE(dev, &dev_attr_ecc);

	DEVICE_CREATE_FILE(dev, &dev_attr_dcb_max_backlog);

	DEVICE_CREATE_FILE(dev, &dev_attr_counter_src);
	DEVICE_CREATE_FILE(dev, &dev_attr_counter_update);
#endif
	DEVICE_CREATE_FILE(dev, &dev_attr_simulate);
	DEVICE_CREATE_FILE(dev, &dev_attr_free_tblocks);
	DEVICE_CREATE_FILE(dev, &dev_attr_oneshot);
	DEVICE_CREATE_FILE(dev, &dev_attr_user_abort);
	DEVICE_CREATE_FILE(dev, &dev_attr_cap_status);
	DEVICE_CREATE_FILE(dev, &dev_attr_pit_stop);
	DEVICE_CREATE_FILE(dev, &dev_attr_burst_delay);
	DEVICE_CREATE_FILE(dev, &dev_attr_burst_len);
	DEVICE_CREATE_FILE(dev, &dev_attr_burst_es_channel);

	DEVICE_CREATE_FILE(dev, &dev_attr_pulse_def);

	DEVICE_CREATE_FILE(dev, &dev_attr_trig);
	DEVICE_CREATE_FILE(dev, &dev_attr_ext_clk);
	DEVICE_CREATE_FILE(dev, &dev_attr_mas_clk);
	DEVICE_CREATE_FILE(dev, &dev_attr_sample_size);
	DEVICE_CREATE_FILE(dev, &dev_attr_word_size);
	DEVICE_CREATE_FILE(dev, &dev_attr_activate_event_on_arm);
	DEVICE_CREATE_FILE(dev, &dev_attr_stub_live_copy_to_user);

	DEVICE_CREATE_FILE(dev, &dev_attr_clock_count_immediate);
	DEVICE_CREATE_FILE(dev, &dev_attr_clock_count_latched);
#ifndef WAV232
	DEVICE_CREATE_FILE(dev, &dev_attr_BeforeDuringAfter);
	DEVICE_CREATE_FILE(dev, &dev_attr_ControlTarget);
#endif
	DEVICE_CREATE_FILE(dev, &dev_attr_pre_arm_hook);
	DEVICE_CREATE_FILE(dev, &dev_attr_post_arm_hook);
	DEVICE_CREATE_FILE(dev, &dev_attr_post_shot_hook);
	DEVICE_MK_DEV_SYSFS(dev);
}






static ssize_t show_debug(
	struct device_driver *driver, char * buf)
{
        return sprintf(buf,"%d\n",acq200_debug);
}

extern void acq200_pipe_fiq(void);
extern void acq200_pipe_fiq_end(void);

static ssize_t show_pipe_fiq(
	struct device_driver *driver, char * buf)
{
        return sprintf(buf, "%20s %p\n%20s %p\n%20s %d 0x%04x\n", 
		       "start", acq200_pipe_fiq, 
		       "end", acq200_pipe_fiq_end,
		       "len", PIPE_FIQ_LEN, PIPE_FIQ_LEN);
}


static ssize_t show_asm_consts(
	struct device_driver *driver, char * buf)
{
	int len = 0;
#define OO offsetof
#define DEFPR(sym,val) \
	len += sprintf(buf+len, "#define %-20s %4d /* 0x%02x */\n", sym, val, val)
#define CPR(str,val) \
        len += sprintf(buf+len, "#define %-20s %d\n", str, val)

	len += sprintf(buf+len, "/* %s */\n", ACQ200_FIFO_H_VERSION_STR);
	CPR("CONSTS_VERSION", ACQ200_FIFO_H_VERSION);
	CPR("FIQDEBUG", FIQDEBUG);
	CPR("RBLEN", RBLEN);
#ifdef FIQDEBUG
	DEFPR( "DG_CAFEBABE", OO(struct DevGlobs, CAFEBABE));
	DEFPR( "DG_FEEDCODE", OO(struct DevGlobs, FEEDCODE));
	DEFPR( "DG_DEADBEEF", OO(struct DevGlobs, DEADBEEF));
#endif

	DEFPR( "RB_IPUT",      OO(struct acq200_dma_ring_buffer, iput));
	DEFPR( "RB_IGET",      OO(struct acq200_dma_ring_buffer, iget));
	DEFPR( "RB_NPUT",      OO(struct acq200_dma_ring_buffer, nput));
	DEFPR( "RB_NGET",      OO(struct acq200_dma_ring_buffer, nget));
	DEFPR( "RB_BUFFERS",   OO(struct acq200_dma_ring_buffer, buffers));

	DEFPR( "IPC_EMPTIES",  OO(struct IPC, empties));
	DEFPR( "IPC_ACTIVE",   OO(struct IPC, active));
	DEFPR( "IPC_ENDSTOPS", OO(struct IPC, endstops));

	DEFPR( "STATS_NFINTS",          OO(struct stats, num_fifo_ints));
	DEFPR( "STATS_DMA_BLOCKS",      OO(struct stats, dma_blocks));
	DEFPR( "STATS_COLD_FIFO_HISTO", OO(struct stats, cold_fifo_histo));
	DEFPR( "STATS_HOT_FIFO_HISTO",  OO(struct stats, hot_fifo_histo));
	DEFPR( "STATS_STARVE_FIFCON",   OO(struct stats, starve_fifcon));

	DEFPR( "DG_STATS",      OO(struct DevGlobs, stats));
	DEFPR( "DG_IPC",        OO(struct DevGlobs, ipc));
	DEFPR( "DG_WO",         OO(struct DevGlobs, wo));
	DEFPR( "DG_FPGA_VA",    OO(struct DevGlobs, fpga.regs.va));
	DEFPR( "DG_ISTACK",     OO(struct DevGlobs, istack));
	DEFPR( "DG_FIFERR",     OO(struct DevGlobs, fiferr));
	DEFPR( "DG_FIFERR_MASK",OO(struct DevGlobs, FIFERR));
	DEFPR( "DG_HEAD",       OO(struct DevGlobs, head));

	DEFPR( "WO_NEXT_LOAD",  OO(struct DMC_WORK_ORDER, next_load));
	return len;
#undef DEFPR
}
static DRIVER_ATTR(asm_consts,S_IRUGO|S_IWUGO,show_asm_consts,0);

static ssize_t show_data_addresses(
	struct device_driver *driver, char * buf)
{
	int len = 0;
#define DEFPR(sym) \
	len += sprintf(buf+len, "%-40s 0x%08lx\n", #sym, (long)sym)
	
	DEFPR( DG );
	DEFPR( DG->ipc );
	DEFPR( DG->wo );
	DEFPR( DG->fpga.regs.va );
	
	return len;
#undef DEFPR
}


static ssize_t store_unsigned_decimal_range(
	struct device_driver *driver, const char * buf, size_t count,
	unsigned *field, unsigned _min, unsigned _max)
{
	unsigned xx;

	if (sscanf(buf,"%u",&xx) && IN_RANGE(xx, _min, _max)){
		*field = xx;
	}
	return strlen(buf);
}

static ssize_t show_unsigned_decimal(
	struct device_driver *driver, char * buf, unsigned *field)
{
        return sprintf(buf,"%u\n", *field);
}

#define DEFINE_DG_RANGE_FIELD(field, _min, _max)			\
static ssize_t show_field_##field(					\
	struct device_driver * driver, char * buf)			\
{									\
	return show_unsigned_decimal(driver, buf, &DG->field);		\
}									\
									\
static ssize_t store_field_##field(					\
	struct device_driver * driver, const char * buf, size_t count)	\
{									\
	return store_unsigned_decimal_range(				\
             driver, buf, count, &DG->field, _min, _max);		\
}									\
static DRIVER_ATTR(field, S_IRUSR|S_IWUSR,				\
		   show_field_##field, store_field_##field)

DEFINE_DG_RANGE_FIELD(empty_fill_threshold, 1, RBLEN);
DEFINE_DG_RANGE_FIELD(put_max_empties, 1, 16000);
DEFINE_DG_RANGE_FIELD(get_max_active, 1, 16000);
DEFINE_DG_RANGE_FIELD(active_batch_threshold, 0, 16000);
DEFINE_DG_RANGE_FIELD(init_endstops, 1, 128);
DEFINE_DG_RANGE_FIELD(eoc_int_modulo_mask, 0, 15);

extern char *acq200_fifo_verid(void);


static ssize_t show_version(
	struct device_driver *driver, char * buf)
{
        return sprintf(buf,"%s\n%s\n", acq200_fifo_verid(), VERID);
}



static ssize_t store_debug(
	struct device_driver *driver, const char * buf, size_t count)
{
	sscanf(buf,"%d",&acq200_debug);
	return strlen(buf);
}


static ssize_t show_int_clk(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d %dkHz %dMHz\n", 
		       acq200_clk_hz, 
		       acq200_clk_hz/1000,
		       acq200_clk_hz/1000000);
}

static ssize_t store_int_clk(
	struct device_driver * driver, const char * buf, size_t count)
{
	char unit;

	switch (sscanf(buf, "%d%c", &acq200_clk_hz, &unit)){
	case 2:
		switch(unit){
		case 'k':
			acq200_clk_hz *= 1000;
			break;
		case 'M':
			acq200_clk_hz *= 1000000;
			break;
		default:
			return strlen(buf);
		}
		/* fall thru */
	case 1:
		acq200_setIntClkHz(acq200_clk_hz);
		break;
	default:
		break;
	}
	return strlen(buf);
}

static ssize_t show_int_clk_div(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%u\n", arch_get_int_clk_div());
}

static ssize_t store_int_clk_div(
	struct device_driver * driver, const char * buf, size_t count)
{
	u32 div;
	if (sscanf(buf, "%u", &div) == 1){
		div = max(2U, div);
		arch_set_int_clk_div(div);
		deactivateSignal(CAPDEF->ext_clk);
		activateSignal(CAPDEF->int_clk_src);
	};
	return strlen(buf);
}



static ssize_t show_busywait(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d\n",DG->busywait);
}

static ssize_t store_busywait(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf(buf,"%d",&DG->busywait);
	return strlen(buf);
}

static ssize_t show_len(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d 0x%08x\n",LEN, LEN);
}

static ssize_t store_len(
	struct device_driver * driver, const char * buf, size_t count)
{
	unsigned len;
	if (sscanf(buf,"%u",&len) == 1 || sscanf(buf, "0x%x",&len) == 1){
		LEN = min(len, len_buf(DG));
	}
	return strlen(buf);
}

static ssize_t show_postlen(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d\n",CAPDEF->demand_postlen);
}

static ssize_t store_postlen(
	struct device_driver * driver, const char * buf, size_t count)
{
	unsigned len;
	sscanf(buf,"%u",&len);
	
	CAPDEF->demand_postlen = min(len, len_buf(DG));
	return strlen(buf);
}


	
static ssize_t set_reset_fifo(
	struct device_driver * driver, const char * buf, size_t count)
/* this should act on device, not driver, but there's only ever one device ..*/
{
	acq200_reset_fifo();
	return strlen(buf);
}
static ssize_t show_hitide(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d\n", DG->hitide);
}

static ssize_t store_hitide(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf(buf, "%d", &DG->hitide);
	return strlen(buf);
}

static ssize_t show_lotide(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d\n", DG->lotide);
}

static ssize_t store_lotide(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf(buf, "%d", &DG->lotide);
	return strlen(buf);
}
static ssize_t show_fpga_isr_fiq(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d\n", DG->use_fiq);
}

static ssize_t store_fpga_isr_fiq(
	struct device_driver * driver, const char * buf, size_t count)
{
	int try_fiq;

	if (sscanf(buf, "%d", &try_fiq) == 1 ){
		DG->use_fiq = set_fpga_isr_use_fiq(try_fiq);
	}
	return strlen(buf);
}

static ssize_t show_max_alloc(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d\n", DG->max_alloc);
}

static ssize_t store_max_alloc(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf(buf, "%d", &DG->max_alloc);
	return strlen(buf);
}


static ssize_t show_shot(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d\n", DG->shot);
}

static ssize_t store_shot(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf(buf, "%d", &DG->shot);
	return strlen(buf);
}

#define STREQN(s1, s2 ) (strncmp(s1, s2, strlen(s2)) == 0)

static ssize_t show_btype(
	struct device_driver * driver, char * buf)
{
	int rc;

	switch(DG->btype){
	case BTYPE_ACQ216:
		rc = sprintf(buf, "acq216"); break;
	case BTYPE_WAV232:
		rc = sprintf(buf, "wav232"); break;
	default:
		rc = sprintf(buf, "ERROR: bad btype"); break;
	}
	return rc;
}

static ssize_t store_btype(
	struct device_driver * driver, const char * buf, size_t count)
{
	if (STREQN(buf, "acq216" )){
		DG->btype = BTYPE_ACQ216;
	}else if (STREQN(buf, "wav232" )){
		DG->btype = BTYPE_WAV232;
	}else{
		return -ENODEV;
	}

	on_personality_change();
	return count;
}


static ssize_t show_transformer_blocksize(
	struct device_driver * driver, char * buf)
{
	int blocklen = DG->bigbuf.tblocks.blocklen;

        return sprintf(buf,"%d %08x %dM %s\n", 
		       blocklen, blocklen, blocklen>>20,
		       DG->bigbuf.tblocks.tmp.start==0? "(nul)": "registered");
}

static ssize_t store_transformer_blocksize(
	struct device_driver * driver, const char * buf, size_t count)
/* user enters bs in integer MB */
{
#ifdef PGMCOMOUT
/* @@todo - this seems like a BAD IDEA */
	int len;

	if (sscanf(buf, "%d", &len) == 1){
		DG->bigbuf.tblocks.blocklen = len<<20;   
		
		if (DG->bigbuf.tblocks.tmp.start){
			free_tblock_list();
		}
		build_tblock_list();
	}
#endif
	return strlen(buf);
}
static DRIVER_ATTR(transformer_bs,S_IRUGO | S_IWUGO, 
		   show_transformer_blocksize, store_transformer_blocksize);


#if !defined(WAV232)


extern void search_epos(
	struct Phase* phase, int search_metric);



static void make_phase_backup(struct Phase* phase)
{
#define MAXOLD 10
	static struct Phase old_phases[MAXOLD];
	static int ii;

	memcpy(&old_phases[ii], phase, PHASE_SZ);
	phase->orig = &old_phases[ii];
	phase->orig->is_orig = 1;
	
	if (++ii >= MAXOLD) ii = 0;
#undef MAXOLD
}


static void find_epos(int search_metric)
{
/* ident old phase (only works for ONE WORKTODO) */

	struct Phase* phase;

	list_for_each_entry(phase, &DMC_WO->phases, list){
		if (phase->event_count){
			make_phase_backup(phase);

			if (PREV_PHASE(phase) != phase){
				make_phase_backup(PREV_PHASE(phase));
				search_epos(phase, search_metric);
			}
		}
	}
}
static ssize_t show_epos(struct device_driver * driver, char * buf)
{
	return sprintf(buf, "%s\n",
		       DMC_WO->epos_found? "FOUND": "WARNING: not found");
}



static ssize_t store_epos(
	struct device_driver * driver, const char * buf, size_t count)
{
	int search_metric = 1;

	sscanf(buf, "%d", &search_metric);
	find_epos(search_metric);
	return strlen(buf);
}


static DRIVER_ATTR(epos,S_IRUGO | S_IWUGO, show_epos, store_epos);

static ssize_t show_pit_count(struct device_driver * driver, char * buf)
{
	return sprintf(buf, "%d\n", DMC_WO->pit_count);
}
static DRIVER_ATTR(pit_count, S_IRUGO, show_pit_count, 0);

static ssize_t show_es_search(
	struct device_driver * driver, char * buf)
{
	return sprintf(buf, "%s\n",
		       DMC_WO->epos_found? "FOUND": "WARNING: not found");
}

extern void find_es_anywhere(void);

static ssize_t store_es_search(
	struct device_driver * driver, const char * buf, size_t count)
{
	int search_metric = 1;

	sscanf(buf, "%d", &search_metric);
	find_es_anywhere();
	return strlen(buf);
}
static DRIVER_ATTR(es_search,S_IRUGO | S_IWUGO, 
		   show_es_search, store_es_search);


static ssize_t show_finalize_phases(
	struct device_driver * driver, char * buf)
{
	return 0;
}


static ssize_t store_finalize_phases(
	struct device_driver * driver, const char * buf, size_t count)
{
	unsigned start_sample = 0;
	struct Phase* phase;

	list_for_each_entry(phase, &DMC_WO->phases, list){
#if 0
		dbg(1, "phase_len %d required %d %s",
		    phase->actual_len, phase->required_len,
		    phase->actual_len > phase->required_len? "rollup": "");

		if (phase->actual_len > phase->required_len){
			acq200_phase_rollup_excess(phase);
		}
#else
		acq200_phase_rollup_excess(phase);
#endif
	}	

	list_for_each_entry(phase, &DMC_WO->phases, list){

		dbg(1, "phase \"%s\" %p start %d len %d",
		    phase->name, phase, phase->start_sample, phase_len(phase));

		if (phase_len(phase)){
			acq200_phase_gather_tblocks(phase);
			phase->start_sample = start_sample;
			start_sample += phase->actual_samples;

			dbg(1, "phase %p start %d actual %d",
			    phase, phase->start_sample, phase->actual_samples);
		}
	}

	return strlen(buf);
}


static DRIVER_ATTR(finalize_phases,S_IRUGO|S_IWUGO, 
		   show_finalize_phases, store_finalize_phases);


static ssize_t store_transformer(
	struct device_driver * driver, const char * buf, size_t count)
{
	int cursor = DG->bigbuf.tblocks.cursor;
	struct Phase* phase;
	struct TblockListElement *tble;

	if (strncmp(buf, "CLEAR", 5) == 0){
		int ib;
		
		for (ib = DG->bigbuf.tblocks.nblocks; ib--; ){
			DG->bigbuf.tblocks.the_tblocks[ib].touched = 0;
		}
		cursor = DG->bigbuf.tblocks.cursor = 0;
		DG->bigbuf.tblocks.cursor_complete = 0;
		info("CLEAR accepted");
		goto cleanup;
	}

#ifndef ACQ132
/* @todo - no short transform for ACQ132 */
	if (cursor == 0){
		struct BIGBUF *bb = &DG->bigbuf;
		struct Phase* phase1 = 0;
		int nphases = 0;

		list_for_each_entry(phase, &DMC_WO->phases, list){
			++nphases;
			if (phase1 == 0){
				phase1 = phase;
			}
		}

		if (nphases == 1 &&
		    phase_len(phase) * sample_size() < 3*TBLOCK_LEN/2){
			acq200_fifo_part_transform(phase);
			cursor = bb->tblocks.nblocks;
			goto cursor_complete;
		}
	}
#endif

	/* iterate tblocks in phase order */
	list_for_each_entry(phase, &DMC_WO->phases, list){
		list_for_each_entry(tble, &phase->tblocks, list){
			struct TBLOCK *tb = tble->tblock;
			if (atomic_read(&tb->in_phase) && tb->touched == 0){
				acq200_fifo_bigbuf_transform(tb->iblock);
				tb->touched = 1;
				++cursor;
				goto cleanup;
			}
		}
	}
#ifndef ACQ132
cursor_complete:
#endif
	DG->bigbuf.tblocks.cursor_complete = 1;
	
cleanup:
	DG->bigbuf.tblocks.cursor = cursor;
	return strlen(buf);
}

#endif   /* #if !defined(WAV232) */


static ssize_t show_transformer(
	struct device_driver * driver, char * buf)
{
	struct BIGBUF *bb = &DG->bigbuf;
	int cursor = bb->tblocks.cursor;
	int is_complete = (cursor && !DG->bigbuf.tblocks.cursor_complete);

	return sprintf(buf, "%d %s\n", cursor, is_complete? "COMPLETE": "");
}


static DRIVER_ATTR(transformer,S_IRUGO | S_IWUGO, 
		   show_transformer, store_transformer);



static ssize_t show_jiffies(
	struct device_driver * driver, char * buf)
{
	return sprintf(buf, "%ld\n", jiffies);
}

static DRIVER_ATTR(jiffies,S_IRUGO, show_jiffies, 0);






static ssize_t show_use_ob_clock(
	struct device_driver * driver, char * buf)
{
	return sprintf(buf, "%d\n", DG->use_ob_clock );
}

static ssize_t store_use_ob_clock(
	struct device_driver * driver, const char * buf, size_t count)
{
	int enable;

	if (sscanf(buf, "%d", &enable) == 1){
		DG->use_ob_clock = enable != 0;
	}
	return strlen(buf);
}

static DRIVER_ATTR(use_ob_clock,S_IRUGO | S_IWUGO, 
		   show_use_ob_clock, store_use_ob_clock);



static ssize_t show_pulse_number(
	struct device_driver * driver, char * buf)
{
	return sprintf(buf, "%lu\n", DG->pulse_number);
}


static ssize_t store_pulse_number(
	struct device_driver * driver, const char * buf, size_t count)
{
	unsigned long pn;

	if (sscanf(buf, "%lu", &pn) == 1){
		DG->pulse_number = pn;
	}
	return strlen(buf);
}

static DRIVER_ATTR(pulse_number,S_IRUGO | S_IWUGO, 
		   show_pulse_number, store_pulse_number);






static ssize_t show_pci_abort(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%08x %08x\n", 
		       DG->mbox_abort_mask, DG->mbox_abort_value);
}
static ssize_t store_pci_abort(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf(buf, "%x %x", &DG->mbox_abort_mask, &DG->mbox_abort_value);
	return strlen(buf);
}


static DRIVER_ATTR(pci_abort,S_IRUGO | S_IWUGO, 
 		   show_pci_abort, store_pci_abort);


static ssize_t show_transformer_blt(
	struct device_driver * driver, char * buf)
{
	struct TBLOCKLIST *tbl = &DG->bigbuf.tblocks;
	int iblt = -1;
	char *cblt = "error";

	if (tbl->blt == blt_memcpy){
		iblt = 0;
		cblt = "blt_memcpy";
	}else if (tbl->blt == blt_null){
		iblt = 1;
		cblt = "blt_null";
	}else if (tbl->blt == blt_dma){
		iblt = 2;
		cblt = "blt_dma";
	}
	return sprintf(buf, "%d %s\n", iblt, cblt);
}

static ssize_t store_transformer_blt(
	struct device_driver * driver, const char * buf, size_t count)
{
	struct TBLOCKLIST *tbl = &DG->bigbuf.tblocks;
	int iblt;

	if (sscanf(buf, "%d", &iblt) == 1){
		switch(iblt){
		case 0:
			tbl->blt = blt_memcpy; break;
		case 1:
			tbl->blt = blt_null; break;
		case 2:
			tbl->blt = blt_dma; break;
		}
	}
	return strlen(buf);
}

static DRIVER_ATTR(transformer_blt,S_IRUGO | S_IWUGO, 
		   show_transformer_blt, store_transformer_blt);







static ssize_t show_bh_unmasks_eoc(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d\n", DG->bh_unmasks_eoc);
}

static ssize_t store_bh_unmasks_eoc(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf(buf, "%d", &DG->bh_unmasks_eoc);
	return strlen(buf);
}

static ssize_t show_bigbuf_read_method(
	struct device_driver * driver, char * buf)
{
        return sprintf(
		buf,"%s\n", 
		acq200_fifo_get_bigbuf_read_method_raw()?
		"raw": "channel");
}

static ssize_t store_bigbuf_read_method(
	struct device_driver * driver, const char * buf, size_t count)
{
	char mode;
	sscanf(buf, "%c", &mode);

	switch(mode){
	case 'R':
	case 'r':
		acq200_fifo_set_bigbuf_read_method_raw(1);
		break;
	default:
		acq200_fifo_set_bigbuf_read_method_raw(0);
	}


	return strlen(buf);
}
static DRIVER_ATTR(debug_read_raw,S_IRUGO|S_IWUGO,
		   show_bigbuf_read_method, store_bigbuf_read_method);






static ssize_t set_clear_bigbuf(
	struct device_driver * driver, const char * buf, size_t count)
{
	memset(va_buf(DG), 0, len_buf(DG));
	return strlen(buf);
}

static ssize_t set_ramp_bigbuf(
	struct device_driver * driver, const char * buf, size_t count)
{
	int ibuf;
	int maxbuf = len_buf(DG)/sizeof(u32);
	u32 *pbuf = (u32*)va_buf(DG);

	for ( ibuf = 0; ibuf != maxbuf; ++ibuf){
		pbuf[ibuf] = ibuf;
	}
	return strlen(buf);
}


static void create_testpattern(short *pfill, int nchan, int  slen)
{
        int ichan = 0;
        int isample = 0;
        int ifill;

        for (ifill = 0; ifill != slen; ++ifill){
                pfill[ifill] = (ichan << 8) | (isample&0x00ff);
                                                                               
                if (++ichan >= nchan){
                        ichan = 0;
                        isample += 1;
                }
        }
}

static void create_testpattern_channel(
	short *pfill, int ichan, int nchan, int  slen)
{
        int isample = 0;
        int ifill;

        for (ifill = 0; ifill < slen; ifill += nchan, isample++){
                pfill[ifill+ichan] = (ichan << 12) | (isample&0x0fff);
        }
}




static ssize_t set_testpattern_bigbuf(
	struct device_driver * driver, const char * buf, size_t count)
{
	int itest;

	if (sscanf(buf, "%d", &itest) == 1){
		switch(itest){
		case 1: case 2: case 3: case 4: 
		case 5: case 6: case 7: case 8: {
			short *bbuf = (short*)va_buf(DG);
			int len = len_buf(DG)/sizeof(short);

			create_testpattern_channel(
				bbuf, itest, NCHAN, len);
			break;			
		}
		default: {
			short *bbuf = (short*)va_buf(DG);
			int len = len_buf(DG)/sizeof(short);

			create_testpattern(bbuf, NCHAN, len);
			break;
		}
		case 0:
			memset(va_buf(DG), 0, len_buf(DG));
		}
	}
	return strlen(buf);
}



static  ssize_t show_bb_len(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf,"%d 0x%08x %dM\n", 
		       len_buf(DG), len_buf(DG), len_buf(DG)/0x100000);
}

static u32 dma_test_pci_addr   = 0x71200000;
static u32 dma_test_local_addr = 0xc2000000;
static u32 dma_test_length     = 0x00100000;

static ssize_t show_dma_test_setup(
	struct device_driver * driver, char * buf)
{
        return sprintf(
		buf,
		"0x%08x 0x%08x 0x%08x\n",
		dma_test_pci_addr,
		dma_test_local_addr,
		dma_test_length );
}

static ssize_t store_dma_test_setup(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf( buf, "0x%08x 0x%08x 0x%08x",
		&dma_test_pci_addr,
		&dma_test_local_addr,
		&dma_test_length );
	return strlen(buf);
}
static DRIVER_ATTR(dma_test_setup, S_IRUSR|S_IWUSR,
		   show_dma_test_setup, store_dma_test_setup);


#define TOLOCAL 1
#define TOPCI   0

static void run_dma_test(int incoming)
{
	iop321_start_ppmu();
	acq200_post_dmac_request(
		1, 
		dma_test_local_addr,
		0,
		dma_test_pci_addr,
		dma_test_length,
		incoming );
	iop321_stop_ppmu();
}

static ssize_t run_dma_test_pci_to_local(
	struct device_driver * driver, char * buf)
{
	run_dma_test(TOLOCAL);
	return 0;
}

static ssize_t run_dma_test_local_to_pci(
	struct device_driver * driver, const char * buf, size_t count)
{
	run_dma_test(TOPCI);
	return strlen(buf);
}

static DRIVER_ATTR(run_dma_test, S_IRUSR|S_IWUSR, 
		   run_dma_test_pci_to_local, 
		   run_dma_test_local_to_pci);




static ssize_t show_global_irq_mask(
	struct device_driver * driver, char * buf)
{
        return sprintf(
		buf,
		"0x%08x\n",
		DG->global_irq_mask );
}

static ssize_t store_global_irq_mask(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf( buf, "0x%08x", &DG->global_irq_mask);
	return strlen(buf);
}
static DRIVER_ATTR(global_irq_mask, S_IRUSR|S_IWUSR,
		   show_global_irq_mask, store_global_irq_mask);


static ssize_t show_FIFERR(
	struct device_driver * driver, char * buf)
{
        return sprintf(
		buf,
		"0x%08x\n",
		DG->FIFERR );
}

static ssize_t store_FIFERR(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf( buf, "0x%08x", &DG->FIFERR);
	return strlen(buf);
}
static DRIVER_ATTR(FIFERR, S_IRUSR|S_IWUSR,
		   show_FIFERR, store_FIFERR);



static ssize_t show_cdog_max_jiffies(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf, "%d\n", DG->cdog_max_jiffies );
}

static ssize_t store_cdog_max_jiffies(
	struct device_driver * driver, const char * buf, size_t count)
{
	sscanf( buf, "%d", &DG->cdog_max_jiffies);
	return strlen(buf);
}
static DRIVER_ATTR(cdog_max_jiffies, S_IRUSR|S_IWUSR,
		   show_cdog_max_jiffies, store_cdog_max_jiffies);


static ssize_t show_DMA_BLOCK_LEN(
	struct device_driver * driver, char * buf)
{
        return sprintf(buf, "%d\n", DMA_BLOCK_LEN);
}

static DRIVER_ATTR(DMA_BLOCK_LEN_bytes, S_IRUSR, show_DMA_BLOCK_LEN, 0);


static DRIVER_ATTR(daq_enable,S_IRUGO|S_IWUGO,show_daq_enable,set_daq_enable);
static DRIVER_ATTR(debug, S_IRUGO|S_IWUGO, show_debug, store_debug);
static DRIVER_ATTR(pipe_fiq, S_IRUGO, show_pipe_fiq, 0);
static DRIVER_ATTR(busywait, S_IRUGO|S_IWUGO, show_busywait, store_busywait);
static DRIVER_ATTR(data_structs, S_IRUGO, show_data_addresses, 0);
static DRIVER_ATTR(int_clk, S_IRUGO|S_IWUGO, show_int_clk, store_int_clk);
static DRIVER_ATTR(int_clk_div, S_IRUGO|S_IWUGO, 
		   show_int_clk_div, store_int_clk_div);
static DRIVER_ATTR(length, S_IRUGO|S_IWUGO, show_len, store_len);
static DRIVER_ATTR(post_length, S_IRUGO|S_IWUGO, show_postlen, store_postlen);
static DRIVER_ATTR(reset_fifo, S_IWUGO, 0, set_reset_fifo);
static DRIVER_ATTR(version, S_IRUGO, show_version, 0);
static DRIVER_ATTR(hitide,S_IRUGO | S_IWUGO, show_hitide, store_hitide);
static DRIVER_ATTR(lotide,S_IRUGO | S_IWUGO, show_lotide, store_lotide);
static DRIVER_ATTR(use_fiq, S_IRUGO | S_IWUGO, 
		   show_fpga_isr_fiq, store_fpga_isr_fiq);
static DRIVER_ATTR(max_alloc,S_IRUGO | S_IWUGO,show_max_alloc,store_max_alloc);
static DRIVER_ATTR(clear_bigbuf, S_IWUGO, 0, set_clear_bigbuf);
static DRIVER_ATTR(ramp_bigbuf, S_IWUGO, 0, set_ramp_bigbuf);
static DRIVER_ATTR(bb_len, S_IRUGO, show_bb_len, 0);
static DRIVER_ATTR(shot, S_IRUGO|S_IWUGO, show_shot, store_shot);
static DRIVER_ATTR(btype, S_IRUGO|S_IWUGO, show_btype, store_btype);

static DRIVER_ATTR(bh_unmasks_eoc, S_IRUGO|S_IWUGO, 
		   show_bh_unmasks_eoc, store_bh_unmasks_eoc);
static DRIVER_ATTR(testpattern,S_IWUGO, 0, set_testpattern_bigbuf);

int mk_sysfs(struct device_driver *driver)
{
	DRIVER_CREATE_FILE(driver, &driver_attr_debug);
	DRIVER_CREATE_FILE(driver, &driver_attr_pipe_fiq);
	DRIVER_CREATE_FILE(driver, &driver_attr_busywait);
	DRIVER_CREATE_FILE(driver, &driver_attr_data_structs);
	DRIVER_CREATE_FILE(driver, &driver_attr_int_clk);
	DRIVER_CREATE_FILE(driver, &driver_attr_int_clk_div);
	DRIVER_CREATE_FILE(driver, &driver_attr_length);
	DRIVER_CREATE_FILE(driver, &driver_attr_post_length);
	DRIVER_CREATE_FILE(driver, &driver_attr_reset_fifo);
	DRIVER_CREATE_FILE(driver, &driver_attr_use_fiq);
	DRIVER_CREATE_FILE(driver, &driver_attr_version);
	DRIVER_CREATE_FILE(driver, &driver_attr_asm_consts);
	DRIVER_CREATE_FILE(driver, &driver_attr_hitide);
	DRIVER_CREATE_FILE(driver, &driver_attr_lotide);
	DRIVER_CREATE_FILE(driver, &driver_attr_max_alloc);
	DRIVER_CREATE_FILE(driver, &driver_attr_clear_bigbuf);
	DRIVER_CREATE_FILE(driver, &driver_attr_ramp_bigbuf);
	DRIVER_CREATE_FILE(driver, &driver_attr_daq_enable);
	DRIVER_CREATE_FILE(driver, &driver_attr_bb_len);
	DRIVER_CREATE_FILE(driver, &driver_attr_shot);
	DRIVER_CREATE_FILE(driver, &driver_attr_btype);
	DRIVER_CREATE_FILE(driver, &driver_attr_bh_unmasks_eoc);
	DRIVER_CREATE_FILE(driver, &driver_attr_pci_abort);
	DRIVER_CREATE_FILE(driver, &driver_attr_debug_read_raw);
#if !defined(WAV232)
	DRIVER_CREATE_FILE(driver, &driver_attr_epos);
	DRIVER_CREATE_FILE(driver, &driver_attr_es_search);
	DRIVER_CREATE_FILE(driver, &driver_attr_finalize_phases);
	DRIVER_CREATE_FILE(driver, &driver_attr_pit_count);
#endif
	DRIVER_CREATE_FILE(driver, &driver_attr_transformer_bs);
	DRIVER_CREATE_FILE(driver, &driver_attr_transformer);
	DRIVER_CREATE_FILE(driver, &driver_attr_transformer_blt);
	DRIVER_CREATE_FILE(driver, &driver_attr_use_ob_clock);
	DRIVER_CREATE_FILE(driver, &driver_attr_testpattern);
	
	DRIVER_CREATE_FILE(driver, &driver_attr_dma_test_setup);
	DRIVER_CREATE_FILE(driver, &driver_attr_run_dma_test);

	DRIVER_CREATE_FILE(driver, &driver_attr_global_irq_mask);
	DRIVER_CREATE_FILE(driver, &driver_attr_FIFERR);
	DRIVER_CREATE_FILE(driver, &driver_attr_cdog_max_jiffies);
	DRIVER_CREATE_FILE(driver, &driver_attr_jiffies);
	DRIVER_CREATE_FILE(driver, &driver_attr_pulse_number);

	DRIVER_CREATE_FILE(driver, &driver_attr_empty_fill_threshold);
	DRIVER_CREATE_FILE(driver, &driver_attr_put_max_empties);
	DRIVER_CREATE_FILE(driver, &driver_attr_get_max_active);
	DRIVER_CREATE_FILE(driver, &driver_attr_active_batch_threshold);
	DRIVER_CREATE_FILE(driver, &driver_attr_init_endstops);
	DRIVER_CREATE_FILE(driver, &driver_attr_eoc_int_modulo_mask);
	DRIVER_CREATE_FILE(driver, &driver_attr_DMA_BLOCK_LEN_bytes);

	acq200_transform_mk_sysfs(driver);
	return 0;
}
void rm_sysfs(struct device_driver *driver)
{
	driver_remove_file(driver, &driver_attr_debug);
	driver_remove_file(driver, &driver_attr_pipe_fiq);
	driver_remove_file(driver, &driver_attr_busywait);
	driver_remove_file(driver, &driver_attr_data_structs);
	driver_remove_file(driver, &driver_attr_int_clk);
	driver_remove_file(driver, &driver_attr_length);
	driver_remove_file(driver, &driver_attr_reset_fifo);
	driver_remove_file(driver, &driver_attr_use_fiq);
	driver_remove_file(driver, &driver_attr_version);
	driver_remove_file(driver, &driver_attr_asm_consts);
	driver_remove_file(driver, &driver_attr_hitide);
	driver_remove_file(driver, &driver_attr_lotide);
	driver_remove_file(driver, &driver_attr_max_alloc);
	driver_remove_file(driver, &driver_attr_clear_bigbuf);
	driver_remove_file(driver, &driver_attr_ramp_bigbuf);
	driver_remove_file(driver, &driver_attr_daq_enable);
	driver_remove_file(driver, &driver_attr_bb_len);
	driver_remove_file(driver, &driver_attr_shot);
	driver_remove_file(driver, &driver_attr_btype);
	driver_remove_file(driver, &driver_attr_bh_unmasks_eoc);
	driver_remove_file(driver, &driver_attr_pci_abort);
	driver_remove_file(driver, &driver_attr_debug_read_raw);
#if !defined(WAV232)
	driver_remove_file(driver, &driver_attr_epos);
	driver_remove_file(driver, &driver_attr_es_search);
	driver_remove_file(driver, &driver_attr_finalize_phases);
	driver_remove_file(driver, &driver_attr_pit_count);
#endif
	driver_remove_file(driver, &driver_attr_transformer_bs);
	driver_remove_file(driver, &driver_attr_transformer);
	driver_remove_file(driver, &driver_attr_transformer_blt);
	driver_remove_file(driver, &driver_attr_use_ob_clock);
	driver_remove_file(driver, &driver_attr_testpattern);

	driver_remove_file(driver, &driver_attr_dma_test_setup);
	driver_remove_file(driver, &driver_attr_run_dma_test);

	driver_remove_file(driver, &driver_attr_global_irq_mask);
	driver_remove_file(driver, &driver_attr_FIFERR);
	driver_remove_file(driver, &driver_attr_jiffies);
	driver_remove_file(driver, &driver_attr_pulse_number);

	driver_remove_file(driver, &driver_attr_empty_fill_threshold);
	driver_remove_file(driver, &driver_attr_put_max_empties);
	driver_remove_file(driver, &driver_attr_get_max_active);
	driver_remove_file(driver, &driver_attr_active_batch_threshold);
	driver_remove_file(driver, &driver_attr_init_endstops);
	driver_remove_file(driver, &driver_attr_eoc_int_modulo_mask);
	driver_remove_file(driver, &driver_attr_DMA_BLOCK_LEN_bytes);

	acq200_transform_rm_sysfs(driver);
}




extern struct proc_dir_entry* proc_acq200;




static int acq200_proc_block_stats(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
#define DG_PRINTF( glob, fmt ) \
    len += PRINTF( "%30s:" fmt "\n", #glob, DG->glob )

	len = 0;
	
	DG_PRINTF( stats.refill_blocks, "%Lu" );
	DG_PRINTF( stats.sendfile_bytes, "%lu" );

	return len;
#undef DG_PRINTF
#undef PRINTF
}


/***********************************************************************
 *
 * FIFO HISTOGRAMS
 *
 ***********************************************************************/

#define NLINES 10
#define HMARK  "*"
#define HBLANK " "
#define INTROFMT "%10s :"
#define INTROFMD "%10d :"
#define RULER  "----------------"
#define NL "\n"
#define BINS   "0123456789abcdef"
#define TITLE INTROFMT "%-16s"

struct histogram {
	unsigned *data;
	int itotal;
	char *title;
};

static char *histo_line(struct histogram *hg, int iline)
{
	static char a_line[80];
	int ibin;
	int threshold;

	switch(iline){
	case -1:
		sprintf(a_line,INTROFMT RULER, "");
		return a_line;
	case -2:
		sprintf(a_line, INTROFMT BINS, "");
		return a_line;
	case -3:
		sprintf(a_line, TITLE, "", hg->title);
		return a_line;
	default:
		;
	}

	if (hg->itotal == 0){	       
		for (ibin = 0, hg->itotal = 1; ibin != NHISTO; ++ibin){
			hg->itotal += hg->data[ibin];
		}
	}

	threshold = hg->itotal*iline/NLINES;
	sprintf(a_line, INTROFMD, threshold);
	
	for (ibin = 0; ibin != NHISTO; ++ibin){
		strcat(a_line, hg->data[ibin]>threshold? HMARK: HBLANK);
	}
	return a_line;
}

static int acq200_proc_coldpoint_histo(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
#if defined (ACQ216) || defined (WAV232)
	struct histogram cold_fifo = {
		.data = DG->stats.cold_fifo_histo,
		.title = "cold fifo isr"
	};

	struct histogram cold_fifo2 = {
		.data = DG->stats.cold_fifo_histo2,
		.title = "cold fifo eoc"
	};
#endif
#if defined(ACQ196) || defined(ACQ132)
	struct histogram hot_fifo = {
		.data = DG->stats.hot_fifo_histo,
		.title = "hot fifo isr"
	};
	struct histogram ao_fifo = {
		.data = DG->stats. ACQ196_AO_HISTO,
		.title = "FAWG isr"
	};
#endif
	struct histogram hot_fifo2 = {
		.data = DG->stats.hot_fifo_histo2,
		.title = "hot fifo eoc"
	};

	struct histogram *hg[] = {
#if defined(ACQ196) || defined(ACQ132)
		&hot_fifo, 
#endif
#if defined (ACQ216) || defined (WAV232)
		&cold_fifo, &cold_fifo2, 
#endif
		&hot_fifo2,
#if defined(ACQ196) || defined(ACQ132)
		&ao_fifo
#endif
	};
#define NIG (sizeof(hg)/sizeof(struct histogram *))

	int iline;
	int ig;

	for ( iline = NLINES-1, len = 0; iline >= 0; --iline ){
		for (ig = 0; ig != NIG; ++ig){
			len += PRINTF("%s",   histo_line(hg[ig], iline));
		}
		len += PRINTF("\n");
	}

	for (iline = 1; iline <= 3; ++iline){
		for (ig = 0; ig != NIG; ++ig){
			len += PRINTF( "%s", histo_line(hg[ig],-iline));
		}
		len += PRINTF("\n");
	}
	return len;

#undef HMARK
#undef HBLANK
#undef PRINTF
#undef NLINES
}



static int _proc_histo_detail(
	char *buf, char **start, off_t offset, int len,
        int* eof, void* data, struct histogram* hg )
{
	int ibin;
	len = 0;
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
	len = PRINTF("Detail for %s\n", hg->title);

	for (ibin = 0; ibin != NHISTO; ++ibin){
		len += PRINTF("%2d : %8d\n", ibin, hg->data[ibin]);
	}

	return len;
}


#ifdef ACQ216

static int acq200_proc_coldpoint_histo_detail(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	struct histogram cold_fifo = {
		.data = DG->stats.cold_fifo_histo,
		.title = "cold fifo isr"
	};
	return _proc_histo_detail(buf, start, offset, len,
				  eof, data, &cold_fifo);
}

#else
static int acq200_proc_hotpoint_histo_detail(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	struct histogram hot_fifo = {
		.data = DG->stats.hot_fifo_histo,
		.title = "hot fifo isr"
	};
	return _proc_histo_detail(buf, start, offset, len,
				  eof, data, &hot_fifo);
}
#endif










static int acq200_proc_ipc_state(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
#define IPC_PRINTF( field )						\
        len += PRINTF( "%8s: %8p %5d %5d %5d %5d %8d %8d %4d %4d\n",	\
                #field,							\
                DG->ipc->field.buffers,					\
		RBLEN, DG->ipc->field.iput,				\
		DG->ipc->field.iget,					\
		rb_entries( &DG->ipc->field ),				\
		DG->ipc->field.nput, DG->ipc->field.nget,		\
                DG->ipc->field.lotide,					\
                DG->ipc->field.hitide )

	len = 0;
	len += PRINTF("%8s: %8s %5s %5s %5s %5s %8s %8s %4s %4s\n", 
		      "ipc", "buffers", "len", "put", "get", "nQ", 
		      "nput", "nget", "lt", "ht");
	IPC_PRINTF(empties);
	IPC_PRINTF(active);
	IPC_PRINTF(endstops);

	return len;
#undef IPC_PRINTF
#undef PRINTF
}


static int acq200_proc_streambuf(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
/*
 * Warning - needs lock prot 
 */	
	struct list_head* clients = &DG->dcb.clients;

	len = 0;

	if (!list_empty(clients)){
		struct DataConsumerBuffer *dcb;
		
		list_for_each_entry(dcb, clients, list) {	
			len += u32rb_printf(buf+len, &dcb->rb);
		}
	}else{
		len += PRINTF("No Consumer\n");
	}

	return len;
#undef PRINTF
}





static int acq200_proc_stat_workorder(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
#define WPRINTF( field, fmt ) \
        len += PRINTF( "%30s: "fmt"\n", #field, DMC_WO->field )

	len = 0;

	WPRINTF(direction, "%d");
	WPRINTF(buf, "%p");
	WPRINTF(pa, "0x%08x");
	WPRINTF(wo_len, "%d");
	WPRINTF(next_empty, "%d");
	WPRINTF(next_load, "%d");
	if (DMC_WO->error){
		WPRINTF(error, "%s");
	}else{
		PRINTF("%30s: %s\n", "", "NO ERRORS");
	}
	return len;
#undef WPRINTF
#undef PRINTF
}


static int acq200_proc_bda(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
#define WPRINTF( field, fmt ) \
        len += PRINTF( "%30s: "fmt"\n", #field, DMC_WO->field )
#define IDENT(fn) DMC_WO->getNextEmpty == (fn)? #fn:

	len = 0;
	len += PRINTF("%30s->%s\n",
		      "getNextEmpty",
#ifdef WAV232
		      IDENT(wav232_getNextEmpty)
#else
		      IDENT(default_getNextEmpty)
		      IDENT(bda_getNextEmpty)
#endif
		      IDENT(NULL) "unknown" );
 
	return len;
#undef IDENT
#undef WPRINTF
#undef PRINTF
}

static int acq200_proc_stat_pits(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) sprintf(buf+mylen, fmt, ## args)
#define WPRINTF( field, fmt ) \
        mylen += PRINTF( "%30s: "fmt"\n", #field, DMC_WO->field )

	int ipit;
	int mylen = 0;
	int line_len = 10;
	WPRINTF(pit_stop, "%d");
	WPRINTF(pit_count, "%d");

	for (ipit = 0;mylen < len-line_len && ipit !=DMC_WO->pit_count;++ipit){
		line_len = PRINTF( "pit %10d F:%08x OFFSET:0x%08x\n",
			       ipit, 
			       DG->pit_store.the_pits[ipit].status,
			       DG->pit_store.the_pits[ipit].offset );
		mylen += line_len;
	}
	return mylen;
#undef WPRINTF
#undef PRINTF
}


static int acq200_proc_stat_bda(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) sprintf(buf+mylen, fmt, ## args)
	int mylen = 0;
	struct BDA* bda = &DG->stats.bda_times;

	int btime = ABS(bda->before - DG->stats.start_jiffies);
	int dtime = ABS(bda->during - DG->stats.start_jiffies);
	int atime = ABS(bda->after  - DG->stats.start_jiffies);
	int state = DMC_WO->bda_blocks.state;	/* instantaneous state */

	mylen += PRINTF( "B:%10d D:%10d A:%10d blocks\n",
			DMC_WO->bda_blocks.before,
			DMC_WO->bda_blocks.during,
			 DMC_WO->bda_blocks.after);
	mylen += PRINTF( "B:%10d D:%10d A:%10d jiffies state:%s\n",
		bda->before, bda->during, bda->after,
		state == BDA_IDLE? "BDA_IDLE":
		state == BDA_BEFORE? "BDA_BEFORE":
		state == BDA_DURING? "BDA_DURING":
		state == BDA_AFTER? "BDA_AFTER": "BDA_DONE");
	mylen += PRINTF( "B:%10d D:%10d A:%10d ms\n",
			 btime, dtime, atime);				       
	return mylen;
#undef PRINTF
}


static int acq200_proc_stat_events(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	int ipit;
	int llen = 0;
#define PRINTF(fmt, args...) sprintf(buf+_len, fmt, ## args)
#define WPRINTF( field, fmt ) \
        _len += PRINTF( "%30s: "fmt"\n", #field, DMC_WO->field )

	int _len = 0;

	WPRINTF(pit_stop, "%d");
	WPRINTF(pit_count, "%d");

	for (ipit = 0; ipit != DMC_WO->pit_count; ++ipit){
		if (_len + llen > len){
			_len += PRINTF("...\n");
			break;
		}
		_len += llen = PRINTF( "pit %3d F:%08x OFFSET:0x%08x va:%p\n",
			       ipit, 
			       DG->pit_store.the_pits[ipit].status,
			       DG->pit_store.the_pits[ipit].offset,
			       va_buf(DG)+DG->pit_store.the_pits[ipit].offset);
	}
	return _len;
#undef DG_PRINTF
#undef WPRINTF
#undef PRINTF
}
static int acq200_proc_stat_dg(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
#define DGPRINTF( field, fmt ) \
        len += PRINTF( "%30s: "fmt"\n", #field, DG->field )

	len = 0;

	DGPRINTF( open_count, "%d" );
	DGPRINTF( ifill, "%d" );
	DGPRINTF( dma_handle, "0x%08x" );
	DGPRINTF( direction, "%d" );
	DGPRINTF( fiferr, "0x%08x" );
	DGPRINTF( FIFERR, "0x%08x" );
	DGPRINTF( head,   "0x%p" );
	DGPRINTF( finished_with_engines, "%d" );
	DGPRINTF( stats.starve_line, "%d" );
	DGPRINTF( stats.starve_fifcon, "%08x" );	
	DGPRINTF( stats.burst_events_too_fast, "%d" );
	DGPRINTF( stats.local_pulse_count, "%d" );
	len += PRINTF( "%30s: " "0x%08x %s\n", 
		       "errflags", DG->fiferr&DG->FIFERR, 
		       ARCH_FIFERR_DESCRIPTION(DG->fiferr&DG->FIFERR));
	len += PRINTF( "%30s: " "%p\n", "DG", DG);
	return len;
#undef DGPRINTF
#undef PRINTF
}


static int acq200_proc_tblock_cursor(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
#define DGPRINTF( field, fmt ) \
        len += PRINTF( "%30s: "fmt"\n", #field, DG->field )

	len = 0;

	DGPRINTF( bigbuf.tblocks.blocklen, "%x" );
	DGPRINTF( bigbuf.tblocks.nblocks, "%d" );
	DGPRINTF( bigbuf.tblocks.cursor, "%d" );
	DGPRINTF( bigbuf.tblocks.blt, "%p" );
	DGPRINTF( bigbuf.tblocks.transform, "%p" );

	return len;
#undef DGPRINTF
#undef PRINTF
}



static int acq200_proc_free_tblocks(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	struct TblockListElement* tle;
	int iblock = 0;

	len = 0;

#define PRINTF(fmt, args...) len += sprintf(buf+len, fmt, ## args)

	PRINTF("list of free tblocks\n");

	list_for_each_entry(tle, &DG->bigbuf.free_tblocks, list){
		PRINTF("%2d %2d %08x\n", 
		       iblock, tle->tblock->iblock, tle->tblock->offset);
		++iblock;
	}
#undef PRINTF
	return len;
}



static int list_length(struct list_head* list)
{
	struct list_head* pos;
	int entries = 0;

	list_for_each(pos, list){
		++entries;
	}

	return entries;
}

static int acq200_proc_empty_tblocks(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	struct TblockListElement* tle;
	int iblock = 0;

	len = 0;

#define PRINTF(fmt, args...) len += sprintf(buf+len, fmt, ## args)

	PRINTF("list of empty tblocks %d\n", 
	       list_length(&DG->bigbuf.empty_tblocks) );

	list_for_each_entry(tle, &DG->bigbuf.empty_tblocks, list){
		PRINTF("%2d %2d %08x\n", 
		       iblock, tle->tblock->iblock, tle->tblock->offset);
		++iblock;
	}

	PRINTF("pool of tles: %d\n", list_length(&DG->bigbuf.pool_tblocks));
#undef PRINTF
	return len;
}



static void *tblocks_seq_start(struct seq_file *sfile, loff_t *pos)
/* *pos == iblock. returns next TBLOCK* */
{
	if (*pos >= DG->bigbuf.tblocks.nblocks){
		return NULL;
	}else{
		return pos;
	}
}

static void *tblocks_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos >= DG->bigbuf.tblocks.nblocks){
		return NULL;
	}else{
		return pos;
	}
}

static void tblocks_seq_stop(struct seq_file *s, void *v)
{

}

static int tblocks_seq_show(struct seq_file *sfile, void *v)
{
	int ib = *(int*)v;
	struct TBLOCK *tblock = &DG->bigbuf.tblocks.the_tblocks[ib];
	unsigned offset;
	const char* stat = "-";
	int len;

	tblock_lock(tblock);
	switch(atomic_read(&tblock->in_phase)){
	case 1:			
		if (tblock->touched){
			stat = "C";
		}else{
			stat = "R";
		}
		break;
	case 0:
		stat = "-"; 
		break;
	default:
		stat = "B";             /* C/R would be nicer */
	}
	offset = tblock->offset;
	tblock_unlock(tblock);


	len = seq_printf(sfile,
			"<bl id=\"%03d\" addr=\"%08x\" st=\"%s\"/>\n", 
		       ib, offset, stat);

	return len;
}

static int tblocks_proc_open(struct inode *inode, struct file *file)
{
	static struct seq_operations tblocks_seq_ops = {
		.start = tblocks_seq_start,
		.next  = tblocks_seq_next,
		.stop  = tblocks_seq_stop,
		.show  = tblocks_seq_show
	};
	return seq_open(file, &tblocks_seq_ops);
}

static void acq200_create_proc_tblocks(
	struct proc_dir_entry* root, const char *fname)
{
	static struct file_operations tblocks_proc_fops = {
		.owner = THIS_MODULE,
		.open = tblocks_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = seq_release
	};
	struct proc_dir_entry *tblocks_entry =
		create_proc_entry(fname, S_IRUGO, root);	
	if (tblocks_entry){
		tblocks_entry->proc_fops = &tblocks_proc_fops;
	}
}


static int acq200_proc_tblocks_brief(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	int ib;
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
#define DGPRINTF( field, fmt ) \
        len += PRINTF( "%30s: "fmt"\n", #field, DG->field )

	len = 0;

	for (ib = 0; ib != DG->bigbuf.tblocks.nblocks; ++ib){
		struct TBLOCK *tblock = &DG->bigbuf.tblocks.the_tblocks[ib];
		len += PRINTF( "%d,", atomic_read(&tblock->in_phase));
	}  
	len += PRINTF("\n");
	return len;
#undef DGPRINTF
#undef PRINTF
}


static int acq200_proc_capdef(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) len += sprintf(buf+len, fmt, ## args)
#define FPRINTF( base, field, fmt ) \
        PRINTF( "%30s: "fmt"\n", #base "." #field, base->field )

	len = 0;
	
	FPRINTF(CAPDEF, demand_len,     "%d");
	FPRINTF(CAPDEF, demand_postlen, "%d");
	FPRINTF(CAPDEF, demand_prelen,  "%d");
	FPRINTF(CAPDEF, channel_mask,   "%x");
	FPRINTF(CAPDEF, _nchan,         "%d");
	FPRINTF(CAPDEF, _word_size,     "%d");
	FPRINTF(CAPDEF, mode,           "%d");
	FPRINTF(CAPDEF, pit_stop,       "%d");

	return len;
#undef FPRINTF
#undef PRINTF
}

static int u32rb_diags(char* buf, struct u32_ringbuffer* rb) 
{
	int len = 0;
#define PRINTF(fmt, args...) len += sprintf(buf+len, fmt, ## args)

	PRINTF("%30s: %d\n", "nput", rb->nput);
	PRINTF("%30s: %d\n", "nget", rb->nget);
	PRINTF("%30s: %d\n", "backlog", rb->nput - rb->nget);
	PRINTF("%30s: %d\n", "noput", rb->noput);
#undef PRINTF
	return len;
}

static int acq200_proc_dcb(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) len += sprintf(buf+len, fmt, ## args)
#define FPRINTF( base, field, fmt ) \
        PRINTF( "%30s: "fmt"\n", #base "." #field, base.field )
	struct DataConsumerBuffer dcb = {};

#if 0
	spin_lock(&DG->dcb.lock);
	if (DG->dcb.buffer){
		memcpy(&dcb, DG->dcb.buffer, sizeof(dcb));
	}
	spin_unlock(&DG->dcb.lock);
#else
#warning WORKTODO: dcb comment out
#endif
	len = 0;
	
	FPRINTF(dcb, handle,      "0x%08x");
	FPRINTF(dcb, last_start,  "0x%08x");
	FPRINTF(dcb, last_finish, "0x%08x");

	FPRINTF(dcb, overrun,     "  %d");
	FPRINTF(dcb, burst_number,"  %d");
	FPRINTF(dcb, scc.scc,         "  %Lu");
	FPRINTF(dcb, scc.scc_since_e2,"  %d");
	FPRINTF(dcb, state,       "  %d");
	FPRINTF(dcb, pdt.state,   "  %d");
	FPRINTF(dcb, pdt.pulses_left, "  %d");

	len += u32rb_diags(buf+len, &dcb.rb);
	return len;
#undef FPRINTF
#undef PRINTF
}

static int acq200_proc_stat_event_counts(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) len += sprintf(buf+len, fmt, ## args)
#define DG_PRINTF( glob, fmt ) \
        PRINTF( "%30s:" fmt "\n", #glob, DG->glob )

	len = 0;

	DG_PRINTF(stats.event0_count, "%d");
	DG_PRINTF(stats.event1_count, "%d");
	DG_PRINTF(stats.event0_count2, "%d");
	DG_PRINTF(stats.event1_count2, "%d");
	DG_PRINTF(stats.event0_too_early, "%d");
	DG_PRINTF(stats.event1_too_early, "%d");

	DG_PRINTF(stats.dcb_flow_control_throttled_count, "%d");
	DG_PRINTF(stats.dcb_flow_control_event_discards, "%d");
	DG_PRINTF(stats.dcb_flow_control_discards, "%d");
	return len;
#undef DG_PRINTF
#undef PRINTF
}


static int acq200_proc_phases(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) len += sprintf(buf+len, fmt, ## args)
#define FPRINTF( base, field, fmt ) \
        len += PRINTF( "%30s: "fmt"\n", #base "." #field, base->field )
#define TPRINTF        PRINTF( "%c %8s %9s %9s %c %8s %8s %8s %8s %8s\n",	\
			       'S', "phase", 				\
			       "demand", "actual", '1', 		\
			       "start_off", "end_off", 			\
			       "next", "prev", "onPIT" )
#define PPRINTF(phase)						\
        PRINTF( "%c %p %9d %9d %c%c %08x %08x %p %p %p\n",	\
                phase == DMC_WO->now? '*':			\
		phase->is_orig? 'o':'_',			\
		phase,						\
		phase->demand_len, phase_len(phase),		\
                phase->is_oneshot?'1':'c',			\
                phase->event_count? 'E':'_',			\
		phase->start_off, phase->end_off,		\
		phase->list.next, phase->list.prev,		\
                phase->onPIT)					\
		 
	struct Phase *phase;

	len = 0;

	TPRINTF;

	list_for_each_entry(phase, &DMC_WO->phases, list){
		if (phase == 0){
			PRINTF("NULL Phase\n");
			break;
		}else{
			PPRINTF(phase);
			if (phase->orig){
				PPRINTF(phase->orig);
			}
		}
	}
	return len;
#undef FPRINTF
#undef PRINTF
}


static int acq200_proc_phase_tblocks(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) len += sprintf(buf+len, fmt, ## args)
	struct Phase *phase;

	len = find_event_diag(buf, 128);

	list_for_each_entry(phase, &DMC_WO->phases, list){
		if (phase == 0){
			break;
		}else{
			PRINTF("Phase %16s %p: samples  %7d"
			       "start %7d end %7d\n", 
			       phase->name,
			       phase, phase_num_samples(phase),
			       phase->start_sample,
			       phase->start_sample+phase_num_samples(phase));
			len += acq200_phase_tblocks_diag(phase, buf+len);
		}
	}	
	return len;
#undef PRINTF
}



static const char* getStatus(void)
{
	switch(DMC_WO_getState()){
	case ST_RUN:
		return "RUN";
	case ST_ARM:
		return "ARMED";
	default:
		if (DMC_WO->finished_code == 1){
			return "OK";
		}else if (DMC_WO->finished_code < 0){
			return "FAIL";
		}else{
			return "NOT FINISHED";
		}
	}
}



static int acq200_proc_stat_timing(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	int process_ms = 10*(DG->stats.end_jiffies-DG->stats.start_jiffies);
	int process_us = calc_process_us();
	int iblock;
	char nbuf[20];
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
#define FMT "%30s: "
#define DPRINTF( field, fmt ) \
        len += PRINTF( FMT fmt "\n", #field, DG->stats.field )
#define LPRINTF( fmt, title, data ) \
        len += PRINTF( FMT fmt "\n", title, data)

	len = 0;

	LPRINTF("%d", "shot", DG->shot);
	len += PRINTF( "------------- Setting --------------\n");
	LPRINTF("%d", "caplen (bytes)", CAPDEF->demand_len);
	if (sample_size()){
		LPRINTF("%d", "caplen (samples)", 
			CAPDEF->demand_len/sample_size());
	}
	LPRINTF("%04x", "channel mask", CAPDEF->channel_mask);
	len += PRINTF( "------------- Result --------------\n");
	LPRINTF("%s", "status", getStatus());
	LPRINTF("%d", "samples", SAMPLES);    // WORKTODO
	LPRINTF("%lu", "buffer lap count", DMC_WO->bb_lap_count);
	if (DMC_WO->now){
		LPRINTF("%lu", "phase lap count", DMC_WO->now->lap_count);
	}
	DPRINTF(refill_blocks, "%Lu");
	DPRINTF(num_fifo_ints, "%d");
	DPRINTF(num_eoc_ints, "%d");
	DPRINTF(num_eoc_bh, "%d");
	DPRINTF(num_eoc_bh2, "%d");
	DPRINTF(num_dmc_run, "%d");
	DPRINTF(num_eoc_nomatches, "%d");
	DPRINTF(busy_pollcat, "%d" );
	DPRINTF(cdog_trips, "%d");

	len += PRINTF( FMT "%d\n", "blocklen",  DG->dma_block_len);
	len += PRINTF( FMT "[",	"#blocks in dma chain");

	for (iblock = 0; iblock != MAX_DMA_BLOCKS; ++iblock){
		len += PRINTF(" %5d", DG->stats.dma_blocks[iblock]);
		if (iblock == 7){			
			len += PRINTF("\n" FMT " ", " ");
		}		
	}

        len += PRINTF("]\n");

	LPRINTF("%d", "process msecs", process_ms );
	if ( process_ms ){
		LPRINTF("%d", "fifo ints/msec",
			DG->stats.num_fifo_ints/process_ms);
		LPRINTF("%d", "eoc ints/msec",
			DG->stats.num_eoc_ints/process_ms);
	}else{
		LPRINTF("%c", "fifo ints/msec", 'X' );
		LPRINTF("%c", "eoc ints/msec", 'X' );
	}
	LPRINTF("%d", "process usecs", process_us );

	if (DG->stats.num_fifo_ints){
		LPRINTF("%d", "fifo usecs/int",
			process_us/DG->stats.num_fifo_ints);
	}else{
		LPRINTF("%c", "fifo usecs/int", 'X' );
	}	
	_show_measured_sample_rate(nbuf, sizeof(nbuf));
	LPRINTF("%s", "measured sample rate", nbuf);

	return len;
#undef DPRINTF
#undef LPRINTF
#undef PRINTF
}

static int acq200_proc_status_status(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
	len = 0;
	return PRINTF( "%s\n", getStatus());
#undef PRINTF
}
static int acq200_proc_status_samples(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	long spb = DMA_BLOCK_LEN/sample_size();
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
	len = 0;


	return PRINTF( "%d %d %d %Lu\n", 
		       SAMPLES, SAMPLES_PRE, SAMPLES_POST,
		       DG->stats.refill_blocks*spb);
#undef PRINTF
}


static struct proc_dir_entry* proc_status;


static void create_proc_status_entries(void)
{
	proc_status = proc_mkdir("status", proc_acq200);

#define CPRE( name, func ) \
        create_proc_read_entry( name, 0, proc_status, func, 0 )
	CPRE("status",  acq200_proc_status_status);
	CPRE("samples", acq200_proc_status_samples);
#undef CPRE
}

static void delete_proc_status_entries(void)
{
#define RMP( name ) remove_proc_entry( name, proc_status )

	RMP("status");
	RMP("samples");
#undef RMP
}

static int acq200_proc_dumpfiq(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	struct pt_regs regs;
	char *bp = buf;
	int ireg;
	int maxreg = sizeof(struct pt_regs)/sizeof(u32);
#define PRTREG(reg) \
        bp += sprintf(bp, "r%02d :0x%08lx\n", reg, regs.uregs[reg])

	get_fiq_regs(&regs);
	
	for (ireg = 0; ireg < maxreg; ++ireg){
		PRTREG(ireg);
	}
#undef PRTREG
	return bp-buf;
}


extern int acq200_getES_DIAG(void* buf, int max_buf);

static int acq200_proc_dumpES(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	*eof = 1;

	if (offset == 0 && len >= 128){
		return acq200_getES_DIAG(buf, len);
	}else{
		return 0;
	}
}

void create_proc_entries(void)
{
#define CPRE( name, func ) \
        create_proc_read_entry( name, 0, proc_acq200, func, 0 )
	CPRE("dump_ES",    acq200_proc_dumpES);
	CPRE("dump_regs",  acq200_proc_dumpregs);
	CPRE("dump_fiq",   acq200_proc_dumpfiq);
	CPRE("stat_dg",    acq200_proc_stat_dg);
	CPRE("stat_block", acq200_proc_block_stats);
	CPRE("stat_ipc",   acq200_proc_ipc_state);
	CPRE("stat_wo",    acq200_proc_stat_workorder);
	CPRE("stat_ev",    acq200_proc_stat_events);
	CPRE("stat_ev_count", acq200_proc_stat_event_counts);
	CPRE("stat_timing",acq200_proc_stat_timing);
	CPRE("stat_the_pits", acq200_proc_stat_pits);
	CPRE("stat_bda", acq200_proc_stat_bda);
	CPRE("coldpoint_histo",acq200_proc_coldpoint_histo);
#ifdef ACQ216
	CPRE("coldpoint_histo_detail",acq200_proc_coldpoint_histo_detail);
#else
	CPRE("hotpoint_histo_detail",acq200_proc_hotpoint_histo_detail);
#endif
	acq200_create_proc_tblocks(proc_acq200, "tblocks");
	CPRE("tblocks2", acq200_proc_tblock_cursor);
	CPRE("tbmap", acq200_proc_tblocks_brief);
	CPRE("tblocks_free", acq200_proc_free_tblocks);
	CPRE("tblocks_empty", acq200_proc_empty_tblocks);
	CPRE("capdef", acq200_proc_capdef);
	CPRE("phases", acq200_proc_phases);
	CPRE("phase_tblocks", acq200_proc_phase_tblocks);
	CPRE("stream_buf", acq200_proc_streambuf);
	CPRE("dcb", acq200_proc_dcb);

	CPRE("bda", acq200_proc_bda);
	create_proc_status_entries();

	DEVICE_CREATE_PROC_ENTRIES(proc_acq200);
#undef CPRE
}

void delete_proc_entries(void)
{
#define RMP( name ) remove_proc_entry( name, proc_acq200 )

	delete_proc_status_entries();
	RMP("dump_ES");
	RMP("dump_regs");
	RMP("dump_fiq");
	RMP("stat_dg");
	RMP("stat_block");
	RMP("stat_ipc");
	RMP("stat_wo");
	RMP("stat_ev");
	RMP("stat_ev_count");
	RMP("stat_timing");
	RMP("stat_the_pits");
	RMP("stat_bda");
	RMP("coldpoint_histo");
	RMP("coldpoint_histo_detail");
	RMP("tbblock_cursor");
	RMP("tblocks");
	RMP("tblocks2");
	RMP("tbcount");
	RMP("capdef");
	RMP("phases");
	RMP("phase_tblocks");
	RMP("dcb");
	RMP("bda");
#undef RMP
}
