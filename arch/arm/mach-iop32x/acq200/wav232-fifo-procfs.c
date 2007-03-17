#define DTACQ_MACH 0
#define WAV232

#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"
#include "wav232-fifo.h"         /* WORKTODO:CAPDEF_PRIVATE dubious load dep */
#include "acq200-fifo.h"
#include "wav232.h"


#ifndef __ASSEMBLER__
static inline u32 arch_get_int_clk_div(void)
{
	return *ACQ200_CLKDAT;
}

static inline u32 arch_set_int_clk_div(u32 clkdiv)
{
	return *ACQ200_CLKDAT = clkdiv;
}

#endif

struct device_driver;
struct device;

static ssize_t show_daq_enable(struct device_driver * driver, char * buf);
static ssize_t set_daq_enable(
	struct device_driver * driver, const char * buf, size_t count);

#ifndef WAV232
/** @@todo - CUTMEPLEASE */
static ssize_t show_event_generic(int ev, struct device * dev, char * buf);
static ssize_t store_event_generic(
	int ev, struct device * dev, const char * buf, size_t count);
#endif

static int acq200_proc_dumpregs(
	char *buf, char **start, off_t offset, int len,
	int* eof, void* data );

static void wav232_mk_dev_sysfs(struct device *dev);

static int getNumTblocks(void);
static ssize_t store_transformer(
	struct device_driver * driver, const char * buf, size_t count);

static void wav232_create_proc_entries(struct proc_dir_entry* root);

#define DEVICE_MK_DEV_SYSFS(dev) wav232_mk_dev_sysfs(dev)
#define DEVICE_CREATE_PROC_ENTRIES(root) wav232_create_proc_entries(root)


static int swap_alt_len;

static void transform_swap_alt_blocks(short *to, short* from, int nwords)
{
	int swap_words = swap_alt_len/2; /* words in swap block      */
	int loop_step = swap_words*2;    /* words processed per loop */

	dbg(1,"to: %p from:%p nwords:%d", to, from, nwords);

	for( ; nwords > 0; 
	     nwords -= loop_step, to += loop_step, from += loop_step){
		dbg(2, "to: %p from:%p cpy:%d", to, from, swap_words);

		memcpy(to+swap_words, from, swap_words*sizeof(short));
		memcpy(to, from+swap_words, swap_words*sizeof(short));
	}
}

void acq200_setChannelMask(unsigned mask)
{
	unsigned bits_to_set = 0;
	u32 syscon;
	int nchan = 0;


	CAPDEF->channel_mask = 0;	

	if ((mask & 0xff000000) != 0){
		nchan = 32;
		CAPDEF->channel_mask = 0xffffffff;
		bits_to_set = WAV232_SYSCON_CH_CONFIG_32;
	}else if ((mask & 0x00ff0000) != 0){
		nchan = 24;
		CAPDEF->channel_mask = 0x00ffffff;
		bits_to_set = WAV232_SYSCON_CH_CONFIG_24;
	}else if ((mask & 0x0000ff00) != 0){
		nchan = 16;
		CAPDEF->channel_mask = 0x0000ffff;
		bits_to_set = WAV232_SYSCON_CH_CONFIG_16;
	}else if ((mask & 0x000000ff) != 0){
		nchan = 8;
		CAPDEF->channel_mask = 0x000000ff;
		bits_to_set = WAV232_SYSCON_CH_CONFIG_08;
	}else{
		/** revert to default 32 channel behaviour */
		nchan = 32;
		CAPDEF->channel_mask = 0xffffffff;
		bits_to_set = WAV232_SYSCON_CH_CONFIG_32;
	}

	CAPDEF_set_nchan(nchan);

	syscon = *WAV232_SYSCON;
	syscon &= ~WAV232_SYSCON_CH_CONFIG_XX;
	syscon |= bits_to_set;
	*WAV232_SYSCON = syscon;
}





#include "acq200-fifo-procfs.c"


int sample_size(void)
{
	return WAVDEF->ntracks * WAVDEF->nchan * sizeof(short);
}


static u16 get_last_stored_value(int itrack, int ichannel)
{
	struct BIGBUF *bb = &DG->bigbuf;
	int samples_in_block = bb->tblocks.blocklen/sample_size();
/* compute INDEX from track_end (one past) */
	unsigned last_offset = WAVDEF->track_ends[itrack][ichannel] - 1;
	int iblock = last_offset/samples_in_block;
	int block_start = iblock*bb->tblocks.blocklen;
	int block_start_sample = block_start/sample_size();
	int ct = itrack*WAVDEF->nchan + ichannel;
	short *bsp = va_buf_s(DG) + samples_in_block*ct;

	int isample_in_block = last_offset - block_start_sample;

	return bsp[isample_in_block];
}

static void get_stored_extents(
	int itrack, int ichannel, short **first, short **last )
{
	struct BIGBUF *bb = &DG->bigbuf;
	int samples_in_block = bb->tblocks.blocklen/sample_size();
/* compute INDEX from track_end (one past) */
	unsigned last_offset = WAVDEF->track_ends[itrack][ichannel] - 1;
	int iblock = last_offset/samples_in_block;
	int block_start = iblock*bb->tblocks.blocklen;
	int block_start_sample = block_start/sample_size();
	int ct = itrack*WAVDEF->nchan + ichannel;
	short *bsp = va_buf_s(DG) + samples_in_block*ct;

	int isample_in_block = last_offset - block_start_sample;

	*first = bsp;
	*last  = bsp+isample_in_block;
}
static void fill_to_end_block(int iblock, int itrack, int ichannel)
{
	struct BIGBUF *bb = &DG->bigbuf;
	unsigned last_offset = WAVDEF->track_ends[itrack][ichannel];
	int block_start = iblock*bb->tblocks.blocklen;
	int block_start_sample = block_start/sample_size();
	int samples_in_block = bb->tblocks.blocklen/sample_size();
	int last_sample_in_block = samples_in_block + block_start_sample;

#define LOCDEB(lv) dbg(3,"%20s %d", #lv, lv)
#define LOCDEBF(lv, fmt) dbg(3, "%20s " fmt, #lv, lv)
	dbg(3, "iblock %d, itrack %d, ichannel %d", iblock, itrack, ichannel);
	LOCDEBF(bb, "%p");
	LOCDEB(last_offset);
	LOCDEB(block_start);
	LOCDEB(block_start_sample);
	LOCDEB(samples_in_block);
	LOCDEB(last_sample_in_block);

	if (last_sample_in_block > last_offset){
		int ct = itrack*WAVDEF->nchan + ichannel;
		short last_value = get_last_stored_value(itrack, ichannel);
		int bss = iblock*bb->tblocks.blocklen/sizeof(short);
		short *bsp = va_buf_s(DG) + bss + samples_in_block*ct;
		int isample_in_block = last_offset > block_start_sample?
			last_offset - block_start_sample: 0;

		LOCDEB(ct);
		LOCDEB(last_value);
		LOCDEBF(bsp, "%p");
		LOCDEB(isample_in_block);

		for ( ; isample_in_block < samples_in_block; ){
			bsp[isample_in_block++] = last_value;
		}

		LOCDEB(isample_in_block);
		LOCDEBF( &bsp[isample_in_block-1], "%p");

		WAVDEF->fill_state[itrack][ichannel] = 
			FILL_STATE_FILL | (iblock<<16) | 
			(unsigned)(last_value&0x0ffff);

		dbg(1, "WAVDEF->fill_state[%d][%d] = 0x%08x\n", 
		    itrack, ichannel, WAVDEF->fill_state[itrack][ichannel] );
	}
	/* else block is already fill for this ct */


#undef LOCDEBF
#undef LOCDEB
}

static void copy_to_end_block(int iblock, int itrack, int ichannel)
{
	struct BIGBUF *bb = &DG->bigbuf;
	unsigned last_offset = WAVDEF->track_ends[itrack][ichannel];
	int block_start = iblock*bb->tblocks.blocklen;
	int block_start_sample = block_start/sample_size();
	int samples_in_block = bb->tblocks.blocklen/sample_size();
	int last_sample_in_block = samples_in_block + block_start_sample;

#define LOCDEB(lv) dbg(3,"%20s %d", #lv, lv)
#define LOCDEBP(lv) dbg(3,"%20s %p", #lv, lv)
#define LOCDEBF(lv, fmt) dbg(3, "%20s " fmt, #lv, lv)
	dbg(3, "iblock %d, itrack %d, ichannel %d", iblock, itrack, ichannel);
	LOCDEBF(bb, "%p");
	LOCDEB(last_offset);
	LOCDEB(block_start);
	LOCDEB(block_start_sample);
	LOCDEB(samples_in_block);
	LOCDEB(last_sample_in_block);

	if (last_sample_in_block > last_offset){
		int ct = itrack*WAVDEF->nchan + ichannel;
		int bss = iblock*bb->tblocks.blocklen/sizeof(short);
		short *bsp = va_buf_s(DG) + bss + samples_in_block*ct;
		int isample_in_block = last_offset > block_start_sample?
			last_offset - block_start_sample: 0;
		short *first;
		short *last;
		short *pfrom;

		get_stored_extents(itrack, ichannel, &first, &last);

		LOCDEB(ct);
		LOCDEBF(bsp, "%p");
		LOCDEB(isample_in_block);
		LOCDEBP(first);
		LOCDEBP(last);
		
		for ( pfrom = first, last++; 
		      isample_in_block < samples_in_block; ){

			bsp[isample_in_block++] = *pfrom++;

			if (pfrom >= last){
				pfrom = first;
			}
		}

		LOCDEB(isample_in_block);
		LOCDEBF( &bsp[isample_in_block-1], "%p");

		WAVDEF->fill_state[itrack][ichannel] = 
			FILL_STATE_COPY | (iblock<<16) | (unsigned)last[-1];

		dbg(1, "WAVDEF->fill_state[%d][%d] = 0x%08x\n", 
		    itrack, ichannel, WAVDEF->fill_state[itrack][ichannel] );

	}
	/* else block is already fill for this ct */


#undef LOCDEBF
#undef LOCDEB
}

static void wav232_fifo_iterate_to_end_block(
	int iblock,
	void (*operate)(int iblock, int itrack, int ichannel ) )
{
/*
 * this runs on cooked data [t][c][s]:
 *
	for each track
			 for each channel
					  calc end value
					  calc end point
					  fill entries in this block
*/
	int itrack;
	int ichannel;
	
	for(itrack = 0; itrack < WAVDEF->ntracks; ++itrack){
		for(ichannel = 0; ichannel < WAVDEF->nchan; ++ichannel){
			operate(iblock, itrack, ichannel);
		}
	}
}


void wav232_transform3(short *to, short *from, int nsamples)
/*
 * wav232:
 * input data: [t][c][s]
 * output data: [s][c&f8][c1..4.t]
 *
 * strategy: collate c1..4 t0, c1..4t1, etc
 * NB changed calling scheme: nwords is the length of each run.
 * so the dimension of from is:
 * from [ntracks][nchannels][nwords]
 * and the dimension of to is:
 * to [nwords][nchannels/4][ntracks][4]
 */
{
#define FROM(cts, isample) from[(cts) + (isample)]
#define C0 (*to++)
#define C1 (*to++)
#define C2 (*to++)
#define C3 (*to++)

#define LOCDEB(lv) dbg(3,"%20s %d", #lv, lv)
#define LOCDEBF(lv, fmt) dbg(3, "%20s " fmt, #lv, lv)

#ifdef PGMCOMOUT
#define DEBFROM \
if ( ((isample&0x3ff)==0 || isample==nsamples-1) && \
     (ichannel==0||ichannel==nchannels-1) )\
dbg(4, "to %p = s:%2d c:%2d t:%d FROM %p [%04x]", \
    to, isample, ichannel, itrack, &FROM(cts+dc, isample), cts+dc)
#else
#define DEBFROM
#endif

	int ntracks = WAVDEF->ntracks;
	int nchannels = WAVDEF->nchan;
	int isample, ichannel, itrack;

	LOCDEBF(to, "%p");
	LOCDEBF(from, "%p");
	LOCDEB(nsamples);
	LOCDEB(ntracks);
	LOCDEB(nchannels);
	
	for (isample = 0; isample != nsamples; ++isample){
		for (ichannel = 0; ichannel < nchannels; ichannel += 4){
			for (itrack = 0; itrack < ntracks; ++itrack){
				int cts = nsamples*
					(itrack*nchannels + ichannel);
				int dc = 0;

				DEBFROM;
				C0 = FROM(cts+dc, isample); dc += nsamples;
				DEBFROM;
				C1 = FROM(cts+dc, isample); dc += nsamples;
				DEBFROM;
				C2 = FROM(cts+dc, isample); dc += nsamples;
				DEBFROM;
				C3 = FROM(cts+dc, isample);
			}
		}
	}

#undef LOCDEB
#undef LOCDEBF

#undef C0
#undef C1
#undef C2
#undef C3
}


static int wav232_fifo_bigbuf_transform(int blocknum)
{
	struct TBLOCK* tblock = &DG->bigbuf.tblocks.the_tblocks[blocknum];
	short *tb_va = tblock_va(blocknum);
	short *tb_tmp = va_tblock_tmp(DG);
	
	unsigned nwords = DG->bigbuf.tblocks.blocklen/sizeof(short);
	unsigned nsamples = DG->bigbuf.tblocks.blocklen/sample_size();

	if (!tb_tmp) return -1;

	dbg(1,"transform(%p %p %d %d)", tb_tmp, tb_va, nwords, NCHAN); 

	
	wav232_transform3(tb_tmp, tb_va, nsamples);

	tblock_lock(tblock);
	if (swap_alt_len){
		transform_swap_alt_blocks(tb_va, tb_tmp, nwords);
	}else{
		DG->bigbuf.tblocks.blt(tb_va, tb_tmp, nwords);
	}
	tblock_unlock(tblock);

	return 0;
}

static int getNumTblocks(void)
/* transform BEFORE capture */
{
	int len;

	if ((len = DG->bigbuf.tblocks.blocklen)){
		return WAVDEF->buffer_maxlen/len + 
			(WAVDEF->buffer_maxlen%len? 1: 0);
	}else{
		return -1;
	}
}


static ssize_t store_transformer(
	struct device_driver * driver, const char * buf, size_t count)
{
	if (DG->bigbuf.tblocks.cursor < getNumTblocks()){
		dbg(1,"wav232 flavor");
		wav232_fifo_bigbuf_transform(DG->bigbuf.tblocks.cursor);
		DG->bigbuf.tblocks.cursor++;
	}
	return strlen(buf);
}





static ssize_t show_tracksel(
	struct device * dev, 
	struct device_attribute *attr,	
	char * buf)
{
	int tracksel;

	if ((*WAV232_SYSCON&WAV232_SYSCON_TRMD_SOFT) != 0){
		tracksel = (*WAV232_SYSCON&WAV232_SYSCON_TRSEL)>>
			WAV232_SYSCON_TRSEL_SHL;
	}else{
/*
 * WORKTODO : where DO we see the hw tracksel? ASNWER: d234
 */
		tracksel = (*ACQ200_DIOCON & 0x1c) >> 2;
	}
        return sprintf(buf,"%d\n", tracksel);
}

static DEVICE_ATTR(tracksel, S_IRUGO, show_tracksel, 0);


static ssize_t show_tracksel_sim(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	u32 syscon = *WAV232_SYSCON;

        if ((syscon&WAV232_SYSCON_TRMD_SOFT) != 0){
		int tracksel = (syscon & WAV232_SYSCON_TRSEL) >> 
			WAV232_SYSCON_TRSEL_SHL;
		return sprintf(buf, "%c%d\n", 'S', tracksel);
	}else{
		return sprintf(buf, "H\n");
	}
}

static ssize_t show_buffer_maxlen(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf,"%d 0x%08x\n",
		       WAVDEF->buffer_maxlen, WAVDEF->buffer_maxlen);
}

static ssize_t store_buffer_maxlen(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	unsigned maxlen;

	if (sscanf(buf, "0x%x", &maxlen) == 1 || 
	    sscanf(buf, "%u", &maxlen)   == 1    ){
		if (maxlen < 128){
			/* treat input as a tblock count */
			maxlen *= DG->bigbuf.tblocks.blocklen;
		}
		if (maxlen > len_buf(DG)){
			maxlen = len_buf(DG);
		}
		WAVDEF->buffer_maxlen = maxlen;
	}
	return strlen(buf);
}
static DEVICE_ATTR(buffer_maxlen, S_IRUGO|S_IWUGO,
		   show_buffer_maxlen, store_buffer_maxlen);


static ssize_t store_tracksel_sim(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	char csel;
	char tsel; 
/* use %c to allow adjacent num no spaces */
	int nscan = sscanf(buf, "%c%c", &csel, &tsel); 
	u32 syscon = *WAV232_SYSCON;
	int tracksel;

	dbg(1, "nscan:%d csel:%d tsel:%c", nscan, csel, tsel);

	switch (nscan){
	case 2:
		tracksel = tsel - '0';
		if ( csel == 'S' && tracksel >=0 && tracksel <= 7 ){
			syscon &= ~WAV232_SYSCON_TRSEL;
			syscon |= WAV232_SYSCON_TRMD_SOFT |
				(tracksel << WAV232_SYSCON_TRSEL_SHL); 
			dbg(1, "tracksel %d", tracksel);
			break;
		}else{
			; /* fall thru */
		}
	default:
		syscon &= ~WAV232_SYSCON_TRMD_SOFT;
	}
	*WAV232_SYSCON = syscon;
	return strlen(buf);
}
static DEVICE_ATTR(tracksel_sim, S_IRUGO|S_IWUGO,
		   show_tracksel_sim, store_tracksel_sim);


static ssize_t show_nchannels(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	int nchannels;
	
	switch (*WAV232_SYSCON & WAV232_SYSCON_CH_CONFIG){
	default:
		nchannels = 32; break;
	case WAV232_SYSCON_CH_CONFIG_24:
		nchannels = 24; break;
	case WAV232_SYSCON_CH_CONFIG_16:
		nchannels = 16; break;
	case WAV232_SYSCON_CH_CONFIG_08:
		nchannels = 8; break;
	}
        return sprintf(buf,"%d\n", nchannels);
}

static ssize_t store_nchannels(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int nchannels = 0;
	u32 ch_config;
	u32 syscon = *WAV232_SYSCON;

	sscanf(buf, "%u", &nchannels);

	switch (nchannels){
	case 32:
		ch_config = WAV232_SYSCON_CH_CONFIG_32; break;
	case 24:
		ch_config = WAV232_SYSCON_CH_CONFIG_24; break;
	case 16:
		ch_config = WAV232_SYSCON_CH_CONFIG_16; break;
	case 8:
		ch_config = WAV232_SYSCON_CH_CONFIG_08; break;
	default:
		return strlen(buf);
	}

	WAVDEF->nchan = nchannels;

	syscon &= ~WAV232_SYSCON_CH_CONFIG;
	syscon |= ch_config;

	*WAV232_SYSCON = syscon;

	return strlen(buf);
}
static DEVICE_ATTR(nchannels, S_IRUGO|S_IWUGO,
		   show_nchannels, store_nchannels);

static ssize_t show_int_clk_src(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	char *src = "bug";

	switch(*WAV232_CLKCON & WAV232_CLKCON_CS_MASK){
	case WAV232_CLKCON_CS_DI0: 
		src = "DI0"; break;
	case WAV232_CLKCON_CS_DI1:
		src = "DI1"; break;
	case WAV232_CLKCON_CS_DI2:
		src = "DI2"; break;
	case WAV232_CLKCON_CS_DI3:
		src = "DI3"; break;
	case WAV232_CLKCON_CS_DI4:
		src = "DI4"; break;
	case WAV232_CLKCON_CS_DI5:
		src = "DI5"; break;
	case WAV232_CLKCON_CS_80M:
		src = "80M"; break;
	case WAV232_CLKCON_CS_66M:
	default:
		src = "66M"; break;
	}
        return sprintf(buf,"%s\n", src);
}
static DEVICE_ATTR(int_clk_src, S_IRUGO, show_int_clk_src, 0);

static ssize_t show_ext_trg_status(
	struct device * dev, 
	struct device_attribute *attr,  
	char * buf)
{
        return sprintf(buf,"%d\n", (*WAV232_DIOSFR&WAV232_DIOSFR_ET_STA) != 0);
}
static DEVICE_ATTR(ext_trg_status, S_IRUGO, show_ext_trg_status, 0);

static ssize_t show_ext_clk_status(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", (*WAV232_DIOSFR&WAV232_DIOSFR_EC_STA) != 0);
}
static DEVICE_ATTR(ext_clk_status, S_IRUGO, show_ext_clk_status, 0);


static ssize_t show_ntracks(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	u32 syscon = *WAV232_SYSCON;
	int ntracks = 
		((syscon & WAV232_SYSCON_NTRACKS) >> WAV232_SYSCON_NTRACKS_SHL)
		+ 1;

        return sprintf(buf,"%d\n", ntracks);
}

static ssize_t store_ntracks(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	u32 syscon = *WAV232_SYSCON;
	int ntracks;
	int nscan = sscanf(buf, "%u", &ntracks);

	if (nscan == 1 && ntracks >= 1 && ntracks <= 8){
		syscon &= ~WAV232_SYSCON_NTRACKS;
		syscon |= (ntracks-1) << WAV232_SYSCON_NTRACKS_SHL;
		WAVDEF->ntracks = ntracks;
	}
	*WAV232_SYSCON = syscon;
	return strlen(buf);
}
static DEVICE_ATTR(ntracks, S_IRUGO|S_IWUGO,
		   show_ntracks, store_ntracks);


static ssize_t clear_all_tracks(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int track;
	int channel;

	for (track = 0; track != WAV232_MAXTRACKS; ++track){
		for (channel = 0; channel != WAV232_MAXCHAN; ++channel){
			WAVDEF->track_ends[track][channel] = 0;
			WAVDEF->fill_state[track][channel] = 0;
		}
	}
	return strlen(buf);
}

static DEVICE_ATTR(clear_all_tracks, S_IWUGO, 0, clear_all_tracks);

static ssize_t show_fill_to_end(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%d %s\n",
		       WAVDEF->fill_cursor, 
		       WAVDEF->fill_cursor == getNumTblocks()?
		       "COMPLETE": "");
}

static ssize_t store_fill_to_end(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (WAVDEF->fill_cursor < getNumTblocks()){
		wav232_fifo_iterate_to_end_block(
			WAVDEF->fill_cursor, fill_to_end_block);
		WAVDEF->fill_cursor++;
	}
	return strlen(buf);
}


static DEVICE_ATTR(fill_to_end,S_IRUGO | S_IWUGO, 
		   show_fill_to_end, store_fill_to_end);


static ssize_t show_copy_to_end(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%d %s\n",
		       WAVDEF->fill_cursor, 
		       WAVDEF->fill_cursor == getNumTblocks()?
		       "COMPLETE": "");
}

static ssize_t store_copy_to_end(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	if (WAVDEF->fill_cursor < getNumTblocks()){
		wav232_fifo_iterate_to_end_block(
			WAVDEF->fill_cursor, copy_to_end_block);
		WAVDEF->fill_cursor++;
	}
	return strlen(buf);
}


static DEVICE_ATTR(copy_to_end,S_IRUGO | S_IWUGO, 
		   show_copy_to_end, store_copy_to_end);


static ssize_t show_swap_alt_len(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "0x%x %d\n", swap_alt_len, swap_alt_len);
}

static ssize_t store_swap_alt_len(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "0x%x", &swap_alt_len) || sscanf(buf, "%d", &swap_alt_len);
	return strlen(buf);
}


static DEVICE_ATTR(swap_alt_len,S_IRUGO | S_IWUGO, 
		   show_swap_alt_len, store_swap_alt_len);





static ssize_t show_fill_to_end0(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	return sprintf(buf, "%d %s\n",
		       WAVDEF->fill_cursor, 
		       WAVDEF->fill_cursor == getNumTblocks()?
		       "COMPLETE": "");
}

static ssize_t store_fill_to_end0(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	static int itrack;
	static int ichannel;
	int selector = 0;

	if (sscanf(buf, "%d", &selector)){
		switch(selector){
		case 1:
		case 0:
			break;
		case -1:
			itrack = ichannel = WAVDEF->fill_cursor = 0;
			break;
		default:
			return strlen(buf);
		}
	}
			
	if (WAVDEF->fill_cursor < getNumTblocks()){
		fill_to_end_block(WAVDEF->fill_cursor, itrack, ichannel);

		if (selector == 1){
			if (++ichannel > WAVDEF->nchan){
				ichannel = 0;
				if (++itrack > WAVDEF->ntracks){
					itrack = 0;
					++WAVDEF->fill_cursor;
				}
			}
		}
	}
	/* now increment itrack, ichannel, iblock */

	return strlen(buf);
}

static DEVICE_ATTR(fill_to_end0,S_IRUGO | S_IWUGO, 
		   show_fill_to_end0, store_fill_to_end0);



static void wav232_mk_dev_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_tracksel);
	DEVICE_CREATE_FILE(dev, &dev_attr_tracksel_sim);
	DEVICE_CREATE_FILE(dev, &dev_attr_nchannels);
	DEVICE_CREATE_FILE(dev, &dev_attr_int_clk_src);
	DEVICE_CREATE_FILE(dev, &dev_attr_ext_trg_status);
	DEVICE_CREATE_FILE(dev, &dev_attr_ext_clk_status);
	DEVICE_CREATE_FILE(dev, &dev_attr_ntracks);
	DEVICE_CREATE_FILE(dev, &dev_attr_fill_to_end);
	DEVICE_CREATE_FILE(dev, &dev_attr_copy_to_end);
	DEVICE_CREATE_FILE(dev, &dev_attr_swap_alt_len);
	DEVICE_CREATE_FILE(dev, &dev_attr_fill_to_end0);
	DEVICE_CREATE_FILE(dev, &dev_attr_buffer_maxlen);
	DEVICE_CREATE_FILE(dev, &dev_attr_clear_all_tracks);
}


static ssize_t show_daq_enable(
	struct device_driver * driver, char * buf)
{
	int enable = (*WAV232_SYSCON&WAV232_SYSCON_DAQEN) != 0;

        return sprintf(buf,"%d\n", enable);
}


static ssize_t set_daq_enable(
	struct device_driver * driver, const char * buf, size_t count)
{
	int enable;
	sscanf(buf, "%d", &enable);

	info("%d", enable);

	if (enable){
		*WAV232_SYSCON |= WAV232_SYSCON_DAQEN;
	}else{
		*WAV232_SYSCON &= ~WAV232_SYSCON_DAQEN;
	}
	return strlen(buf);
}

#ifndef WAV232
/** @@todo events not relevant to WAV. Check we didn't _mean_ trigger */

static ssize_t show_event_generic(int ev, struct device * dev, char * buf)
{
#ifdef WORKTODO
	return sprintf(buf, "ev%d %s %s",
		ev,
		CAPDEF->ev[ev].DIx == ACQ200_SYSCON_EV_DI3? "DI3":
		CAPDEF->ev[ev].DIx == ACQ200_SYSCON_EV_DI4? "DI4":
		CAPDEF->ev[ev].DIx == ACQ200_SYSCON_EV_DI5? "DI5":
		"none",
		CAPDEF->ev[ev].sense == ACQ200_SYSCON_EV_RISING?
		"rising": "falling" );
#else
	return sprintf(buf, "WORKTODO\n" );
#endif
	
}

static ssize_t store_event_generic(
	int ev, struct device * dev, const char * buf, size_t count)
{
#ifdef WORKTODO
	int dix;
	char tok2[20];

	if ( sscanf(buf, "DI%d %10s", &dix, tok2) == 2){
		switch(dix){
		default:
			return 0;
		case 3:
			CAPDEF->ev[ev].DIx = ACQ200_SYSCON_EV_DI3;
			break;
		case 4:
			CAPDEF->ev[ev].DIx = ACQ200_SYSCON_EV_DI4;
			break;
		case 5:
			CAPDEF->ev[ev].DIx = ACQ200_SYSCON_EV_DI5;
			break;			
		}
		CAPDEF->ev[ev].sense = strcmp(tok2, "rising")==0?
			ACQ200_SYSCON_EV_RISING: ACQ200_SYSCON_EV_FALLING;
		return strlen(buf);
	}else{
		return 0;
	}
#else
	return 0;
#endif
}

#endif


int acq200_dumpregs_diag(char* buf, int len)
{
	char *bp = buf;
#define APPEND(reg) \
        bp += snprintf(bp, len - (bp-buf), "%20s:0x%08X\n", #reg, *reg)
	
	APPEND(ACQ200_BDR);
	APPEND(ACQ200_FIFCON);
	APPEND(WAV232_SYSCON);
	APPEND(WAV232_CLKCON);
	APPEND(ACQ200_CLKDAT);
	APPEND(ACQ200_DIOCON);
	APPEND(WAV232_DIOSFR);

	return bp-buf;
}
static int acq200_proc_dumpregs(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	return acq200_dumpregs_diag(buf, len);
}

#define M1 0x100000
#define K64 0x10000

static int wav232_proc_fill_report(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	int ichannel, itrack;
#define PRINTF(fmt, args...) sprintf(buf+len, fmt, ## args)
	len = 0;

	if (!WAVDEF){
		return PRINTF("ERROR:WAVDEF NOT DEFINED!\n");
	}

	for (ichannel = 0; ichannel != WAVDEF->nchan; ++ichannel){

		dbg(1,"ichannel %d", ichannel);

		len += PRINTF( "%02d:", ichannel+1 );
		for (itrack = 0; itrack != WAVDEF->ntracks; ++itrack){
			unsigned fillstate = 
				WAVDEF->fill_state[itrack][ichannel];
			unsigned tlen = WAVDEF->track_ends[itrack][ichannel];
			unsigned fblock = (fillstate&FILL_STATE_BLOCK)>>16;
			char mode = 
				(fillstate&FILL_STATE_FILL)? 'F':
				(fillstate&FILL_STATE_COPY)? 'C':
				(fillstate&FILL_STATE_WRITE)?'W': ' ';
			char tlenu;

			dbg(1, "itrack %d", itrack);

			if (tlen < 1000 ){
				tlenu = ' ';
			}else if (tlen < 999 * 1024 ){
				tlenu = 'k';
				tlen /= 1024;
				tlen = max(1U, tlen); /*handle 999..1023 gap */
			}else{
				tlenu = 'M';
				tlen /= 1024*1024;
				tlen = max(1U, tlen);
			}
				
			len += PRINTF("%c%03dB%03d%c ", 
				      mode, fblock, tlen, tlenu);
		}
		len += PRINTF("\n");
	}
	
	return len;
#undef PRINTF
}



static void wav232_create_proc_entries(struct proc_dir_entry* root)
{
#define CPRE( name, func ) \
        create_proc_read_entry( name, 0, root, func, 0 )	

	CPRE("wav232_fill", wav232_proc_fill_report);
#undef CPRE
}
