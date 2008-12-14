/* ------------------------------------------------------------------------- */
/* acq216-fifo-procfs.c                                                      */
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
#define ACQ216 

#include <linux/seq_file.h>

#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"
#include "acq200-fifo.h"
#include "acq216.h"



static const char* report_fifo_flags(u32 flags);

#define ARCH_FIFERR_DESCRIPTION(flags) report_fifo_flags(flags)

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

static void acq216_mk_dev_sysfs(struct device *dev);
static void acq216_create_proc_entries(struct proc_dir_entry* root);


#define DEVICE_MK_DEV_SYSFS(dev) acq216_mk_dev_sysfs(dev)
#define DEVICE_CREATE_PROC_ENTRIES(root) acq216_create_proc_entries(root)



static inline u32 arch_get_int_clk_div(void)
{
	return *ACQ200_CLKDAT;
}

static inline u32 arch_set_int_clk_div(u32 clkdiv)
{
	return *ACQ200_CLKDAT = clkdiv;
}




#include "acq200-fifo-procfs.c"




DEFINE_EVENT_ATTR(1);
DEFINE_SIGNAL_ATTR(ob_clk_src);




static ssize_t show_channel_mapping(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
#define PRINTF(fmt, args...) len += sprintf(buf+len, fmt, ## args)
	int len = 0;
	int lchan = 0;
	int nchan = NCHAN;
	int active = 0;

	for (lchan = 1; active <= nchan; ++lchan){
		int pchan = acq200_lookup_pchan(lchan);
		if (pchan >= 0){
			PRINTF("%d %d\n", lchan, pchan);
		}else{
			active += 1;
		}

		if (lchan > 96){
			err("lost the plot at 96");
			return len;
		}
	}
	return len;
#undef PRINTF
}
static DEVICE_ATTR(channel_mapping, S_IRUGO, show_channel_mapping, 0);


static ssize_t show_channel_mapping_bin(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	short *sb = (short*)buf;
	int lchan = 0;
	int nchan = NCHAN;

	for (lchan = 1; lchan <= nchan; ++lchan){
		*sb++ = (short)acq200_lookup_pchan(lchan);
	}
	return nchan*sizeof(short);
#undef PRINTF
}
static DEVICE_ATTR(channel_mapping_bin, S_IRUGO, show_channel_mapping_bin, 0);

static u32 ob_clock_word;
static int actual_khz;

static ssize_t show_ob_clock_word(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	if (ob_clock_word&ACQ200_CLKDAT_CLKDIV){
		return sprintf(buf, "0x%08x %d kHz derived %d kHz\n", 
		       ob_clock_word, 
		       actual_khz*(ob_clock_word&ACQ200_CLKDAT_CLKDIV),
		       actual_khz);
	}else{
		return sprintf(buf, "0x%08x %d kHz\n",
			       ob_clock_word, actual_khz);
	}
}


static ssize_t store_ob_clock_word(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	u32 obw;

	if (sscanf(buf, "0x%08x %d", &obw, &actual_khz) == 2){
		*ACQ200_CLKCON &= 
			~(ACQ200_CLKCON_EC_MASK|ACQ200_CLKCON_EXTCLK);

		activateSignal(CAPDEF->ob_clk_src);

		CAPDEF->int_clk_src->DIx = 
				ACQ200_CLKCON_CS_OBC>>ACQ200_CLKCON_CS_SHIFT;
		activateSignal(CAPDEF->int_clk_src);

		*ACQ200_CLKDAT = obw;
		*ACQ200_CLKCON |= ACQ200_CLKCON_EC_OBC;
		ob_clock_word = obw;
	}


	return strlen(buf);
}


static DEVICE_ATTR(ob_clock_word, S_IRUGO|S_IWUGO,
		   show_ob_clock_word, store_ob_clock_word);




static ssize_t show_clock_freq(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
	u32 fw = *ACQ200_CLKDAT;
	u32 clkdiv = fw & ACQ200_CLKDAT_CLKDIV;
	u32 n66 = (fw & ACQ200_CLKDAT_FREQ) >> ACQ200_CLKDAT_FREQ_SHL;

	u32 clkcon = *ACQ200_CLKCON;
	const char* clock_in_use;

/** for the case of EXTCLK, we have the divider to ourselves,
 *  so adjust for best readback.
 *  NB: this may not be strictly true - could be derived clock output
 * this puts onus on app software to be VERY careful about sequence :-(.
 */
	if ((clkcon & ACQ200_CLKCON_EXTCLK) && 
	    !(clkcon & ACQ200_CLKCON_CLKMAS)      ){
		if (n66 > 220){
			clkdiv -= 1;
		}else if (n66 < 100){
			clkdiv *= 2;
		}else if (n66 < 200){
			clkdiv += 1;
		}
		if (clkdiv < 2 ) clkdiv = 2;
		if (clkdiv > 255) clkdiv = 255;

		*ACQ200_CLKDAT = 
			(ob_clock_word & ~ACQ200_CLKDAT_CLKDIV) | clkdiv;
	}
	if (clkcon&ACQ200_CLKCON_EXTCLK){
		if ((clkcon&ACQ200_CLKCON_EC_MASK) == ACQ200_CLKCON_EC_OBC){
			clock_in_use = "AI";
		}else{
			clock_in_use = "not ai";
		}
	}else{
		clock_in_use = "AI div";
	}

	if (n66 > 0 && n66 < 255){
		if (clkcon & ACQ200_CLKCON_EXTCLK){
			return sprintf(buf,"%d kHz %d x %d / %d %s\n", 
				       acq200_rounding(66667*clkdiv/n66, n66), 
				       clkdiv, 66667, n66,
				       clock_in_use);
		}else{
			return sprintf(buf,"%d kHz %d / %d %s\n", 
				       acq200_rounding(66667/n66, n66),
				       66667, n66,
				       clock_in_use);
		}
	}else{
		return sprintf(buf, "OOR\n");
	}
}

static DEVICE_ATTR(clock_freq, S_IRUGO|S_IWUGO, show_clock_freq, 0);


static ssize_t show_pipeline_offset(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d,%d,%d,%d [4,8,12,16]\n", 
		       CAPDEF->pipeline_offset[0],
		       CAPDEF->pipeline_offset[1],
		       CAPDEF->pipeline_offset[2],
		       CAPDEF->pipeline_offset[3]
		);
}

static ssize_t store_pipeline_offset(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	sscanf(buf, "%d,%d,%d,%d", 
	       &CAPDEF->pipeline_offset[0],
	       &CAPDEF->pipeline_offset[1],
	       &CAPDEF->pipeline_offset[2],
	       &CAPDEF->pipeline_offset[3]
	);
	return strlen(buf);
}
static DEVICE_ATTR(pipeline_offset, S_IRUGO|S_IWUGO,
		   show_pipeline_offset, store_pipeline_offset);


static ssize_t show_gate(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	int gate = (*ACQ200_SYSCON & ACQ200_SYSCON_GATEMODE) != 0;
	return sprintf(buf, "%d\n",  gate);	
}

static ssize_t store_gate(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int gate;
	if (sscanf(buf, "%d", &gate) == 1){
		if (gate){
			*ACQ200_SYSCON |= ACQ200_SYSCON_GATEMODE;
		}else{
			*ACQ200_SYSCON &= ~ACQ200_SYSCON_GATEMODE;
		}
	}
	return strlen(buf);
}
static DEVICE_ATTR(gate, S_IRUGO|S_IWUGO, show_gate, store_gate);

static ssize_t show_antiphase(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	int antiphase = (*ACQ200_SYSCON & ACQ200_SYSCON_ANTIPHASE) != 0;
	return sprintf(buf, "%d\n",  antiphase);	
}

static ssize_t store_antiphase(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int antiphase;
	if (sscanf(buf, "%d", &antiphase) == 1){
		acq216_setAntiPhase(antiphase);
	}
	return strlen(buf);
}
static DEVICE_ATTR(antiphase, S_IRUGO|S_IWUGO, 
		   show_antiphase, store_antiphase);


static ssize_t show_capcom_count(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"%d\n", *ACQ216_CCT_CON & ACQ216_CCT_CAPCOM);
}

static ssize_t store_capcom_count(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
/** set capcom counter (samples). 0 => disables */
{
	unsigned capcom;

	if (sscanf(buf, "0x%x", &capcom) || 
	    sscanf(buf, "%u", &capcom) || 
	    sscanf(buf, "%x", &capcom)        ){
		u32 cct_con = *ACQ216_CCT_CON;
		cct_con &= ~ACQ216_CCT_UPDCTRL;
		cct_con &= ~ACQ216_CCT_CAPCOM;

		capcom = min(capcom, ACQ216_CCT_CAPCOM);

		if (capcom){
			cct_con |= 
				ACQ216_CCT_UPDCTRL_CC<<ACQ216_CCT_UPDCTRL_SHL;
			cct_con |= capcom;
		}
		*ACQ216_CCT_CON = cct_con;
	}
	return strlen(buf);
}
static DEVICE_ATTR(capcom, S_IRUGO|S_IWUGO,
		   show_capcom_count, store_capcom_count);




static ssize_t show_translen(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	int transient = (*ACQ200_SYSCON & ACQ200_SYSCON_TRANSMODE) != 0;
	return sprintf(buf, "%d\n",  transient? *ACQ216_TRANSLEN: 0);	
}	

static ssize_t store_translen(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, size_t count)
{
	int translen;

	if (sscanf(buf, "%d", &translen) == 1){
		if (translen){
			if (translen > 0xffff) translen = 0xffff;
			*ACQ216_TRANSLEN = translen;
			*ACQ200_SYSCON |= ACQ200_SYSCON_TRANSMODE;
		}else{
			*ACQ216_TRANSLEN = 0;
			*ACQ200_SYSCON &= ~ACQ200_SYSCON_TRANSMODE;
		}			
	}
	return strlen(buf);
}
static DEVICE_ATTR(translen, S_IRUGO|S_IWUGO, show_translen, store_translen);




static void acq216_mk_dev_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_clock_freq);
	DEVICE_CREATE_FILE(dev, &dev_attr_ob_clock_word);
	DEVICE_CREATE_FILE(dev, &dev_attr_event1);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mapping);
	DEVICE_CREATE_FILE(dev, &dev_attr_channel_mapping_bin);
	DEVICE_CREATE_FILE(dev, &dev_attr_ob_clk_src);
	DEVICE_CREATE_FILE(dev, &dev_attr_pipeline_offset);
	DEVICE_CREATE_FILE(dev, &dev_attr_gate);
	DEVICE_CREATE_FILE(dev, &dev_attr_antiphase);
	DEVICE_CREATE_FILE(dev, &dev_attr_translen);
	DEVICE_CREATE_FILE(dev, &dev_attr_capcom);
}

void acq200_setChannelMask(unsigned mask)
{
	int nchan;
	unsigned syscon = *ACQ200_SYSCON;
	unsigned daqen = syscon&ACQ200_SYSCON_DAQEN;
	unsigned ap = syscon&ACQ200_SYSCON_ANTIPHASE;
	unsigned fifen = *ACQ200_FIFCON&ACQ200_FIFCON_HC_ENABLE;
	unsigned lchan, mm;

	if (fifen){
		info("refusing to change CMASK with FIFO ENABLED");
		return;
	}
#define MASK_HAS_ONLY(mask, bits)				\
        (((mask) & ~(bits)) == 0 && ((mask) & (bits)) != 0)



	if (!ap){
		if (MASK_HAS_ONLY(mask, 0x000f)){
			nchan = 4; 
			syscon = ACQ200_SYSCON_CH_CONFIG_04;	
			mask = 0x000f;
		}else if (MASK_HAS_ONLY(mask, 0x00ff)){
			nchan = 8; 
			syscon = ACQ200_SYSCON_CH_CONFIG_08;
			mask = 0x00ff;
		}else if (MASK_HAS_ONLY(mask, 0x0fff)){
			nchan = 12; 
			syscon = ACQ200_SYSCON_CH_CONFIG_12;
			mask = 0x0fff;
		}else{
			nchan = 16;
			syscon = ACQ200_SYSCON_CH_CONFIG_16;
			mask = 0xffff;
		}
	}else{
		if (MASK_HAS_ONLY(mask, 0x000f)){
			nchan = 4;
			syscon = ACQ200_SYSCON_CH_CONFAP_02;
			mask = 0x000f;
		}else if (MASK_HAS_ONLY(mask, 0x00f0)){
			nchan = 4;
			syscon = ACQ200_SYSCON_CH_CONFAP_04;
			mask = 0x00f0;
		}else if (MASK_HAS_ONLY(mask, 0x0ff0)){
			nchan = 8;
			syscon = ACQ200_SYSCON_CH_CONFAP_08;
			mask = 0x0ff0;
		}else{
			nchan = 12;
			syscon = ACQ200_SYSCON_CH_CONFAP_12;
			mask = 0xfff0;
		}
	}

	for (lchan = mm = 1; mm; mm<<=1, lchan++){
		acq200_setChannelEnabled(
			acq200_lookup_pchan(lchan), (mm&mask) != 0);
	}

	CAPDEF_set_nchan(nchan);
	CAPDEF->channel_mask = mask;

	*ACQ200_SYSCON &= ~(ACQ200_SYSCON_CH_CONFIG|ACQ200_SYSCON_DAQEN);
	*ACQ200_SYSCON |= syscon | daqen;
}



static ssize_t show_daq_enable(
	struct device_driver * driver, char * buf)
{
	int enable = (*ACQ200_SYSCON&ACQ200_SYSCON_DAQEN) != 0;

        return sprintf(buf,"%d\n", enable);
}


static ssize_t set_daq_enable(
	struct device_driver * driver, const char * buf, size_t count)
{
	int enable;
	sscanf(buf, "%d", &enable);

	if (enable){
		*ACQ200_SYSCON |= ACQ200_SYSCON_DAQEN;
	}else{
		*ACQ200_SYSCON &= ~ACQ200_SYSCON_DAQEN;
	}
	return strlen(buf);
}



int acq200_dumpregs_diag(char* buf, int len)
{
	char *bp = buf;
#define APPEND(reg) \
        bp += snprintf(bp, len - (bp-buf), "%20s:0x%08X\n", #reg, *reg)
	
	APPEND(ACQ200_BDR);
	APPEND(ACQ200_FIFCON);
	APPEND(ACQ200_SYSCON);
	APPEND(ACQ200_CLKCON);
	APPEND(ACQ200_CLKDAT);
	APPEND(ACQ200_DIOCON);
	APPEND(ACQ216_OFFSET_DACS);
	APPEND(ACQ216_TCR_IMM);
	APPEND(ACQ216_TCR_LAT);
	APPEND(ACQ216_TRANSLEN);
	APPEND(ACQ216_CCT_CON);
	APPEND(ACQ216_BLOCKID);

	return bp-buf;
}


static int acq200_proc_dumpregs(
	char *buf, char **start, off_t offset, int len,
                int* eof, void* data )
{
	return acq200_dumpregs_diag(buf, len);
}


static const char* report_fifo_flags(u32 flags)
{
	static char buf[80];
#define PREFIX strlen("ACQ200_FIFCON_")

#define FPRINT(flags, FLAGS)			\
        if (((flags)&(FLAGS)) != 0){		\
		strcat(buf, &#FLAGS[PREFIX]);		\
		strcat(buf, " ");		\
	}

	buf[0] = '\0';

	FPRINT(flags, ACQ200_FIFCON_HOTFULL);
	FPRINT(flags, ACQ200_FIFCON_HOTEMPTY);
	FPRINT(flags, ACQ200_FIFCON_HOTOVER);
	FPRINT(flags, ACQ200_FIFCON_HOTUNDER);

	FPRINT(flags, ACQ200_FIFCON_COLDFULL);
	FPRINT(flags, ACQ200_FIFCON_COLDEMPTY);
	FPRINT(flags, ACQ200_FIFCON_COLDOVER);
	FPRINT(flags, ACQ200_FIFCON_COLDUNDER);

	return buf;
}


struct IPC_DUMP {
	struct acq200_dma_ring_buffer rb;
	int index;
	struct iop321_dma_desc* cursor;
};

#define THIS(s) ((struct IPC_DUMP*)(s->private))


static void* ipc_seq_get(struct seq_file *s)
{
	if (RB_IS_EMPTY(THIS(s)->rb)){
		memset(THIS(s), 0, sizeof(struct IPC_DUMP));
		dbg(1, "EMPTY");
		return NULL;
	}else{
		struct iop321_dma_desc *cursor = 0;
		
		rb_get(&THIS(s)->rb, &cursor);
		THIS(s)->cursor = cursor;
		THIS(s)->index++;

		return THIS(s);
	}
}

static void* ipc_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;
	return ipc_seq_get(s);
}

static void* ipc_seq_start(
	struct seq_file *s, loff_t *pos, struct acq200_dma_ring_buffer *ipc)
{
	loff_t n = *pos;

	dbg(1, "Hello pos %Lu\n", n);

	s->private = kmalloc(sizeof(struct IPC_DUMP), GFP_KERNEL);
	memset(THIS(s), 0, sizeof(struct IPC_DUMP));
	THIS(s)->rb = *ipc;

	if (n == 0){
		seq_printf(s, "<ipc name=\"%s\">\n", ipc->name);
	}
	while (n--){
		if (ipc_seq_get(s) == NULL){
			break;
		}
	}
	
	return ipc_seq_get(s);
}


static void ipc_seq_stop(struct seq_file* s, void *v)
{

	dbg(1, "Goodbye %p index %d", v, THIS(s)->index);

	if (v == NULL){
		seq_printf(s, "</ipc>\n");
	}
	kfree(THIS(s));
}

const char* acq216_identifyInsert(void* fun);

static int seq_printf_dma_desc(
	struct seq_file* s, int ix, struct iop321_dma_desc *dmad, char id)
{
	seq_printf(s, "\t<dmad ix=\"%d\" id=\"%c\" >\n", ix, id);

	if (dmad != NULL){
		seq_printf(s, "\t\t<NDA>%08x</NDA>\n", dmad->NDA);
		seq_printf(s, "\t\t<PDA>%08x</PDA>\n", dmad->PDA);
		seq_printf(s, "\t\t<PUAD>%08x</PUAD>\n", dmad->PUAD);
		seq_printf(s, "\t\t<LAD>%08x</LAD>\n", dmad->LAD);	
		seq_printf(s, "\t\t<BC>%d</BC>\n",     dmad->BC);
		seq_printf(s, "\t\t<DC>%08x</DC>\n",   dmad->DC);
		seq_printf(s, "\t\t<pa>%08x</pa>\n",   dmad->pa);
	}
	seq_printf(s, "\t</dmad>\n");
	return 0;
}

static int ipc_seq_show_PBChain(
	struct seq_file *s, int index, struct PrebuiltChain *pbc)
{
	int ix;

	seq_printf(s, "<PrebuiltChain ix=\"%d\" length=\"%d\" "
		   "fifo_to_local=\"%d\" local_to_host=\"%d\" "
		   "insert=\"%s\">\n",
		   index, 
		   pbc->length, 
		   pbc->fifo_to_local, pbc->local_to_host,
		   acq216_identifyInsert(pbc->insert)  );

	for (ix = 0; ix != pbc->length; ++ix){
		seq_printf_dma_desc(s, ix, pbc->the_chain[ix], pbc->id[ix]);
	}
		       
	seq_printf(s, "</PrebuiltChain>\n" );
	return 0;
}

static int ipc_seq_show(struct seq_file *s, void *v)
{
	struct iop321_dma_desc* cursor = ((struct IPC_DUMP*)v)->cursor;
	int index = ((struct IPC_DUMP*)v)->index;

	if (isPBChainDesc(cursor)){
		return ipc_seq_show_PBChain(s, index, getPBChain(cursor));
	}else if (cursor->clidat != NULL){
		return ipc_seq_show_PBChain(
			s, index, (struct PrebuiltChain *)cursor->clidat);
	}else{
		return seq_printf_dma_desc(s, index, cursor, 'x');
	}
	return 0;
}


static void* endstops_seq_start(struct seq_file *s, loff_t *pos)
{
	return ipc_seq_start(s, pos, &DG->ipc->endstops);
}

static void* empties_seq_start(struct seq_file *s, loff_t *pos)
{
	return ipc_seq_start(s, pos, &DG->ipc->empties);
}

static void* active_seq_start(struct seq_file *s, loff_t *pos)
{
	return ipc_seq_start(s, pos, &DG->ipc->active);
}


static int endstops_proc_open(struct inode *inode, struct file *file)
{
	static struct seq_operations seq_ops = {
		.start = endstops_seq_start,
		.next = ipc_seq_next,
		.stop = ipc_seq_stop,
		.show = ipc_seq_show
	};
	return seq_open(file, &seq_ops);
}


static int empties_proc_open(struct inode *inode, struct file *file)
{
	static struct seq_operations seq_ops = {
		.start = empties_seq_start,
		.next = ipc_seq_next,
		.stop = ipc_seq_stop,
		.show = ipc_seq_show
	};
	return seq_open(file, &seq_ops);
}

static int active_proc_open(struct inode *inode, struct file *file)
{
	static struct seq_operations seq_ops = {
		.start = active_seq_start,
		.next = ipc_seq_next,
		.stop = ipc_seq_stop,
		.show = ipc_seq_show
	};
	return seq_open(file, &seq_ops);
}

static struct file_operations endstops_proc_fops = {
	.owner = THIS_MODULE,
	.open = endstops_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

static struct file_operations empties_proc_fops = {
	.owner = THIS_MODULE,
	.open = empties_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

static struct file_operations active_proc_fops = {
	.owner = THIS_MODULE,
	.open = active_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};


static void acq216_create_proc_entries(struct proc_dir_entry* root)
{
        struct proc_dir_entry *endstops_entry =
		create_proc_entry("endstops.ipc", S_IRUGO, root);
        struct proc_dir_entry *empties_entry =
		create_proc_entry("empties.ipc", S_IRUGO, root);
        struct proc_dir_entry *active_entry =
		create_proc_entry("active.ipc", S_IRUGO, root);

	if (endstops_entry){
		endstops_entry->proc_fops = &endstops_proc_fops;
	}
	if (empties_entry){
		empties_entry->proc_fops = &empties_proc_fops;
	}
	if (active_entry){
		active_entry->proc_fops = &active_proc_fops;
	}
}
