
#include <linux/kernel.h>
#include <linux/seq_file.h>

#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <asm/uaccess.h>


#include "acq200_debug.h"
#include "ringbuffer.h"

#include "acq200-fifo.h"

#define EXCLUDE_PBC_INLINES
#include "prebuiltChainUtils.h"

struct IPC_DUMP {
	struct acq200_dma_ring_buffer rb;
	int index;
	struct iop321_dma_desc* cursor;
};

#define THIS(s) ((struct IPC_DUMP*)(s->private))



void prebuilt_insert_local (
	struct PrebuiltChain *_this, 
	struct iop321_dma_desc* _new)
{
	int ilocal = _this->fifo_to_local;
	int l2l = _this->local_to_local;

	mk_link(_this, ilocal, _new, 0);
	_this->the_chain[ilocal] = _new;
	_new->NDA = _this->the_chain[ilocal+1]->pa;	
	_new->clidat = _this;                 /* id we are part of a pbc now */

	if (l2l){
		_this->the_chain[l2l]->MM_SRC = _new->LAD;
	}
}

void prebuilt_insert_local_host(
	struct PrebuiltChain *_this, 
	struct iop321_dma_desc* _new)
{
	int ilocal = _this->fifo_to_local;
	mk_link(_this, ilocal, _new,  0);
	_this->the_chain[ilocal] = _new;
	_new->NDA = _this->the_chain[ilocal+1]->pa;
	_this->the_chain[ _this->local_to_host]->LAD = _new->LAD;
	_new->clidat = _this;                 /* id we are part of a pbc now */
}

void prebuilt_insert_local_host_nodata(
	struct PrebuiltChain *_this, 
	struct iop321_dma_desc* _new)
{
	int ilocal = _this->fifo_to_local;
	mk_link(_this, ilocal, _new, 0);
	_this->the_chain[ilocal] = _new;
	_new->NDA = _this->the_chain[ilocal+1]->pa;
	_new->clidat = _this;                 /* id we are part of a pbc now */
}



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

static const char* pbc_identifyInsert(void* fun);

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
		   pbc_identifyInsert(pbc->insert)  );

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

static const char* pbc_identifyInsert(void* fun)
{
#define FUNID(fun) { fun, #fun }
	struct LUT {
		void* fun;
		const char* fname; 
	} lut[] = {
		FUNID(prebuilt_insert_local),
		FUNID(prebuilt_insert_local_host),
		FUNID(prebuilt_insert_local_host_nodata)
	};
#define NLUT (sizeof(lut)/sizeof(struct LUT))

	int ilut;

	for (ilut = 0; ilut != NLUT; ++ilut){
		if (fun == lut[ilut].fun){
			return lut[ilut].fname + strlen("prebuilt_insert_");
		}
	}

	return "function not identified";
}


void pbc_create_proc_entries(struct proc_dir_entry* root)
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
