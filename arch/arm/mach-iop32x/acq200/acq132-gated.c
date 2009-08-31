/* acq132-gated.c acq132 gated capture special stuff                         */
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
 Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                   */
/* ------------------------------------------------------------------------- */

#define DTACQ_MACH 2
#define ACQ132
#define ACQ_IS_INPUT 1

#define MODEL_VERID							\
	"$Id: acq132-fifo.c,v 1.13 2006/10/04 11:14:12 pgm Exp $ B1012\n"

#define FPGA_INT   IRQ_ACQ100_FPGA
#define FPGA_INT_MASK (1<<FPGA_INT)

#define ACQ132_VERID "$Revision: 1.13 $ " __DATE__ " " __TIME__

#define MTTR2	0x80

#include <linux/platform_device.h>

#include "acq200-fifo-top.h"

#include "acq200-fifo-local.h"

#include "acq200-fifo.h"
#include "acq132.h"


#include "acq196-AO.h"



int gpg_busy;
module_param(gpg_busy, int, 0444);

#define GPG_RESET_ON_OPEN	0x1
#define GPG_RESET_ON_CLOSE	0x2
#define GPG_RESET_ON_STOP	0x4
int use_gpg_reset;
module_param(use_gpg_reset, int, 0600);

static spinlock_t gpg_lock = SPIN_LOCK_UNLOCKED;
static pid_t gpg_owner;



static void gpg_onEndShot(void* notused)
{
	if (gpg_owner){
		if (use_gpg_reset&GPG_RESET_ON_STOP){
			reset_gpg();	
		}
		kill_proc(gpg_owner, SIGHUP, 1);			
	}
}

static struct Hookup gpg_end_of_shot_hook = {
	.the_hook = gpg_onEndShot
};
static int acq132_gate_pulse_open(struct inode *inode, struct file *file)
{
	spin_lock(&gpg_lock);
	if (gpg_busy || DMC_WO_getState() != ST_STOP){
		spin_unlock(&gpg_lock);
		return -EBUSY;
	}else{
		gpg_busy = 1;
		gpg_owner = current->pid;
		acq200_add_end_of_shot_hook(&gpg_end_of_shot_hook);
		if (use_gpg_reset&GPG_RESET_ON_OPEN){
			reset_gpg();
		}
	}

	spin_unlock(&gpg_lock);	
	return 0;
}

#define GATE_TO	2	/* timeout, jiffies .. ensures minimal loading */

static ssize_t acq132_gate_pulse_write(
	struct file *file, const char *buf, size_t len, loff_t *offset
	)
{
	static DECLARE_WAIT_QUEUE_HEAD(wq);

	int isend;
	u32 data;
	
	for (isend = 0; isend < len; isend += sizeof(data)){
		int rc;

		if (copy_from_user(&data, buf+isend, sizeof(data))){
			return -EFAULT;
		}
		do {
			rc = wait_event_interruptible_timeout(
				wq, !pulse_fifo_full(), GATE_TO);
		} while (rc == 0);

		if (rc < 0){
			return -ERESTARTSYS;
		}
		// rc > 0 .. we're good to write .. 

		dbg(1, "write %d stat: 0x%08x", isend, *ACQ132_FIFSTAT);

		*ACQ132_GATE_PULSE_FIFO = data;
	}
	*offset += isend;
	return isend;
}

static ssize_t acq132_gate_pulse_release(struct inode *inode, struct file *file)
{
	spin_lock(&gpg_lock);
	gpg_busy = 0;
	gpg_owner = 0;
	acq200_del_end_of_shot_hook(&gpg_end_of_shot_hook);
	if (use_gpg_reset&GPG_RESET_ON_CLOSE){
		reset_gpg();
	}
	spin_unlock(&gpg_lock);
	return 0;
}


struct file_operations acq132_gate_pulse_ops = {
	.open = acq132_gate_pulse_open,
	.release = acq132_gate_pulse_release,
	.write = acq132_gate_pulse_write
};

