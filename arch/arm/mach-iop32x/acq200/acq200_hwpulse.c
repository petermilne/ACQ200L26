/* ------------------------------------------------------------------------- */
/* acq200_hwpulse.c - configure acq196 hw pulse gen replaces acq200-pulse.c  */
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

#include <linux/list.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <linux/fs.h>     	/* This is where libfs stuff is declared */
#include <asm/uaccess.h>	/* copy_to_user */


#include <linux/kthread.h>


/* keep debug local to this module */
#define acq200_debug acq200_pulse_debug   


#include "acq200_debug.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq196.h"

int acq200_pulse_debug;
module_param(acq200_pulse_debug, int, 0664);


int acq200_pulse_on_time_ms = 10;
module_param(acq200_pulse_on_time_ms, int, 0664);

int stub = 0;
module_param(stub, int, 0664);

#define VERID "$Revision: 1.1 $ build B1000 "

char acq200_pulse_driver_name[] = "acq200_hwpulse";
char acq200_pulse_driver_string[] = "controls hardware pulse generator";
char acq200_pulse_driver_version[] = VERID __DATE__;
char acq200_pulse_copyright[] = "Copyright (c) 2006 D-TACQ Solutions Ltd";

static struct PulseDef GPD = {
	.pulse_count = 0,
	.ibit = 0x01,
	.active_high = 1,
	.delay_ms = 100,
	.start_delay = 20	
};

 
struct Signal* trig;
struct Signal* gendo;



#define MAX

static u32 make_pgtimer_word(void)
{
#define FIELD_FROM_ZERO(x, shl) (((x)>1? ((x)-1): 0) << (shl))
	u32 the_word = 0;
	if (GPD.pulse_count == 0){
		return the_word;
	}

	if (GPD.pulse_count > ACQ196_PGTIMER_MAX_PULSECOUNT){
		GPD.pulse_count = ACQ196_PGTIMER_MAX_PULSECOUNT;
	}
	if (acq200_pulse_on_time_ms > ACQ196_PGTIMER_MAX_ONTIME){
		acq200_pulse_on_time_ms = ACQ196_PGTIMER_MAX_ONTIME;
	}
	if (GPD.start_delay > ACQ196_PGTIMER_MAX_DELAY){
		GPD.start_delay = ACQ196_PGTIMER_MAX_DELAY;
	}
	if (GPD.delay_ms - acq200_pulse_on_time_ms > 
		ACQ196_PGTIMER_MAX_OFFTIME){
		GPD.delay_ms = 
			acq200_pulse_on_time_ms + ACQ196_PGTIMER_MAX_OFFTIME;
	}

	the_word |= FIELD_FROM_ZERO(GPD.delay_ms - acq200_pulse_on_time_ms,
				    ACQ196_PGTIMER_OFFTIME_SHL);
	the_word |= FIELD_FROM_ZERO(
		GPD.pulse_count, ACQ196_PGTIMER_PULSECOUNT_SHL);
	the_word |= FIELD_FROM_ZERO(
		acq200_pulse_on_time_ms, ACQ196_PGTIMER_ONTIME_SHL);
	the_word |= FIELD_FROM_ZERO(
		GPD.start_delay, ACQ196_PGTIMER_DELTIMER_SHL);

	return the_word;
}
/** "driver" */


static void hwpulse_enable(void)
{
	*ACQ196_PGCSS |= ACQ196_PGCSS_GENEN;
}

static void hwpulse_disable(void)
{
	*ACQ196_PGCSS &= ~ACQ196_PGCSS_GENEN;
}


static void hwpulse_stop(void) 
{
	u32 the_word = make_pgtimer_word();
	the_word &= ~ACQ196_PGTIMER_PULSECOUNT_MASK;
	*ACQ196_PGTIMER = the_word;
	hwpulse_disable();	

	dbg(1, " write 0x%08x", the_word);
}

static void hwpulse_start(void)
{
	u32 the_word = make_pgtimer_word();
	*ACQ196_PGTIMER = the_word;
	hwpulse_enable();

	dbg(1, "write 0x%08x", the_word);
}

/** work: we have no hooks to Start Stop @@worktodo
 *  since we don't want to re-release, set up a work thread to poll the 
 *  state of the system. 
 *  Poll rate doesn't have to be fast - 2Hz, so load is negligible
 */
#define WORK_RATE (HZ/2)

static int please_stop;

static int hwpulse_work(void *clidata) 
{
	struct PulseDef my_gpd;
	int pulse_running = 0;
	wait_queue_head_t waitq;
	init_waitqueue_head(&waitq);

	

	while(1){
		wait_event_interruptible_timeout(
				waitq, please_stop, WORK_RATE);

		if (!please_stop && DMC_WO->state == ST_RUN){
			if (pulse_running){
				if (GPD.pulse_count == 0){
					hwpulse_stop();
					pulse_running = 0;
				}else if (memcmp(&my_gpd, &GPD, sizeof(GPD))){
					hwpulse_start();
					memcpy(&my_gpd, &GPD, sizeof(GPD));
				}				
			}else{
				if (GPD.pulse_count != 0){
					hwpulse_start();
					pulse_running = 1;
					memcpy(&my_gpd, &GPD, sizeof(GPD));
				}
			}			
		}else{
			if (pulse_running){
				hwpulse_stop();
				pulse_running = 0;
			}

			if (please_stop){
				please_stop = 0;
				break;
			}
		}
	}
	
	return 0;
}



static void _start_work(int start) 
{
	static struct task_struct *the_worker;
	

	if (start){
		the_worker = kthread_run(hwpulse_work, NULL, "hwpulse_work");
	}else{
		wait_queue_head_t waitq;
		init_waitqueue_head(&waitq);

		if (the_worker != 0){
			wait_event_interruptible_timeout(
						waitq, !please_stop, 
						WORK_RATE*2);
		}
	}
}

static void start_work(void)
{
	_start_work(1);
}
static void stop_work(void)
{
	_start_work(0);	      
}








/** DUP */

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


/** 
 *  @@null_signal - default signal class
 */
static int def_commit(struct Signal* signal)
{
	dbg(1, "");
	return 0;
}

struct Signal* createSignal(
	const char* name, 
	int minDIx, int maxDIx,
	int DIx, int rising, int is_active,
	int (*commit)(struct Signal* signal)
)
{
	static struct Signal def_signal = {
		.is_active = 0,
		.commit = def_commit
	};
	struct Signal* signal = kzalloc(SIGNAL_SZ, GFP_KERNEL);
	memcpy(signal, &def_signal, SIGNAL_SZ);
	strncpy(signal->name, name, sizeof(signal->name)-1);
	signal->_minDIx = minDIx;
	signal->_maxDIx = maxDIx;
	signal->DIx = DIx;
	signal->rising = rising;
	signal->is_active = is_active;
	if (commit){
		signal->commit = commit;
	}
	return signal;
}

void destroySignal(struct Signal* signal)
{
	kfree(signal);
}


/** DUP ends */

static int _commitPulseTrig(struct Signal *signal)
{
	u32 reg = *ACQ196_PGCSS;
	reg &= ~(ACQ196_PGCSS_TRIGRISING|ACQ196_PGCSS_TRIGSEL);

	reg |= signal->DIx <<  ACQ196_PGCSS_TRIGSEL_SHL;
	if (signal->rising){
		reg |= ACQ196_PGCSS_TRIGSEL;
	}

	*ACQ196_PGCSS = reg;
	return 0;
}

static int _commitPulseGenDO(struct Signal *signal)
{
	u32 reg = *ACQ196_PGCSS;
	reg &= ~(ACQ196_PGCSS_DONOT|ACQ196_PGCSS_DOSEL);

	reg |= signal->DIx <<  ACQ196_PGCSS_DOSEL_SHL;
	if (!signal->rising){
		reg |= ACQ196_PGCSS_DONOT;
	}

	*ACQ196_PGCSS = reg;
	return 0;
}



static ssize_t show_trig(
	struct device* dev, 
	struct device_attribute *attr,
	char *buf)
{
	return show_signal(trig, dev, buf);
}


static ssize_t store_trig(
	struct device *dev, 
	struct device_attribute *attr,
	const char *buf, 
	size_t count)
{
	return store_signal(trig, dev, buf, count);
}


static DEVICE_ATTR(trig, S_IRUGO|S_IWUGO, show_trig, store_trig);

static ssize_t show_gendo(
	struct device* dev, 
	struct device_attribute *attr,
	char *buf)
{
	return show_signal(gendo, dev, buf);
}


static ssize_t store_gendo(
	struct device *dev, 
	struct device_attribute *attr,
	const char *buf, 
	size_t count)
{
	return store_signal(gendo, dev, buf, count);
}


static DEVICE_ATTR(genDO, S_IRUGO|S_IWUGO, show_gendo, store_gendo);



static ssize_t show_pulse_def(
	struct device * dev, 
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf,"c=%-2d ms=%-4d bit=%d active_high=%d delay=%d\n", 
		       GPD.pulse_count, GPD.delay_ms,
		       GPD.ibit, GPD.active_high,
		       GPD.start_delay);
}

static ssize_t store_pulse_def(
	struct device * dev, 
	struct device_attribute *attr,
	const char * buf, 
	size_t count)
{
	struct PulseDef pd = GPD;
	int nfields = sscanf(buf, "c=%d ms=%d bit=%d active_high=%d delay=%d", 
			     &pd.pulse_count,
			     &pd.delay_ms, &pd.ibit, &pd.active_high,
			     &pd.start_delay);

	if (nfields >= 1){
		GPD = pd;
	}
	return strlen(buf);
}
static DEVICE_ATTR(pulse_def, S_IRUGO|S_IWUGO,
		   show_pulse_def, store_pulse_def);


static ssize_t show_version(
	struct device *dev,
	struct device_attribute *attr,
	char * buf)
{
        return sprintf(buf, "%s\n%s\n%s\n%s\n",
		       acq200_pulse_driver_name,
		       acq200_pulse_driver_string,
		       acq200_pulse_driver_version,
		       acq200_pulse_copyright
		);
}

static DEVICE_ATTR(version, S_IRUGO, show_version, 0);



static int mk_pulse_sysfs(struct device *dev)
{
	DEVICE_CREATE_FILE(dev, &dev_attr_version);
	DEVICE_CREATE_FILE(dev, &dev_attr_pulse_def);
	DEVICE_CREATE_FILE(dev, &dev_attr_trig);
	DEVICE_CREATE_FILE(dev, &dev_attr_genDO);
	return 0;
}

static void init_signals(void)
{
	trig = createSignal("trig", 0, 5, 3, 0, 1, _commitPulseTrig);
	trig->has_internal_option = 1;
	trig->commit(trig);
	gendo = createSignal("genDO", 0, 5, 1, 1, 1, _commitPulseGenDO);
	gendo->has_internal_option = 1;
	gendo->is_output = 1;
	gendo->commit(gendo);
}

static void delete_signals(void)
{
	destroySignal(trig);
	destroySignal(gendo);
}
static void acq200_pulse_dev_release(struct device *dev)
{
	info("");
}

static int acq200_pulse_probe(struct device *dev)
{
	info("");
	mk_pulse_sysfs(dev);
	return 0;
}

static int acq200_pulse_remove(struct device *dev)
{
	return 0;
}	


static struct device_driver acq200_pulse_driver = {
	.name     = "acq200_pulse",
	.probe    = acq200_pulse_probe,
	.remove   = acq200_pulse_remove,
	.bus	  = &platform_bus_type,	
};


static u64 dma_mask = 0x00000000ffffffff;

static struct platform_device acq200_pulse_device = {
	.name = "acq200_pulse",
	.id   = 0,
	.dev = {
		.release    = acq200_pulse_dev_release,
		.dma_mask   = &dma_mask
	}

};


static int __init acq200_pulse_init( void )
{
	acq200_debug = acq200_pulse_debug;

	info("%s %s %s",
	     acq200_pulse_driver_name,
	     acq200_pulse_driver_version, acq200_pulse_copyright);

	init_signals();
	start_work();
	return driver_register(&acq200_pulse_driver) ||
		platform_device_register(&acq200_pulse_device);
}


static void __exit
acq200_pulse_exit_module(void)
{
	info("");
	stop_work();
	delete_signals();
	platform_device_unregister(&acq200_pulse_device);
	driver_unregister(&acq200_pulse_driver);
}

module_init(acq200_pulse_init);
module_exit(acq200_pulse_exit_module);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter.Milne@d-tacq.com");
MODULE_DESCRIPTION("Module for 2G Hardware Pulse Generator");

