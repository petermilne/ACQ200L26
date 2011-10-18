/* ------------------------------------------------------------------------- */
/* acq200-signal.  - signal support					     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2003-9 Peter Milne, D-TACQ Solutions Ltd
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

#include "acq200_debug.h"
#include "acq200-signal.h"



ssize_t acq200_show_signal(
	struct Signal* signal, struct device * dev, char * buf)
{
	const char *hilo;

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
		hilo = signal->rising? signal->key_hi: signal->key_lo;
		return sprintf(buf, "%s D%c%d %s %s\n",
			       signal->name, 
			       signal->is_output? 'O': 'I',
/* frig DIx for case of phys bits already set (ob_clk_src) gimme C++ */
			       signal->DIx <= 16? signal->DIx: 16,
			       signal->is_output? "": hilo,
			       signal->is_active? "ACTIVE":"inactive");
	}
}


ssize_t acq200_store_signal(
	struct Signal* signal,
	struct device * dev, const char * _buf, size_t count)
{
	char buf[80];
	char name[64];
	char edge[64];
	char *src = edge;
	int xx;
	int ok = __LINE__;
	int nc;
	char io;
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

	nc = sscanf(buf, "%s %s", name, src);

	if (signal->has_internal_option && OKL(nc == 2) &&
	    OKL(strcmp(name, signal->name) == 0) && 
	    OKL(strcmp(src, "internal") == 0)){
		signal->DIx = DIX_INTERNAL;
		activateSignal(signal);
		OKOK;
	}else if (OKL(nc == 2) &&
	    OKL(strcmp(name, signal->name) == 0) && 
	    OKL(strcmp(src, "none") == 0)){
		deactivateSignal(signal);
		setSignal(signal, DIX_NONE, 0);
		OKOK;
	}else if (OKL((nc=sscanf(buf,"%s D%c%d %s",name,&io,&xx,edge)) >= 3) &&
		  OKL(strcmp(name, signal->name) == 0) &&
		  OKL((signal->is_output == 1 && io == 'O') || 
		      (signal->is_output == 0 && io == 'I')   ) ){

		if ((nc == 3 || OKL(strcmp(edge, signal->key_lo) == 0)) &&
		     OKL(setSignal(signal, xx, 0) == 0)                ){
			activateSignal(signal);
			OKOK;
		}else if (OKL(strcmp(edge, signal->key_hi) == 0)  &&
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
		return -EINVAL;
	}else{
		return count;
	}
}


struct Signal* acq200_createSignal(
	const char* name, 
	int minDIx, int maxDIx,
	int DIx, int rising, int is_active,
	int (*commit)(struct Signal* signal)
)
{
	static struct Signal def_signal = {
		.is_active = 0,
		.commit = acq200_def_commit,
		.key_lo = "falling",
		.key_hi = "rising"
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


struct Signal* acq200_createLevelSignal(
	const char* name, 
	int minDIx, int maxDIx,
	int DIx, int rising, int is_active,
	int (*commit)(struct Signal* signal)
)
{
	struct Signal *signal =  createSignal(
		name, minDIx, maxDIx, DIx, rising, is_active, commit);
	signal->key_lo = "low";
	signal->key_hi = "high";
	return signal;
}

void acq200_destroySignal(struct Signal* signal)
{
	kfree(signal);
}

/** 
 *  @@null_signal - default signal class
 */
int acq200_def_commit(struct Signal* signal)
{
	dbg(1, "");
	return 0;
}


EXPORT_SYMBOL_GPL(acq200_show_signal);
EXPORT_SYMBOL_GPL(acq200_store_signal);
EXPORT_SYMBOL_GPL(acq200_createSignal);
EXPORT_SYMBOL_GPL(acq200_createLevelSignal);
EXPORT_SYMBOL_GPL(acq200_destroySignal);
EXPORT_SYMBOL_GPL(acq200_def_commit);


