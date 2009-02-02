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

#ifndef __ACQ200_SIGNAL_H__
#define __ACQ200_SIGNAL_H__


#include <linux/list.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif


#ifndef IN_RANGE
#define IN_RANGE(xx, ll, rr) ((xx)>=(ll)&&(xx)<=(rr))
#endif


/*
 * class Signal - generic per device, per function line handling
 * external interface deals in logical values, internal is board, func
 * specific
 */
struct Signal {
	char name[16];
	int is_active;
	int was_active;
	int DIx;                
	int rising;

	int _minDIx, _maxDIx;
	int has_internal_option;
	int is_output;
	const char *key_hi;
	const char *key_lo;

	int (*commit)(struct Signal* signal);
};

#define DIX_INTERNAL -2
#define DIX_NONE -1


ssize_t acq200_show_signal(
	struct Signal* signal, struct device * dev, char * buf);

static inline ssize_t show_signal(
	struct Signal* signal, struct device * dev, char * buf)
{
	return acq200_show_signal(signal, dev, buf);
}

ssize_t acq200_store_signal(
	struct Signal* signal,
	struct device * dev, const char * _buf, size_t count);

static inline ssize_t store_signal(
	struct Signal* signal,
	struct device * dev, const char * _buf, size_t count)
{
	return acq200_store_signal(signal, dev, _buf, count);
}
/**
 * createSignal
 * name, minDIx, maxDIx, DIx, rising, is_active, commit
 */
struct Signal* acq200_createSignal(
	const char* name, 
	int minDIx, int maxDIx,
	int DIx, int rising, int is_active,
	int (*commit)(struct Signal* signal)
);
static inline struct Signal* createSignal(
	const char* name, 
	int minDIx, int maxDIx,
	int DIx, int rising, int is_active,
	int (*commit)(struct Signal* signal) )
{
	return acq200_createSignal(name, minDIx, maxDIx, DIx, rising,
				   is_active, commit);
}



struct Signal* acq200_createLevelSignal(
	const char* name, 
	int minDIx, int maxDIx,
	int DIx, int rising, int is_active,
	int (*commit)(struct Signal* signal)
);
static inline struct Signal* createLevelSignal(
	const char* name, 
	int minDIx, int maxDIx,
	int DIx, int rising, int is_active,
	int (*commit)(struct Signal* signal)
	)
{
	return acq200_createLevelSignal(
		name, minDIx, maxDIx, DIx, rising, is_active, commit);
}

void acq200_destroySignal(struct Signal* signal);
#define SIGNAL_SZ (sizeof(struct Signal))

static inline void destroySignal(struct Signal* signal)
{
	acq200_destroySignal(signal);
}


int acq200_def_commit(struct Signal* signal);

#define createNullSignal() createSignal("null",0,0,0,0,0,0)

static inline int setSignal(struct Signal* signal, int DIx, int rising)
{
	if (DIx == DIX_NONE){
		signal->DIx = DIX_NONE;
		return 0;
	}else if (IN_RANGE(DIx, signal->_minDIx, signal->_maxDIx)){
		signal->DIx = DIx;
		signal->rising = rising;
		return 0;
	}else{
		return -1;
	}
}

static inline int enableSignal(struct Signal* signal, int enable)
{
	if (signal->DIx != DIX_NONE){
		return signal->is_active = enable;
	}else{
		return 0;
	}
}
static inline int signalCommit(struct Signal* signal)
{
	if (signal->DIx != DIX_NONE){
		return signal->commit(signal);
	}else{
		return 0;
	}
}


static inline void activateSignal(struct Signal* signal)
{
	dbg(1, "%s", signal->name);
	enableSignal(signal, 1);
	signalCommit(signal);
}
static inline void deactivateSignal(struct Signal* signal)
{
	dbg(1, "%s", signal->name);
	signal->was_active = signal->is_active;
	enableSignal(signal, 0);
	signalCommit(signal);
}
static inline void reactivateSignal(struct Signal* signal)
{
	dbg(1, "%s", signal->name);
	enableSignal(signal, signal->was_active);
	signalCommit(signal);
}




#endif		/* __ACQ200_SIGNAL_H__ */
