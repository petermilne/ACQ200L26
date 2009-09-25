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
#include "acq200-signal2.h"


ssize_t acq200_show_signal(
	struct Signal* signal, struct device * dev, char * buf)
{
	if (!signal){
		err("ERROR: null signal");
		return -ENODEV;
	}

	return sprintf(buf, "%s %s %s %s\n", 
		       signal->name, 
		       signalLookup(signal, SIG_P1, signal->px[SIG_P1].ix),
		       signalLookup(signal, SIG_P2, signal->px[SIG_P2].ix),
		       signal->is_active? "ACTIVE": "inactive");
}


static void signalHelp(struct Signal *signal)
{
	char buf[128];
	const char** pp;
	int ip;

	info("%s:%s", "name", signal->name);
	
	for (ip = SIG_P1; ip <= SIG_P2; ++ip){
		buf[0] = '\0';
		if ((pp = signal->px[ip].lut) != 0){
			int ik;
			for (ik = 0; pp[ik]; ++ik){
				if (strlen(buf)+strlen(pp[ik]) < sizeof(buf)-1){
					strcat(buf, pp[ik]);
					strcat(buf, " ");
				}
			}
		}
		info("%d:%s", ip, buf);
	}		
}
ssize_t acq200_store_signal(
	struct Signal* signal,
	struct device * dev, const char * _buf, size_t count)
{
	char buf[80];
	char name[64];
	char p1[64];
	char p2[64];
	int nscan;

	p1[0] = p2[0] = '\0';

	if (!signal){
		err("ERROR: null signal");
		return -ENODEV;
	}

	buf[79] = '\0';
	strncpy(buf, _buf, min((int)count,78));
	if (strchr(buf, '\n')){
		*strchr(buf, '\n') = '\0';
	}
	
	nscan = sscanf(buf, "%s %s %s", name, p1, p2);

	if (nscan < 1){
		err("name p1 p2 - try help for list");
		return -ENODEV;
	}
	if (strcmp(name, "help") == 0){
		signalHelp(signal);
		return -ENODEV;
	}else if (strcmp(signal->name, name) != 0){
		err("signal name must match \"%s\" \"%s\"",
			signal->name, name);
		return -ENODEV;
	}
	if (nscan < 2){
		err("no parameters");
		return -ENODEV;
	}else{
		int ix1 = signalLookupIndex(signal, SIG_P1, p1);
		if (ix1 >= 0){
			signal->px[SIG_P1].ix = ix1;	
		}else{
			err("illegal p1 \"%s\"", p1);
			return -ENODEV;
		}
		if (nscan == 3){
			int ix2 = signalLookupIndex(signal, SIG_P2, p2);
			if (ix2 >= 0){
				signal->px[SIG_P2].ix = ix2;
			}else{
				err("illegal p2 \"%s\"", p2);
				return -ENODEV;
			}
		}
	}

	dbg(1, "p1:\"%s\" p2:\"%s\"",  p1, p2);

	activateSignal(signal);
	return count;
}


#define _ISIGS	"DI0", "DI1", "DI2", "DI3", "DI4", "DI5"
#define _OSIGS   "DO0", "DO1", "DO2", "DO3", "DO4", "DO5"
#define _ISIGS07 _ISIGS, "DI06", "DI07"

static const char * _SIGDEF_EVS[] = {
	"none", _ISIGS, 0
};
static const char * _SIGDEF_EDG[] = {
	"falling", "rising", 0
};
static const char * _SIGDEF_LVL[] = {
	"low", "high", 0
};
static const char * _SIGDEF_CKS[] = {
	_ISIGS, "internal", 0
};
static const char * _SIGDEF_OCKS[] = {
	_OSIGS, "none", 0
};
static const char * _SIGDEF_DI07[] = {
	_ISIGS07, 0			
};
static const char * _SIGDEF_CKS07[] = {
	_ISIGS07, "internal", "none", 0
};

static const char** menus[] = {
	[SIGDEF_NUL] = 0,
	[SIGDEF_EVS] = _SIGDEF_EVS,
	[SIGDEF_EDG] = _SIGDEF_EDG,
	[SIGDEF_LVL] = _SIGDEF_LVL,
	[SIGDEF_CKS] = _SIGDEF_CKS,
	[SIGDEF_OCKS] = _SIGDEF_OCKS,
	[SIGDEF_DI07] = _SIGDEF_DI07,
	[SIGDEF_CKS07] = _SIGDEF_CKS07
};

#define NMENUS (sizeof(menus)/sizeof(const char*))

const char** signalFactory(enum SIGDEF def)
{
	if (def < 0 || def >= NMENUS){
		return 0;
	}else{
		return menus[def];
	}
}

struct Signal* acq200_createSignal(
	const char* name, 
	const char** _p1, int init_p1,
	const char** _p2, int init_p2,
	int (*commit)(struct Signal* signal))
{
	struct Signal* signal = kzalloc(SIGNAL_SZ, GFP_KERNEL);
	strncpy(signal->name, name, sizeof(signal->name)-1);
	signal->px[SIG_P1].lut = _p1;
	signal->px[SIG_P1].ix = init_p1;
	signal->px[SIG_P2].lut = _p2;
	signal->px[SIG_P2].ix = init_p2;
	signal->commit = commit;
	signal->show = acq200_show_signal;
	signal->store = acq200_store_signal;
	return signal;
}


const char* signalLookup(struct Signal * signal, int px, int key)
{
	if (signal->px[px].lut != 0){
		int ikey;
		const char* lbl;
		for (ikey = 0; (lbl = signal->px[px].lut[ikey]) != 0; ++ikey){
			if (ikey == key){
				return lbl;
			}
		}
	}

	return "";
}
int signalLookupIndex(struct Signal *signal, int px, const char* key)
{
	dbg(2, "01:key \"%s\"", key);

	if (signal->px[px].lut != 0){
		int ikey;
		const char* lbl;
		for (ikey = 0; (lbl = signal->px[px].lut[ikey]) != 0; ++ikey){
			dbg(2, "50:key \"%s\" lbl \"%s\"", key, lbl);
			if (strcmp(lbl, key) == 0){
				dbg(2, "98: return %d", ikey);
				return ikey;
			}
		}
	}

	dbg(2, "99: return -1");
	return -1;
}

void activateSignal(struct Signal* signal)
{
	dbg(1, "%s", signal->name);
	enableSignal(signal, 1);
	signalCommit(signal);
}
void deactivateSignal(struct Signal* signal)
{
	dbg(1, "%s", signal->name);
	signal->was_active = signal->is_active;
	enableSignal(signal, 0);
	signalCommit(signal);
}
void reactivateSignal(struct Signal* signal)
{
	dbg(1, "%s", signal->name);
	enableSignal(signal, signal->was_active);
	signalCommit(signal);
}


int enableSignal(struct Signal* signal, int enable)
{
	const char* key = signalLookup(signal, 0, signal->px[0].ix);
	if (key && strcmp(key, "none")){
		return signal->is_active = enable;
	}else{
		signal->is_active = 0;
		return 0;
	}
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
EXPORT_SYMBOL_GPL(acq200_destroySignal);
EXPORT_SYMBOL_GPL(acq200_def_commit);


