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



#ifndef IN_RANGE
#define IN_RANGE(xx, ll, rr) ((xx)>=(ll)&&(xx)<=(rr))
#endif


/*
 * class Signal - generic per device, per function line handling
 * external interface deals in logical values, internal is board, func
 * specific
 */

struct device;

#define SIG_P1	0
#define SIG_P2	1

#define SIG_LINE SIG_P1
#define SIG_EDGE SIG_P2

struct Signal {
	char name[16];
	int is_active;
	int was_active;

	struct {
		const char **lut;
		int ix;
	}
		px[2];

	int (*commit)(struct Signal* signal);
};


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
	const char** _p1, int init_p1,
	const char** _p2, int init_p2,
	int (*commit)(struct Signal* signal)
);

#define createSignal acq200_createSignal
#define createLevelSignal acq200_createSignal


void acq200_destroySignal(struct Signal* signal);
#define SIGNAL_SZ (sizeof(struct Signal))

#define destroySignal acq200_destroySignal

int acq200_def_commit(struct Signal* signal);

#define createNullSignal() createSignal("null",0,0,0,0,0)

const char* signalLookup(struct Signal * signal, int px, int value);
int signalLookupIndex(struct Signal *signal, int px, const char* key);

int enableSignal(struct Signal* signal, int enable);

static inline int signalCommit(struct Signal* signal)
{
	if (signal->commit == 0){
		return 0;
	}else{
		return signal->commit(signal);
	}
}


static inline int signalLine(struct Signal *signal)
{
	return signal->px[SIG_LINE].ix;
}
static inline int signalEdge(struct Signal *signal)
{
	return signal->px[SIG_EDGE].ix;
}

enum SIGDEF {
	SIGDEF_NUL,
	SIGDEF_EVS,
	SIGDEF_EDG,
	SIGDEF_LVL,
	SIGDEF_CKS,
	SIGDEF_OCKS,
	SIGDEF_DI07,
	SIGDEF_CKS07
};

/* index of "none" entry in OCKS @@todo HACK! use lookup .. */
#define SIG_OCKS_NONE 6

const char** signalFactory(enum SIGDEF def);

#define SIG(code)	signalFactory(SIGDEF_##code)

void activateSignal(struct Signal* signal);
void deactivateSignal(struct Signal* signal);
void reactivateSignal(struct Signal* signal);




#endif		/* __ACQ200_SIGNAL_H__ */
