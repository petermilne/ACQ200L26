/* ------------------------------------------------------------------------- */
/* refill-client.c - consumer for refill data eg mean device                 */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2010 Peter Milne, D-TACQ Solutions Ltd
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

#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/moduleparam.h>
#include <linux/proc_fs.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#include <linux/fs.h>     	/* This is where libfs stuff is declared */
#include <asm/uaccess.h>	/* copy_to_user */

#include "acqX00-port.h"
/* keep debug local to this module */
#define acq200_debug acq200_mean_debug   

#include "acq200_debug.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq32busprot.h"

/** @@todo : future could be multiple RefillClients */

void acq200_addRefillClient(RefillClient client)
{
	spin_lock(&DG->refillClient.lock);
	DG->refillClient.client = client;
	spin_unlock(&DG->refillClient.lock);
}

void acq200_delRefillClient(RefillClient client)
{
	acq200_addRefillClient(0);
} 

void acq200_runRefillClient(void *data, int nbytes)
{
	RefillClient client;
	spin_lock(&DG->refillClient.lock);
	client = DG->refillClient.client;
	if (client){
		client(data, nbytes);
	}
	spin_unlock(&DG->refillClient.lock);	
}
