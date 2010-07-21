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

#include <linux/kernel.h>
#include <linux/list.h>


#include "acqX00-port.h"
/* keep debug local to this module */
#define acq200_debug acq200_mean_debug   

#include "acq200_debug.h"
#include "acq200-fifo-local.h"     /* DG */

#include "acq32busprot.h"


void acq200_addRefillClient(struct RefillClient *client)
{
	spin_lock(&DG->refillClients.lock);
	list_add_tail(&client->list, &DG->refillClients.clients);
	spin_unlock(&DG->refillClients.lock);
}

void acq200_delRefillClient(struct RefillClient *client)
{
	spin_lock(&DG->refillClients.lock);
	list_del(&client->list);
	spin_unlock(&DG->refillClients.lock);

	acq200_addRefillClient(0);
} 

void acq200_runRefillClient(void *data, int nbytes)
{
	struct RefillClient *client;

	spin_lock(&DG->refillClients.lock);
	list_for_each_entry(client, &DG->refillClients.clients, list){
		client->action(data, nbytes);
	}
	spin_unlock(&DG->refillClients.lock);
}
