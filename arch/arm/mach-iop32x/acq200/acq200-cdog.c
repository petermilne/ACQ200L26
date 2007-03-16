/* ------------------------------------------------------------------------- */
/* acq200-cdog.c clock watchdog - report lost cloc                           */
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

#include "acq200-fifo-local.h"
#include "acq200_minors.h"

#include <linux/poll.h>

#include "acq200-rb.h"

#include <asm/arch/iop321-dma.h>

#include "acq200-fifo.h"
#include "acq200-fifo-tblock.h"
#include "acq200-stream-api.h"

#include <linux/dma-mapping.h>
#include <linux/kdev_t.h>


#include <linux/moduleparam.h>

extern u32 acq200_get_fifsta(void);

static int 
run_cdog_helper(void)
{
	static char* envp[] = {
		"HOME=/",
		"PATH=/usr/bin:/bin:/usr/sbin:/sbin",
		0
	};
	char *argv[2];
	int rc;

        argv[0] = "/sbin/acq200_cdog_helper";
	argv[1] = 0;


	dbg( 1, "call_usermodehelper %s\n", argv[0] );

	rc = call_usermodehelper(argv [0], argv, envp, 0);

	if ( rc != 0 ) err( "call done returned %d", rc );

	return 0;
}


int acq200_cdog(int mode)
/* clock watchdog() */
{
	static int last_fints;
	static long oj;
	static int maxclk_jiffies;
	long dj;


	if (!DG->cdog_max_jiffies){
		return 0;               /** 0 disables */
	}

	switch(mode){
	case CDOG_INIT:
		maxclk_jiffies = DG->cdog_max_jiffies;
		oj = 0;
		return 0;
	case CDOG_REFRESH:
		if (oj == 0){
			oj = jiffies;
			return 0;
		}
		break;
	default:
		BUG();
	}

	dj = jiffies - oj;

	if (DG->stats.num_fifo_ints < last_fints || jiffies < oj){
		/* new shot or rollover */
		last_fints = DG->stats.num_fifo_ints;
		oj = jiffies;
		maxclk_jiffies = DG->cdog_max_jiffies;
	}else if (last_fints != DG->stats.num_fifo_ints){
#ifdef THIS_IS_DODGY
		/* timeout adapts to clock rate */
		maxclk_jiffies = (dj + maxclk_jiffies)/2;
		maxclk_jiffies = max(maxclk_jiffies, 20);
#endif
		last_fints = DG->stats.num_fifo_ints;
		oj = jiffies;	
	}else{
		if ( DG->stats.num_fifo_ints && dj > maxclk_jiffies){
			u32 fifsta = acq200_get_fifsta();
			char *buf = kmalloc(4096, GFP_KERNEL);
			char *bp;


			if ((fifsta & DG->FIFERR) != 0){
				if (!DG->fiferr){
					DG->fiferr = fifsta & DG->FIFERR;
				}
				bp = buf + sprintf(buf, "CDOG FIFERR\n"
					"%20s : 0x%08x\n" "%20s : 0x%08x\n",
						   "status", fifsta,
						   "FIFERR", DG->FIFERR
					);
			}else{
				bp = buf + sprintf(buf, "LOST THE CLOCK\n"
			      "n fi:%d last:%d jiffies:%ld oj:%ld "
			      "(%ld > %d)\n",
			      DG->stats.num_fifo_ints, last_fints, jiffies, oj,
			      dj, maxclk_jiffies);
			      bp += acq200_dumpregs_diag(bp, 4096);
			}
			
			acq200_dumpregs_diag(bp, 4096);			
			info("\n%s", buf);
			kfree(buf);
			finish_with_engines(-__LINE__);
			run_cdog_helper();
			return 1;
		}
	}
	/* else ... let those jiffies ramp ! */
	return 0;
}
