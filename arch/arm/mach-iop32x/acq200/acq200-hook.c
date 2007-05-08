#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/mm.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <asm-arm/fiq.h>
#ifdef PGMCOMOUT263
#include <asm-arm/proc-armv/cache.h>
#endif

#include <linux/proc_fs.h>

#include <linux/string.h>


#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#include <linux/module.h>
#endif

#define DMA_BLOCK_LEN 0xdeadbeef  /* FIXME PLEASE - not used here */


#include "acq200-fifo-local.h"
#include "acq200_debug.h"

#include "acq200-fifo-tblock.h"

static void _run_helper(struct ArgBlock *argblock, int wait)
{
	static char* envp[] = {
		"HOME=/",
		"PATH=/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin",
		0
	};
	int rc;
	
	if (argblock->argc == 0){
		return;
	}

	dbg(1, "call_usermodehelper %s\n", argblock->argv[0]);

	rc = call_usermodehelper(
		argblock->argv[0], 
		argblock->argv, envp, wait);

        dbg(1, "call done returned %d", rc);	
}

void run_pre_arm_hook(void)
{
	_run_helper(&DG->pre_arm_hook, 1);
}
void run_post_arm_hook(void)
{
	_run_helper(&DG->post_arm_hook, 0);
}

void run_post_shot_hook(void)
{
	_run_helper(&DG->post_shot_hook, 1);
}

int show_hook(struct ArgBlock *argBlock, char* buf, int maxbuf)
{
	int len = 0;
	int ii;
	int argc = argBlock->argc;

	buf[0] = '\0';

	for (ii = 0; ii < argc; ++ii){
		char *arg = argBlock->argv[ii];
		int arglen = strlen(arg);		
		char tchar;

		len += sprintf(buf+len, "%s", arg);

		if (ii+1 == argc){
			if (arg[arglen-1] == '\n'){
				continue;
			}else{
				tchar = '\n';
			}
		}else{
			tchar = ' ';
		}
		len += sprintf(buf+len, "%c", tchar);
	}
	
	return len;
}


static int isspace(char c)
{
	return strchr( " \t", c) != 0;
}
#define MAXARGS 20

int store_hook(struct ArgBlock *argBlock, const char* buf, int count)
{
	char *cursor;
	char *base;
	int in_ws= 1;
	int iarg = 0;

	if (argBlock->base == 0){
		/** @@todo mem leak */
		argBlock->base = kmalloc(PAGE_SIZE, GFP_KERNEL);

		assert(argBlock->base);

		argBlock->argv = (char **)argBlock->base;
		argBlock->argv[0] = (char *)&argBlock->argv[MAXARGS];

		dbg(1,"kmalloc:argBlock %p base %p", argBlock, argBlock->base);
	}

#define RUMP (PAGE_SIZE - (MAXARGS*sizeof(char **)))

	count = min((unsigned long)count, RUMP);

	memcpy(argBlock->argv[0], buf, count);

/** @todo - ignores quotes, splits on spaces */
	for (cursor = base = argBlock->argv[0]; 
	     cursor - base < count; ++cursor){

		if (in_ws){
			if (!isspace(*cursor)){
				argBlock->argv[iarg++] = cursor;
				in_ws = 0;
			}
		}else{
			if (isspace(*cursor)){
				*cursor = '\0';
				in_ws = 1;
			}
		}


		if (iarg >= MAXARGS-1){
			break;
		}
	}
	argBlock->argv[iarg] = 0;
	argBlock->argc = iarg;

	dbg(1, "argBlock %p argc %d", argBlock, argBlock->argc);
	*cursor = '\0';
	return cursor - base;			
}
