/* ------------------------------------------------------------------------- */
/* acq200-mm.c - iomemory map, PBIBAR set up                                 */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2003 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>

    With acknowledgements to Rory Bolt, mm-321.c

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
/*
 * linux/arch/arm/mach-iop3xx/mm.c
 *
 * Low level memory intialization for IOP321 based systems
 *
 * Author: Rory Bolt <rorybolt@pacbell.net>
 * Copyright (C) 2002 Rory Bolt
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>         /* EXPORT_SYMBOL() */

#include <linux/bootmem.h>        /* alloc_bootmem */
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/ioport.h>

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/arch.h>         /* struct machine_desc */
#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <asm/setup.h>             /* struct meminfo, struct tag */

#ifndef CONFIG_ARCH_ACQ200
#error This file specific to ARCH_ACQ200
#endif

#include "acq200_debug.h"


#include <asm-arm/arch-iop32x/iop321.h>
#include <asm-arm/arch-iop32x/acq200.h>

#define MKPBLR( len ) (0xffffffff & ~((len)-1))


#define INIT_RESOURCE(_name, _start, _end, _flags) \
	{ .name = _name, .start = _start, .end = _end, .flags = _flags }


static struct resource mem_res[] = {
	INIT_RESOURCE( "PBI",	      0,	0,     IORESOURCE_MEM ),
	INIT_RESOURCE( "Kernel code", 0,     0,     IORESOURCE_MEM ),
	INIT_RESOURCE( "Kernel data", 0,     0,     IORESOURCE_MEM ),
	INIT_RESOURCE( "bigbuf",      0,     0,     IORESOURCE_MEM ),
	INIT_RESOURCE( "MUmem",       0,     0,     IORESOURCE_MEM ),
	INIT_RESOURCE( "tblock",      0,     0,     IORESOURCE_MEM )
};

#define pbi         mem_res[0]
#define kernel_code mem_res[1]
#define kernel_data mem_res[2]
#define bigbuf      mem_res[3]
#define mumem       mem_res[4]
#define tblock      mem_res[5]


/* WORKTODO - won't work in under 128MB ?? 96 should be enough */

#define LINUX_IDEAL_MB 128

#define BYTES( mb ) ((mb)<<20)
#define MB( bytes ) ((bytes)>>20)


static int _acq200_get_resource( 
	struct resource *dst, const struct resource *src )
{
	int length = src->end - src->start;

	if ( length && dst ){
		memcpy( dst, src, sizeof( struct resource ) );
	}
	return length;
}
int acq200_get_bigbuf_resource( struct resource *resource )
/* exported function hook for driver module to obtain resource info */
{
	return _acq200_get_resource( resource, &bigbuf );
}

int acq200_get_mumem_resource( struct resource *resource )
/* exported function hook for driver module to obtain resource info */
{
	return _acq200_get_resource( resource, &mumem );
}

int acq200_get_tblock_resource( struct resource *resource )
{
	return _acq200_get_resource( resource, &tblock );
}


static void p_request_resource(struct resource *parent, struct resource *self)
{
	info( "%10s %08lx %08lx", self->name, 
			(long)self->start, (long)self->end);
	request_resource(parent, self);
}
extern void __init iop321_map_io(void);


static void __init acq200_tblock_init(void)
{
	void *va = alloc_bootmem(ACQ200_TBLOCK_SIZE);

	info("tblock va %p", va);
	if ( va ){
		tblock.start = (unsigned)va;
		tblock.end   = tblock.start + ACQ200_TBLOCK_SIZE;
	}else{
		err( "failed to allocate tblock size 0x%08x", 
		     ACQ200_TBLOCK_SIZE );
	}
}

#define MUMEM_LEN 

static void __init acq200_mumem_init(void)
{
	void *mumem_va = __alloc_bootmem(ACQ200_MU_QTOTSZ, ACQ200_MU_QALIGN,0);

	if ( mumem_va ){
		mumem.start = (unsigned)mumem_va;
		mumem.end   = mumem.start + ACQ200_MU_QTOTSZ;
	}else{
		err( "failed to allocate mumem size 0x%08x", 
		     ACQ200_MU_QTOTSZ );
	}
}


static void __init acq200_bigbuf_init(struct meminfo *mi)
{
	int bigbuf_len = 0;
	void* bigbuf_va;
	unsigned mem_size = mi->bank[0].size;

	if (MB(mem_size) <= 64){
		bigbuf_len = BYTES( 1 );
	}else if(MB(mem_size) <= 128){
		bigbuf_len = BYTES( 48 );
	}else{
		bigbuf_len = mem_size - BYTES(LINUX_IDEAL_MB);
	}
	
	info("%15s %08x %15s %08x", "mem", mem_size, "ideal buf", bigbuf_len);

	while ( (bigbuf_va = alloc_bootmem(bigbuf_len)) == 0 ){
		info( "alloc_bootmem( 0x%08x ) failed\n", bigbuf_len );
		bigbuf_len >>= 1;
		if ( bigbuf_len < PAGE_SIZE ){
			err( "bigbuf_len < PAGE_SIZE, dropping out\n" );
			return;
		}
	}

	info("%15s %p %15s %08x", "bigbuf va", bigbuf_va, "len", bigbuf_len);

	bigbuf.start = (unsigned)bigbuf_va;
	bigbuf.end   =  bigbuf.start+bigbuf_len;
}



void __init
acq200_request_standard_resources(
	struct meminfo *mi, struct machine_desc *mdesc)
/* Called Last */
{
	struct resource *res;
	int i;

	kernel_code.start  = init_mm.start_code;
	kernel_code.end    = init_mm.end_code - 1;
	kernel_data.start  = init_mm.end_code;
	kernel_data.end    = init_mm.brk - 1;


	acq200_bigbuf_init(mi);
	acq200_tblock_init();
	acq200_mumem_init();
/* 
 * PGM Query - will this work for multiple banks?
 * will we ever _have_ multiple banks do we care ?
 * prefer to use a static entry for SysRAM and then can lose half this setup
 */
	for (i = 0; i < mi->nr_banks; i++) {
		unsigned long virt_start, virt_end;

		if (mi->bank[i].size == 0)
			continue;

		virt_start = __phys_to_virt(mi->bank[i].start);
		virt_end   = virt_start + mi->bank[i].size - 1;

		res = alloc_bootmem_low(sizeof(*res));
		res->name  = "System RAM";
		res->start = virt_start;
		res->end   = virt_end;
		res->flags = IORESOURCE_MEM | IORESOURCE_BUSY;
		
		request_resource(&iomem_resource, res);

		if (kernel_code.start >= res->start &&
		    kernel_code.end <= res->end)
			request_resource(res, &kernel_code);

		if (kernel_data.start >= res->start &&
		    kernel_data.end <= res->end)
			request_resource(res, &kernel_data);


		p_request_resource(res, &bigbuf);
		p_request_resource(res, &tblock);
		p_request_resource(res, &mumem);
	}
}


#define MASK_PBAR(pbar, bits) \
        do { *(pbar) &= ~PBAR_MASK; *(pbar) |= (bits); } while(0)

#define INIT_PBLR(pblr, len) (*(pblr) = 0xffffffff^((len)-1))


void __init acq100_init_pbi(void)
/*
  CS0 16/FLASH FLASH Memory 
  CS1 8/FLASH Core Logic CPLDs 
  CS2 32/Standard FPGA Communications to ADC and DAC logic 
  CS3 8/FLASH Serial Port UART on the RTM 
  CS4 32/Standard Ethernet Controller on the RTM 
  CS5 32/FLASH Digital I/O on the RTM	
*/
{

#define PBAR4_BITS (PBAR_BUS_32|PBAR_FLASH|PBAR_RCWAIT_1|PBAR_ADWAIT_4)
#define PBAR5_BITS (PBAR_BUS_32|PBAR_FLASH|PBAR_RCWAIT_1|PBAR_ADWAIT_4)
	MASK_PBAR(IOP321_PBBAR2, PBAR_BUS_32);
        INIT_PBLR(IOP321_PBLR2,  ACQ100_FPGA_LEN);

	MASK_PBAR(IOP321_PBBAR4, PBAR4_BITS);
	INIT_PBLR(IOP321_PBLR4,  ACQ100_ETHERNET_LEN);


	MASK_PBAR(IOP321_PBBAR5, PBAR5_BITS);
	INIT_PBLR(IOP321_PBLR5,  ACQ200_EXTERNIO_LEN);
#undef PBAR4_BITS
#undef PBAR5_BITS
}


void __init acq200_init_pbi(void)
/*
  CS0 16/FLASH FLASH Memory 
  CS1 8/FLASH Core Logic CPLDs 
  CS2 ??
  CS3 8/FLASH Serial Port UART on the RTM 
  CS4 ??
  CS5 16/FLASH Digital I/O on the RTM	
*/
{
#define PBAR5_BITS (PBAR_BUS_16|PBAR_FLASH|PBAR_RCWAIT_1|PBAR_ADWAIT_4)

	MASK_PBAR(IOP321_PBBAR5, PBAR5_BITS);
	INIT_PBLR(IOP321_PBLR5,  ACQ200_EXTERNIO_LEN);

#undef PBAR5_BITS
}

static void _map_io(void)
{
	iop3xx_map_io();

	allocate_resource( &iomem_resource, &pbi,
			   ACQ200_PBI_END-ACQ200_PBI_START,
			   ACQ200_PBI_START,
			   ACQ200_PBI_END,
			   ACQ200_PBI_END-ACQ200_PBI_START, NULL, NULL );
}


// WORKTODO - one mapping per PBIx ? */
static struct map_desc acq200_io_desc[] __initdata = {
 /* virtual     physical      length        type */
 
// { 0xc0000000,  ACQ200_PCIMEM, ACQ200_PCIMEM_SIZE, MT_DEVICE },
 /* on-board devices */
	
	{ 
		.virtual	= ACQ200_PBI_START,  
		.pfn		= __phys_to_pfn(ACQ200_PBI_START_P),   
		.length		= ACQ200_PBI_END-ACQ200_PBI_START,   
		.type		= MT_DEVICE 
	},
/* set up a static mapping for first 32MB of PCI space.
 * this is because ioremap() memory can fault in FIQ
 * let's hope it works ...
 */
	{
		.virtual	= ACQ200_PCI_VSTAT,		   /* va */
		.pfn		= __phys_to_pfn(ACQ200_PCIMEM),    /* pa */
		.length		= 0x2000000,			   /* 32MB */
		.type		= MT_DEVICE	       
				/* uncached: DMA does the blocks anyway */
	}
};


void __init acq200_map_io(void)
/* Called Second */
{
	_map_io();
	iotable_init(acq200_io_desc, ARRAY_SIZE(acq200_io_desc));
	acq200_init_pbi();
}

// WORKTODO - one mapping per PBIx ? */
static struct map_desc acq100_io_desc[] __initdata = {
 /* virtual     physical      length        type */
 
// { 0xc0000000,  ACQ200_PCIMEM, ACQ200_PCIMEM_SIZE, MT_DEVICE },
 /* on-board devices */
	
	{ 
		.virtual	= ACQ200_PBI_START,  
		.pfn		= __phys_to_pfn(ACQ200_PBI_START_P),   
		.length		= ACQ200_PBI_END-ACQ200_PBI_START,   
		.type		= MT_DEVICE 
	},
	{
		.virtual	= ACQ100_PCIMEM_START,
		.pfn		= __phys_to_pfn(ACQ100_PCIMEM_P),
		.length		= ACQ100_PCIMEM_END-ACQ100_PCIMEM_START,
		.type		= MT_DEVICE
	}		
};

extern int acq100_is_system_slot_enabled(void);

void __init acq100_map_io(void)
{
	_map_io();
	/** PBI mapping NEEDED to find out if we are system slot or not! */
	iotable_init(acq100_io_desc, 1);
	acq100_init_pbi();
	if (acq100_is_system_slot_enabled()){
		info("configure system slot mappings");
		iotable_init(acq200_io_desc+1, ARRAY_SIZE(acq200_io_desc)-1);
	}else{
		info("configure pci outbound memory");
		iotable_init(acq100_io_desc+1, ARRAY_SIZE(acq100_io_desc)-1);
	}

}

#define ONEGB     0x40000000
/*
 * this should work but doesn't:
#define ONEGB_OOB 0x00010000
*/


 /* discarding 1MB seems wasteful, but for 40p WTF */
#define ONEGB_OOB 0x00100000  

void __init fixup_acq200(
	struct machine_desc *desc, 
	struct tag *params,
	char **cmdline, 
	struct meminfo *mi )
/* Called First */
{
	struct tag *tag;
/* dodgy - better hope there IS an ATAG_MEM or we loop forever ... */

	
	for_each_tag( tag, params ){
		if ( tag->hdr.tag == ATAG_MEM ){
			info( "ATAG_MEM found 0x%08x size 0x%08x",
			      tag->u.mem.start, tag->u.mem.size      );

			if (tag->u.mem.size == ONEGB){
				tag->u.mem.size -= ONEGB_OOB;
				info("ONEGB lop top %x", ONEGB_OOB);
			}
			break;
		}
	}
}



struct resource* acq200_pbi_resource = &pbi;
EXPORT_SYMBOL(acq200_pbi_resource);
