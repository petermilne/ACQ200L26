/* ------------------------------------------------------------------------- */
/* drivers/mtd/maps/acq200.c MTD device mapping for ACQ200                   */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2003 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>

    Template with acknowledgements to iq80321.c by Rory Bolt

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


#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <linux/ioport.h>
#include <asm/arch-iop32x/iop321.h>
#include <asm/arch-iop32x/acq200.h>

#define BUSWIDTH 	2

static struct mtd_info *mymtd;

static struct map_info acq200_map = {
	.name = "ACQ200 flash",
	.size = ACQ200_FLASH_LEN,
	.bankwidth = BUSWIDTH,
	.phys = ACQ200_FLASH_P,
	.virt = ACQ200_FLASH_V,
};

#define RO_P( _name, _offset, _size ) \
{ name: _name, size: _size, offset: _offset, mask_flags: MTD_WRITEABLE }
#define RW_P( _name, _offset, _size ) \
{ name: _name, size: _size, offset: _offset }

static struct mtd_partition acq200_partitions[] = {
	RO_P( "0 Bootldr", 0x00000000, 0x00020000 ),
	RW_P( "1 Env",     0x00020000, 0x00020000 ),
	RO_P( "2 SafeKrn", 0x00040000, 0x00180000 ),
	RW_P( "3 FPGA",    0x001c0000, 0x00040000 ),
	RO_P( "4 SafeRd",  0x00200000, 0x00200000 ),
	RW_P( "5 FieldKrn",0x00400000, 0x00200000 ),
	RW_P( "6 FieldRd ",0x00600000, 0x00200000 ),
	RW_P( "7 extra",   0x00800000, 0x003c0000 ),
	RW_P( "8 home",    0x00bc0000, 0x00380000 ),
	RW_P( "9 cal",     0x00f40000, 0x000c0000 ),
};

#define NB_OF(x)  (sizeof(x)/sizeof(x[0]))

#define ACQ200_FLASH_REGION_LEN 0x08000000
#define ACQ200_FLASH_REGION_START ACQ200_FLASH
#define ACQ200_FLASH_REGION_END   \
        (ACQ200_FLASH_REGION_START+ACQ200_FLASH_REGION_LEN)

static void acq200_request_mem_regions(void)
{
	static struct resource res = {
		.name = "acq200 flash",
		.flags = IORESOURCE_MEM
	};
	int ipart;

	allocate_resource(acq200_pbi_resource, &res, 
			  ACQ200_FLASH_REGION_LEN,
			  ACQ200_FLASH_REGION_START,
			  ACQ200_FLASH_REGION_END,
			  ACQ200_FLASH_REGION_LEN,
			  NULL, NULL);

	for ( ipart = 0; ipart != NB_OF(acq200_partitions); ++ipart ){
		struct mtd_partition *part = &acq200_partitions[ipart];
		request_mem_region( ACQ200_FLASH+part->offset,
					    part->size, part->name );
	}
}


static int __init acq200_init_map(void)
{
	acq200_request_mem_regions();

	simple_map_init(&acq200_map);
	mymtd = do_map_probe("cfi_probe", &acq200_map);
	mymtd->owner = THIS_MODULE;
	
	add_mtd_partitions(mymtd, acq200_partitions, NB_OF(acq200_partitions));

	return 0;
}

static void __exit acq200_cleanup_map(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
	}	
}

module_init(acq200_init_map);
module_exit(acq200_cleanup_map);


MODULE_LICENSE("GPL");

