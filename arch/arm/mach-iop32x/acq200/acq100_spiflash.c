/* ------------------------------------------------------------------------- */
/* acq200/acq100_spiflash.c spi device description for ACQ1{64}              */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2009 Peter Milne, D-TACQ Solutions Ltd
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
#include <linux/types.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/arch/iop321_spi.h>


static struct spi_board_info acq100_spidevices[] = {
	{
		/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.mode = SPI_CS_HIGH 
	}
};

struct iop321_spi_info acq100_spi_info = {
	.ext_clk_hz = 0,
	.board_size = 1,
	.board_info = acq100_spidevices
};

static u64 spi_dmamask = 0xffffffffUL;

struct platform_device iop321_spi = {
	.name = "iop321-spi",
	.id = 0,
	.num_resources = 0,
	.dev = {
		.dma_mask = &spi_dmamask,
		.coherent_dma_mask = 0xffffffffUL,
		.platform_data = &acq100_spi_info
	}
};

static struct platform_device *acq100_devices[] __initdata = {
	&iop321_spi
};

static int __init acq100_spiflash_init(void)
{
	platform_add_devices(acq100_devices, ARRAY_SIZE(acq100_devices));
	return 0;
}

static void __exit acq100_spiflash_exit(void)
{
	
}


module_init(acq100_spiflash_init);
module_exit(acq100_spiflash_exit);

MODULE_DESCRIPTION("ACQ100 SPIFLASH Driver");
MODULE_AUTHOR("Peter Milne, <peter.milne@d-tacq.com>");
MODULE_LICENSE("GPL");
