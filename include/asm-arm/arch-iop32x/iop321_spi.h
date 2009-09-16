/* ------------------------------------------------------------------------- */
/* drivers/spi/iop321_spi.h spi device driver interface for IOP321           */
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


#ifndef __IOP321_SPI_H
#define __IOP321_SPI_H

struct spi_board_info;
struct iop321_spi_info;

struct iop321_spi_info {
	unsigned long ext_clk_hz;	/* 0 => use internal clock */
	unsigned long board_size;
	struct spi_board_info *board_info;

	void (*set_cs)(struct iop321_spi_info *spi, int cs, int pol);
};

#endif	/* __IOP321_SPI_H */
