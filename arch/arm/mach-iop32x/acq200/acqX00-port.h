/* ------------------------------------------------------------------------- */
/* acqX00-port.h portability layer                                           */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2007 Peter Milne, D-TACQ Solutions Ltd
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



#define COPY_TO_USER(dest,src,size)			\
	do {						\
		if (copy_to_user(dest, src, size)){	\
			return -EFAULT;			\
		}					\
	}						\
		while(0);

#define COPY_FROM_USER(dest,src,size)			\
	do {						\
		if (copy_from_user(dest, src, size)){	\
			return -EFAULT;			\
		}					\
	}						\
		while(0);


struct device;
struct device_attribute;

extern void acq200_device_create_file(
	struct device * dev, struct device_attribute * attr,
	const char *file, int line);

#define DEVICE_CREATE_FILE(dev, attr) \
	acq200_device_create_file(dev, attr, __FILE__, __LINE__)

struct device_driver;
struct driver_attribute;

extern void acq200_driver_create_file(
	struct device_driver *drv, struct driver_attribute * attr,
	const char* file, int line);

#define DRIVER_CREATE_FILE(drv, attr) \
	acq200_driver_create_file(drv, attr, __FILE__, __LINE__)
