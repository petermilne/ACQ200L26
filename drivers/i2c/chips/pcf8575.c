/*
    pcf8575.c - Part of lm_sensors, Linux kernel modules for hardware
             monitoring
    Copyright (C) 2004  Peter Milne peter.milne AT d-tacq . com

    Copied from pcf8574.c
    Copyright (c) 2000  Frodo Looijaard <frodol@dds.nl>, and others
    Ported to Linux 2.6 by Aurelien Jarno <aurel32@debian.org> with 

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#define I2C_DRIVERID_PCF8575 0xf038

/* A few notes about the PCF8575:

* The PCF8575 is an 16-bit I/O expander for the I2C bus produced by
  Philips Semiconductors. 

*/

#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>

#include "pcf8575.h"

#define DEVICE_CREATE_FILE(dev, attr)					\
	do {								\
		if (device_create_file(dev, attr) != 0) return -ENODEV;	\
	} while(0)

/* Addresses to scan */
static unsigned short normal_i2c[] = { 
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_2(pcf8575, pcf8575a);

/* Initial values */
#define PCF8575_INIT 255	/* All outputs on (input mode) */

/* Each client has this additional data */
struct pcf8575_data {
	struct i2c_client client;
	struct semaphore update_lock;

	u16 read, write;			/* Register values */
};

static int pcf8575_attach_adapter(struct i2c_adapter *adapter);
static int pcf8575_detect(struct i2c_adapter *adapter, int address, int kind);
static int pcf8575_detach_client(struct i2c_client *client);
static void pcf8575_init_client(struct i2c_client *client);
static struct pcf8575_data *pcf8575_update_client(struct device *dev);

/* This is the driver that will be inserted */
static struct i2c_driver pcf8575_driver = {
	.driver = {
		.name		= "pcf8575",
	},
	.id		= I2C_DRIVERID_PCF8575,
	.attach_adapter	= pcf8575_attach_adapter,
	.detach_client	= pcf8575_detach_client,
};

static int i2c_write_word(struct i2c_client *client, u16 value)
{
	return i2c_master_send(client, (char*)&value, 2);
}

static int i2c_read_word(struct i2c_client *client)
{
	u16 value;
	i2c_master_recv(client, (char*)&value, 2);
	return value;
}


/*
 * Kernel API
 */

u16 pcf8575_read(struct device *dev)
{
	struct pcf8575_data *data = pcf8575_update_client(dev);
	return data->read;
}

u16 pcf8575_write(
	struct device *dev, 
	u16 write_val)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf8575_data *data = i2c_get_clientdata(client);

	data->write = write_val;
	i2c_write_word(client, data->write);
	return write_val;
}

/* following are the sysfs callback functions */
static ssize_t show_read(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%04x\n", pcf8575_read(dev));
}

static DEVICE_ATTR(read, S_IRUGO, show_read, NULL);

static ssize_t show_write(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	struct pcf8575_data *data = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%04x\n", data->write);
}

static ssize_t set_write(
	struct device *dev, 
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	pcf8575_write(dev, simple_strtoul(buf, NULL, 16));
	return count;
}

static DEVICE_ATTR(write, S_IWUGO | S_IRUGO, show_write, set_write);

/*
 * Real code
 */

static int pcf8575_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, pcf8575_detect);
}

/* This function is called by i2c_detect */
int pcf8575_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct pcf8575_data *data;
	int err = 0;
	const char *client_name = "";

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		goto exit;

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet. */
	if (!(data = kmalloc(sizeof(struct pcf8575_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct pcf8575_data));

	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &pcf8575_driver;
	new_client->flags = 0;

	/* Now, we would do the remaining detection. But the PCF8575 is plainly
	   impossible to detect! Stupid chip. */

	/* Determine the chip type */
	if (kind <= 0) {
		kind = pcf8575;
	}

	client_name = "pcf8575";

	/* Fill in the remaining client fields and put it into the global list */
	strlcpy(new_client->name, client_name, I2C_NAME_SIZE);

	init_MUTEX(&data->update_lock);

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_free;
	
	/* Initialize the PCF8575 chip */
	pcf8575_init_client(new_client);

	/* Register sysfs hooks */
	DEVICE_CREATE_FILE(&new_client->dev, &dev_attr_read);
	DEVICE_CREATE_FILE(&new_client->dev, &dev_attr_write);
	return 0;

/* OK, this is not exactly good programming practice, usually. But it is
   very code-efficient in this case. */

      exit_free:
	kfree(data);
      exit:
	return err;
}

static int pcf8575_detach_client(struct i2c_client *client)
{
	int err;

	if ((err = i2c_detach_client(client))) {
		dev_err(&client->dev,
			"Client deregistration failed, client not detached.\n");
		return err;
	}

	kfree(i2c_get_clientdata(client));
	return 0;
}

/* Called when we have found a new PCF8575. */
static void pcf8575_init_client(struct i2c_client *client)
{
	struct pcf8575_data *data = i2c_get_clientdata(client);
	data->write = PCF8575_INIT;
	i2c_write_word(client, data->write);
}

static struct pcf8575_data *pcf8575_update_client(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf8575_data *data = i2c_get_clientdata(client);

	down(&data->update_lock);
	dev_dbg(&client->dev, "Starting pcf8575 update\n");
	data->read = i2c_read_word(client); 
	up(&data->update_lock);
	
	return data;
}

static int __init pcf8575_init(void)
{
	return i2c_add_driver(&pcf8575_driver);
}

static void __exit pcf8575_exit(void)
{
	i2c_del_driver(&pcf8575_driver);
}


MODULE_AUTHOR( "Peter Milne <peter.milne AT D-TACQ DOT COM>");
MODULE_DESCRIPTION("PCF8575 driver");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL_GPL(pcf8575_read);
EXPORT_SYMBOL_GPL(pcf8575_write);

module_init(pcf8575_init);
module_exit(pcf8575_exit);

