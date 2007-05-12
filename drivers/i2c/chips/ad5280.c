/*
    ad5280.c - Part of lm_sensors, Linux kernel modules for hardware
             monitoring
    Copyright (C) 2004  Peter Milne peter.milne  www.d-tacq.com

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

#define I2C_DRIVERID_AD5280 0xf039

/* A few notes about the AD5280:

* AD5280: 1 x 8 bit pot, 2 bits DIO
* AD5282: 2 x 8 bit pot, 1 bit DIO
*
* Explicit support currently only for first POT, no DIO.
* Not sure how to autodetect an AD5282 anyway
*/

#define DEVICE_CREATE_FILE(dev, attr)					\
	do {								\
		if (device_create_file(dev, attr) != 0) return -ENODEV;	\
	} while(0)


#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>

/* Addresses to scan */
static unsigned short normal_i2c[] = { 
	0x2c, 0x2d, 0x2e, 0x2f, I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_2(ad5280, ad5282);

/* Initial values */
#define AD5280_INIT 128	/* All outputs on (input mode) */

/* Each client has this additional data */
struct ad5280_data {
	struct i2c_client client;
	struct semaphore update_lock;

	u16 read, write;			/* Register values */
};

static int ad5280_attach_adapter(struct i2c_adapter *adapter);
static int ad5280_probe(struct i2c_adapter *adapter, int address, int kind);
static int ad5280_detach_client(struct i2c_client *client);
static void ad5280_init_client(struct i2c_client *client);
static struct ad5280_data *ad5280_update_client(struct device *dev);

/* This is the driver that will be inserted */
static struct i2c_driver ad5280_driver = {
	.driver = {
		.name		= "ad5280",
	},
	.id		= I2C_DRIVERID_AD5280,
	.attach_adapter	= ad5280_attach_adapter,
	.detach_client	= ad5280_detach_client,
};

static int i2c_write_word(struct i2c_client *client, u16 value)
{
	value = htons(value);
	return i2c_master_send(client, (char*)&value, 2);
}

static int i2c_read_word(struct i2c_client *client)
{
	u16 value;
	i2c_master_recv(client, (char*)&value, 2);
	return ntohs(value);
}

/* following are the sysfs callback functions */
static ssize_t show_read(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	struct ad5280_data *data = ad5280_update_client(dev);
	return sprintf(buf, "%04x\n", data->read);
}

static DEVICE_ATTR(read, S_IRUGO, show_read, NULL);

static ssize_t show_write(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	struct ad5280_data *data = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%04x\n", data->write);
}

static ssize_t set_write(
	struct device *dev, 
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ad5280_data *data = i2c_get_clientdata(client);
	data->write = simple_strtoul(buf, NULL, 16);
	i2c_write_word(client, data->write);
	return count;
}

static DEVICE_ATTR(write, S_IWUGO | S_IRUGO, show_write, set_write);



static ssize_t show_wiper(
	struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	struct ad5280_data *data = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%d\n", ((int)data->write&0xff) - 0x80);
}
static ssize_t set_wiper(
	struct device *dev, 
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int wiper;

	if (sscanf(buf, "%d", &wiper) || sscanf(buf, "0x%x", &wiper)){
		struct i2c_client *client = to_i2c_client(dev);
		struct ad5280_data *data = i2c_get_clientdata(client);

		wiper += 0x80;

		if (wiper < 0) wiper = 0;
		if (wiper > 255) wiper = 255;

		data->write &= ~255;
		data->write |= wiper;
		i2c_write_word(client, data->write);
		return strlen(buf);
	}else{
		return -1;
	}
}

static DEVICE_ATTR(wiper, S_IWUGO | S_IRUGO, show_wiper, set_wiper);

/*
 * Real code
 */

static int ad5280_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, ad5280_probe);
}

/* This function is called by i2c_detect */
int ad5280_probe(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct ad5280_data *data;
	int err = 0;
	const char *client_name = "";

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		goto exit;

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet. */
	if (!(data = kmalloc(sizeof(struct ad5280_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct ad5280_data));

	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &ad5280_driver;
	new_client->flags = 0;

	/* Now, we would do the remaining detection. But the AD5280 is plainly
	   impossible to detect! Stupid chip. */

	/* Determine the chip type */
	if (kind <= 0) {
		kind = ad5280;
	}

	client_name = "ad5280";

	/* Fill in the remaining client fields and put it into the global list */
	strlcpy(new_client->name, client_name, I2C_NAME_SIZE);

	init_MUTEX(&data->update_lock);

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_free;
	
	/* Initialize the AD5280 chip */
	ad5280_init_client(new_client);

	/* Register sysfs hooks */
	DEVICE_CREATE_FILE(&new_client->dev, &dev_attr_read);
	DEVICE_CREATE_FILE(&new_client->dev, &dev_attr_write);
	DEVICE_CREATE_FILE(&new_client->dev, &dev_attr_wiper);
	return 0;

/* OK, this is not exactly good programming practice, usually. But it is
   very code-efficient in this case. */

      exit_free:
	kfree(data);
      exit:
	return err;
}

static int ad5280_detach_client(struct i2c_client *client)
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

/* Called when we have found a new AD5280. */
static void ad5280_init_client(struct i2c_client *client)
{
	struct ad5280_data *data = i2c_get_clientdata(client);
	data->write = AD5280_INIT;
	i2c_write_word(client, data->write);
}

static struct ad5280_data *ad5280_update_client(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ad5280_data *data = i2c_get_clientdata(client);

	down(&data->update_lock);
	dev_dbg(&client->dev, "Starting ad5280 update\n");
	data->read = i2c_read_word(client); 
	up(&data->update_lock);
	
	return data;
}

static int __init ad5280_init(void)
{
	return i2c_add_driver(&ad5280_driver);
}

static void __exit ad5280_exit(void)
{
	i2c_del_driver(&ad5280_driver);
}


MODULE_AUTHOR( "Peter Milne <peter.milne AT D-TACQ DOT COM>");
MODULE_DESCRIPTION("AD5280 driver");
MODULE_LICENSE("GPL");

module_init(ad5280_init);
module_exit(ad5280_exit);
