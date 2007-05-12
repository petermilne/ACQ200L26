/*
 * ad5391.c -  Part of lm_sensors, Linux kernel modules for hardware
 * monitoring.
 * Copyright (C) 2005 Peter Milne www.d-tacq.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


/*
 * Chip driver for Analog Devices AD539x 16 channel bit DAC
 *
 * AD5390: 16ch, 14 bit
 * AD5391: 16ch, 12 bit (tested with this one)
 * AD5392:  8ch  14 bit
 *
 * module based on:
 * fscher.c Copyright (C) 2003, 2004 Reinhard Nissl <rnissl@gmx.de>
 *
 * Application control of device is via sysfs files:
 *
 * reset - echo 1 > reset to reset
 *
 * channel values are set as signed decimal strings as follows:
 * chNN  { NN: 01 .. 16 } - output value
 * _cNN  { NN: 01 .. 16 } - cal constant
 * _mNN  { NN: 01 .. 16 } - gain value
 *
 */

#define AD539x 5391                  /* actual model number */
#define I2C_DRIVERID_AD539X 0xf037   /* @@todo get official i2c-id.h */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>


/* 
 * debug stuff - this live in a header file.
 */

#include <linux/moduleparam.h>


#include "ad539x.h"

#define VERID "ad539x B1003"

#define acq200_debug ad539x_debug  

#define FN __FUNCTION__
#define KE KERN_ERR
#define KI KERN_INFO
#define KW KERN_WARNING
//#define KD KERN_DEBUG
#define KD KI

#define dbg(lvl,format, arg...)	\
	do {						\
		if(acq200_debug>=lvl)                   \
			printk (KD "%s: " format "\n",	\
				FN , ## arg); 		\
	} while(0)

#define err(format, arg...) printk(KE "%s: " format "\n", FN , ## arg)
#define info(format, arg...) printk(KI "%s: " format "\n", FN , ## arg)
#define warn(format, arg...) printk(KW "%s: " format "\n", FN , ## arg)



int ad539x_debug = 0;
/*
 * this didn't work - not compatible with i2c-sensors.h 
module_param(ad539x_debug, int, 0666);
         so we use sysfs instead
 */




/*
 * Addresses to scan
 */

static unsigned short normal_i2c[] = { 0x56, I2C_CLIENT_END };

#if (AD539x == 5391)
I2C_CLIENT_INSMOD_1(ad5391);
#else
I2C_CLIENT_INSMOD_1(ad5390);
#endif

/*
 * Internal Reg defs from datasheet 555455492AD5390_1_2_a.pdf
 */

#define NCHAN 16


/*
 * maps a quad bytes instruction sent to the chip
 */
struct ad539x_i2c_quad {
	u8 addr;
	u8 ptr;
	u8 msb;
	u8 lsb;
};

#define REG_SEL_DATA 0xc0
#define REG_SEL_C    0x80
#define REG_SEL_M    0x40
#define REG_SEL_SFR  0x00

#define SFR_ADDR_NOP            0x0
#define SFR_ADDR_WRITE_CLR_CODE 0x1
#define SFR_ADDR_CLR            0x2
/* 0x3 - 0x7 not used */
#define SFR_ADDR_SPD            0x8
#define SFR_ADDR_SPU            0x9
/* 0xa not used */
/* 0xb not used */
#define SFR_ADDR_CRW            0xc
#define SFR_ADDR_CRR            0xc                /* READ!! */
#define SFR_ADDR_MON            0x9
#define SFR_ADDR_RESET          0xf


#define SFR_DONT_CARE 0

#define CR_PD_HIZ               0x2000            /* alt: 100K to ground */
#define CR_INTERNAL_REF_2p5     0x1000            /* alt: 1.25V */
#define CR_BOOST                0x0800
#define CR_INTERNAL_REF         0x0400
#define CR_CHANNEL_MONITOR      0x0200
#define CR_THERMAL_MONITOR      0x0100
/* don't care 0x00f0 */
#define CR_TOGGLE_15_08         0x0008
#define CR_TOGGLE_07_00         0x0004
/* don't care 0x0002 */

#define PTR_QUAD_MODE 0x00

static inline u8 msb(u16 value) { return value >> 8; }
static inline u8 lsb(u16 value) { return value&0x00ff; }


struct ad539x_data;


/*
 * Methods
 */
static int ad539x_attach_adapter(struct i2c_adapter *adapter);
static int ad539x_detect(struct i2c_adapter *adapter, int address, int kind);
static int ad539x_detach_client(struct i2c_client *client);
static void ad539x_update_client(struct i2c_client *client);
static void ad539x_init_client(struct i2c_client *client);

static int ad539x_reset(struct i2c_client *client, struct ad539x_data *data);
static int _ad539x_set_channel(
		struct i2c_client *client, int chaddr, u16 value);
static int ad539x_set_channel_gain(
		struct i2c_client *client, int chaddr, u16 value);
static int ad539x_set_channel_offset(
		struct i2c_client *client, int chaddr, u16 value);
static int ad539x_write_value4(
	struct i2c_client *client, struct ad539x_i2c_quad *data);



/*
 * Driver data (common to all clients)
 */
 
static struct i2c_driver ad539x_driver = {
	.driver = {
		.name		= "ad539x",
	},
	.id		= I2C_DRIVERID_AD539X,
	.attach_adapter	= ad539x_attach_adapter,
	.detach_client	= ad539x_detach_client,
};

/*
 * Client data (each client gets its own)
 */

struct ad539x_data {
	struct semaphore update_lock;
	char valid; /* zero until following fields are valid */
	unsigned long last_updated; /* in jiffies */

	int is_12bit;
	struct Channel {         /* client: signed values hw is different */
		short value;
		short offset;
		short gain;
	}
	channels[NCHAN];
};

static inline short maxval(struct ad539x_data *data) {
	return data->is_12bit? 2047: 16383;
}
static inline short minval(struct ad539x_data *data) {
	return data->is_12bit? -2048: -16384;
}


/*
 * module scope data
 */

#define AD539X_CLIENT_SZ \
        (sizeof(struct i2c_client) + sizeof(struct ad539x_data))
/*
 * Sysfs stuff - luckily max 16 channels.
 */

/** Value */
static ssize_t show_ch (struct ad539x_data *, char *, int);
static ssize_t set_ch (struct i2c_client *, struct ad539x_data *, 
		       const char *, size_t, int);

/** Offset Cal */
static ssize_t show__c (struct ad539x_data *, char *, int);
static ssize_t set__c(struct i2c_client *, struct ad539x_data *, 
		       const char *, size_t, int);
/** Gain Cal */
static ssize_t show__m (struct ad539x_data *, char *, int);
static ssize_t set__m (struct i2c_client *, struct ad539x_data *, 
		       const char *, size_t, int);

#define sysfs_ch_r(kind, chans, ch)				\
static ssize_t show_##kind##chans (				\
	struct device *,					\
	struct device_attribute *attr,				\
	char *);						\
static ssize_t show_##kind##chans (				\
	struct device *dev,					\
	struct device_attribute *attr,				\
	char *buf)						\
{								\
	struct i2c_client *client = to_i2c_client(dev);		\
	struct ad539x_data *data = i2c_get_clientdata(client);	\
	ad539x_update_client(client);				\
	return show_##kind(data, buf, ch );			\
}

#define sysfs_ch_w(kind, chans, ch)				\
static ssize_t set_##kind##chans (				\
	struct device *,					\
	struct device_attribute *attr,				\
	const char *, size_t);					\
static ssize_t set_##kind##chans (				\
	struct device *dev,					\
	struct device_attribute *attr,				\
	const char *buf, size_t count)				\
{								\
	struct i2c_client *client = to_i2c_client(dev);		\
	struct ad539x_data *data = i2c_get_clientdata(client);	\
	return set_##kind(client, data, buf, count, ch);	\
}

#define sysfs_ch_rw(kind, chans, chn) \
sysfs_ch_r(kind, chans, chn) \
sysfs_ch_w(kind, chans, chn) \
static DEVICE_ATTR(kind##chans, S_IRUGO | S_IWUGO, \
                   show_##kind##chans, set_##kind##chans);

#define sysfs_r(kind, offset, reg)				\
static ssize_t show_##kind (struct ad539x_data *, char *, int); \
static ssize_t show_##kind##offset (				\
	struct device *,					\
	struct device_attribute *attr,				\
	char *);						\
static ssize_t show_##kind##offset (				\
	struct device *dev,					\
	struct device_attribute *attr,				\
	char *buf)						\
{								\
	struct i2c_client *client = to_i2c_client(dev);		\
	struct ad539x_data *data = i2c_get_clientdata(client);	\
	ad539x_update_client(client);				\
	return show_##kind(data, buf, (offset));		\
}

#define sysfs_w(kind, offset, reg)					\
static ssize_t set_##kind (						\
	struct i2c_client *, struct ad539x_data *,			\
	const char *, size_t, int, int);				\
static ssize_t set_##kind##offset (					\
	struct device *,						\
	struct device_attribute *attr,					\
	const char *, size_t);						\
static ssize_t set_##kind##offset (					\
	struct device *dev,						\
	struct device_attribute *attr,					\
	const char *buf, size_t count)					\
{									\
	struct i2c_client *client = to_i2c_client(dev);			\
	struct ad539x_data *data = i2c_get_clientdata(client);		\
	return set_##kind(client, data, buf, count, (offset), reg);	\
}


#define sysfs_ro(kind, reg) \
sysfs_r(kind, 0, reg) \
static DEVICE_ATTR(kind, S_IRUGO, show_##kind##0, NULL);



#define sysfs_rw(kind, reg) \
sysfs_r(kind, 0, reg) \
sysfs_w(kind, 0, reg) \
static DEVICE_ATTR(kind, S_IRUGO | S_IWUGO, show_##kind##0, set_##kind##0);


#define sysfs_channel(chans, chn) \
        sysfs_ch_rw(ch, chans, chn); \
        sysfs_ch_rw(_m, chans, chn); \
        sysfs_ch_rw(_c, chans, chn)

#define sysfs_reset      sysfs_rw(reset, 0)

#define sysfs_debug      sysfs_rw(debug, 0)
/*
 * static def of all the sysfs hooks 
 */

sysfs_reset;
sysfs_debug;
/** static def of 16 channels. To bad no for() loop */
sysfs_channel(01, 0);
sysfs_channel(02, 1);
sysfs_channel(03, 2);
sysfs_channel(04, 3);
sysfs_channel(05, 4);
sysfs_channel(06, 5);
sysfs_channel(07, 6);
sysfs_channel(08, 7);
#if AD539x != 5392                              /* 16 channel versions .. */
sysfs_channel(09, 8);
sysfs_channel(10, 9);
sysfs_channel(11,10);
sysfs_channel(12,11);
sysfs_channel(13,12);
sysfs_channel(14,13);
sysfs_channel(15,14);
sysfs_channel(16,15);
#endif

#define DEVICE_CREATE_FILE(dev, attr)					\
	do {								\
		if (device_create_file(dev, attr) != 0) return -ENODEV;	\
	} while(0)


#define device_create_file_reset(client) \
        DEVICE_CREATE_FILE(&client->dev, &dev_attr_reset);

#define device_create_file_debug(client) \
        DEVICE_CREATE_FILE(&client->dev, &dev_attr_debug);

#define device_create_file_channel(client, chans) \
do { \
        DEVICE_CREATE_FILE(&client->dev, &dev_attr_ch##chans); \
        DEVICE_CREATE_FILE(&client->dev, &dev_attr__m##chans); \
        DEVICE_CREATE_FILE(&client->dev, &dev_attr__c##chans); \
} while(0)

static int ad539x_mk_sysfs(struct i2c_client *client)
{
	device_create_file_reset(client);
	device_create_file_debug(client);

	device_create_file_channel(client, 01);
	device_create_file_channel(client, 02);
	device_create_file_channel(client, 03);
	device_create_file_channel(client, 04);
	device_create_file_channel(client, 05);
	device_create_file_channel(client, 06);
	device_create_file_channel(client, 07);
	device_create_file_channel(client, 08);
#if AD539x != 5392                              /* 16 channel versions .. */
	device_create_file_channel(client, 09);
	device_create_file_channel(client, 10);
	device_create_file_channel(client, 11);
	device_create_file_channel(client, 12);
	device_create_file_channel(client, 13);
	device_create_file_channel(client, 14);
	device_create_file_channel(client, 15);
	device_create_file_channel(client, 16);
#endif
	return 0;
}


static u16 ad539x_scale(struct ad539x_data *data, short value)
{
	if (data->is_12bit){
		return (value + 0x800) << 2;
	}else{
		return value + 0x2000;
	}
}

static int validate_ch(
	struct ad539x_data *data, short *pvalue, const char *buf, size_t count)
{
	int sdata;

	if (sscanf(buf, "%d", &sdata) == 1){
		*pvalue = SENSORS_LIMIT(sdata, minval(data), maxval(data));
	}else{
		return -1;
	}
	return count;
}

static ssize_t set_ch(struct i2c_client *client, struct ad539x_data *data,
			   const char *buf, size_t count, int chn)
{
	short *pvalue = &data->channels[chn].value;

	count = validate_ch(data, pvalue, buf, count);

	if (count > 0){
		_ad539x_set_channel(client, chn, ad539x_scale(data, *pvalue));
	}

	return count;
}


static ssize_t show_ch_value(char *buf, short value)
{
	return sprintf(buf, "%d\n", value);
}


static ssize_t show_ch(struct ad539x_data *data, char *buf, int chn)
{
	return show_ch_value(buf, data->channels[chn].value);
}


/** Offset Cal */
static ssize_t show__c (struct ad539x_data *data, char *buf, int chn)
{
	return show_ch_value(buf, data->channels[chn].offset);
}
static ssize_t set__c(struct i2c_client * client, struct ad539x_data *data, 
		       const char *buf, size_t count, int chn)
{
	short* pvalue = &data->channels[chn].offset;

	count = validate_ch(data, pvalue, buf, count);

	if (count > 0){
		ad539x_set_channel_offset(
			client, chn, ad539x_scale(data, *pvalue));
	}

	return count;
}
/** Gain Cal */
static ssize_t show__m (struct ad539x_data *data, char *buf, int chn)
{
	return show_ch_value(buf, data->channels[chn].gain);
}

static ssize_t set__m (struct i2c_client *client, struct ad539x_data *data, 
		       const char *buf, size_t count, int chn)
{
	short* pvalue = &data->channels[chn].gain;

	count = validate_ch(data, pvalue, buf, count);

	if (count > 0){
		ad539x_set_channel_gain(
			client, chn, ad539x_scale(data, *pvalue));
	}

	return count;
}



static ssize_t set_reset(struct i2c_client *client, struct ad539x_data *data,
			   const char *buf, size_t count, int nr, int reg)
{
	ad539x_reset(client, data);
	return count;
}

static ssize_t show_reset(struct ad539x_data *data, char *buf, int nr)
{
	return sprintf(buf, "Reset\n");
}


static ssize_t set_debug(struct i2c_client *client, struct ad539x_data *data,
			   const char *buf, size_t count, int nr, int reg)
{
	sscanf(buf, "%d", &ad539x_debug);
	return count;
}

static ssize_t show_debug(struct ad539x_data *data, char *buf, int nr)
{
	return sprintf(buf, "%d\n",ad539x_debug );
}



static int ad539x_set_sfr(
	struct i2c_client *client, unsigned sfr_addr, unsigned value)
{
	struct ad539x_i2c_quad quad;

	quad.addr = client->addr;
	quad.ptr = PTR_QUAD_MODE | sfr_addr;
	quad.msb = REG_SEL_SFR | msb(value);
	quad.lsb = lsb(value);

	return ad539x_write_value4(client, &quad);
	
}
static int ad539x_reset(struct i2c_client *client, struct ad539x_data *data)
{
	int rc = 0;

	rc = ad539x_set_sfr(client, SFR_ADDR_RESET, SFR_DONT_CARE);
	
	udelay(200);

	rc = ad539x_set_sfr(client, SFR_ADDR_SPU, SFR_DONT_CARE);
	rc = ad539x_set_sfr(client, SFR_ADDR_CRW, 
		       CR_INTERNAL_REF_2p5|CR_INTERNAL_REF);

	return rc;
}

static int _ad539x_set_channel(struct i2c_client *client, int chaddr, u16 value)
{
	struct ad539x_i2c_quad quad;

	quad.addr = client->addr;
	quad.ptr = PTR_QUAD_MODE | chaddr;
	quad.msb = REG_SEL_DATA | msb(value);
	quad.lsb = lsb(value);
	
	return ad539x_write_value4(client, &quad);
}

static int ad539x_set_channel_offset(struct i2c_client *client, int chaddr, u16 value)
{
	struct ad539x_i2c_quad quad;

	quad.addr = client->addr;
	quad.ptr = PTR_QUAD_MODE | chaddr;
	quad.msb = REG_SEL_C | msb(value);
	quad.lsb = lsb(value);
	
	return ad539x_write_value4(client, &quad);
}

static int ad539x_set_channel_gain(struct i2c_client *client, int chaddr, u16 value)
{
	struct ad539x_i2c_quad quad;

	quad.addr = client->addr;
	quad.ptr = PTR_QUAD_MODE | chaddr;
	quad.msb = REG_SEL_M | msb(value);
	quad.lsb = lsb(value);
	
	return ad539x_write_value4(client, &quad);
}


int ad539x_set_channel(struct device *dev, int chaddr, u16 value)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ad539x_data *data = i2c_get_clientdata(client);
	
	return _ad539x_set_channel(client, chaddr, ad539x_scale(data, value));
}

static int ad539x_write_value4(
	struct i2c_client *client, struct ad539x_i2c_quad *msg)
{
	char *dbgp = &msg->ptr;
	int rc;

	dbg(1, "send: 0x%02x 0x%02x 0x%02x 0x%02x",
	    dbgp[-1], dbgp[0], dbgp[1], dbgp[2] );

 /* driver sends addr for us */
	rc = i2c_master_send(client, &msg->ptr, 3);

	return rc;
}


static void ad539x_update_client(struct i2c_client *client)
{
	/** easy - it's Write Only !! */
}

static int ad539x_detach_client(struct i2c_client *client)
{
	return 0;
}
static void ad539x_init_client(struct i2c_client *client)
{
/* @@todo - what happens here ?? */
}




static int ad539x_attach_adapter(struct i2c_adapter *adapter)
{
/** @@todo --- just what class _is_ our adapter ? 
	if (!(adapter->class & I2C_ADAP_CLASS_SMBUS))
		return 0;
*/	

	return i2c_probe(adapter, &addr_data, ad539x_detect);
}


static int ad539x_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct ad539x_data *data = 0;
	int err = 0;
	
	if (!(new_client = kmalloc(AD539X_CLIENT_SZ, GFP_KERNEL))){
		err = -ENOMEM;
		goto exit;
	}

	memset(new_client, 0, AD539X_CLIENT_SZ);
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &ad539x_driver;
	new_client->flags = 0;

	strncpy(new_client->name, "ad539x", I2C_NAME_SIZE);

	data = (struct ad539x_data *) (new_client + 1);
#if (AD539x == 5391)
	data->is_12bit = 1;
#endif

	i2c_set_clientdata(new_client, data);
	data->valid = 0;
	init_MUTEX(&data->update_lock);

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client))){
		goto exit_free;
	}

	ad539x_init_client(new_client);
	ad539x_mk_sysfs(new_client);

	return 0;

 exit_free:
	kfree(new_client);
 exit:
	return err;

}



static int __init sensors_ad539x_init(void)
{
	info("%s", VERID);
	dbg(1, "sizeof struct ad539x_i2c_quad: %d", 
	     sizeof(struct ad539x_i2c_quad));

	return i2c_add_driver(&ad539x_driver);
}

static void __exit sensors_ad539x_exit(void)
{
	i2c_del_driver(&ad539x_driver);
}

MODULE_AUTHOR("Peter Milne www.d-tacq.com");
MODULE_DESCRIPTION("AD539x multi channel DAC i2c driver");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL_GPL(ad539x_set_channel);

module_init(sensors_ad539x_init);
module_exit(sensors_ad539x_exit);
