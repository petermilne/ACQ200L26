/*
 * File:         arch/blackfin/kernel/bfin_dma_5xx.c
 * Based on:
 * Author:
 *
 * Created:
 * Description:  This file contains the simple DMA Implementation for Blackfin
 *
 * Modified:
 *               Copyright 2004-2006 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
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
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/param.h>

#include <asm/dma.h>
#include <asm/cacheflush.h>

/* Remove unused code not exported by symbol or internally called */
#define REMOVE_DEAD_CODE

/**************************************************************************
 * Global Variables
***************************************************************************/

static struct dma_channel dma_ch[MAX_BLACKFIN_DMA_CHANNEL];
#if defined (CONFIG_BF561)
static struct dma_register *base_addr[MAX_BLACKFIN_DMA_CHANNEL] = {
	(struct dma_register *) DMA1_0_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_1_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_2_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_3_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_4_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_5_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_6_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_7_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_8_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_9_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_10_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_11_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_0_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_1_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_2_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_3_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_4_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_5_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_6_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_7_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_8_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_9_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_10_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_11_NEXT_DESC_PTR,
	(struct dma_register *) MDMA1_D0_NEXT_DESC_PTR,
	(struct dma_register *) MDMA1_S0_NEXT_DESC_PTR,
	(struct dma_register *) MDMA1_D1_NEXT_DESC_PTR,
	(struct dma_register *) MDMA1_S1_NEXT_DESC_PTR,
	(struct dma_register *) MDMA2_D0_NEXT_DESC_PTR,
	(struct dma_register *) MDMA2_S0_NEXT_DESC_PTR,
	(struct dma_register *) MDMA2_D1_NEXT_DESC_PTR,
	(struct dma_register *) MDMA2_S1_NEXT_DESC_PTR,
	(struct dma_register *) IMDMA_D0_NEXT_DESC_PTR,
	(struct dma_register *) IMDMA_S0_NEXT_DESC_PTR,
	(struct dma_register *) IMDMA_D1_NEXT_DESC_PTR,
	(struct dma_register *) IMDMA_S1_NEXT_DESC_PTR,
};
#else
static struct dma_register *base_addr[MAX_BLACKFIN_DMA_CHANNEL] = {
	(struct dma_register *) DMA0_NEXT_DESC_PTR,
	(struct dma_register *) DMA1_NEXT_DESC_PTR,
	(struct dma_register *) DMA2_NEXT_DESC_PTR,
	(struct dma_register *) DMA3_NEXT_DESC_PTR,
	(struct dma_register *) DMA4_NEXT_DESC_PTR,
	(struct dma_register *) DMA5_NEXT_DESC_PTR,
	(struct dma_register *) DMA6_NEXT_DESC_PTR,
	(struct dma_register *) DMA7_NEXT_DESC_PTR,
#if (defined(CONFIG_BF537) || defined(CONFIG_BF534) || defined(CONFIG_BF536))
	(struct dma_register *) DMA8_NEXT_DESC_PTR,
	(struct dma_register *) DMA9_NEXT_DESC_PTR,
	(struct dma_register *) DMA10_NEXT_DESC_PTR,
	(struct dma_register *) DMA11_NEXT_DESC_PTR,
#endif
	(struct dma_register *) MDMA_D0_NEXT_DESC_PTR,
	(struct dma_register *) MDMA_S0_NEXT_DESC_PTR,
	(struct dma_register *) MDMA_D1_NEXT_DESC_PTR,
	(struct dma_register *) MDMA_S1_NEXT_DESC_PTR,
};
#endif

/*------------------------------------------------------------------------------
 *       Set the Buffer Clear bit in the Configuration register of specific DMA
 *       channel. This will stop the descriptor based DMA operation.
 *-----------------------------------------------------------------------------*/
static void clear_dma_buffer(unsigned int channel)
{
	dma_ch[channel].regs->cfg |= RESTART;
	SSYNC();
	dma_ch[channel].regs->cfg &= ~RESTART;
	SSYNC();
}

int __init blackfin_dma_init(void)
{
	int i;

	printk(KERN_INFO "Blackfin DMA Controller\n");

	for (i = 0; i < MAX_BLACKFIN_DMA_CHANNEL; i++) {
		dma_ch[i].chan_status = DMA_CHANNEL_FREE;
		dma_ch[i].regs = base_addr[i];
		mutex_init(&(dma_ch[i].dmalock));
	}

	return 0;
}

arch_initcall(blackfin_dma_init);

/*
 *	Form the channel find the irq number for that channel.
 */
#if !defined(CONFIG_BF561)

static int bf533_channel2irq(unsigned int channel)
{
	int ret_irq = -1;

	switch (channel) {
	case CH_PPI:
		ret_irq = IRQ_PPI;
		break;

#if (defined(CONFIG_BF537) || defined(CONFIG_BF534) || defined(CONFIG_BF536))
	case CH_EMAC_RX:
		ret_irq = IRQ_MAC_RX;
		break;

	case CH_EMAC_TX:
		ret_irq = IRQ_MAC_TX;
		break;

	case CH_UART1_RX:
		ret_irq = IRQ_UART1_RX;
		break;

	case CH_UART1_TX:
		ret_irq = IRQ_UART1_TX;
		break;
#endif

	case CH_SPORT0_RX:
		ret_irq = IRQ_SPORT0_RX;
		break;

	case CH_SPORT0_TX:
		ret_irq = IRQ_SPORT0_TX;
		break;

	case CH_SPORT1_RX:
		ret_irq = IRQ_SPORT1_RX;
		break;

	case CH_SPORT1_TX:
		ret_irq = IRQ_SPORT1_TX;
		break;

	case CH_SPI:
		ret_irq = IRQ_SPI;
		break;

	case CH_UART_RX:
		ret_irq = IRQ_UART_RX;
		break;

	case CH_UART_TX:
		ret_irq = IRQ_UART_TX;
		break;

	case CH_MEM_STREAM0_SRC:
	case CH_MEM_STREAM0_DEST:
		ret_irq = IRQ_MEM_DMA0;
		break;

	case CH_MEM_STREAM1_SRC:
	case CH_MEM_STREAM1_DEST:
		ret_irq = IRQ_MEM_DMA1;
		break;
	}
	return ret_irq;
}

# define channel2irq(channel) bf533_channel2irq(channel)

#else

static int bf561_channel2irq(unsigned int channel)
{
	int ret_irq = -1;

	switch (channel) {
	case CH_PPI0:
		ret_irq = IRQ_PPI0;
		break;
	case CH_PPI1:
		ret_irq = IRQ_PPI1;
		break;
	case CH_SPORT0_RX:
		ret_irq = IRQ_SPORT0_RX;
		break;
	case CH_SPORT0_TX:
		ret_irq = IRQ_SPORT0_TX;
		break;
	case CH_SPORT1_RX:
		ret_irq = IRQ_SPORT1_RX;
		break;
	case CH_SPORT1_TX:
		ret_irq = IRQ_SPORT1_TX;
		break;
	case CH_SPI:
		ret_irq = IRQ_SPI;
		break;
	case CH_UART_RX:
		ret_irq = IRQ_UART_RX;
		break;
	case CH_UART_TX:
		ret_irq = IRQ_UART_TX;
		break;

	case CH_MEM_STREAM0_SRC:
	case CH_MEM_STREAM0_DEST:
		ret_irq = IRQ_MEM_DMA0;
		break;
	case CH_MEM_STREAM1_SRC:
	case CH_MEM_STREAM1_DEST:
		ret_irq = IRQ_MEM_DMA1;
		break;
	case CH_MEM_STREAM2_SRC:
	case CH_MEM_STREAM2_DEST:
		ret_irq = IRQ_MEM_DMA2;
		break;
	case CH_MEM_STREAM3_SRC:
	case CH_MEM_STREAM3_DEST:
		ret_irq = IRQ_MEM_DMA3;
		break;

	case CH_IMEM_STREAM0_SRC:
	case CH_IMEM_STREAM0_DEST:
		ret_irq = IRQ_IMEM_DMA0;
		break;
	case CH_IMEM_STREAM1_SRC:
	case CH_IMEM_STREAM1_DEST:
		ret_irq = IRQ_IMEM_DMA1;
		break;
	}
	return ret_irq;
}

# define channel2irq(channel) bf561_channel2irq(channel)

#endif

/*------------------------------------------------------------------------------
 *	Request the specific DMA channel from the system.
 *-----------------------------------------------------------------------------*/
int request_dma(unsigned int channel, char *device_id)
{

	pr_debug("request_dma() : BEGIN \n");
	mutex_lock(&(dma_ch[channel].dmalock));

	if ((dma_ch[channel].chan_status == DMA_CHANNEL_REQUESTED)
	    || (dma_ch[channel].chan_status == DMA_CHANNEL_ENABLED)) {
		mutex_unlock(&(dma_ch[channel].dmalock));
		pr_debug("DMA CHANNEL IN USE  \n");
		return -EBUSY;
	} else {
		dma_ch[channel].chan_status = DMA_CHANNEL_REQUESTED;
		pr_debug("DMA CHANNEL IS ALLOCATED  \n");
	}

	mutex_unlock(&(dma_ch[channel].dmalock));

	dma_ch[channel].device_id = device_id;
	dma_ch[channel].irq_callback = NULL;

	/* This is to be enabled by putting a restriction -
	 * you have to request DMA, before doing any operations on
	 * descriptor/channel
	 */
	pr_debug("request_dma() : END  \n");
	return channel;
}
EXPORT_SYMBOL(request_dma);

int set_dma_callback(unsigned int channel, dma_interrupt_t callback, void *data)
{
	int ret_irq = 0;

	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	if (callback != NULL) {
		int ret_val;
		ret_irq = channel2irq(channel);

		dma_ch[channel].data = data;

		ret_val =
		    request_irq(ret_irq, (void *)callback, IRQF_DISABLED,
				dma_ch[channel].device_id, data);
		if (ret_val) {
			printk(KERN_NOTICE
			       "Request irq in DMA engine failed.\n");
			return -EPERM;
		}
		dma_ch[channel].irq_callback = callback;
	}
	return 0;
}
EXPORT_SYMBOL(set_dma_callback);

void free_dma(unsigned int channel)
{
	int ret_irq;

	pr_debug("freedma() : BEGIN \n");
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	/* Halt the DMA */
	disable_dma(channel);
	clear_dma_buffer(channel);

	if (dma_ch[channel].irq_callback != NULL) {
		ret_irq = channel2irq(channel);
		free_irq(ret_irq, dma_ch[channel].data);
	}

	/* Clear the DMA Variable in the Channel */
	mutex_lock(&(dma_ch[channel].dmalock));
	dma_ch[channel].chan_status = DMA_CHANNEL_FREE;
	mutex_unlock(&(dma_ch[channel].dmalock));

	pr_debug("freedma() : END \n");
}
EXPORT_SYMBOL(free_dma);

void dma_enable_irq(unsigned int channel)
{
	int ret_irq;

	pr_debug("dma_enable_irq() : BEGIN \n");
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	ret_irq = channel2irq(channel);
	enable_irq(ret_irq);
}
EXPORT_SYMBOL(dma_enable_irq);

void dma_disable_irq(unsigned int channel)
{
	int ret_irq;

	pr_debug("dma_disable_irq() : BEGIN \n");
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	ret_irq = channel2irq(channel);
	disable_irq(ret_irq);
}
EXPORT_SYMBOL(dma_disable_irq);

int dma_channel_active(unsigned int channel)
{
	if (dma_ch[channel].chan_status == DMA_CHANNEL_FREE) {
		return 0;
	} else {
		return 1;
	}
}
EXPORT_SYMBOL(dma_channel_active);

/*------------------------------------------------------------------------------
*	stop the specific DMA channel.
*-----------------------------------------------------------------------------*/
void disable_dma(unsigned int channel)
{
	pr_debug("stop_dma() : BEGIN \n");

	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	dma_ch[channel].regs->cfg &= ~DMAEN;	/* Clean the enable bit */
	SSYNC();
	dma_ch[channel].chan_status = DMA_CHANNEL_REQUESTED;
	/* Needs to be enabled Later */
	pr_debug("stop_dma() : END \n");
	return;
}
EXPORT_SYMBOL(disable_dma);

void enable_dma(unsigned int channel)
{
	pr_debug("enable_dma() : BEGIN \n");

	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	dma_ch[channel].chan_status = DMA_CHANNEL_ENABLED;
	dma_ch[channel].regs->curr_x_count = 0;
	dma_ch[channel].regs->curr_y_count = 0;

	dma_ch[channel].regs->cfg |= DMAEN;	/* Set the enable bit */
	SSYNC();
	pr_debug("enable_dma() : END \n");
	return;
}
EXPORT_SYMBOL(enable_dma);

/*------------------------------------------------------------------------------
*		Set the Start Address register for the specific DMA channel
* 		This function can be used for register based DMA,
*		to setup the start address
*		addr:		Starting address of the DMA Data to be transferred.
*-----------------------------------------------------------------------------*/
void set_dma_start_addr(unsigned int channel, unsigned long addr)
{
	pr_debug("set_dma_start_addr() : BEGIN \n");

	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	dma_ch[channel].regs->start_addr = addr;
	SSYNC();
	pr_debug("set_dma_start_addr() : END\n");
}
EXPORT_SYMBOL(set_dma_start_addr);

void set_dma_next_desc_addr(unsigned int channel, unsigned long addr)
{
	pr_debug("set_dma_next_desc_addr() : BEGIN \n");

	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	dma_ch[channel].regs->next_desc_ptr = addr;
	SSYNC();
	pr_debug("set_dma_start_addr() : END\n");
}
EXPORT_SYMBOL(set_dma_next_desc_addr);

void set_dma_x_count(unsigned int channel, unsigned short x_count)
{
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	dma_ch[channel].regs->x_count = x_count;
	SSYNC();
}
EXPORT_SYMBOL(set_dma_x_count);

void set_dma_y_count(unsigned int channel, unsigned short y_count)
{
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	dma_ch[channel].regs->y_count = y_count;
	SSYNC();
}
EXPORT_SYMBOL(set_dma_y_count);

void set_dma_x_modify(unsigned int channel, short x_modify)
{
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	dma_ch[channel].regs->x_modify = x_modify;
	SSYNC();
}
EXPORT_SYMBOL(set_dma_x_modify);

void set_dma_y_modify(unsigned int channel, short y_modify)
{
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	dma_ch[channel].regs->y_modify = y_modify;
	SSYNC();
}
EXPORT_SYMBOL(set_dma_y_modify);

void set_dma_config(unsigned int channel, unsigned short config)
{
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	dma_ch[channel].regs->cfg = config;
	SSYNC();
}
EXPORT_SYMBOL(set_dma_config);

unsigned short
set_bfin_dma_config(char direction, char flow_mode,
		    char intr_mode, char dma_mode, char width)
{
	unsigned short config;

	config =
	    ((direction << 1) | (width << 2) | (dma_mode << 4) |
	     (intr_mode << 6) | (flow_mode << 12) | RESTART);
	return config;
}
EXPORT_SYMBOL(set_bfin_dma_config);

void set_dma_sg(unsigned int channel, struct dmasg * sg, int nr_sg)
{
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	dma_ch[channel].regs->cfg |= ((nr_sg & 0x0F) << 8);

	dma_ch[channel].regs->next_desc_ptr = (unsigned int)sg;

	SSYNC();
}
EXPORT_SYMBOL(set_dma_sg);

/*------------------------------------------------------------------------------
 *	Get the DMA status of a specific DMA channel from the system.
 *-----------------------------------------------------------------------------*/
unsigned short get_dma_curr_irqstat(unsigned int channel)
{
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	return dma_ch[channel].regs->irq_status;
}
EXPORT_SYMBOL(get_dma_curr_irqstat);

/*------------------------------------------------------------------------------
 *	Clear the DMA_DONE bit in DMA status. Stop the DMA completion interrupt.
 *-----------------------------------------------------------------------------*/
void clear_dma_irqstat(unsigned int channel)
{
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));
	dma_ch[channel].regs->irq_status |= 3;
}
EXPORT_SYMBOL(clear_dma_irqstat);

/*------------------------------------------------------------------------------
 *	Get current DMA xcount of a specific DMA channel from the system.
 *-----------------------------------------------------------------------------*/
unsigned short get_dma_curr_xcount(unsigned int channel)
{
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	return dma_ch[channel].regs->curr_x_count;
}
EXPORT_SYMBOL(get_dma_curr_xcount);

/*------------------------------------------------------------------------------
 *	Get current DMA ycount of a specific DMA channel from the system.
 *-----------------------------------------------------------------------------*/
unsigned short get_dma_curr_ycount(unsigned int channel)
{
	BUG_ON(!(dma_ch[channel].chan_status != DMA_CHANNEL_FREE
	       && channel < MAX_BLACKFIN_DMA_CHANNEL));

	return dma_ch[channel].regs->curr_y_count;
}
EXPORT_SYMBOL(get_dma_curr_ycount);

void *dma_memcpy(void *dest, const void *src, size_t size)
{
	int direction;	/* 1 - address decrease, 0 - address increase */
	int flag_align;	/* 1 - address aligned,  0 - address unaligned */
	int flag_2D;	/* 1 - 2D DMA needed,	 0 - 1D DMA needed */

	if (size <= 0)
		return NULL;

	if ((unsigned long)src < memory_end)
		blackfin_dcache_flush_range((unsigned int)src,
					    (unsigned int)(src + size));

	bfin_write_MDMA_D0_IRQ_STATUS(DMA_DONE | DMA_ERR);

	if ((unsigned long)src < (unsigned long)dest)
		direction = 1;
	else
		direction = 0;

	if ((((unsigned long)dest % 2) == 0) && (((unsigned long)src % 2) == 0)
	    && ((size % 2) == 0))
		flag_align = 1;
	else
		flag_align = 0;

	if (size > 0x10000)	/* size > 64K */
		flag_2D = 1;
	else
		flag_2D = 0;

	/* Setup destination and source start address */
	if (direction) {
		if (flag_align) {
			bfin_write_MDMA_D0_START_ADDR(dest + size - 2);
			bfin_write_MDMA_S0_START_ADDR(src + size - 2);
		} else {
			bfin_write_MDMA_D0_START_ADDR(dest + size - 1);
			bfin_write_MDMA_S0_START_ADDR(src + size - 1);
		}
	} else {
		bfin_write_MDMA_D0_START_ADDR(dest);
		bfin_write_MDMA_S0_START_ADDR(src);
	}

	/* Setup destination and source xcount */
	if (flag_2D) {
		if (flag_align) {
			bfin_write_MDMA_D0_X_COUNT(1024 / 2);
			bfin_write_MDMA_S0_X_COUNT(1024 / 2);
		} else {
			bfin_write_MDMA_D0_X_COUNT(1024);
			bfin_write_MDMA_S0_X_COUNT(1024);
		}
		bfin_write_MDMA_D0_Y_COUNT(size >> 10);
		bfin_write_MDMA_S0_Y_COUNT(size >> 10);
	} else {
		if (flag_align) {
			bfin_write_MDMA_D0_X_COUNT(size / 2);
			bfin_write_MDMA_S0_X_COUNT(size / 2);
		} else {
			bfin_write_MDMA_D0_X_COUNT(size);
			bfin_write_MDMA_S0_X_COUNT(size);
		}
	}

	/* Setup destination and source xmodify and ymodify */
	if (direction) {
		if (flag_align) {
			bfin_write_MDMA_D0_X_MODIFY(-2);
			bfin_write_MDMA_S0_X_MODIFY(-2);
			if (flag_2D) {
				bfin_write_MDMA_D0_Y_MODIFY(-2);
				bfin_write_MDMA_S0_Y_MODIFY(-2);
			}
		} else {
			bfin_write_MDMA_D0_X_MODIFY(-1);
			bfin_write_MDMA_S0_X_MODIFY(-1);
			if (flag_2D) {
				bfin_write_MDMA_D0_Y_MODIFY(-1);
				bfin_write_MDMA_S0_Y_MODIFY(-1);
			}
		}
	} else {
		if (flag_align) {
			bfin_write_MDMA_D0_X_MODIFY(2);
			bfin_write_MDMA_S0_X_MODIFY(2);
			if (flag_2D) {
				bfin_write_MDMA_D0_Y_MODIFY(2);
				bfin_write_MDMA_S0_Y_MODIFY(2);
			}
		} else {
			bfin_write_MDMA_D0_X_MODIFY(1);
			bfin_write_MDMA_S0_X_MODIFY(1);
			if (flag_2D) {
				bfin_write_MDMA_D0_Y_MODIFY(1);
				bfin_write_MDMA_S0_Y_MODIFY(1);
			}
		}
	}

	/* Enable source DMA */
	if (flag_2D) {
		if (flag_align) {
			bfin_write_MDMA_S0_CONFIG(DMAEN | DMA2D | WDSIZE_16);
			bfin_write_MDMA_D0_CONFIG(WNR | DI_EN | DMAEN | DMA2D | WDSIZE_16);
		} else {
			bfin_write_MDMA_S0_CONFIG(DMAEN | DMA2D);
			bfin_write_MDMA_D0_CONFIG(WNR | DI_EN | DMAEN | DMA2D);
		}
	} else {
		if (flag_align) {
			bfin_write_MDMA_S0_CONFIG(DMAEN | WDSIZE_16);
			bfin_write_MDMA_D0_CONFIG(WNR | DI_EN | DMAEN | WDSIZE_16);
		} else {
			bfin_write_MDMA_S0_CONFIG(DMAEN);
			bfin_write_MDMA_D0_CONFIG(WNR | DI_EN | DMAEN);
		}
	}

	while (!(bfin_read_MDMA_D0_IRQ_STATUS() & DMA_DONE))
		;

	bfin_write_MDMA_D0_IRQ_STATUS(bfin_read_MDMA_D0_IRQ_STATUS() |
				      (DMA_DONE | DMA_ERR));

	bfin_write_MDMA_S0_CONFIG(0);
	bfin_write_MDMA_D0_CONFIG(0);

	if ((unsigned long)dest < memory_end)
		blackfin_dcache_invalidate_range((unsigned int)dest,
						 (unsigned int)(dest + size));

	return dest;
}
EXPORT_SYMBOL(dma_memcpy);

void *safe_dma_memcpy(void *dest, const void *src, size_t size)
{
	int flags = 0;
	void *addr;
	local_irq_save(flags);
	addr = dma_memcpy(dest, src, size);
	local_irq_restore(flags);
	return addr;
}
EXPORT_SYMBOL(safe_dma_memcpy);
