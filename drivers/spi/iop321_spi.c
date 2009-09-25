/* ------------------------------------------------------------------------- */
/* drivers/spi/iop321_spi.c spi device driver for IOP321                     */
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
/* with acknowledgements to linux/drivers/spi/spi_s3c24xx.c
 *
 * Copyright (c) 2006 Ben Dooks
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
*/
#define DEBUG 1

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/hardware.h>
#include <asm/arch/iop321.h>
#include <asm/arch/iop321_spi.h>
#include <asm/arch/iop321-irqs.h>


/* copy bits from pcs-regs.h that are the same ... */

#define SSCR0_DSS		0x0000000f	/* Data Size Select (mask) */
#define SSCR0_DataSize(x)  ((x) - 1)		/* Data Size Select [4..16] */
#define SSCR0_FRF		0x00000030	/* FRame Format (mask) */
#define SSCR0_Motorola		(0x0 << 4)	/* Motorola's Serial Peripheral Interface (SPI) */
#define SSCR0_ECS	(1 << 6)	/* External clock select */
#define SSCR0_SSE	(1 << 7)	/* Synchronous Serial Port Enable */
#define SSCR0_SCR	(0x0000ff00)	/* Serial Clock Rate (mask) */
#define SSCR0_SerClkDiv(x) ((((x) - 2)/2) << 8) /* Divisor [2..512] */

#define SSCR1_RIE	(1 << 0)	/* Receive FIFO Interrupt Enable */
#define SSCR1_TIE	(1 << 1)	/* Transmit FIFO Interrupt Enable */
#define SSCR1_LBM	(1 << 2)	/* Loop-Back Mode */
#define SSCR1_SPO	(1 << 3)	/* Motorola SPI SSPSCLK polarity setting */
#define SSCR1_SPH	(1 << 4)	/* Motorola SPI SSPSCLK phase setting */
#define SSCR1_MWDS	(1 << 5)	/* Microwire Transmit Data Size */
#define SSCR1_TFT	(0x000003c0)	/* Transmit FIFO Threshold (mask) */
#define SSCR1_TxTresh(x) (((x) - 1) << 6) /* level [1..16] */
#define SSCR1_RFT	(0x00003c00)	/* Receive FIFO Threshold (mask) */
#define SSCR1_RxTresh(x) (((x) - 1) << 10) /* level [1..16] */

#define SSSR_TNF	(1 << 2)	/* Transmit FIFO Not Full */
#define SSSR_RNE	(1 << 3)	/* Receive FIFO Not Empty */
#define SSSR_BSY	(1 << 4)	/* SSP Busy */
#define SSSR_TFS	(1 << 5)	/* Transmit FIFO Service Request */
#define SSSR_RFS	(1 << 6)	/* Receive FIFO Service Request */
#define SSSR_ROR	(1 << 7)	/* Receive FIFO Overrun */

#define SSSR_TFL	0x00000f00
#define SSSR_RFL	0x0000f000

#define SSSR_RxCount(sssr) ((sssr)&SSSR_RNE? ((sssr)&SSSR_RFL) >> 12: 0)

#ifdef DEBUG
int iop321_debug;
module_param(iop321_debug, int, 0664);
#define DEV_DBG	if (iop321_debug) dev_dbg
#else
#define DEV_DBG	
#endif


/* knobs to tweak bus params - should be in the descriptor, but this is easier */
int SP0;
module_param(SP0, int, 0644);
int SPH;
module_param(SPH, int, 0644);

int do_every_time=1;	/* init every setup - shouldn't be necessary */
module_param(do_every_time, int, 0644);

struct iop321_spi {
	/* bitbang has to be first */
	struct spi_bitbang	 bitbang;
	struct completion	 done;

	void __iomem		*regs;
	int			 irq;
	int			 len;
	int			 tx_count;
	int			 rx_count;

	void			(*set_cs)(struct iop321_spi_info *spi,
					  int cs, int pol);

	/* data buffers */
	const unsigned char	*tx;
	unsigned char		*rx;

	struct clk		*clk;
	struct resource		*ioarea;
	struct spi_master	*master;
	struct spi_device	*curdev;
	struct device		*dev;
	struct iop321_spi_info  *pdata;
};

static inline struct iop321_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void iop321_ssp_init(void)
{
	*IOP3XX_SSCR0 = 
		SSCR0_DataSize(8)|
		SSCR0_Motorola|
		SSCR0_SerClkDiv(2);

	*IOP3XX_SSCR1 = 
		(SP0 ? SSCR1_SPO: 0)|(SPH? SSCR1_SPH: 0)|
		SSCR1_SPO|
		SSCR1_RIE|
		SSCR1_TxTresh(1)|
		SSCR1_RxTresh(1);  

	*IOP3XX_SSCR0 |= SSCR0_SSE;	     
}
static void iop321_spi_setcs(struct spi_board_info *spi, int cs, int pol)
{
/* default: no cs */
}

static void iop321_spi_chipsel(struct spi_device *spi, int value)
{
	struct iop321_spi *hw = to_hw(spi);
	unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;

	return;			/* @todo ... fails after this .. */
	switch (value) {
	default:
	case BITBANG_CS_INACTIVE:
		hw->set_cs(hw->pdata, spi->chip_select, cspol^1);
		break;

	case BITBANG_CS_ACTIVE:
		hw->set_cs(hw->pdata, spi->chip_select, cspol);
		break;
	}
}

static int iop321_spi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct iop321_spi *hw = to_hw(spi);
	DEV_DBG(&spi->dev, "setup\n");
	if (do_every_time){
		iop321_ssp_init();
	}
	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		;
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}


static int iop321_spi_setup(struct spi_device *spi)
{
	int ret;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if ((spi->mode & SPI_LSB_FIRST) != 0)
		return -EINVAL;

	ret = iop321_spi_setupxfer(spi, NULL);
	if (ret < 0) {
		dev_err(&spi->dev, "setupxfer returned %d\n", ret);
		return ret;
	}

	DEV_DBG(&spi->dev, "%s: mode %d, %u bpw, %d hz\n",
		__FUNCTION__, spi->mode, spi->bits_per_word,
		spi->max_speed_hz);

	return 0;
}

static inline unsigned int hw_txbyte(struct iop321_spi *hw, int count)
{
	unsigned rc = hw->tx ? hw->tx[count] : 0;
	DEV_DBG(hw->dev, "TX:%02x\n", rc);
	return rc;
}

static void iop321_tx(struct iop321_spi *hw)
{	
	while(hw->tx_count < hw->len && (*IOP3XX_SSSR & SSSR_TNF) != 0){
		*IOP3XX_SSDR = hw_txbyte(hw, hw->tx_count++);
	}	
}

static void iop321_rx(struct iop321_spi *hw, int nread)
{
	while(nread-- > 0){
		u32 rx = *IOP3XX_SSDR;
		if (hw->rx && hw->rx_count < hw->len){
			hw->rx[hw->rx_count++] = rx;	
			DEV_DBG(hw->dev, "RX:%02x\n", rx);
		}else{
			DEV_DBG(hw->dev, "RX_%02x\n", rx);
		}
	}
}

static inline int tx_complete(struct iop321_spi *hw)
{
	return hw->tx_count >= hw->len;
}
static inline int rx_complete(struct iop321_spi *hw)
{
	return hw->rx == 0 || hw->rx_count >= hw->len;
}
static inline int txrx_complete(struct iop321_spi *hw)
{
	return tx_complete(hw) && rx_complete(hw);
}

static int irq_report_ok;

static int iop321_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct iop321_spi *hw = to_hw(spi);

	DEV_DBG(hw->dev, "txrx: tx %p, rx %p, len %d\n",
		t->tx_buf, t->rx_buf, t->len);

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->len = t->len;
	hw->tx_count = hw->rx_count = 0;

	/* send the first byte[s] */

	irq_report_ok = 1;
	*IOP3XX_SSCR1 |= SSCR1_TIE|SSCR1_RIE;
	iop321_tx(hw);

	while (!txrx_complete(hw)){
		wait_for_completion(&hw->done);
	}

	return hw->tx_count;
}



static irqreturn_t iop321_spi_irq(int irq, void *dev)
{
	struct iop321_spi *hw = dev;
	u32 sssr = *IOP3XX_SSSR;
	static u32 old_sssr;

	if (sssr != old_sssr || irq_report_ok){
		DEV_DBG(hw->dev, "IRQ:SSSR %08x\n", sssr);
		old_sssr = sssr;
		irq_report_ok = 0;
	}
	iop321_tx(hw);
	if (tx_complete(hw)){	
		DEV_DBG(hw->dev, "IRQ: TX complete\n");	
		*IOP3XX_SSCR1 &= ~SSCR1_TIE;
	}
	if (sssr&SSSR_RFS){
		iop321_rx(hw, SSSR_RxCount(sssr));
	}

	if (rx_complete(hw)){
		DEV_DBG(hw->dev, "RX complete\n");
		*IOP3XX_SSCR1 &= ~SSCR1_RIE;
	}

	if (txrx_complete(hw)){	
		complete(&hw->done);
	}

	return IRQ_HANDLED;
}

static int register_associated_devices(
	struct spi_master *master, struct iop321_spi *hw)
{
	struct spi_board_info *bi = &hw->pdata->board_info[0];
	struct spi_device *newdev;
	int i;

	if (!spi_master_get(master)){
		dev_err(hw->dev,
			 "spi_new_device will fail on spi_master_get\n");
	}


	for (i = 0; i < hw->pdata->board_size; i++, bi++) {
		printk(KERN_INFO "register_associated_devices() %d %s\n", i, bi->modalias);
		dev_info(hw->dev, "registering %s\n", bi->modalias);

		bi->controller_data = hw;
		newdev = spi_new_device(master, bi);
		if (newdev == NULL){
			dev_err(hw->dev, "spi_new_device returned NULL\n");
		}
	}
	return 0;
}

static int iop321_spi_probe(struct platform_device *pdev)
{
	struct iop321_spi *hw;
	struct spi_master *master;	
//	struct resource *res;
	int err = 0;

	printk(KERN_INFO "iop321_spi_probe()\n");
	DEV_DBG(&pdev->dev, "iop321_spi_probe()\n");
	master = spi_alloc_master(&pdev->dev, sizeof(struct iop321_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct iop321_spi));

	hw->master = spi_master_get(master);
	hw->pdata = pdev->dev.platform_data;
	hw->dev = &pdev->dev;

	if (hw->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_no_pdata;
	}

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);

	/* setup the state for the bitbang driver */

	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = iop321_spi_setupxfer;
	hw->bitbang.chipselect     = iop321_spi_chipsel;
	hw->bitbang.txrx_bufs      = iop321_spi_txrx;
	hw->bitbang.master->setup  = iop321_spi_setup;

	DEV_DBG(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */
#if 0
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	hw->ioarea = request_mem_region(res->start, (res->end - res->start)+1,
					pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_no_iores;
	}
	hw->regs = ioremap(res->start, (res->end - res->start)+1);
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}
	hw->irq = platform_get_irq(pdev, 0);
#endif
	hw->irq = IRQ_IOP321_SSP;	/* @todo get from resource? why? */
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	err = request_irq(hw->irq, iop321_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}

	iop321_ssp_init();

#if 0
	if (!hw->pdata->set_cs){
		hw->set_cs = iop321_spi_setcs;
	}else{
		hw->set_cs = hw->pdata->set_cs;
	}
#endif
	/* register our spi controller */

	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}
	
	return register_associated_devices(master, hw);

 err_register:

 err_no_clk:
	free_irq(hw->irq, hw);

 err_no_irq:


 err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

 err_no_iores:
 err_no_pdata:
	spi_master_put(hw->master);

 err_nomem:
	return err;
}

static int iop321_spi_remove(struct platform_device *dev)
{
	struct iop321_spi *hw = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);

	free_irq(hw->irq, hw);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	spi_master_put(hw->master);
	return 0;
}


#ifdef CONFIG_PM

static int iop321_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct iop321_spi *hw = platform_get_drvdata(pdev);

	return 0;
}

static int iop321_spi_resume(struct platform_device *pdev)
{
	struct iop321_spi *hw = platform_get_drvdata(pdev);

	return 0;
}

#else
#define iop321_spi_suspend NULL
#define iop321_spi_resume  NULL
#endif

static struct platform_driver iop321_spidrv = {
	.probe		= iop321_spi_probe,
	.remove		= iop321_spi_remove,
	.suspend	= iop321_spi_suspend,
	.resume		= iop321_spi_resume,
	.driver		= {
		.name	= "iop321-spi",
		.owner	= THIS_MODULE,
	},
};

static int __init iop321_spi_init(void)
{
	printk(KERN_INFO "iop321_spi_init\n");
        return platform_driver_register(&iop321_spidrv);
}

static void __exit iop321_spi_exit(void)
{
        platform_driver_unregister(&iop321_spidrv);
}

module_init(iop321_spi_init);
module_exit(iop321_spi_exit);

MODULE_DESCRIPTION("IOP321 SPI Driver");
MODULE_AUTHOR("Peter Milne, <peter dot milne@d-tacq.com>");
MODULE_LICENSE("GPL");
