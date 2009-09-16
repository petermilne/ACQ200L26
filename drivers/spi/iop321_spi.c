/* ------------------------------------------------------------------------- */
/* drivers/spi/iop32x_spi.c spi device driver for IOP321                     */
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
/* with acknowledgements to linux/drivers/spi/spi_s3c24xx.c
 *
 * Copyright (c) 2006 Ben Dooks
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/hardware.h>
#include <asm/arch/iop321.h>
#include <asm/arch/iop321_spi.h>

struct iop32x_spi {
	/* bitbang has to be first */
	struct spi_bitbang	 bitbang;
	struct completion	 done;

	void __iomem		*regs;
	int			 irq;
	int			 len;
	int			 count;

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

static inline struct iop32x_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void iop32x_spi_setcs(struct spi_board_info *spi, int cs, int pol)
{
/* default: no cs */
}

static void iop32x_spi_chipsel(struct spi_device *spi, int value)
{
	struct iop32x_spi *hw = to_hw(spi);
	unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;

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

static int iop32x_spi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct iop32x_spi *hw = to_hw(spi);

	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}

static int iop32x_spi_setup(struct spi_device *spi)
{
	int ret;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if ((spi->mode & SPI_LSB_FIRST) != 0)
		return -EINVAL;

	ret = iop32x_spi_setupxfer(spi, NULL);
	if (ret < 0) {
		dev_err(&spi->dev, "setupxfer returned %d\n", ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n",
		__FUNCTION__, spi->mode, spi->bits_per_word,
		spi->max_speed_hz);

	return 0;
}

static void iop32x_tx(struct iop32x_spi *hw)
{

}
static inline unsigned int hw_txbyte(struct iop32x_spi *hw, int count)
{
	return hw->tx ? hw->tx[count] : 0;
}

static int iop32x_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct iop32x_spi *hw = to_hw(spi);

	dev_dbg(&spi->dev, "txrx: tx %p, rx %p, len %d\n",
		t->tx_buf, t->rx_buf, t->len);

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->len = t->len;
	hw->count = 0;

	/* send the first byte[s] */
	iop32x_tx(hw);
	wait_for_completion(&hw->done);

	return hw->count;
}

// IRQ_IOP32X_SSP

static irqreturn_t iop32x_spi_irq(int irq, void *dev)
{
	struct iop32x_spi *hw = dev;
#if 0
	unsigned int spsta = readb(hw->regs + S3C2410_SPSTA);
	unsigned int count = hw->count;

	if (spsta & S3C2410_SPSTA_DCOL) {
		dev_dbg(hw->dev, "data-collision\n");
		complete(&hw->done);
		goto irq_done;
	}

	if (!(spsta & S3C2410_SPSTA_READY)) {
		dev_dbg(hw->dev, "spi not ready for tx?\n");
		complete(&hw->done);
		goto irq_done;
	}

	hw->count++;

	if (hw->rx)
		hw->rx[count] = readb(hw->regs + S3C2410_SPRDAT);

	count++;

	if (count < hw->len)
		writeb(hw_txbyte(hw, count), hw->regs + S3C2410_SPTDAT);
	else
#endif
	if (hw->count >= hw->len){
		complete(&hw->done);
	}

	//irq_done:
	return IRQ_HANDLED;
}

static int iop32x_spi_probe(struct platform_device *pdev)
{
	struct iop32x_spi *hw;
	struct spi_master *master;
	struct spi_board_info *bi;
	struct resource *res;
	int err = 0;
	int i;

	master = spi_alloc_master(&pdev->dev, sizeof(struct iop32x_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct iop32x_spi));

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
	hw->bitbang.setup_transfer = iop32x_spi_setupxfer;
	hw->bitbang.chipselect     = iop32x_spi_chipsel;
	hw->bitbang.txrx_bufs      = iop32x_spi_txrx;
	hw->bitbang.master->setup  = iop32x_spi_setup;

	dev_dbg(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */

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
#if 0
	hw->regs = ioremap(res->start, (res->end - res->start)+1);
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}
#endif
	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	err = request_irq(hw->irq, iop32x_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}

	/* program defaults into the registers */

	/* setup any gpio we can */

#if 0
	if (!hw->pdata->set_cs){
		hw->set_cs = iop32x_spi_setcs;
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

	/* register all the devices associated */

	bi = &hw->pdata->board_info[0];
	for (i = 0; i < hw->pdata->board_size; i++, bi++) {
		dev_info(hw->dev, "registering %s\n", bi->modalias);

		bi->controller_data = hw;
		spi_new_device(master, bi);
	}

	return 0;

 err_register:

 err_no_clk:
	free_irq(hw->irq, hw);

 err_no_irq:


 err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

 err_no_iores:
 err_no_pdata:
	spi_master_put(hw->master);;

 err_nomem:
	return err;
}

static int iop32x_spi_remove(struct platform_device *dev)
{
	struct iop32x_spi *hw = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);

	free_irq(hw->irq, hw);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	spi_master_put(hw->master);
	return 0;
}


#ifdef CONFIG_PM

static int iop32x_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct iop32x_spi *hw = platform_get_drvdata(pdev);

	return 0;
}

static int iop32x_spi_resume(struct platform_device *pdev)
{
	struct iop32x_spi *hw = platform_get_drvdata(pdev);

	return 0;
}

#else
#define iop32x_spi_suspend NULL
#define iop32x_spi_resume  NULL
#endif

static struct platform_driver iop32x_spidrv = {
	.probe		= iop32x_spi_probe,
	.remove		= iop32x_spi_remove,
	.suspend	= iop32x_spi_suspend,
	.resume		= iop32x_spi_resume,
	.driver		= {
		.name	= "iop32x-spi",
		.owner	= THIS_MODULE,
	},
};

static int __init iop32x_spi_init(void)
{
        return platform_driver_register(&iop32x_spidrv);
}

static void __exit iop32x_spi_exit(void)
{
        platform_driver_unregister(&iop32x_spidrv);
}

module_init(iop32x_spi_init);
module_exit(iop32x_spi_exit);

MODULE_DESCRIPTION("IOP32X SPI Driver");
MODULE_AUTHOR("Peter Milne, <peter dot milne@d-tacq.com>");
MODULE_LICENSE("GPL");
