/*
 * linux/drivers/spi/spi-slave-ambarella.c
 * NOTICE:
 * 1. !!!!!!!! THIS SPISLAVE CAN only works in SPI_MODE_3 !!!!!!!
 * 2. If dma is applied, tx/rx must be BLOCK*N (BLOCK=8)
 * History:
 *	2012/02/21 - [Zhenwu Xue]  created file
 *	2019/10/25 - [xuliang Zhang] modify and fixup bugs
 * Copyright (C) 2004-2012, Ambarella, Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/poll.h>
#include <linux/pinctrl/consumer.h>

#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/kfifo.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/cdev.h>
#include <linux/atomic.h>
#include <plat/spi.h>
#include <plat/dma.h>

/* inc this number for each version */
#define SLAVE_SPI_VERSION_NUM (0x0001)

#define SPI_RXFIS_MASK 		0x00000010
#define SPI_RXOIS_MASK 		0x00000008
#define SPI_RXUIS_MASK 		0x00000004
#define SPI_TXOIS_MASK 		0x00000002
#define SPI_TXEIS_MASK 		0x00000001

#define SPI_SR_BUSY_MSK (0x1)
#define SPI_SR_TXNF_MSK (0x2)
#define SPI_SR_TXEPT_MSK (0x4)
#define SPI_SR_TXE_MSK	(0x20)

#define SPI_FIFO_DEPTH		(SPI_DATA_FIFO_SIZE_32)
#define SLAVE_SPI_BUF_SIZE	PAGE_SIZE

#define SPI_DMA_ALIGN_8_MASK	(0xFFFFFFF8)	/* down-align-to 8 */
#define SPI_DMA_ALIGN_16_MASK	(0xFFFFFFF0)	/* down-align-to 16 */

#define FIFO_DEPTH_MULTI	(4)

#define SPI_DMA_TX	(0)
#define SPI_DMA_RX	(1)

#define SPI_TMOD_TXONLY		(1 << 8)
#define SPI_TMOD_RXONLY		(2 << 8)
#define SPI_TMODE_TXRX		(0 << 8)
#define SPI_TMODE_MSK		(3 << 8)

#define SPI_DMA_RX_EN		(1)
#define SPI_DMA_TX_EN		(2)
#define SPI_DMA_EN_NONE		(0)

#define SPI_FAULTINJECT_MASK   (2)

#define CAN_WAKEUP_BLOCK_R (0x1)
#define CAN_WAKEUP_BLOCK_W (0x2)
#define CAN_WAKEUP_POLL (0x4)

/* 1:stream dma memory;    0:corherent dma memory */
#define USING_STREAM_DMA		(1)

/* export to user-space used by ioctl, not support now */
#define SPI_SLAVE_MAGIC   (250)
struct slavespi_ioctl_args {
};

/* slavespi controller */
struct ambarella_slavespi {
	int irq;
	void __iomem	*regbase;

	dev_t 	devno;
	struct cdev	cdevx;
	struct device	*device;
	const char *dname;
	struct class *dclass;

	u32	bpw;
	u32	spi_mode;
	u32	fifo_depth;

	u32	buf_size;
	void	*w_buf;
	void	*r_buf;

	u32	fault_inject;	/* introduce @CV2FS */

	u32	dma_used;
	u32	dma_size_mask;
	u32	phys;
	struct	device *pltdev;
	struct dma_chan	*tx_dma_chan;
	struct dma_chan	*rx_dma_chan;
	u8	*tx_dma_buf;
	u8	*rx_dma_buf;
	dma_addr_t	tx_dma_phys;
	dma_addr_t	rx_dma_phys;

	dma_cookie_t	tx_cookie;
	dma_cookie_t	rx_cookie;

	struct dma_async_tx_descriptor	*tx_dma_desc;
	struct dma_async_tx_descriptor	*rx_dma_desc;
	u32 tx_dma_inprocess;
	u32 rx_dma_inprocess;
	u32 rx_dma_size;
	u32 tx_dma_size;

	u32 stat_rx_cnt;	/* packets */
	u32 stat_rx_drop;	/* bytes */
	u32 stat_rxov_cnt;	/* bad packet */
	u32 stat_txe_cnt;	/* shift packet */
	struct proc_dir_entry *stat_proc;

	struct kfifo	w_fifo;
	struct kfifo	r_fifo;

	u32 want_cnt;
	spinlock_t w_buf_lock;
	spinlock_t r_buf_lock;
	spinlock_t wakeup_lock;

	atomic_t open_once;
	u32 wakeup_mask;	/* bit0: read, bit1: write , bit2 poll */
	wait_queue_head_t wq_rhead;
	wait_queue_head_t wq_whead;
	wait_queue_head_t wq_poll;
	struct tasklet_struct wakeup_helper;
};

void ambarella_spislave_reset(struct ambarella_slavespi *priv)
{
	writel(0, priv->regbase + SPI_SSIENR_OFFSET);
	udelay(1);
	writel(1, priv->regbase + SPI_SSIENR_OFFSET);
}

static void ambarella_slave_fill_txfifo(struct ambarella_slavespi *priv)
{
	u32 i = 0;
	u32 tx_len = 0;
	u32 tx_wfifo_len = 0;
	u32 tx_hwfifo_free = 0;

	tx_wfifo_len = kfifo_len(&priv->w_fifo);
	tx_hwfifo_free = priv->fifo_depth - readl_relaxed(priv->regbase + SPI_TXFLR_OFFSET);
	if (priv->bpw > 8) {
		tx_hwfifo_free = tx_hwfifo_free * 2;
	}

	tx_len = kfifo_out(&priv->w_fifo, priv->w_buf, min_t(u32, tx_wfifo_len, tx_hwfifo_free));
	if (priv->bpw > 8) {
		tx_len = tx_len >> 1;
	}

	for (i = 0; i < tx_len; i++) {
		if (priv->bpw > 8) {
			writel(((u16 *)priv->w_buf)[i], priv->regbase + SPI_DR_OFFSET);
		} else {
			writel(((u8 *)priv->w_buf)[i], priv->regbase + SPI_DR_OFFSET);
		}
	}
}

static int
ambarella_spislave_dma_allocate(struct ambarella_slavespi *priv,
	bool dma_to_memory)
{
	int ret = 0;
	u8 *dma_buf;
	dma_addr_t dma_phys;
	struct dma_chan *dma_chan;

	dma_chan = dma_request_slave_channel(priv->pltdev,
					dma_to_memory ? "rx" : "tx");
	if (IS_ERR(dma_chan)) {
		ret = PTR_ERR(dma_chan);
		if (ret != -EPROBE_DEFER)
			printk(KERN_ALERT "slavespi: Dma channel is not available: %d \n", ret);
		return ret;
	}

#if (USING_STREAM_DMA > 0)
	dma_buf = kzalloc(priv->buf_size, GFP_KERNEL);
	if (!dma_buf) {
		ret = -ENOMEM;
		printk(KERN_ALERT "slavespi: alloc memory failed for %s \n",
					dma_to_memory ? "rx" : "tx");
		dma_release_channel(dma_chan);
		dma_chan = NULL;
		return ret;
	}

	if (dma_to_memory) {
		dma_phys = dma_map_single(priv->pltdev, dma_buf, priv->buf_size, DMA_FROM_DEVICE);
	} else {
		dma_phys = dma_map_single(priv->pltdev, dma_buf, priv->buf_size, DMA_TO_DEVICE);
	}

	if (dma_mapping_error(priv->pltdev, dma_phys)) {
		printk("slavespi: dma streaming mapping error for %s \n",
					dma_to_memory ? "rx" : "tx");
		kfree(dma_buf);
		dma_buf = NULL;
	}
#else
	dma_buf = dma_alloc_coherent(priv->pltdev, priv->buf_size,
				&dma_phys, GFP_KERNEL);
#endif

	if (!dma_buf) {
		printk(KERN_ALERT "slavespi: Not able to allocate the dma buffer\n");
		dma_release_channel(dma_chan);
		return -ENOMEM;
	}

	if (dma_to_memory) {
		priv->rx_dma_chan = dma_chan;
		priv->rx_dma_buf = dma_buf;
		priv->rx_dma_phys = dma_phys;
	} else {
		priv->tx_dma_chan = dma_chan;
		priv->tx_dma_buf = dma_buf;
		priv->tx_dma_phys = dma_phys;
	}

	return ret;
}

static void
ambarella_spi_slave_dma_free(struct ambarella_slavespi *priv,
	bool dma_to_memory)
{
	u8 *dma_buf;
	dma_addr_t dma_phys;
	struct dma_chan *dma_chan;

	if (dma_to_memory) {
		dma_buf = priv->rx_dma_buf;
		dma_chan = priv->rx_dma_chan;
		dma_phys = priv->rx_dma_phys;
		priv->rx_dma_chan = NULL;
		priv->rx_dma_buf = NULL;
	} else {
		dma_buf = priv->tx_dma_buf;
		dma_chan = priv->tx_dma_chan;
		dma_phys = priv->tx_dma_phys;
		priv->tx_dma_buf = NULL;
		priv->tx_dma_chan = NULL;
	}

	if (!dma_chan) {
		return;
	}

#if (USING_STREAM_DMA > 0)
	if (dma_to_memory) {
		dma_unmap_single(priv->pltdev, dma_phys, priv->buf_size, DMA_FROM_DEVICE);
	} else {
		dma_unmap_single(priv->pltdev, dma_phys, priv->buf_size, DMA_TO_DEVICE);
	}
	kfree(dma_buf);
	dma_buf = NULL;
#else
	dma_free_coherent(priv->pltdev, priv->buf_size, dma_buf, dma_phys);
#endif
	dma_release_channel(dma_chan);
}

static void
ambarella_slavespi_start_tx_dma(struct ambarella_slavespi *priv);

static int
ambarella_slavespi_start_rx_dma(struct ambarella_slavespi *priv);

static void slavespi_drain_rxfifo(struct ambarella_slavespi *priv);

static void ambarella_rx_dma_callback(void *dma_param)
{
	u32 risr = 0;
	u32 copied = 0;
	u32 avail_size = 0;

	struct dma_tx_state state;
	enum dma_status status;
	struct ambarella_slavespi *priv = (struct ambarella_slavespi *)dma_param;

	risr = readl(priv->regbase + SPI_RISR_OFFSET);
	if (risr & SPI_RXOIS_MASK) {
		priv->stat_rxov_cnt++;
		readl(priv->regbase + SPI_RXOICR_OFFSET);
	}

	status = dmaengine_tx_status(priv->rx_dma_chan, priv->rx_cookie, &state);
	if (status == DMA_COMPLETE) {
		priv->stat_rx_cnt++;
		avail_size = priv->rx_dma_size - state.residue;

#if (USING_STREAM_DMA > 0)
		/* let cpu own this buf to read data */
		dma_sync_single_for_cpu(priv->pltdev, priv->rx_dma_phys,
					priv->rx_dma_size, DMA_FROM_DEVICE);
#endif
		copied = kfifo_in(&priv->r_fifo, priv->rx_dma_buf, avail_size);
		ambarella_slavespi_start_rx_dma(priv);		/* re-start rx-dma again */
	}

	wake_up_interruptible(&priv->wq_poll);

	if (avail_size > copied) {
		priv->stat_rx_drop += (avail_size - copied);
	}
}

static int ambarella_slavespi_start_rx_dma(struct ambarella_slavespi *priv)
{
	struct dma_slave_config rx_cfg;
	memset(&rx_cfg, 0, sizeof(rx_cfg));

	priv->rx_dma_size = priv->fifo_depth;

	rx_cfg.src_addr	 = priv->phys + SPI_DR_OFFSET;

	/* Attention: src_maxburst should not big
	 * or get duplicated data for the last valid byte;
	 * ref data-sheet for more info
	*/
	if (priv->bpw <= 8) {
		rx_cfg.src_addr_width	= DMA_SLAVE_BUSWIDTH_1_BYTE;
		rx_cfg.src_maxburst	= 8;
	} else {
		rx_cfg.src_addr_width	= DMA_SLAVE_BUSWIDTH_2_BYTES;
		rx_cfg.src_maxburst	= 8;
	}
	rx_cfg.direction = DMA_DEV_TO_MEM;
	BUG_ON(dmaengine_slave_config(priv->rx_dma_chan, &rx_cfg) < 0);

	priv->rx_dma_desc = dmaengine_prep_slave_single(priv->rx_dma_chan,
		priv->rx_dma_phys, priv->rx_dma_size, DMA_DEV_TO_MEM,  DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!priv->rx_dma_desc) {
		printk(KERN_DEBUG "slavespi: Not able to get desc for Rx dma \n");
		return -EIO;
	}

	priv->rx_dma_desc->callback = ambarella_rx_dma_callback;
	priv->rx_dma_desc->callback_param = priv;

#if (USING_STREAM_DMA > 0)
	/* let device own this buf to write received data from outside */
	dma_sync_single_for_device(priv->pltdev, priv->rx_dma_phys,
		priv->rx_dma_size, DMA_FROM_DEVICE);
#endif

	priv->rx_cookie = dmaengine_submit(priv->rx_dma_desc);
	dma_async_issue_pending(priv->rx_dma_chan);

	return 0;
}

static void ambarella_tx_dma_callback(void *dma_param)
{
	u32 sr = 0;
	u32 copied = 0;
	u32 w_fifo_size = 0;
	u32 w_fifo_len = 0;

	struct dma_tx_state state;
	enum dma_status status;
	struct ambarella_slavespi *priv = NULL;

	priv = (struct ambarella_slavespi *)dma_param;

	sr = readl(priv->regbase + SPI_SR_OFFSET);
	if (sr & SPI_SR_TXE_MSK) {
		priv->stat_txe_cnt++;
	}

	status = dmaengine_tx_status(priv->tx_dma_chan, priv->tx_cookie, &state);
	if (status == DMA_COMPLETE) {
		w_fifo_len = kfifo_len(&priv->w_fifo);
		w_fifo_len &= priv->dma_size_mask;
		if (w_fifo_len > 0) {	/* block data using dma to transfer */
			w_fifo_size = min_t(u32, w_fifo_len, priv->buf_size);

#if (USING_STREAM_DMA > 0)
			/* let cpu own this buf to write data into dma buf */
			dma_sync_single_for_cpu(priv->pltdev, priv->tx_dma_phys,
							priv->buf_size, DMA_TO_DEVICE);
#endif
			copied = kfifo_out(&priv->w_fifo, priv->tx_dma_buf, w_fifo_size);
			priv->tx_dma_size = copied;
			ambarella_slavespi_start_tx_dma(priv);
		} else {
			priv->tx_dma_inprocess = 0;	/* can over-write dma buf */
		}
	}

	wake_up_interruptible(&priv->wq_poll);
}

void ambarella_slavespi_start_tx_dma(struct ambarella_slavespi *priv)
{
	struct dma_slave_config tx_cfg;

	/* TX DMA : mem->dev.txfifo */
	tx_cfg.dst_addr	 = priv->phys + SPI_DR_OFFSET;
	if (priv->bpw <= 8) {
		tx_cfg.dst_addr_width	= DMA_SLAVE_BUSWIDTH_1_BYTE;
		tx_cfg.dst_maxburst	= 8;
	} else {
		tx_cfg.dst_addr_width	= DMA_SLAVE_BUSWIDTH_2_BYTES;
		tx_cfg.dst_maxburst	= 8;
	}

	tx_cfg.direction = DMA_MEM_TO_DEV;

	BUG_ON(dmaengine_slave_config(priv->tx_dma_chan, &tx_cfg) < 0);

#if (USING_STREAM_DMA > 0)
	/* now let device own this buf to tranfer data to outside */
	dma_sync_single_for_device(priv->pltdev, priv->tx_dma_phys,
					priv->tx_dma_size, DMA_TO_DEVICE);
#endif

	priv->tx_dma_desc = dmaengine_prep_slave_single(priv->tx_dma_chan,
		priv->tx_dma_phys, priv->tx_dma_size, DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);

	BUG_ON (!priv->tx_dma_desc);

	priv->tx_dma_desc->callback	= ambarella_tx_dma_callback;
	priv->tx_dma_desc->callback_param = priv;
	priv->tx_cookie = dmaengine_submit(priv->tx_dma_desc);
	dma_async_issue_pending(priv->tx_dma_chan);
}

static void wakeup_tasklet(unsigned long data)
{
	u32 mask = 0;
	unsigned long flags;
	struct ambarella_slavespi *priv	= (struct ambarella_slavespi *)data;

	spin_lock_irqsave(&priv->wakeup_lock, flags);
	mask = priv->wakeup_mask;
	priv->wakeup_mask = 0;
	spin_unlock_irqrestore(&priv->wakeup_lock, flags);

	if (mask & CAN_WAKEUP_POLL) {
		wake_up_interruptible(&priv->wq_poll);
	}

	if (mask & CAN_WAKEUP_BLOCK_W) {
		wake_up_interruptible(&priv->wq_whead);
	}

	if (mask & CAN_WAKEUP_BLOCK_R) {
		wake_up_interruptible(&priv->wq_rhead);
	}
}


/* drain any data in hardware rxfifo , used with spinlock */
void slavespi_drain_rxfifo(struct ambarella_slavespi *priv)
{
	u32 i = 0;
	u32 len = 0;
	u32 rxflr = 0;

	rxflr = readl(priv->regbase + SPI_RXFLR_OFFSET);
	if (0 == rxflr) {	/* data-fifo-entry = 0, no data avail */
		return ;
	}

	for (i = 0; i < rxflr; i++) {
		if (priv->bpw > 8) {
			((u16 *)priv->r_buf)[i] = readl(priv->regbase + SPI_DR_OFFSET);
		} else {
			((u8 *)priv->r_buf)[i] = readl(priv->regbase + SPI_DR_OFFSET);
		}
	}
	if (priv->bpw > 8) {
		rxflr <<= 1;
	}

	len = kfifo_in(&priv->r_fifo, priv->r_buf, rxflr);
	if (rxflr > len) {
		priv->stat_rx_drop += (rxflr - len);
	}

	return ;
}

static irqreturn_t ambarella_slavespi_isr(int irq, void *dev_data)
{
	u32 isr = 0;
	u32 imr = 0;
	u32 risr = 0;

	struct ambarella_slavespi *priv	= dev_data;

	isr = readl(priv->regbase + SPI_ISR_OFFSET);
	risr = readl(priv->regbase + SPI_RISR_OFFSET);

	if ((risr & SPI_RXOIS_MASK) || (risr & SPI_TXOIS_MASK)) {
		if (risr & SPI_RXOIS_MASK) {
			readl(priv->regbase + SPI_RXOICR_OFFSET);
			printk(KERN_ALERT "slavespi: speed too fast, rx-overflow \n");
		}
		if (risr & SPI_TXOIS_MASK) {
			readl(priv->regbase + SPI_TXOICR_OFFSET);
			printk(KERN_ALERT "slavespi: speed too fast, tx-overflow \n");
		}

		ambarella_spislave_reset(priv);
	}

	if (isr & SPI_TXEIS_MASK) {
		spin_lock(&priv->w_buf_lock);
		if (kfifo_is_empty(&priv->w_fifo)) {
			/* w_fifo is empty then disable txfifo empty interrupt */
			imr = readl(priv->regbase + SPI_IMR_OFFSET);
			imr &= (~SPI_TXEIS_MASK);
			writel(imr, priv->regbase + SPI_IMR_OFFSET);
			priv->wakeup_mask |= CAN_WAKEUP_BLOCK_W | CAN_WAKEUP_POLL;
		} else {
			ambarella_slave_fill_txfifo(priv);
			priv->wakeup_mask |= CAN_WAKEUP_POLL;
			if (kfifo_len(&priv->w_fifo) < (FIFO_DEPTH_MULTI * priv->fifo_depth)) {
				priv->wakeup_mask |= CAN_WAKEUP_BLOCK_W;
			}

			imr = readl(priv->regbase + SPI_IMR_OFFSET);
			writel(imr | SPI_TXEIS_MASK, priv->regbase + SPI_IMR_OFFSET);
		}
		spin_unlock(&priv->w_buf_lock);
	}

	if ((!priv->dma_used) && (isr & SPI_RXFIS_MASK)) {
		spin_lock(&priv->r_buf_lock);
		slavespi_drain_rxfifo(priv);
		if (!kfifo_is_empty(&priv->r_fifo)) {
			priv->wakeup_mask |= CAN_WAKEUP_POLL;		/* wakeup poll for read */
		}
		if (kfifo_len(&priv->r_fifo) >= priv->want_cnt) {
			priv->wakeup_mask |= CAN_WAKEUP_BLOCK_R;	/* wakeup timeout block read */
		}
		spin_unlock(&priv->r_buf_lock);
	}

	writel(0, priv->regbase + SPI_ISR_OFFSET);
	writel(0, priv->regbase + SPI_RISR_OFFSET);

	readl(priv->regbase + SPI_RXOICR_OFFSET);
	readl(priv->regbase + SPI_TXOICR_OFFSET);
	readl(priv->regbase + SPI_ICR_OFFSET);		/* double sure to clear */

	tasklet_schedule(&priv->wakeup_helper);
	return IRQ_HANDLED;
}

static int ambarella_slavespi_sfifo_init(struct ambarella_slavespi *priv)
{
	int err = 0;

	priv->w_buf = kzalloc(priv->buf_size, GFP_KERNEL);
	if (!priv->w_buf) {
		printk(KERN_ALERT "slavespi: Not able to allocate the write buffer\n");
		goto w_buf_err;
	}

	if (!kfifo_initialized(&priv->w_fifo)) { /* wfifo = 256PAGE */
		err = kfifo_alloc(&priv->w_fifo, priv->buf_size << 8, GFP_KERNEL);
		if (err) {
			printk(KERN_ALERT "slavespi: Alloc w_kfifo failed \n");
			goto w_fifo_err;
		}
	}

	priv->r_buf = kzalloc(priv->buf_size, GFP_KERNEL);
	if (!priv->r_buf) {
		printk(KERN_ALERT "slavespi: Not able to allocate the read buffer\n");
		goto r_buf_err;
	}

	if (!kfifo_initialized(&priv->r_fifo)) { /* rfifo = 256PAGE */
		err = kfifo_alloc(&priv->r_fifo, priv->buf_size << 8, GFP_KERNEL);
		if (err) {
			printk(KERN_ALERT "slavespi: Alloc r_kfifo failed \n");
			goto r_fifo_err;
		}
	}

	kfifo_reset(&priv->w_fifo);
	kfifo_reset(&priv->r_fifo);

	return 0;


r_fifo_err:
	kfree(priv->r_buf);
	priv->r_buf = NULL;

r_buf_err:
	kfifo_free(&priv->w_fifo);

w_fifo_err:
	kfree(priv->w_buf);
	priv->w_buf = NULL;

w_buf_err:
	return -ENOMEM;
}

static void ambarella_slavespi_sfifo_deinit(struct ambarella_slavespi *priv)
{
	if (priv->r_buf) {
		kfree(priv->r_buf);
		priv->r_buf = NULL;
	}

	if (kfifo_initialized(&priv->r_fifo)) {
		kfifo_free(&priv->r_fifo);
	}

	if (priv->w_buf) {
		kfree(priv->w_buf);
		priv->w_buf = NULL;
	}

	if (kfifo_initialized(&priv->w_fifo)) {
		kfifo_free(&priv->w_fifo);
	}
}

static void ambarella_slavespi_hardware_init(struct ambarella_slavespi *priv)
{
	int err = 0;
	u32 inject = 0;
	struct clk *clk = NULL;
	struct pinctrl *pins = NULL;
	spi_ctrl0_reg_t ctrl0_reg;

	clk = clk_get(NULL, "gclk_ssi2");
	if (IS_ERR_OR_NULL(clk)) {
		printk(KERN_ALERT "spislave: no slave spi clock \n");
	} else {
		clk_set_rate(clk, 192000000);
	}

	writel(0, priv->regbase + SPI_SSIENR_OFFSET);

	if (priv->fault_inject) {	/* disable faultinject during init */
		writel(SPI_FAULTINJECT_MASK, priv->regbase + SPI_FAULTINJECT_OFFSET);
	}

	ctrl0_reg.w = 0;
	ctrl0_reg.s.dfs = (priv->bpw - 1);
	ctrl0_reg.s.frf = 0;

	if (priv->spi_mode & SPI_CPHA) {
		ctrl0_reg.s.scph = 1;
	} else {
		ctrl0_reg.s.scph = 0;
	}
	if (priv->spi_mode & SPI_CPOL) {
		ctrl0_reg.s.scpol = 1;
	} else {
		ctrl0_reg.s.scpol = 0;
	}

	ctrl0_reg.s.tmod = SPI_WRITE_READ;
	if (priv->spi_mode & SPI_LSB_FIRST) {
		ctrl0_reg.s.tx_lsb = 1;
		ctrl0_reg.s.rx_lsb = 1;
	} else {
		ctrl0_reg.s.tx_lsb = 0;
		ctrl0_reg.s.rx_lsb = 0;
	}

	ctrl0_reg.s.byte_ws = 1;		/* byte-wise fifo configure */
	if (priv->bpw > 8) {
		ctrl0_reg.s.byte_ws = 0;	/* bit-per-word >8 bits, no byte_wise */
	}

	if (0 == priv->dma_used) {		/* only dma-mode support byte-wise, irq can not support byte-wise */
		ctrl0_reg.s.byte_ws = 0;
	}

	priv->fifo_depth = SPI_FIFO_DEPTH;
	if (ctrl0_reg.s.byte_ws) {
		priv->fifo_depth = priv->fifo_depth * 2;
	}

	ctrl0_reg.s.hold = 0;
	ctrl0_reg.s.fc_en = 0;
	ctrl0_reg.s.residue = 1;
	writel(ctrl0_reg.w, priv->regbase + SPI_CTRLR0_OFFSET);

	/* clear any error */
	readl(priv->regbase + SPI_TXOICR_OFFSET);
	readl(priv->regbase + SPI_RXOICR_OFFSET);
	readl(priv->regbase + SPI_RXUICR_OFFSET);
	readl(priv->regbase + SPI_ICR_OFFSET);

	/* disable dma & interrupt */
	writel(SPI_DMA_EN_NONE, priv->regbase + SPI_DMAC_OFFSET);
	writel(0, priv->regbase + SPI_IMR_OFFSET);

	/* TX_Threshold(half-fifo) <=16(32 if byte wise)bytes, >=RX_Threadhold 8bytes */
	writel(((priv->fifo_depth / 2) - 1), priv->regbase + SPI_TXFTLR_OFFSET);
	writel((8 - 1), priv->regbase + SPI_RXFTLR_OFFSET);

	/* configure interrupt or dma-request */
	if (priv->dma_used) {
		writel(0, priv->regbase + SPI_IMR_OFFSET);
		writel(SPI_DMA_TX_EN|SPI_DMA_RX_EN, priv->regbase + SPI_DMAC_OFFSET);
	} else {
		writel(SPI_RXFIS_MASK, priv->regbase + SPI_IMR_OFFSET);
		writel(SPI_DMA_EN_NONE, priv->regbase + SPI_DMAC_OFFSET);
	}

	if (priv->fault_inject) {	/* allow injectfault */
		inject = readl(priv->regbase + SPI_FAULTINJECT_OFFSET);
		inject &= ~SPI_FAULTINJECT_MASK;
		writel(inject, priv->regbase + SPI_FAULTINJECT_OFFSET);
	}

	smp_wmb();
	writel(1, priv->regbase + SPI_SSIENR_OFFSET);

	/* pinctrl */
	pins = devm_pinctrl_get_select_default(priv->pltdev);
	if (IS_ERR(pins)) {
		printk(KERN_ALERT "spislave: Failed to request pinctrl \n");
		err = PTR_ERR(pins);
	}
}
static void ambarella_slavespi_hardware_deinit(struct ambarella_slavespi *priv)
{
	u32 ctrl0 = 0;

	writel(1, priv->regbase + SPI_SSIENR_OFFSET);
	udelay(2);
	writel(0, priv->regbase + SPI_SSIENR_OFFSET);
	udelay(2);

	/* reset ctrl0 to default, disable slave-txe */
	ctrl0 = 0x407;	/* slave.ove = 1 disable tx, frame-bit=7 */
	writel(ctrl0, priv->regbase + SPI_CTRLR0_OFFSET);

	/* clear any error */
	readl(priv->regbase + SPI_TXOICR_OFFSET);
	readl(priv->regbase + SPI_RXOICR_OFFSET);
	readl(priv->regbase + SPI_RXUICR_OFFSET);
	readl(priv->regbase + SPI_ICR_OFFSET);

	/* reset dma-req & interrupt to default */
	writel(SPI_DMA_EN_NONE, priv->regbase + SPI_DMAC_OFFSET);
	writel(0x1F, priv->regbase + SPI_IMR_OFFSET);

	/* reset TX_Threshold RX_Threadhold */
	writel(0, priv->regbase + SPI_TXFTLR_OFFSET);
	writel(0, priv->regbase + SPI_RXFTLR_OFFSET);
}

static int ambarella_slavespi_dma_init(struct ambarella_slavespi *priv)
{
	int err = 0;

	priv->tx_dma_inprocess = 0;
	priv->rx_dma_inprocess = 0;

	err = ambarella_spislave_dma_allocate(priv, SPI_DMA_TX);
	if (err) {
		return err;
	}

	err = ambarella_spislave_dma_allocate(priv, SPI_DMA_RX);
	if (err) {
		ambarella_spi_slave_dma_free(priv, SPI_DMA_TX);
		return err;
	}

	dmaengine_terminate_sync(priv->tx_dma_chan);
	dmaengine_terminate_sync(priv->rx_dma_chan);

	return 0;
}

static void ambarella_slavespi_dma_deinit(struct ambarella_slavespi *priv)
{
	dmaengine_terminate_sync(priv->tx_dma_chan);
	dmaengine_terminate_sync(priv->rx_dma_chan);

	priv->tx_dma_inprocess = 0;
	priv->rx_dma_inprocess = 0;

	ambarella_spi_slave_dma_free(priv, SPI_DMA_TX);
	ambarella_spi_slave_dma_free(priv, SPI_DMA_RX);
}

static int ambarella_slavespi_open_init(struct ambarella_slavespi *priv)
{
	int err = 0;

	priv->stat_txe_cnt = 0;
	priv->stat_rxov_cnt = 0;
	priv->stat_rx_cnt = 0;
	priv->stat_rx_drop = 0;

	err = ambarella_slavespi_sfifo_init(priv);
	if (err) {
		return err;
	}

	if (priv->dma_used) {
		err = ambarella_slavespi_dma_init(priv);
		if (err) {
			ambarella_slavespi_sfifo_deinit(priv);
			return err;
		}
		priv->rx_dma_inprocess = 1;
		err = ambarella_slavespi_start_rx_dma(priv);
		if (err) {
			priv->rx_dma_inprocess = 0;
			ambarella_slavespi_sfifo_deinit(priv);
			return err;
		}
	} else {
		err = devm_request_irq(priv->pltdev, priv->irq, ambarella_slavespi_isr,
			IRQF_TRIGGER_HIGH, dev_name(priv->pltdev), priv);
		if (err) {
			printk(KERN_ALERT "slavespi: Request spi interrupt error \n");
			ambarella_slavespi_sfifo_deinit(priv);
			return err;
		}
	}

	ambarella_slavespi_hardware_init(priv);

	return 0;
}

static void ambarella_slavespi_close_deinit(struct ambarella_slavespi *priv)
{
	priv->stat_txe_cnt = 0;
	priv->stat_rxov_cnt = 0;
	priv->stat_rx_cnt = 0;
	priv->stat_rx_drop = 0;
	priv->wakeup_mask = 0;

	if (priv->dma_used) {
		ambarella_slavespi_dma_deinit(priv);
	} else {
		devm_free_irq(priv->pltdev, priv->irq, priv);
	}

	ambarella_slavespi_sfifo_deinit(priv);
	ambarella_slavespi_hardware_deinit(priv);
}

static int slavespi_open(struct inode *inode, struct file *filp)
{
	int err = 0;
	struct ambarella_slavespi *priv = NULL;

	priv = container_of(inode->i_cdev, struct ambarella_slavespi, cdevx);
	if (!priv) {
		printk(KERN_ALERT "slavespi: no such device \n");
		return -EINVAL;
	}
	filp->private_data = priv;

	if (!atomic_dec_and_test(&priv->open_once)) {
		atomic_inc(&priv->open_once);
		printk(KERN_ALERT "slavespi: device is busy \n");
		return -EBUSY;
	}

	err = ambarella_slavespi_open_init(priv);
	if (err) {
		atomic_inc(&priv->open_once);
	}
	return err;
}

static ssize_t
slavespi_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	int err = 0;
	u32 imr = 0;
	u32 copied = 0;
	u32 dma_copied = 0;
	u32 w_fifo_size = 0;
	u32 w_fifo_len = 0;

	unsigned long flags;
	struct ambarella_slavespi *priv = NULL;

	priv = (struct ambarella_slavespi *)filp->private_data;
	if (!buf || 0 == count) {
		return 0;
	}

	if (kfifo_is_full(&priv->w_fifo)) {
		copied = 0;
		return copied;
	} else {
		err = kfifo_from_user(&priv->w_fifo, buf, count, &copied);
		if (err) {
			printk(KERN_DEBUG "slavespi: get data from user failed, err = %d \n", err);
			return err;
		}
	}

	if (priv->dma_used) {
		if (priv->tx_dma_inprocess) {
			return copied;
		}

		spin_lock(&priv->w_buf_lock);
		if (priv->tx_dma_inprocess) {
			spin_unlock(&priv->w_buf_lock);
			return copied;
		} else {
			priv->tx_dma_inprocess = 1;
			spin_unlock(&priv->w_buf_lock);
		}

		w_fifo_len = kfifo_len(&priv->w_fifo);
		w_fifo_len &= priv->dma_size_mask;
		w_fifo_size = min_t(u32, w_fifo_len, priv->buf_size);
		if (w_fifo_size > 0) {
			dma_sync_single_for_cpu(priv->pltdev, priv->tx_dma_phys,
							priv->buf_size, DMA_TO_DEVICE);
			dma_copied = kfifo_out(&priv->w_fifo, priv->tx_dma_buf, w_fifo_size);
		}
		if (dma_copied > 0) {
			priv->tx_dma_size = dma_copied;
			ambarella_slavespi_start_tx_dma(priv);
		} else {
			priv->tx_dma_inprocess = 0;	/* can over write dma buf */
		}

		return copied;
	}

	/* no dma avail, using interrupt pio to transfer */
	spin_lock_irqsave(&priv->w_buf_lock, flags);
	ambarella_slave_fill_txfifo(priv);
	imr = readl(priv->regbase + SPI_IMR_OFFSET);
	writel((imr | SPI_TXEIS_MASK), priv->regbase + SPI_IMR_OFFSET);
	spin_unlock_irqrestore(&priv->w_buf_lock, flags);
	wait_event_interruptible(priv->wq_whead, !kfifo_is_full(&priv->w_fifo));

	return copied;
}

static ssize_t
slavespi_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int err = 0;
	int copied = 0;
	unsigned long flags;
	struct ambarella_slavespi *priv = NULL;

	if (!buf || 0 == count) {
		return 0;
	}

	priv = (struct ambarella_slavespi *)filp->private_data;
	if (priv->dma_used) {
		if (priv->rx_dma_inprocess) {
			err = kfifo_to_user(&priv->r_fifo, buf, count, &copied);
			if (err) {
				printk(KERN_ALERT "spislave: copy to user failed \n");
				return err;
			} else {
				return copied;
			}
		} else {
			priv->rx_dma_inprocess = 1;
			err = ambarella_slavespi_start_rx_dma(priv);
			if (err) {
				printk(KERN_ALERT "spislave: start rx dma failed \n");
				priv->rx_dma_inprocess = 0;
			}
			return err;
		}
	}

	/* no dma avail , using interrupt pio to tranfer */
	if (filp->f_flags & O_NONBLOCK) {
		spin_lock_irqsave(&priv->r_buf_lock, flags);
		slavespi_drain_rxfifo(priv);
		err = kfifo_to_user(&priv->r_fifo, buf, count, &copied);
		spin_unlock_irqrestore(&priv->r_buf_lock, flags);
		if (err) {
			return err;
		} else {
			return copied;
		}
	}

	/* following is block read */
	priv->want_cnt = count;
	spin_lock_irqsave(&priv->r_buf_lock, flags);
	slavespi_drain_rxfifo(priv);
	if (kfifo_len(&priv->r_fifo) >= priv->want_cnt) {
		err = kfifo_to_user(&priv->r_fifo, buf, priv->want_cnt, &copied);
		spin_unlock_irqrestore(&priv->r_buf_lock, flags);
		if (err) {
			return err;
		} else {
			return copied;
		}
	}
	spin_unlock_irqrestore(&priv->r_buf_lock, flags);

	wait_event_interruptible(priv->wq_rhead, \
			(kfifo_len(&priv->r_fifo) >= priv->want_cnt));

	spin_lock_irqsave(&priv->r_buf_lock, flags);
	slavespi_drain_rxfifo(priv);
	err = kfifo_to_user(&priv->r_fifo, buf, count, &copied);
	spin_unlock_irqrestore(&priv->r_buf_lock, flags);

	if (err) {
		return err;
	} else {
		return copied;
	}
}

static long
slavespi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int slavespi_release(struct inode *inode, struct file *filp)
{
	struct ambarella_slavespi *priv = NULL;
	priv = (struct ambarella_slavespi *)filp->private_data;

	ambarella_slavespi_close_deinit(priv);

	atomic_inc(&priv->open_once);
	return 0;
}

static unsigned int
slavespi_poll(struct file *filp, struct poll_table_struct *wait)
{
	u32 mask = 0;
	struct ambarella_slavespi *priv = NULL;

	priv = (struct ambarella_slavespi *)filp->private_data;
	poll_wait(filp, &priv->wq_poll, wait);

	if (kfifo_len(&priv->r_fifo) > 0) {
		mask |= POLLIN | POLLRDNORM;
	}

	if (!kfifo_is_full(&priv->w_fifo)) {
		mask |= POLLOUT;
	}

	return mask;
}

static struct file_operations slavespi_fops = {
	.open		= slavespi_open,
	.read		= slavespi_read,
	.write		= slavespi_write,
	.unlocked_ioctl	= slavespi_ioctl,
	.release	= slavespi_release,
	.poll		= slavespi_poll,
};

static int ambarella_slavespi_cdev_init(struct ambarella_slavespi *priv)
{
	int err = 0;

	priv->dname = "slavespi";
	err = alloc_chrdev_region(&priv->devno, 0, 1, priv->dname);
	if (err) {
		printk(KERN_ALERT "slavespi: alloc chrdevno failed \n");
		goto cdev_alloc_err;
	}

	cdev_init(&priv->cdevx, &slavespi_fops);
	priv->cdevx.owner = THIS_MODULE;
	cdev_add(&priv->cdevx, priv->devno, 1);

	priv->dclass = class_create(THIS_MODULE, priv->dname);
	if (IS_ERR(priv->dclass)) {
		printk(KERN_ALERT "slavespi: Unable to create class\n");
		goto cdev_class_err;

	}

	priv->device = device_create(priv->dclass, NULL, priv->devno, priv, priv->dname);
	if (IS_ERR(priv->device)) {
		printk(KERN_ALERT "slavespi: Unable to create device\n");
		goto cdev_device_err;
	}

	return 0;

cdev_device_err:
	class_destroy(priv->dclass);
	priv->dclass = NULL;

cdev_class_err:
	cdev_del(&priv->cdevx);
	unregister_chrdev_region(priv->devno, 1);

cdev_alloc_err:
	return (-ENODEV);
}

static void ambarella_slavespi_cdev_deinit(struct ambarella_slavespi *priv)
{
	device_destroy(priv->dclass, priv->devno);
	priv->device = NULL;

	class_destroy(priv->dclass);
	priv->dclass = NULL;

	cdev_del(&priv->cdevx);
	unregister_chrdev_region(priv->devno, 1);
}

static int slavespi_proc_show(struct seq_file *m, void *v)
{
	struct ambarella_slavespi *priv = m->private;

	seq_printf(m, "---show slavespi stat, write to clear ---\n");

	seq_printf(m, "slavespi->txe: %u \n", priv->stat_txe_cnt);
	seq_printf(m, "slavespi->rx_ov: %u \n", priv->stat_rxov_cnt);
	seq_printf(m, "slavespi->rx: %u \n", priv->stat_rx_cnt);
	seq_printf(m, "slavespi->rx_drop_bytes: %u \n", priv->stat_rx_drop);

	return 0;
}

static ssize_t slavespi_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *ppos)
{
	struct ambarella_slavespi *priv = PDE_DATA(file_inode(file));

	priv->stat_rxov_cnt = 0;
	priv->stat_rx_cnt = 0;
	priv->stat_txe_cnt = 0;
	priv->stat_rx_drop = 0;

	printk(KERN_ALERT "slavespi: all stat clear done \n");

	return count;
}

static int slavespi_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, slavespi_proc_show, PDE_DATA(inode));
}

static const struct file_operations slavespi_proc_ops = {
	.owner = THIS_MODULE,
	.open = slavespi_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = slavespi_proc_write,
	.release = single_release,
};

static int ambarella_slavespi_probe(struct platform_device *pdev)
{
	int err = 0;
	int irq = 0;
	void __iomem	*reg = NULL;
	struct resource *res = NULL;
	struct ambarella_slavespi *priv = NULL;

	/* Get Base Address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk(KERN_ALERT "slavespi: get mem resource failed\n");
		err = -ENOENT;
		goto err_out;
	}

	reg = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!reg) {
		printk(KERN_ALERT "slavespi: devm_ioremap() failed\n");
		err = -ENOMEM;
		goto err_out;
	}

	/* Get IRQ NO. */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		printk(KERN_ALERT "slavespi: get irq failed\n");
		err = -ENOENT;
		goto err_out;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		printk(KERN_ALERT "slavespi: no memory\n");
		err = -ENOMEM;
		goto err_out;
	}

	atomic_set(&priv->open_once, 1);
	priv->phys = res->start;
	priv->regbase = reg;
	priv->irq = irq;
	priv->pltdev = &pdev->dev;
	priv->dma_used = 0;
	priv->buf_size = SLAVE_SPI_BUF_SIZE;
	priv->wakeup_mask = 0;

	if (of_find_property(priv->pltdev->of_node, "amb,dma-used", NULL)) {
		priv->dma_used = 1;
	}

	if (of_find_property(priv->pltdev->of_node, "amb,fault-inject", NULL)) {
		priv->fault_inject = 1;
	}

#if 0
	if (of_find_property(priv->pltdev->of_node, "spi-cpol", NULL)) {
		priv->spi_mode |= SPI_CPOL;
	}

	if (of_find_property(priv->pltdev->of_node, "spi-cpha", NULL)) {
		priv->spi_mode |= SPI_CPHA;
	}

	if (of_find_property(priv->pltdev->of_node, "bits-per-word", NULL)) {
		of_property_read_u32_index(priv->pltdev->of_node, "bits-per-word", 0, &priv->bpw);
	}
#else
	priv->bpw = 8;			/* bits-per-word = 8 */
	priv->spi_mode = SPI_MODE_3;	/* [only support mode 3!!!], force!!! */
#endif

	if (of_find_property(priv->pltdev->of_node, "spi-lsb-first", NULL)) {
		priv->spi_mode |= SPI_LSB_FIRST;
	}

	if (priv->bpw > 8) {
		priv->dma_size_mask = SPI_DMA_ALIGN_16_MASK;
	} else {
		priv->dma_size_mask = SPI_DMA_ALIGN_8_MASK;
	}

	init_waitqueue_head(&priv->wq_rhead);
	init_waitqueue_head(&priv->wq_whead);
	init_waitqueue_head(&priv->wq_poll);

	spin_lock_init(&priv->r_buf_lock);
	spin_lock_init(&priv->w_buf_lock);
	spin_lock_init(&priv->wakeup_lock);
	tasklet_init(&priv->wakeup_helper, wakeup_tasklet, (unsigned long)priv);

	err = ambarella_slavespi_cdev_init(priv);
	if (err) {
		goto err_out;
	}

	priv->stat_proc = proc_create_data("slavespi", S_IRUGO|S_IWUSR, NULL,
			&slavespi_proc_ops, priv);
	if (!priv->stat_proc) {
		printk(KERN_ALERT "slavespi without proc supported \n");
	}

	printk(KERN_ALERT "slavespi: probe ok ver:%u using %s \n", SLAVE_SPI_VERSION_NUM,
				priv->dma_used ? "dma-mode":"irq-mode");
	if (priv->dma_used) {
		printk(KERN_ALERT "slavespi: dma-mode is %s \n",
			USING_STREAM_DMA > 0 ? "stream":"corherent");
	}

	platform_set_drvdata(pdev, priv);
	return 0;

err_out:
	printk(KERN_ALERT "spislave probe failed \n");
	return err;
}

static int ambarella_slavespi_remove(struct platform_device *pdev)
{
	struct ambarella_slavespi *priv = NULL;

	priv = platform_get_drvdata(pdev);

	if (priv->stat_proc) {
		remove_proc_entry("slavespi", NULL);
		priv->stat_proc = NULL;
	}

	ambarella_slavespi_cdev_deinit(priv);

	/* shutdown controller */
	writel(0, priv->regbase + SPI_SSIENR_OFFSET);

	return 0;
}


#ifdef CONFIG_PM
static int ambarella_slavespi_suspend_noirq(struct device *dev)
{
	return 0;
}

static int ambarella_slavespi_resume_noirq(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops ambarella_slavespi_dev_pm_ops = {
	.suspend_noirq = ambarella_slavespi_suspend_noirq,
	.resume_noirq = ambarella_slavespi_resume_noirq,
};
#endif

static const struct of_device_id ambarella_slavespi_dt_ids[] = {
	{.compatible = "ambarella,spi-slave", },
	{},
};
MODULE_DEVICE_TABLE(of, ambarella_slavespi_dt_ids);

static struct platform_driver ambarella_slavespi_driver = {
	.probe		= ambarella_slavespi_probe,
	.remove		= ambarella_slavespi_remove,
	.driver		= {
		.name	= "ambarella-spi-slave",
		.owner	= THIS_MODULE,
		.of_match_table = ambarella_slavespi_dt_ids,
#ifdef CONFIG_PM
		.pm	= &ambarella_slavespi_dev_pm_ops,
#endif
	},
};

static int __init ambarella_slavespi_init(void)
{
	return platform_driver_register(&ambarella_slavespi_driver);
}

static void __exit ambarella_slavespi_exit(void)
{
	platform_driver_unregister(&ambarella_slavespi_driver);
}

subsys_initcall(ambarella_slavespi_init);
module_exit(ambarella_slavespi_exit);

MODULE_DESCRIPTION("Ambarella Media Processor SPI Slave Controller");
MODULE_AUTHOR("Zhenwu Xue, <zwxue@ambarella.com>");
MODULE_LICENSE("GPL");
