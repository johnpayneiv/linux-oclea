/*
 * linux/drivers/dma/ambarella_dma.h
 *
 * Header file for Ambarella DMA Controller driver
 *
 * History:
 *	2012/07/10 - [Cao Rongrong] created file
 *
 * Copyright (C) 2012-2029, Ambarella, Inc.
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

#ifndef _AMBARELLA_DMA_H
#define _AMBARELLA_DMA_H

#define NR_DESCS_PER_CHANNEL		64
#define AMBARELLA_DMA_MAX_LENGTH	((1 << 22) - 1)

enum ambdma_status {
	AMBDMA_STATUS_IDLE = 0,
	AMBDMA_STATUS_BUSY,
	AMBDMA_STATUS_STOPPING,
};

/* lli == Linked List Item; aka DMA buffer descriptor, used by hardware */
struct ambdma_lli {
	u32			src;
	u32			dst;
	u32			next_desc;
	u32			rpt_addr;
	u32			xfr_count;
	u32			attr;
	u32			rsvd;
	u32			rpt;
};

struct ambdma_desc {
	struct ambdma_lli		*lli;

	struct dma_async_tx_descriptor	txd;
	struct list_head		tx_list;
	struct list_head		desc_node;
	size_t				total_len;
	bool				is_cyclic;
};

struct ambdma_chan {
	struct dma_chan			chan;
	struct ambdma_device		*amb_dma;

	struct tasklet_struct		tasklet;
	spinlock_t			lock;

	u32				descs_allocated;
	struct list_head		active_list;
	struct list_head		queue_list;
	struct list_head		free_list;
	struct list_head		stopping_list;

	dma_addr_t			rt_addr;
	u32				rt_attr;
	u32				force_stop;
	enum ambdma_status		status;

#ifdef CONFIG_PM
	u32				ch_ctl;
	u32				ch_sta;
	u32				ch_da;
#endif
};

struct ambdma_device {
	struct dma_device		dma_dev;
	struct ambdma_chan		amb_chan[NUM_DMA_CHANNELS];
	struct dma_pool			*lli_pool;
	void __iomem			*regbase;
	void __iomem			*regscr;
	struct regmap			*scr_regmap;
	int				dma_irq;
	/* dummy_desc is used to stop DMA immediately. */
	struct ambdma_lli		*dummy_lli;
	dma_addr_t			dummy_lli_phys;
	u32				*dummy_data;
	dma_addr_t			dummy_data_phys;
	/* support pause/resume/stop */
	u32				non_support_prs : 1;
	u32				nr_channels;
	u32				nr_requests;
	u32				scr_offset;

#ifdef CONFIG_PM
	u32				sel_val0;
	u32				sel_val1;
#endif
};


static inline struct ambdma_desc *to_ambdma_desc(
	struct dma_async_tx_descriptor *txd)
{
	return container_of(txd, struct ambdma_desc, txd);
}

static inline struct ambdma_desc *ambdma_first_active(
	struct ambdma_chan *amb_chan)
{
	return list_first_entry_or_null(&amb_chan->active_list,
			struct ambdma_desc, desc_node);
}

static inline struct ambdma_desc *ambdma_first_stopping(
	struct ambdma_chan *amb_chan)
{
	return list_first_entry_or_null(&amb_chan->stopping_list,
			struct ambdma_desc, desc_node);
}

static inline struct ambdma_chan *to_ambdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct ambdma_chan, chan);
}

static inline void __iomem *dma_chan_ctr_reg(struct ambdma_chan *amb_chan)
{
	return amb_chan->amb_dma->regbase + 0x300 + (amb_chan->chan.chan_id << 4);
}

static inline void __iomem *dma_chan_src_reg(struct ambdma_chan *amb_chan)
{
	return amb_chan->amb_dma->regbase + 0x304 + (amb_chan->chan.chan_id << 4);
}

static inline void __iomem *dma_chan_dst_reg(struct ambdma_chan *amb_chan)
{
	return amb_chan->amb_dma->regbase + 0x308 + (amb_chan->chan.chan_id << 4);
}

static inline void __iomem *dma_chan_sta_reg(struct ambdma_chan *amb_chan)
{
	return amb_chan->amb_dma->regbase + 0x30c + (amb_chan->chan.chan_id << 4);
}

static inline void __iomem *dma_chan_da_reg(struct ambdma_chan *amb_chan)
{
	return amb_chan->amb_dma->regbase + 0x380 + (amb_chan->chan.chan_id << 2);
}

static inline int ambdma_desc_is_error(struct ambdma_desc *amb_desc)
{
	return (amb_desc->lli->rpt & (DMA_CHANX_STA_OE | DMA_CHANX_STA_ME |
			DMA_CHANX_STA_BE | DMA_CHANX_STA_RWE |
			DMA_CHANX_STA_AE)) != 0x0;
}

static inline int ambdma_chan_is_enabled(struct ambdma_chan *amb_chan)
{
	return !!(readl_relaxed(dma_chan_ctr_reg(amb_chan)) & DMA_CHANX_CTR_EN);
}

static dma_cookie_t ambdma_tx_submit(struct dma_async_tx_descriptor *tx);
static void ambdma_dostart(struct ambdma_chan *amb_chan,
		struct ambdma_desc *first);

#endif

