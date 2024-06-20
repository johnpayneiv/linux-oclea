/*
 * /drivers/net/ethernet/ambarella/ambarella_eth.h
 *
 * Copyright (C) 2004-2099, Ambarella, Inc.
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

#ifndef __AMBARELLA_ETH_H__
#define __AMBARELLA_ETH_H__

#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/ptp_clock_kernel.h>

#define AMBETH_NAPI_WEIGHT		32
#define AMBETH_TX_WATCHDOG		(2 * HZ)
#define AMBETH_MII_RETRY_CNT	200
#define AMBETH_FC_PAUSE_TIME	1954

#define DEFAULT_BFSIZE			1536

#define AMBETH_RX_RNG_MIN		(8)
#define AMBETH_TX_RNG_MIN		(4)
#define AMBETH_PHY_REG_SIZE		(32)

#define AMBETH_RXDMA_STATUS	(ETH_DMA_STATUS_OVF | ETH_DMA_STATUS_RI | \
				ETH_DMA_STATUS_RU | ETH_DMA_STATUS_RPS | \
				ETH_DMA_STATUS_RWT)
#define AMBETH_RXDMA_INTEN	(ETH_DMA_INTEN_OVE | ETH_DMA_INTEN_RIE | \
				ETH_DMA_INTEN_RUE | ETH_DMA_INTEN_RSE | \
				ETH_DMA_INTEN_RWE)
#define AMBETH_TXDMA_STATUS	(ETH_DMA_STATUS_TI | ETH_DMA_STATUS_TPS | \
				ETH_DMA_STATUS_TU | ETH_DMA_STATUS_TJT | \
				ETH_DMA_STATUS_UNF)
#if defined(CONFIG_NET_VENDOR_AMBARELLA_INTEN_TUE)
#define AMBETH_TXDMA_INTEN	(ETH_DMA_INTEN_TIE | ETH_DMA_INTEN_TSE | \
				ETH_DMA_INTEN_TUE | ETH_DMA_INTEN_TJE | \
				ETH_DMA_INTEN_UNE)
#else
#define AMBETH_TXDMA_INTEN	(ETH_DMA_INTEN_TIE | ETH_DMA_INTEN_TSE | \
				ETH_DMA_INTEN_TJE | ETH_DMA_INTEN_UNE)
#endif
#define AMBETH_DMA_INTEN	(ETH_DMA_INTEN_NIE | ETH_DMA_INTEN_AIE | \
				ETH_DMA_INTEN_FBE | AMBETH_RXDMA_INTEN | \
				AMBETH_TXDMA_INTEN)

/*------------------------------------------------------------------------*/
#define setbitsl(v, a)   (writel(((v) | readl(a)), (a)))
#define clrbitsl(v, a)   (writel(((~(v)) & readl(a)), (a)))

struct ambeth_gmac_op {
	void (*set_mode)(void *priv);
	void (*set_clock)(void *priv);
};


struct ambeth_desc {
	u32				status;
	u32				length;
	u32				buffer1;
	u32				buffer2;
	u32				des4;
	u32				des5;
	u32				des6;
	u32				des7;
} __attribute((packed));

struct ambeth_rng_info {
	struct sk_buff			*skb;
	dma_addr_t			mapping;
};

struct ambeth_tx_rngmng {
	unsigned long long		cur_tx;
	unsigned long long		dirty_tx;
	struct ambeth_rng_info		*rng_tx;
	struct ambeth_desc		*desc_tx;
};

struct ambeth_rx_rngmng {
	unsigned long long		cur_rx;
	unsigned long long		dirty_rx;
	struct ambeth_rng_info		*rng_rx;
	struct ambeth_desc		*desc_rx;
};

struct ambeth_info {
	unsigned int			rx_count;
	struct ambeth_rx_rngmng		rx;
	unsigned int			tx_count;
	unsigned int			tx_irq_low;
	unsigned int			tx_irq_high;
	struct ambeth_tx_rngmng		tx;
	dma_addr_t			rx_dma_desc;
	dma_addr_t			tx_dma_desc;
	spinlock_t			lock;
	int				oldspeed;
	int				oldduplex;
	int				oldlink;
	int				oldpause;
	int				oldasym_pause;
	u32				flow_ctr;

	struct net_device_stats		stats;
	struct napi_struct		napi;
	struct net_device		*ndev;
	struct device			*dev;

	struct mii_bus			new_bus;
	struct phy_device		*phydev;
	int				pwr_gpio;
	u8				pwr_gpio_active;
	int				rst_gpio;
	u8				rst_gpio_active;
	u32				rst_gpio_delay;
	u32				intf_type;
	u32				intf_type_val;
	u32				intf_type_mask;
	u32				phy_supported;
	u32				fixed_speed;		/* only for phy-less */

	struct ptp_clock		*ptp_clk;
	struct ptp_clock_info		ptp_clk_ops;
	u32				clk_ptp_rate;
	bool				hwts_tx_en;
	bool				hwts_rx_en;
	spinlock_t			ptp_spinlock;

	bool				pps_avail;
	u32				default_adden;
	u32				sub_second_inc;
	u32				systime_flags;

	unsigned char __iomem		*regbase;
	struct regmap			*reg_rct;
	struct regmap			*reg_scr;
	const struct ambeth_gmac_op	*op;
	u32				instance;
	u32				msg_enable;
	u32				bfsize;

	u32				nis_rx;
	u32				nis_tx;
	u32				nis_tx_buf_unavail;
	u32				ais_tx_stopped;
	u32				ais_tx_jabber_timeout;
	u32				ais_rx_overflow;
	u32				ais_tx_underflow;
	u32				ais_rx_buf_unavail;
	u32				ais_rx_stopped;
	u32				ais_rx_wdt_timeout;
	u32				ais_early_tx;
	u32				ais_fatal_error;





	struct completion		comp;
	void				(*selftest_callback)(struct sk_buff *skb, struct net_device *ndev);
	bool				selftest_ok;
	void				*selftest_rx_data;
	int				selftest_rx_len;

	u32				mdio_gpio: 1,
					enhance: 1,
					second_ref_clk_50mhz : 1,
					tx_clk_invert : 1,
					rx_clk_invert : 1,
					phy_enabled : 1,
					ipc_tx : 1,
					ipc_rx : 1,
					dump_tx : 1,
					dump_rx : 1,
					dump_rx_free : 1,
					ahb_mdio_clk_div: 4,
					loopback:1,
					dump_rx_all : 1,
					fixed_mdio: 1;

};

/*----------------------------------------------------------------------------*/

/* IEEE 1588 PTP registers */
#define MAC_PTP_CTRL_OFFSET		0x0700
#define MAC_PTP_SSINC_OFFSET		0x0704
#define MAC_PTP_STSEC_OFFSET		0x0708
#define MAC_PTP_STNSEC_OFFSET		0x070C
#define MAC_PTP_STSEC_UPDATE_OFFSET	0x0710
#define MAC_PTP_STNSEC_UPDATE_OFFSET	0x0714
#define MAC_PTP_ADDEND_OFFSET		0x0718

#define MAC_PPS_TARGET_TIME_SEC		0x071C
#define MAC_PPS_TARGET_TIME_NSEC	0x0720

#define MAC_PPS_CONTROL			0x072C
#define MAC_PPS_INTERVAL		0x0760
#define MAC_PPS_WIDTH			0x0764

#define PPSEN0				BIT(4)	/* enable flexiable output */
#define TRGTBUSY0			BIT(31)
#define PPSCMD(val)			(val)
#define TRGTMODSEL(val)			((val) << 5)
#define TRGTMODSEL_MASK			(3 << 5)

/* MAC_PTP_CTRL_OFFSET bitmap */
#define PTP_CTRL_TSENA		BIT(0)
#define PTP_CTRL_TSCFUPDT	BIT(1)
#define PTP_CTRL_TSINIT		BIT(2)
#define PTP_CTRL_TSUPDT		BIT(3)
#define PTP_CTRL_TSTRIG		BIT(4)
#define PTP_CTRL_TSADDREG	BIT(5)
#define PTP_CTRL_TSENALL	BIT(8)
#define PTP_CTRL_TSCTRLSSR	BIT(9)
#define PTP_CTRL_TSVER2ENA	BIT(10)
#define PTP_CTRL_TSIPENA	BIT(11)
#define PTP_CTRL_TSIPV6ENA	BIT(12)
#define PTP_CTRL_TSIPV4ENA	BIT(13)
#define PTP_CTRL_TSEVNTENA	BIT(14)
#define PTP_CTRL_TSMSTRENA	BIT(15)
#define PTP_CTRL_SNAPTYPSEL	BIT(16)
#define PTP_CTRL_TSENMACADDR	BIT(18)

#define	PTP_SSIR_SSINC_MASK	0xff

struct ambeth_mac_pps_cfg {
	bool available;
	struct timespec64 start;
	struct timespec64 period;
};

int ambeth_set_hwtstamp(struct net_device *dev, struct ifreq *ifr);
int ambeth_get_ts_info(struct net_device *dev, struct ethtool_ts_info *info);
int ambeth_ptp_init(struct platform_device *pdev);
void ambeth_ptp_exit(struct ambeth_info *priv);
void ambeth_get_rx_hwtstamp(struct ambeth_info *lp, struct sk_buff *skb,
		struct ambeth_desc *desc);
void ambeth_get_tx_hwtstamp(struct ambeth_info *lp, struct sk_buff *skb,
		struct ambeth_desc *desc);
void ambeth_tx_hwtstamp_enable(struct ambeth_info *lp, u32 *flags);

void ambeth_self_test(struct net_device *dev, struct ethtool_test *etest, u64 *buf);
int ambeth_get_sset_count(struct net_device *netdev, int sset);
void ambeth_get_strings(struct net_device *netdev, u32 stringset, u8 *data);
void ambeth_get_ethtool_stats(struct net_device *netdev,
	struct ethtool_stats *stats, u64 *data);
void ambeth_interrupt_statis(struct ambeth_info *priv, u32 irq_status);
static void ambeth_adjust_link(struct net_device *ndev);
#endif
