/*
 * /drivers/net/ethernet/ambarella/ambarella_ptp.c
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/time64.h>
#include <linux/ptp_classify.h>
#include <linux/net_tstamp.h>
#include <linux/spinlock.h>
#include <linux/ptp_clock_kernel.h>
#include <plat/eth.h>

#include "ambarella_eth.h"

static void ambeth_set_hwctrl(struct ambeth_info *lp, u32 bit)
{
	u32 ctrl_val, retry = 0;

	ctrl_val = readl(lp->regbase + MAC_PTP_CTRL_OFFSET);
	ctrl_val |= bit;
	writel(ctrl_val, lp->regbase + MAC_PTP_CTRL_OFFSET);

	while (readl(lp->regbase + MAC_PTP_CTRL_OFFSET) & bit) {

		if (++retry > 16) {
			pr_err("program ambarella MAC_PTP_CTRL register with value 0x%x timeout.\n",
					bit);
			break;
		}
		mdelay(8);
	}
}

int ambeth_set_hwtstamp(struct net_device *dev, struct ifreq *ifr)
{
	struct hwtstamp_config config;
	struct ambeth_info *lp = netdev_priv(dev);
	struct timespec64 now;
	u64 ssinc, addend;
	u32 ctrl_val = 0;

	if (!lp->ptp_clk)
		return -ENODEV;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	printk("SIOCSHWTSTAMP: f.%x t.%x f.%x\n",
			config.flags, config.tx_type, config.rx_filter);

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		config.rx_filter = HWTSTAMP_FILTER_NONE;
		break;

	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;

		ctrl_val |= PTP_CTRL_SNAPTYPSEL;
		ctrl_val |= PTP_CTRL_TSIPV4ENA;
		ctrl_val |= PTP_CTRL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_SYNC;

		ctrl_val |= PTP_CTRL_TSEVNTENA;
		ctrl_val |= PTP_CTRL_TSIPV4ENA;
		ctrl_val |= PTP_CTRL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ;

		ctrl_val |= PTP_CTRL_TSMSTRENA;
		ctrl_val |= PTP_CTRL_TSEVNTENA;
		ctrl_val |= PTP_CTRL_TSIPV4ENA;
		ctrl_val |= PTP_CTRL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;

		ctrl_val |= PTP_CTRL_TSVER2ENA;
		ctrl_val |= PTP_CTRL_SNAPTYPSEL;
		ctrl_val |= PTP_CTRL_TSIPV4ENA;
		ctrl_val |= PTP_CTRL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_SYNC;

		ctrl_val |= PTP_CTRL_TSVER2ENA;
		ctrl_val |= PTP_CTRL_TSEVNTENA;
		ctrl_val |= PTP_CTRL_TSIPV4ENA;
		ctrl_val |= PTP_CTRL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ;

		ctrl_val |= PTP_CTRL_TSVER2ENA;
		ctrl_val |= PTP_CTRL_TSMSTRENA;
		ctrl_val |= PTP_CTRL_TSEVNTENA;
		ctrl_val |= PTP_CTRL_TSIPV4ENA;
		ctrl_val |= PTP_CTRL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_EVENT:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;

		ctrl_val |= PTP_CTRL_TSVER2ENA;
		ctrl_val |= PTP_CTRL_SNAPTYPSEL;
		ctrl_val |= PTP_CTRL_TSIPV4ENA;
		ctrl_val |= PTP_CTRL_TSIPV6ENA;
		ctrl_val |= PTP_CTRL_TSIPENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_SYNC:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;

		ctrl_val |= PTP_CTRL_TSVER2ENA;
		ctrl_val |= PTP_CTRL_TSEVNTENA;
		ctrl_val |= PTP_CTRL_TSIPV4ENA;
		ctrl_val |= PTP_CTRL_TSIPV6ENA;
		ctrl_val |= PTP_CTRL_TSIPENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_DELAY_REQ;

		ctrl_val |= PTP_CTRL_TSVER2ENA;
		ctrl_val |= PTP_CTRL_TSMSTRENA;
		ctrl_val |= PTP_CTRL_TSEVNTENA;
		ctrl_val |= PTP_CTRL_TSIPV4ENA;
		ctrl_val |= PTP_CTRL_TSIPV6ENA;
		ctrl_val |= PTP_CTRL_TSIPENA;
		break;

	case HWTSTAMP_FILTER_NTP_ALL:
	case HWTSTAMP_FILTER_ALL:
		config.rx_filter = HWTSTAMP_FILTER_ALL;

		ctrl_val |= PTP_CTRL_TSENALL;
		break;

	default:
		return -ERANGE;
	}

	lp->hwts_tx_en = (config.tx_type == HWTSTAMP_TX_ON);
	lp->hwts_rx_en = !(config.rx_filter == HWTSTAMP_FILTER_NONE);

	if (!lp->hwts_tx_en && !lp->hwts_rx_en) {
		writel(0, lp->regbase + MAC_PTP_CTRL_OFFSET);
	} else {
		/*
		 * Timestamp control
		 *	Timestamp fine enable.
		 *	Timestamp low register rolls after 10e9 ns.
		 * */
		ctrl_val |= PTP_CTRL_TSENA;
		ctrl_val |= PTP_CTRL_TSCFUPDT;
		ctrl_val |= PTP_CTRL_TSCTRLSSR;
		writel(ctrl_val, lp->regbase + MAC_PTP_CTRL_OFFSET);

		if (lp->clk_ptp_rate > 50000000) {
			ssinc = 1000000000ULL / 50000000ULL;
		} else {
			ssinc = 1000000000ULL / 50000000ULL;	/* (10e9 / 50HMZ)) x (60MHZ / R) */
			ssinc *= 60000000;
			do_div(ssinc, lp->clk_ptp_rate);
		}
		writel(ssinc, lp->regbase + MAC_PTP_SSINC_OFFSET);

		addend = (1ULL << 32) * 50000000;
		if (lp->clk_ptp_rate > 50000000)
			do_div(addend, lp->clk_ptp_rate);
		else
			do_div(addend, 60000000);

		writel(addend, lp->regbase + MAC_PTP_ADDEND_OFFSET);
		ambeth_set_hwctrl(lp, PTP_CTRL_TSADDREG);

		ktime_get_real_ts64(&now);

		writel((u32)now.tv_sec, lp->regbase +  MAC_PTP_STSEC_UPDATE_OFFSET);
		writel(now.tv_nsec, lp->regbase + MAC_PTP_STNSEC_UPDATE_OFFSET);
		ambeth_set_hwctrl(lp, PTP_CTRL_TSUPDT);

		ambeth_set_hwctrl(lp, PTP_CTRL_TSINIT);
	}

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ? -EFAULT : 0;
}

int ambeth_get_ts_info(struct net_device *dev, struct ethtool_ts_info *info)
{
	struct ambeth_info *lp = netdev_priv(dev);

	if (lp->ptp_clk) {

		info->so_timestamping = SOF_TIMESTAMPING_TX_SOFTWARE |
					SOF_TIMESTAMPING_TX_HARDWARE |
					SOF_TIMESTAMPING_RX_SOFTWARE |
					SOF_TIMESTAMPING_RX_HARDWARE |
					SOF_TIMESTAMPING_SOFTWARE |
					SOF_TIMESTAMPING_RAW_HARDWARE;

		info->phc_index = ptp_clock_index(lp->ptp_clk);

		info->tx_types = (1 << HWTSTAMP_TX_OFF) | (1 << HWTSTAMP_TX_ON);

		info->rx_filters = ((1 << HWTSTAMP_FILTER_NONE)		|
				(1 << HWTSTAMP_FILTER_PTP_V1_L4_EVENT)	|
				(1 << HWTSTAMP_FILTER_PTP_V1_L4_SYNC)	|
				(1 << HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ)	|
				(1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT)	|
				(1 << HWTSTAMP_FILTER_PTP_V2_L4_SYNC)	|
				(1 << HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ)	|
				(1 << HWTSTAMP_FILTER_PTP_V2_EVENT)	|
				(1 << HWTSTAMP_FILTER_PTP_V2_SYNC)	|
				(1 << HWTSTAMP_FILTER_PTP_V2_DELAY_REQ) |
				(1 << HWTSTAMP_FILTER_ALL));
		return 0;
	} else
		return ethtool_op_get_ts_info(dev, info);
}

static int ambeth_adjust_freq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct ambeth_info *lp =
		container_of(ptp, struct ambeth_info, ptp_clk_ops);
	u32 diff, neg_adj = 0;
	u64 adj, addend;
	unsigned long flags;

	spin_lock_irqsave(&lp->ptp_spinlock, flags);

	if (ppb < 0) {
		ppb = -ppb;
		neg_adj = 1;
	}

	addend = (1ULL << 32) * 50000000;
	if (lp->clk_ptp_rate > 50000000)
		do_div(addend, lp->clk_ptp_rate);
	else
		do_div(addend, 60000000);

	adj = addend;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);
	addend = neg_adj ? (addend - diff) : (addend + diff);

	writel(addend, lp->regbase + MAC_PTP_ADDEND_OFFSET);
	ambeth_set_hwctrl(lp, PTP_CTRL_TSADDREG);

	spin_unlock_irqrestore(&lp->ptp_spinlock, flags);

	return 0;
}

static int ambeth_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
	u32 sec, nsec;
	int neg_adj = 0;
	unsigned long flags;
	struct ambeth_info *lp =
		container_of(ptp, struct ambeth_info, ptp_clk_ops);

	spin_lock_irqsave(&lp->ptp_spinlock, flags);

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	sec = div_u64_rem(delta, 1000000000ULL, &nsec);
	if (neg_adj)
		nsec = 1000000000 - nsec;

	nsec |= (neg_adj << 31);

	writel(sec, lp->regbase + MAC_PTP_STSEC_UPDATE_OFFSET);
	writel(nsec, lp->regbase + MAC_PTP_STNSEC_UPDATE_OFFSET);

	ambeth_set_hwctrl(lp, PTP_CTRL_TSUPDT);

	spin_unlock_irqrestore(&lp->ptp_spinlock, flags);

	return 0;
}

static int ambeth_ptp_enable(struct ptp_clock_info *ptp,
		struct ptp_clock_request *rq, int on)
{
	return -EOPNOTSUPP;
}

static int ambeth_set_time(struct ptp_clock_info *ptp,
		const struct timespec64 *ts)
{
	unsigned long flags;
	struct ambeth_info *lp =
		container_of(ptp, struct ambeth_info, ptp_clk_ops);

	spin_lock_irqsave(&lp->ptp_spinlock, flags);

	writel(ts->tv_sec, lp->regbase + MAC_PTP_STSEC_UPDATE_OFFSET);
	writel(ts->tv_nsec, lp->regbase + MAC_PTP_STNSEC_UPDATE_OFFSET);

	ambeth_set_hwctrl(lp, PTP_CTRL_TSINIT);

	spin_unlock_irqrestore(&lp->ptp_spinlock, flags);

	return 0;
}

static int ambeth_get_time(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	u64 ns;
	unsigned long flags;
	struct ambeth_info *lp =
		container_of(ptp, struct ambeth_info, ptp_clk_ops);

	spin_lock_irqsave(&lp->ptp_spinlock, flags);

	ns = readl(lp->regbase + MAC_PTP_STNSEC_OFFSET);
	ns += readl(lp->regbase + MAC_PTP_STSEC_OFFSET) * 1000000000ULL;
	*ts = ns_to_timespec64(ns);

	spin_unlock_irqrestore(&lp->ptp_spinlock, flags);

	return 0;
}

static struct ptp_clock_info ambarella_ptp_clock_ops = {
	.owner = THIS_MODULE,
	.name = "amba,ptp-clock",
	.max_adj =  0,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0,
	.n_pins = 0,
	.pps = 0,
	.adjfreq = ambeth_adjust_freq,
	.adjtime = ambeth_adjust_time,
	.gettime64 = ambeth_get_time,
	.settime64 = ambeth_set_time,
	.enable = ambeth_ptp_enable,
};

int ambeth_ptp_init(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct ambeth_info *lp = netdev_priv(ndev);
	struct clk *clk;
	u64 tmp;

	lp->ptp_clk_ops = ambarella_ptp_clock_ops;
	lp->hwts_rx_en = false;
	lp->hwts_tx_en = false;

	clk = devm_clk_get(&pdev->dev, "ptp_ref");
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&pdev->dev, "get 'ptp_ref' clock error.\n");
		return -EIO;
	}

	lp->clk_ptp_rate = clk_get_rate(clk);
	if (lp->clk_ptp_rate > 50000000)
		tmp = lp->clk_ptp_rate;
	else if (lp->clk_ptp_rate)	/* fix 60MHZ ? */
		tmp = 60000000;
	else
		return -EINVAL;

	tmp *= 1000000000ULL;
	do_div(tmp, 50000000);
	tmp = tmp - 1000000000ULL - 1;
	lp->ptp_clk_ops.max_adj = (u32)tmp;

	lp->ptp_clk = ptp_clock_register(&lp->ptp_clk_ops, &pdev->dev);

	if (IS_ERR_OR_NULL(lp->ptp_clk)) {
		lp->ptp_clk = NULL;
		dev_warn(&pdev->dev, "CONFIG_PTP_1588_CLOCK is not set\n");
		return -ENODEV;
	}

	spin_lock_init(&lp->ptp_spinlock);

	dev_info(&pdev->dev, "ambarella ptp clock register.\n");

	return 0;
}

void ambeth_get_rx_hwtstamp(struct ambeth_info *lp, struct sk_buff *skb,
		struct ambeth_desc *desc)
{
	struct skb_shared_hwtstamps *shhwtstamp;
	u32 sec, nsec;
	u64 rx_nsec;

	if (!lp->hwts_rx_en)
		return;

	if (!(desc->status & (ETH_ENHANCED_RDES0_LS)))
		return;

	if (!(desc->status & (1 << 7)))
		return;

	shhwtstamp = skb_hwtstamps(skb);
	if (!shhwtstamp)
		return;

	nsec = le32_to_cpu(desc->des6);
	sec = le32_to_cpu(desc->des7);
	rx_nsec = sec * 1000000000ULL + nsec;

	memset(shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamp->hwtstamp = ns_to_ktime(rx_nsec);

}

void ambeth_get_tx_hwtstamp(struct ambeth_info *lp, struct sk_buff *skb,
		struct ambeth_desc *desc)
{
	struct skb_shared_hwtstamps shhwtstamp;
	u32 sec, nsec;
	u64 tx_nsec;

	if (!lp->hwts_tx_en)
		return;

	if (!(desc->status & (ETH_ENHANCED_TDES0_LS)))
		return;

	if (!(desc->status & (1 << 17)))
		return;

	nsec = le32_to_cpu(desc->des6);
	sec = le32_to_cpu(desc->des7);
	tx_nsec = sec * 1000000000ULL + nsec;

	memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));

	shhwtstamp.hwtstamp = ns_to_ktime(tx_nsec);
	skb_tstamp_tx(skb, &shhwtstamp);
}

void ambeth_tx_hwtstamp_enable(struct ambeth_info *lp, u32 *flags)
{
	if (!lp->hwts_tx_en)
		return;
	*flags |= (1 << 25);
}
