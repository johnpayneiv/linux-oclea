/*
 * arch/arm/plat-ambarella/include/plat/idc.h
 *
 * Author: Anthony Ginger <hfjiang@ambarella.com>
 *
 * Copyright (C) 2004-2010, Ambarella, Inc.
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

#ifndef __PLAT_AMBARELLA_IDC_H__
#define __PLAT_AMBARELLA_IDC_H__

#include <plat/chip.h>

/* ==========================================================================*/
#if (CHIP_REV == S2L) || (CHIP_REV == S3) || (CHIP_REV == S3L) || (CHIP_REV == S5)
#define IDC_INSTANCES			3
#elif (CHIP_REV == CV2FS) || (CHIP_REV == CV5)
#define IDC_INSTANCES			6
#else
#define IDC_INSTANCES			4
#endif

#if (CHIP_REV == S2L) || (CHIP_REV == S3) || (CHIP_REV == S3L) || \
	(CHIP_REV == S5) || (CHIP_REV == S5L)
#define IDC0_OFFSET			0x3000
#define IDC1_OFFSET			0x1000
#define IDC2_OFFSET			0x7000
#define IDC3_OFFSET			0x13000
#else
#define IDC0_OFFSET			0x8000
#define IDC1_OFFSET			0x9000
#define IDC2_OFFSET			0xA000
#define IDC3_OFFSET			0xB000
#endif
#if (CHIP_REV == CV5)
#define IDC4_OFFSET			0xE000
#define IDC5_OFFSET			0xF000
#else
#define IDC4_OFFSET			0x19000
#define IDC5_OFFSET			0x1A000
#endif

#define IDC0_BASE			(APB_BASE + IDC0_OFFSET)
#define IDC0_REG(x)			(IDC0_BASE + (x))

#define IDC1_BASE			(APB_BASE + IDC1_OFFSET)
#define IDC1_REG(x)			(IDC1_BASE + (x))

#define IDC2_BASE			(APB_BASE + IDC2_OFFSET)
#define IDC2_REG(x)			(IDC2_BASE + (x))

#define IDC3_BASE			(APB_BASE + IDC3_OFFSET)
#define IDC3_REG(x)			(IDC3_BASE + (x))

#define IDC4_BASE			(APB_BASE + IDC4_OFFSET)
#define IDC4_REG(x)			(IDC4_BASE + (x))

#define IDC5_BASE			(APB_BASE + IDC5_OFFSET)
#define IDC5_REG(x)			(IDC5_BASE + (x))

/* ==========================================================================*/
#define IDC_ENR_OFFSET			0x00
#define IDC_CTRL_OFFSET			0x04
#define IDC_DATA_OFFSET			0x08
#define IDC_STS_OFFSET			0x0c
#define IDC_PSLL_OFFSET			0x10
#define IDC_PSLH_OFFSET			0x14
#define IDC_FMCTRL_OFFSET		0x18
#define IDC_FMDATA_OFFSET		0x1c
#define IDC_PSHS_OFFSET			0x20
#define IDC_DUTYCYCLE_OFFSET		0x24
#define IDC_STRETCHSCL_OFFSET		0x28

#define IDC_ENR_REG_ENABLE		(0x01)
#define IDC_ENR_REG_DISABLE		(0x00)

#define IDC_CTRL_HSMODE			(0x10)
#define IDC_CTRL_STOP			(0x08)
#define IDC_CTRL_START			(0x04)
#define IDC_CTRL_IF			(0x02)
#define IDC_CTRL_ACK			(0x01)
#define IDC_CTRL_CLS			(0x00)

#define IDC_STS_FIFO_EMP		(0x04)
#define IDC_STS_FIFO_FUL		(0x02)

#define IDC_FIFO_BUF_SIZE		(63)

#define IDC_FMCTRL_HSMODE		(0x10)
#define IDC_FMCTRL_STOP			(0x08)
#define IDC_FMCTRL_START		(0x04)
#define IDC_FMCTRL_IF			(0x02)

/* ==========================================================================*/

#endif /* __PLAT_AMBARELLA_IDC_H__ */

