/*
 * arch/arm/plat-ambarella/include/plat/dma.h
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

#ifndef __PLAT_AMBARELLA_DMA_H__
#define __PLAT_AMBARELLA_DMA_H__

#include <plat/chip.h>

/* ==========================================================================*/
#if (CHIP_REV == S2L) || (CHIP_REV == S3) || (CHIP_REV == S3L) || \
	(CHIP_REV == S5) || (CHIP_REV == S5L)
#define DMA_INSTANCES			1
#define NUM_DMA_CHANNELS 		8
#else
#define DMA_INSTANCES			2
#define NUM_DMA_CHANNELS 		16
#endif

#if (CHIP_REV == S2L) || (CHIP_REV == S3) || (CHIP_REV == S3L) || \
	(CHIP_REV == S5) || (CHIP_REV == S5L)
#define DMA0_OFFSET			0x5000
#elif (CHIP_REV == CV1) || (CHIP_REV == CV2)
#define DMA0_OFFSET			0xA000
#else
#define DMA0_OFFSET			0x20000
#endif
#define DMA0_BASE			(AHB_BASE + DMA0_OFFSET)
#define DMA0_REG(x)			(DMA0_BASE + (x))

#if (CHIP_REV == CV1) || (CHIP_REV == CV2)
#define DMA1_OFFSET			0xB000
#else
#define DMA1_OFFSET			0x21000
#endif
#define DMA1_BASE			(AHB_BASE + DMA1_OFFSET)
#define DMA1_REG(x)			(DMA1_BASE + (x))

/* ==========================================================================*/

#if ((CHIP_REV == S3) || (CHIP_REV == S3L) || (CHIP_REV == S5) || (CHIP_REV == S5L))
#define AHBSP_DMA_SEL_BIT_SHIFT		4
#elif (CHIP_REV == CV1)
#define AHBSP_DMA_SEL_BIT_SHIFT		5
#else
#define AHBSP_DMA_SEL_BIT_SHIFT		8
#endif
#define AHBSP_DMA_SEL_BIT_MASK		((1 << AHBSP_DMA_SEL_BIT_SHIFT) - 1)

/* ==========================================================================*/

#define NOR_SPI_TX_DMA_CHAN		0
#define NOR_SPI_RX_DMA_CHAN		1
#define SSI1_TX_DMA_CHAN		2
#define SSI1_RX_DMA_CHAN		3
#define UART_TX_DMA_CHAN		4
#define UART_RX_DMA_CHAN		5
#define I2S_RX_DMA_CHAN			6
#define I2S_TX_DMA_CHAN			7

/* ==========================================================================*/
#define DMA0_CHAN_CTR_REG(x)		DMA0_REG((0x300 + ((x) << 4)))
#define DMA0_CHAN_SRC_REG(x)		DMA0_REG((0x304 + ((x) << 4)))
#define DMA0_CHAN_DST_REG(x)		DMA0_REG((0x308 + ((x) << 4)))
#define DMA0_CHAN_STA_REG(x)		DMA0_REG((0x30c + ((x) << 4)))
#define DMA0_CHAN_DA_REG(x)		DMA0_REG((0x380 + ((x) << 2)))

#define DMA1_CHAN_CTR_REG(x)		DMA1_REG((0x300 + ((x) << 4)))
#define DMA1_CHAN_SRC_REG(x)		DMA1_REG((0x304 + ((x) << 4)))
#define DMA1_CHAN_DST_REG(x)		DMA1_REG((0x308 + ((x) << 4)))
#define DMA1_CHAN_STA_REG(x)		DMA1_REG((0x30c + ((x) << 4)))
#define DMA1_CHAN_DA_REG(x)		DMA1_REG((0x380 + ((x) << 2)))

/* ==========================================================================*/

#define DMA_INT_OFFSET			0x3f0
#define DMA_PAUSE_SET_OFFSET		0x3f4
#define DMA_PAUSE_CLR_OFFSET		0x3f8
#define DMA_EARLY_END_OFFSET		0x3fc

/* ==========================================================================*/

/* DMA_CHANX_CTR_REG */
#define DMA_CHANX_CTR_EN		0x80000000
#define DMA_CHANX_CTR_D			0x40000000
#define DMA_CHANX_CTR_WM		0x20000000
#define DMA_CHANX_CTR_RM		0x10000000
#define DMA_CHANX_CTR_NI		0x08000000
#define DMA_CHANX_CTR_BLK_1024B		0x07000000
#define DMA_CHANX_CTR_BLK_512B		0x06000000
#define DMA_CHANX_CTR_BLK_256B		0x05000000
#define DMA_CHANX_CTR_BLK_128B		0x04000000
#define DMA_CHANX_CTR_BLK_64B		0x03000000
#define DMA_CHANX_CTR_BLK_32B		0x02000000
#define DMA_CHANX_CTR_BLK_16B		0x01000000
#define DMA_CHANX_CTR_BLK_8B		0x00000000
#define DMA_CHANX_CTR_TS_8B		0x00C00000
#define DMA_CHANX_CTR_TS_4B		0x00800000
#define DMA_CHANX_CTR_TS_2B		0x00400000
#define DMA_CHANX_CTR_TS_1B		0x00000000

/* DMA descriptor bit fields */
#define DMA_DESC_EOC			0x01000000
#define DMA_DESC_WM			0x00800000
#define DMA_DESC_RM			0x00400000
#define DMA_DESC_NI			0x00200000
#define DMA_DESC_TS_8B			0x00180000
#define DMA_DESC_TS_4B			0x00100000
#define DMA_DESC_TS_2B			0x00080000
#define DMA_DESC_TS_1B			0x00000000
#define DMA_DESC_BLK_1024B		0x00070000
#define DMA_DESC_BLK_512B		0x00060000
#define DMA_DESC_BLK_256B		0x00050000
#define DMA_DESC_BLK_128B		0x00040000
#define DMA_DESC_BLK_64B		0x00030000
#define DMA_DESC_BLK_32B		0x00020000
#define DMA_DESC_BLK_16B		0x00010000
#define DMA_DESC_BLK_8B			0x00000000
#define DMA_DESC_ID			0x00000004
#define DMA_DESC_IE			0x00000002
#define DMA_DESC_ST			0x00000001

/* DMA_CHANX_STA_REG */
#define DMA_CHANX_STA_DM		0x80000000
#define DMA_CHANX_STA_OE		0x40000000
#define DMA_CHANX_STA_DA		0x20000000
#define DMA_CHANX_STA_DD		0x10000000
#define DMA_CHANX_STA_OD		0x08000000
#define DMA_CHANX_STA_ME		0x04000000
#define DMA_CHANX_STA_BE		0x02000000
#define DMA_CHANX_STA_RWE		0x01000000
#define DMA_CHANX_STA_AE		0x00800000
#define DMA_CHANX_STA_DN		0x00400000

/* DMA_INT_REG */
#define DMA_INT_CHAN(x)			(0x1 << (x))

/* ==========================================================================*/

#ifndef __ASSEMBLER__

#endif /* __ASSEMBLER__ */

/* ==========================================================================*/

#endif /* __PLAT_AMBARELLA_DMA_H__ */

