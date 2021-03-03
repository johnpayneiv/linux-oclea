/*
 * arch/arm/plat-ambarella/include/plat/uart.h
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

#ifndef __PLAT_AMBARELLA_UART_H__
#define __PLAT_AMBARELLA_UART_H__

#include <plat/chip.h>

/* ==========================================================================*/
#if (CHIP_REV == S2L) || (CHIP_REV == S3) || (CHIP_REV == S3L) || (CHIP_REV == S5)
#define	UART_INSTANCES			2
#elif (CHIP_REV == S5L)
#define	UART_INSTANCES			3
#elif (CHIP_REV == CV1)
#define	UART_INSTANCES			7
#else
#define	UART_INSTANCES			5
#endif

/* ==========================================================================*/
#if (CHIP_REV == S2L) || (CHIP_REV == S3) || (CHIP_REV == S3L) || \
	(CHIP_REV == S5) || (CHIP_REV == S5L)
#define UART0_OFFSET			0x5000
#else
#define UART0_OFFSET			0x0000
#endif

#if (CHIP_REV == S2L) || (CHIP_REV == S3) || (CHIP_REV == S3L) || \
	(CHIP_REV == S5) || (CHIP_REV == S5L)
#define UART1_OFFSET			0x32000
#elif (CHIP_REV == CV1) || (CHIP_REV == CV2)
#define UART1_OFFSET			0x16000
#else
#define UART1_OFFSET			0x17000
#endif

#if (CHIP_REV == S5L)
#define UART2_OFFSET			0x33000
#elif (CHIP_REV == CV1) || (CHIP_REV == CV2)
#define UART2_OFFSET			0x17000
#else
#define UART2_OFFSET			0x18000
#endif

#if (CHIP_REV == CV1) || (CHIP_REV == CV2)
#define UART3_OFFSET			0x18000
#else
#define UART3_OFFSET			0x19000
#endif

#if (CHIP_REV == CV1) || (CHIP_REV == CV2)
#define UART4_OFFSET			0x19000
#else
#define UART4_OFFSET			0x1A000
#endif

#define UART5_OFFSET			0x1A000
#define UART6_OFFSET			0x1B000

#define UART0_BASE			(APB_N_BASE + UART0_OFFSET)
#define UART1_BASE			(AHB_N_BASE + UART1_OFFSET)
#define UART2_BASE			(AHB_N_BASE + UART2_OFFSET)
#define UART3_BASE			(AHB_N_BASE + UART3_OFFSET)
#define UART4_BASE			(AHB_N_BASE + UART4_OFFSET)
#define UART5_BASE			(AHB_N_BASE + UART5_OFFSET)
#define UART6_BASE			(AHB_N_BASE + UART6_OFFSET)

#define UART0_REG(x)			(UART0_BASE + (x))
#define UART1_REG(x)			(UART1_BASE + (x))
#define UART2_REG(x)			(UART2_BASE + (x))
#define UART3_REG(x)			(UART3_BASE + (x))
#define UART4_REG(x)			(UART4_BASE + (x))
#define UART5_REG(x)			(UART5_BASE + (x))
#define UART6_REG(x)			(UART6_BASE + (x))
#define UART_REG(n, x)			((n) == 0 ? UART0_REG(x) : \
					 (n) == 1 ? UART1_REG(x) : \
					 (n) == 2 ? UART2_REG(x) : \
					 (n) == 3 ? UART3_REG(x) : \
					 (n) == 4 ? UART4_REG(x) : \
					 (n) == 5 ? UART5_REG(x) : \
						    UART6_REG(x))

/* ==========================================================================*/
#define UART_RB_OFFSET			0x00
#define UART_TH_OFFSET			0x00
#define UART_DLL_OFFSET			0x00
#define UART_IE_OFFSET			0x04
#define UART_DLH_OFFSET			0x04
#define UART_II_OFFSET			0x08
#define UART_FC_OFFSET			0x08
#define UART_LC_OFFSET			0x0c
#define UART_MC_OFFSET			0x10
#define UART_LS_OFFSET			0x14
#define UART_MS_OFFSET			0x18
#define UART_SC_OFFSET			0x1c	/* Byte */
#define UART_DMAE_OFFSET		0x28
#define UART_DMAF_OFFSET		0x40	/* DMA fifo */
#define UART_US_OFFSET			0x7c
#define UART_TFL_OFFSET			0x80
#define UART_RFL_OFFSET			0x84
#define UART_SRR_OFFSET			0x88

/* UART[x]_IE_REG */
#define UART_IE_PTIME			0x80
#define UART_IE_ETOI			0x20
#define UART_IE_EBDI			0x10
#define UART_IE_EDSSI			0x08
#define UART_IE_ELSI			0x04
#define UART_IE_ETBEI			0x02
#define UART_IE_ERBFI			0x01

/* UART[x]_II_REG */
#define UART_II_MODEM_STATUS_CHANGED	0x00
#define UART_II_NO_INT_PENDING		0x01
#define UART_II_THR_EMPTY		0x02
#define UART_II_RCV_DATA_AVAIL		0x04
#define UART_II_RCV_STATUS		0x06
#define UART_II_CHAR_TIMEOUT		0x0c

/* UART[x]_FC_REG */
#define UART_FC_RX_ONECHAR		0x00
#define UART_FC_RX_QUARTER_FULL		0x40
#define UART_FC_RX_HALF_FULL		0x80
#define UART_FC_RX_2_TO_FULL		0xc0
#define UART_FC_TX_EMPTY		0x00
#define UART_FC_TX_2_IN_FIFO		0x10
#define UART_FC_TX_QUATER_IN_FIFO	0x20
#define UART_FC_TX_HALF_IN_FIFO		0x30
#define UART_FC_XMITR			0x04
#define UART_FC_RCVRR			0x02
#define UART_FC_FIFOE			0x01

/* UART[x]_LC_REG */
#define UART_LC_DLAB			0x80
#define UART_LC_BRK			0x40
#define UART_LC_EVEN_PARITY		0x10
#define UART_LC_ODD_PARITY		0x00
#define UART_LC_PEN			0x08
#define UART_LC_STOP_2BIT		0x04
#define UART_LC_STOP_1BIT		0x00
#define UART_LC_CLS_8_BITS		0x03
#define UART_LC_CLS_7_BITS		0x02
#define UART_LC_CLS_6_BITS		0x01
#define UART_LC_CLS_5_BITS		0x00
/*	quick defs */
#define	UART_LC_8N1			0x03
#define	UART_LC_7E1			0x0a

/* UART[x]_MC_REG */
#define UART_MC_SIRE			0x40
#define UART_MC_AFCE			0x20
#define UART_MC_LB			0x10
#define UART_MC_OUT2			0x08
#define UART_MC_OUT1			0x04
#define UART_MC_RTS			0x02
#define UART_MC_DTR			0x01

/* UART[x]_LS_REG */
#define UART_LS_FERR			0x80
#define UART_LS_TEMT			0x40
#define UART_LS_THRE			0x20
#define UART_LS_BI			0x10
#define UART_LS_FE			0x08
#define UART_LS_PE			0x04
#define UART_LS_OE			0x02
#define UART_LS_DR			0x01

/* UART[x]_MS_REG */
#define UART_MS_DCD			0x80
#define UART_MS_RI			0x40
#define UART_MS_DSR			0x20
#define UART_MS_CTS			0x10
#define UART_MS_DDCD			0x08
#define UART_MS_TERI			0x04
#define UART_MS_DDSR			0x02
#define UART_MS_DCTS			0x01

/* UART[x]_US_REG */
#define UART_US_RFF			0x10
#define UART_US_RFNE			0x08
#define UART_US_TFE			0x04
#define UART_US_TFNF			0x02
#define UART_US_BUSY			0x01

/* ==========================================================================*/
#define UART_FIFO_SIZE			(16)

#define DEFAULT_AMBARELLA_UART_MCR	(0)
#define DEFAULT_AMBARELLA_UART_IER	(UART_IE_ELSI | UART_IE_ERBFI | UART_IE_ETOI)

#endif /* __PLAT_AMBARELLA_UART_H__ */

