/*
 * arch/arm/mach-ambarella/sleep.S
 *
 * Author: Cao Rongrong <rrcao@ambarella.com>
 *
 * Copyright (C) 2014-2016, Ambarella, Inc.
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

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/cache.h>
#include <mach/hardware.h>
#include <plat/rct.h>
#include <plat/uart.h>

#define REF_CLK_FREQ			24000000

#define DRAM_DDRC_OFFSET  		0x0800
#define DDRC_CTL_OFFSET			(0x00)
#define DDRC_SELF_REF_OFFSET		(0x1C)

	.text

#ifdef CONFIG_AMBARELLA_SREF_FIFO_EXEC
ENTRY(ambarella_optimize_suspend)
	dsb
	isb

	mov	r1, #0
	cmp	r0, r1
	beq	suspend_with_pwc

	mov	r1, r0
	mov	r2, #0x1f
	bic	r0, r2			/* r0 gpio bank base */
	and	r1, r2			/* r1 gpio offset */

	/* gpio pull down */
	mov	r2, #0
	str	r2, [r0, #0]

	/* delay 100ms: 0x249F00 is about 0x250000 */
	mov	r3, #0x250000

	mov	r4, #RCT_BUS_BASE
	orr	r4, #RCT_OFFSET
	orr	r4, #RCT_TIMER_OFFSET
	ldr	r5, [r4]
	add	r5, r5, r3
rct_time_loop:
	ldr	r6, [r4]
	cmp	r5, r6
	bhi	rct_time_loop

	b	self_refresh

suspend_with_pwc:
	mov	r0, #RCT_BUS_BASE
	orr	r0, #RCT_OFFSET
	orr	r0, #ANA_PWR_OFFSET

	/* Generate the PD signal */
	mov	r1, #0x20
	str	r1, [r0]

self_refresh:
	/* CPU use the delay interval between power-down signal PD
	   and PWC_RSTOB to execute the following code */

	mov	r0, #DRAMC_BASE
	orr	r0, r0, #DRAM_DDRC_OFFSET

	ldr	r1, [r0, #DDRC_CTL_OFFSET]
	bic	r1, r1, #0x1
	str	r1, [r0, #DDRC_CTL_OFFSET]
	mov	r1, #0x80000000
	str	r1, [r0, #DDRC_SELF_REF_OFFSET]
1:
	ldr	r1, [r0, #DDRC_SELF_REF_OFFSET]
	tst	r1, #0x10000000
	beq	1b

	/* Wait for PWC_RSTOB signal to power down cpu */
	b	.

ENDPROC(ambarella_optimize_suspend)

ENTRY(ambarella_optimize_suspend_sz)
	.word 	. - ambarella_optimize_suspend
#endif

#if (CHIP_REV == S3)
#define	IOMUX_UART0_PIN_CFG	0x00000030
#else
#define	IOMUX_UART0_PIN_CFG	0x00000180
#endif

ENTRY(ambarella_cpu_resume)
	/*
	 * init UART(115200, 8N1) to avoid deadloop in printk.
	 * MMU is not enabled yet, so we use physical address here.
	 */
	ldr	r0, =DBGBUS_PHYS_BASE
	mov	r1, #0x1
	str	r1, [r0, #CG_UART0_OFFSET]

	ldr	r0, =(APB_PHYS_BASE + UART0_OFFSET)
	mov	r1, #0x1
	str	r1, [r0, #UART_SRR_OFFSET]
	mov	r1, #0x0
	str	r1, [r0, #UART_SRR_OFFSET]
	mov	r1, #0x80
	str	r1, [r0, #UART_LC_OFFSET]
	ldr	r1, =(REF_CLK_FREQ/16/115200)
	str	r1, [r0, #UART_DLL_OFFSET]
	mov	r1, #0x00
	str	r1, [r0, #UART_DLH_OFFSET]
	mov	r1, #0x03
	str	r1, [r0, #UART_LC_OFFSET]

	/* configure Tx/Rx pin as hw mode, it's chip specific. */

	/* gpio39/40 are used for UART0_rx/tx */
	ldr	r0, =(APB_PHYS_BASE + IOMUX_OFFSET)
	/* read-modify-write */
	ldr	r1, [r0, #IOMUX_REG1_0_OFFSET]
	orr	r1, r1, #IOMUX_UART0_PIN_CFG
	str	r1, [r0, #IOMUX_REG1_0_OFFSET]
	/* read-modify-write */
	ldr	r1, [r0, #IOMUX_REG1_1_OFFSET]
	bic	r1, r1, #IOMUX_UART0_PIN_CFG
	str	r1, [r0, #IOMUX_REG1_1_OFFSET]
	/* read-modify-write */
	ldr	r1, [r0, #IOMUX_REG1_2_OFFSET]
	bic	r1, r1, #IOMUX_UART0_PIN_CFG
	str	r1, [r0, #IOMUX_REG1_2_OFFSET]
	ldr	r1, =0x00000001
	str	r1, [r0, #IOMUX_CTRL_SET_OFFSET]
	ldr	r1, =0x00000000
	str	r1, [r0, #IOMUX_CTRL_SET_OFFSET]

	/* jump to generic resume */
#ifdef CONFIG_THUMB2_KERNEL
	b	cpu_resume_arm
#else
	b	cpu_resume
#endif

ENDPROC(ambarella_cpu_resume)

