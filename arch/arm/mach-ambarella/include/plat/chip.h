/*
 * arch/arm/plat-ambarella/include/plat/chip.h
 *
 * Author: Anthony Ginger <hfjiang@ambarella.com>
 *
 * Copyright (C) 2004-2013, Ambarella, Inc.
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

#ifndef __PLAT_AMBARELLA_CHIP_H__
#define __PLAT_AMBARELLA_CHIP_H__

/* ==========================================================================*/
#define S2L		(12000)
#define S3		(11000)
#define S3L		(13000)
#define S5		(15000)
#define S5L		(16000)
#define S6LM		(17000)
#define CV1		(20000)
#define CV2		(21000)
#define CV22		(22000)
#define CV25		(25000)
#define CV2FS		(26000)
#define CV28		(27000)
#define CV5		(30000)

#define CHIP_ID(x)	((x / 1000))
#define CHIP_MAJOR(x)	((x / 100) % 10)
#define CHIP_MINOR(x)	((x / 10) % 10)

#if defined(CONFIG_ARCH_AMBARELLA_S2L)
#define CHIP_REV	S2L
#elif defined(CONFIG_ARCH_AMBARELLA_S3)
#define CHIP_REV	S3
#elif defined(CONFIG_ARCH_AMBARELLA_S3L)
#define CHIP_REV	S3L
#elif defined(CONFIG_ARCH_AMBARELLA_S5)
#define CHIP_REV	S5
#elif defined(CONFIG_ARCH_AMBARELLA_S5L)
#define CHIP_REV	S5L
#elif defined(CONFIG_ARCH_AMBARELLA_S6LM)
#define CHIP_REV	S6LM
#elif defined(CONFIG_ARCH_AMBARELLA_CV1)
#define CHIP_REV	CV1
#elif defined(CONFIG_ARCH_AMBARELLA_CV2)
#define CHIP_REV	CV2
#elif defined(CONFIG_ARCH_AMBARELLA_CV22)
#define CHIP_REV	CV22
#elif defined(CONFIG_ARCH_AMBARELLA_CV25)
#define CHIP_REV	CV25
#elif defined(CONFIG_ARCH_AMBARELLA_CV2FS)
#define CHIP_REV	CV2FS
#elif defined(CONFIG_ARCH_AMBARELLA_CV28)
#define CHIP_REV	CV28
#elif defined(CONFIG_ARCH_AMBARELLA_CV5)
#define CHIP_REV	CV5
#else
#error "Undefined CHIP_REV"
#endif

/* ==========================================================================*/

#if (CHIP_REV == S3)
#define	CHIP_FIX_2NDCORTEX_BOOT	1
#endif

/* ==========================================================================*/

#ifndef __ASSEMBLER__

extern struct proc_dir_entry *get_ambarella_proc_dir(void);
extern struct dentry *get_ambarella_debugfs_dir(void);
extern unsigned int ambarella_sys_config(void);

#endif

/* ==========================================================================*/

#endif /* __PLAT_AMBARELLA_CHIP_H__ */

