/*
 * Ambarella SCM driver
 *
 * Copyright (C) 2017 Ambarella Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __AMBARELLA_SMC__
#define __AMBARELLA_SMC__

/* cpufreq svc ID 0x1 */
#define AMBA_SCM_SVC_FREQ			0x1
#define AMBA_SCM_CNTFRQ_SETUP_CMD		0x1

/* OTP operation */
#define AMBA_SIP_ACCESS_OTP			0x5
#define AMBA_SIP_GET_AMBA_UNIQUE_ID		0x1

#define AMBA_SCM_SVC_PM				0x2	/* Deprecated */
#define AMBA_SCM_PM_GPIO_SETUP			0x1	/* Deprecated */

#define AMBA_SIP_ACCESS_REG			0x3	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_READ		0x1	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_WRITE		0x2	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_SETBIT		0x3	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_CLRBIT		0x4	/* Deprecated */

#define AMBA_SIP_ACCESS_REG_READ8		0x5	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_READ16		0x6	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_READ32		0x7	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_READ64		0x8	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_WRITE8		0x9	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_WRITE16		0xa	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_WRITE32		0xb	/* Deprecated */
#define AMBA_SIP_ACCESS_REG_WRITE64		0xc	/* Deprecated */

/* XXX svc ID 0xff */
#define AMBA_SCM_SVC_QUERY			0xff
#define AMBA_SCM_QUERY_COUNT			0x00
#define AMBA_SCM_QUERY_UID			0x01
#define AMBA_SCM_QUERY_VERSION			0x03

#define	SVC_SCM_FN(s, f)			((((s) & 0xff) << 8) | ((f) & 0xff))

#define SVC_OF_SMC(c)				(((c) >> 8) & 0xff)
#define FNID_OF_SMC(c)				((c) & 0xff)
#endif
