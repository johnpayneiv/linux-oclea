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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/arm-smccc.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/freezer.h>
#include <soc/ambarella/ambarella_scm.h>

static int ambarella_scm_query(void)
{
	u32 fn;
	u32 cmd;
	struct arm_smccc_res res;

	fn = SVC_SCM_FN(AMBA_SCM_SVC_QUERY, AMBA_SCM_QUERY_VERSION);
	cmd = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64,
			ARM_SMCCC_OWNER_SIP, fn);

	arm_smccc_smc(cmd, 0, 0, 0, 0, 0, 0, 0, &res);

	return res.a0;
}

int ambarella_aarch64_cntfrq_update(void)
{
	u32 fn;
	u32 cmd;
	struct arm_smccc_res res;

	fn = SVC_SCM_FN(AMBA_SCM_SVC_FREQ, AMBA_SCM_CNTFRQ_SETUP_CMD);
	cmd = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64,
			ARM_SMCCC_OWNER_SIP, fn);

	arm_smccc_smc(cmd, 0, 0, 0, 0, 0, 0, 0, &res);

	return res.a0;
}
EXPORT_SYMBOL(ambarella_aarch64_cntfrq_update);

/*
 *  Ambarella memory monitor
 */
int ambarella_scm_monitor_config(size_t addr, uint32_t length, uint32_t mode)
{
	u32 fn, cmd;
	struct arm_smccc_res res;
#ifdef FREEZE_SYSTEM
	int error;
	error = freeze_processes();
	if (error)
		return -EBUSY;
	error = freeze_kernel_threads();
	if (error) {
		thaw_processes();
		return -EBUSY;
	}
#endif

	fn = SVC_SCM_FN(AMBA_SIP_MEMORY_MONITOR, AMBA_SIP_MONITOR_CONFIG);
	cmd = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64,
			ARM_SMCCC_OWNER_SIP, fn);

	arm_smccc_smc(cmd, addr, length, mode, 0, 0, 0, 0, &res);

#ifdef FREEZE_SYSTEM
	thaw_processes();
	thaw_kernel_threads();
#endif

	return res.a0;
}
EXPORT_SYMBOL(ambarella_scm_monitor_config);

int ambarella_scm_monitor_enable(size_t addr, uint32_t length, uint32_t mode)
{
	u32 fn, cmd;
	struct arm_smccc_res res;
#ifdef FREEZE_SYSTEM
	int error;
	error = freeze_processes();
	if (error)
		return -EBUSY;
	error = freeze_kernel_threads();
	if (error) {
		thaw_processes();
		return -EBUSY;
	}
#endif

	fn = SVC_SCM_FN(AMBA_SIP_MEMORY_MONITOR, AMBA_SIP_MONITOR_ENABLE);
	cmd = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64,
			ARM_SMCCC_OWNER_SIP, fn);

	arm_smccc_smc(cmd, addr, length, mode, 0, 0, 0, 0, &res);

#ifdef FREEZE_SYSTEM
	thaw_processes();
	thaw_kernel_threads();
#endif

	return res.a0;
}
EXPORT_SYMBOL(ambarella_scm_monitor_enable);

int ambarella_scm_monitor_disable(size_t addr, uint32_t length, uint32_t mode)
{
	u32 fn, cmd;
	struct arm_smccc_res res;
#ifdef FREEZE_SYSTEM
	int error;
	error = freeze_processes();
	if (error)
		return -EBUSY;
	error = freeze_kernel_threads();
	if (error) {
		thaw_processes();
		return -EBUSY;
	}
#endif

	fn = SVC_SCM_FN(AMBA_SIP_MEMORY_MONITOR, AMBA_SIP_MONITOR_DISABLE);
	cmd = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64,
			ARM_SMCCC_OWNER_SIP, fn);

	arm_smccc_smc(cmd, addr, length, mode, 0, 0, 0, 0, &res);

#ifdef FREEZE_SYSTEM
	thaw_processes();
	thaw_kernel_threads();
#endif

	return res.a0;
}
EXPORT_SYMBOL(ambarella_scm_monitor_disable);

/* Software Reset VP cluster */
int ambarella_scm_soft_reset_vp(void)
{
	u32 fn, cmd;
	struct arm_smccc_res res;

	fn = SVC_SCM_FN(AMBA_SIP_VP_CONFIG, AMBA_SIP_VP_CONFIG_RESET);
	cmd = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64,
			ARM_SMCCC_OWNER_SIP, fn);

	arm_smccc_smc(cmd, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		return -EINVAL;

	return res.a0;
}
EXPORT_SYMBOL(ambarella_scm_soft_reset_vp);

/* uuidbuf space at least 128bits */
int ambarella_otp_get_uuid(u32 *uuidbuf)
{
	u32 fn;
	u32 cmd;
	struct arm_smccc_res res;

	if (!uuidbuf) {
		return -EINVAL;
	}

	fn = SVC_SCM_FN(AMBA_SIP_ACCESS_OTP, AMBA_SIP_GET_AMBA_UNIQUE_ID);
	cmd = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64,
			ARM_SMCCC_OWNER_SIP, fn);

	arm_smccc_smc(cmd, 0, 0, 0, 0, 0, 0, 0, &res);

	if (res.a0) {
		return -EINVAL;
	}

	uuidbuf[0] = res.a1 & 0xFFFFFFFF;
	uuidbuf[1] = (res.a1 >> 32) & 0xFFFFFFFF;
	uuidbuf[2] = res.a2 & 0xFFFFFFFF;
	uuidbuf[3] = (res.a2 >> 32) & 0xFFFFFFFF;

	return res.a0;
}
EXPORT_SYMBOL(ambarella_otp_get_uuid);

/* ---------------------------------------------------------------------------- */
static int smc_code_avail = -1;
int ambarella_smc_deployed(void)
{
	int len;
	const char *method;
	struct device_node *node;

	if (smc_code_avail >= 0)
		return smc_code_avail;

	node = of_find_node_by_name(NULL, "psci");
	if (!node) {
		smc_code_avail  = 0;
		return smc_code_avail;
	}

	method = of_get_property(node, "method", &len);
	if (method == NULL) {
		smc_code_avail = 0;
		return smc_code_avail;
	}

	if (strncmp(method, "smc", 3))
		smc_code_avail = 0;
	else
		smc_code_avail = 1;

	return smc_code_avail;
}
EXPORT_SYMBOL_GPL(ambarella_smc_deployed);

int __init ambarella_scm_init(void)
{
	if (!ambarella_smc_deployed())
		return 0;

	BUG_ON(ambarella_scm_query() != ARM_SMCCC_SMC_64);

	return 0;
}
arch_initcall(ambarella_scm_init);
