/*
 *
 * Author: Cao Rongrong <rrcao@ambarella.com>
 *
 * Copyright (C) 2012-2016, Ambarella, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/arm-smccc.h>
#include <asm/io.h>
#include <linux/of_address.h>
#include <soc/ambarella/ambarella_scm.h>

#if defined(CONFIG_ARCH_AMBARELLA_S6LM) || defined(CONFIG_ARCH_AMBARELLA_CV22) \
	|| defined(CONFIG_ARCH_AMBARELLA_CV2) || defined(CONFIG_ARCH_AMBARELLA_CV25) \
	|| defined(CONFIG_ARCH_AMBARELLA_CV2FS) || defined(CONFIG_ARCH_AMBARELLA_CV28) \
	|| defined(CONFIG_ARCH_AMBARELLA_CV5)
#define AMBARELLA_SECURE_PLATFORM	(1)
#else
#define AMBARELLA_SECURE_PLATFORM	(0)
#endif

#if (AMBARELLA_SECURE_PLATFORM > 0)
/*
 * gclk_coretex and gclk_core can be secure clock
 * other clock not be secure clock now.
*/
static uint32_t amba_clk_is_secure(const char *name)
{
	if ((0 == strcmp(name, "gclk_cortex")) \
		|| (0 == strcmp(name, "gclk_core")) \
		|| (0 == strcmp(name, "pll_out_core")))
		return 1;
	else
		return 0;
}

static int secure_boot_flag = -1;
int amba_secure_boot_detect(void)
{
	void __iomem *rct_base;
	struct device_node *np;

	if (secure_boot_flag >= 0)
		return secure_boot_flag;

	np = of_find_compatible_node(NULL, NULL, "ambarella,rct");
	if (!np)
		np = of_find_compatible_node(NULL, NULL, "syscon");

	if (!np) {
		secure_boot_flag = 0;
		return secure_boot_flag;
	}

	rct_base = of_iomap(np, 0);
	secure_boot_flag = !!(readl(rct_base + 0x34) & (1 << 6));
	iounmap(rct_base);

	return secure_boot_flag;
}
EXPORT_SYMBOL_GPL(amba_secure_boot_detect);

static uint32_t secure_clk_pll_set_rate(const char *name, unsigned long rate,
			unsigned long parent_rate)
{
	uint32_t fn, cmd;
	struct arm_smccc_res res;

	fn = SVC_SCM_FN(AMBA_SIP_SECURITY_CPUFREQ, 0);
	cmd = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64,
			ARM_SMCCC_OWNER_SIP, fn);

	if (0 == strcmp(name, "gclk_cortex"))
		arm_smccc_smc(cmd, 0, rate, parent_rate, 0, 0, 0, 0, &res);

	if ((0 == strcmp(name, "gclk_core")) \
		|| (0 == strcmp(name, "pll_out_core")))
		arm_smccc_smc(cmd, 1, rate, parent_rate, 0, 0, 0, 0, &res);

	if (res.a0 != 0)
		pr_err("change secure pll setttings error for [%s] \n", name);

	return AMBA_SIP_SECURITY_CPUFREQ;
}

extern int ambarella_smc_deployed(void);
uint32_t amba_secure_clk_pll_set_rate(const char *name,
			unsigned long rate, unsigned long parent_rate)
{
	unsigned long ret;

	if (!amba_clk_is_secure(name))
		return 0;

	if (!amba_secure_boot_detect())
		return 0;

	if (!ambarella_smc_deployed())
		return 0;

	ret = secure_clk_pll_set_rate(name, rate, parent_rate);
	return ret;
}

#else

uint32_t amba_secure_clk_pll_set_rate(const char *name,
			unsigned long rate, unsigned long parent_rate)
{
	return 0;
}
#endif
EXPORT_SYMBOL_GPL(amba_secure_clk_pll_set_rate);
