// SPDX-License-Identifier: GPL-2.0
/*
 * FPGA cfg device instantiation for fpga-cfg driver autoload
 *
 * Copyright (C) 2018 DENX Software Engineering
 * Anatolij Gustschin <agust@denx.de>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

static int __init fpga_cfg_loader_init(void)
{
	static struct platform_device *pdev;

	pdev = platform_device_register_simple("fpga-cfg-dev",
						PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("Can't register platform device: %ld\n", PTR_ERR(pdev));
		return PTR_ERR(pdev);
	}

	return 0;
}
subsys_initcall(fpga_cfg_loader_init);
