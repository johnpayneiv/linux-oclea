// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Linaro Ltd.
 *
 * Author: Cao Rongrong <rrcao@ambarella.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/sys_soc.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

/* ==========================================================================*/

#define AHB_CPUID_OFFSET		0x00

#define SYS_CONFIG_OFFSET		0x34
#define SOFT_OR_DLL_RESET_OFFSET	0x68

/* ==========================================================================*/

static struct ambarella_soc_id {
	const char *name;
	unsigned int id;
} soc_ids[] = {
	{ "s5l",   0x00483253 },
	{ "cv2",   0x00435632 },
	{ "cv22",  0x43563253 },
	{ "cv25",  0x43563245 },
	{ "s6lm",  0x00483245 },
	{ "cv28m", 0x4356324C },
	{ "cv5",   0x00435635 },
	{ "cv3",   0x00435636 },
};

static const char * __init ambarella_socinfo_soc_id(unsigned int soc_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(soc_ids); i++)
		if (soc_id == soc_ids[i].id)
			return soc_ids[i].name;
	return NULL;
}

static int __init ambarella_socinfo_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	struct device_node *np;
	struct regmap *cpuid_regmap;
	unsigned int soc_id;

	cpuid_regmap = syscon_regmap_lookup_by_compatible("ambarella,cpuid");
	if (IS_ERR(cpuid_regmap))
		return PTR_ERR(cpuid_regmap);

	regmap_read(cpuid_regmap, AHB_CPUID_OFFSET, &soc_id);

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENODEV;

	soc_dev_attr->family = "Ambarella SoC";

	np = of_find_node_by_path("/");
	of_property_read_string(np, "model", &soc_dev_attr->machine);
	of_node_put(np);

	soc_dev_attr->soc_id = ambarella_socinfo_soc_id(soc_id);
	if (!soc_dev_attr->soc_id) {
		pr_err("Unknown SoC\n");
		kfree(soc_dev_attr);
		return -ENODEV;
	}

	/* please note that the actual registration will be deferred */
	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree(soc_dev_attr);
		return PTR_ERR(soc_dev);
	}

	pr_info("Ambarella SoC %s detected\n", soc_dev_attr->soc_id);

	return 0;
}

static unsigned int ambsys_config;

unsigned int ambarella_sys_config(void)
{
	return ambsys_config;
}
EXPORT_SYMBOL(ambarella_sys_config);

static int __init ambarella_soc_init(void)
{
	struct regmap *rct_regmap;
	int ret;

	rct_regmap = syscon_regmap_lookup_by_compatible("ambarella,rct");
	if (IS_ERR(rct_regmap)) {
		pr_err("failed to get ambarella rct regmap\n");
		return PTR_ERR(rct_regmap);
	}

	regmap_read(rct_regmap, SYS_CONFIG_OFFSET, &ambsys_config);

	/* make sure software reboot bit is low, otherwise WDT cannot reset the chip */
	regmap_update_bits(rct_regmap, SOFT_OR_DLL_RESET_OFFSET, 0x1, 0x0);

	ret = ambarella_socinfo_init();
	if (ret < 0)
		return ret;

	return 0;
}

arch_initcall(ambarella_soc_init);

