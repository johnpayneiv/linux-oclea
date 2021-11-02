/*
 *
 * Author: Cao Rongrong <rrcao@ambarella.com>
 *
 * Copyright (C) 2012-2016, Ambarella, Inc.
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>

static struct proc_dir_entry *ambarella_proc_dir = NULL;
static struct dentry *ambarella_debugfs_dir = NULL;

struct proc_dir_entry *get_ambarella_proc_dir(void)
{
	return ambarella_proc_dir;
}
EXPORT_SYMBOL(get_ambarella_proc_dir);

struct dentry *get_ambarella_debugfs_dir(void)
{
	return ambarella_debugfs_dir;
}
EXPORT_SYMBOL(get_ambarella_debugfs_dir);

static int __init ambarella_init_root_dir(void)
{
	ambarella_proc_dir = proc_mkdir("ambarella", NULL);
	if (IS_ERR_OR_NULL(ambarella_proc_dir)) {
		pr_err("failed to create ambarella root proc dir\n");
		return -ENOMEM;
	}

#if defined(CONFIG_DEBUG_FS)
	ambarella_debugfs_dir = debugfs_create_dir("ambarella", NULL);
	if (IS_ERR_OR_NULL(ambarella_debugfs_dir)) {
		pr_err("failed to create ambarella root debugfs dir\n");
		return -ENOMEM;
	}
#endif

	return 0;
}
core_initcall(ambarella_init_root_dir);

/* Shutdown code for socs with pmic */

static struct i2c_board_info pmic_info = {
	/* Use placeholder address until after i2c_new_device to avoid resource
	   busy error from dts entry */
	I2C_BOARD_INFO("pmic_mp5424", 0x67),
};

void ambarella_power_off(void)
{
	struct i2c_adapter *i2c_adap1;
	struct i2c_client *i2c_dev;
	struct device_node *np;
	struct device_node *i2c_node;
	int i2c_bus = -1;
	int i2c_addr = 0x69;

	np = of_find_node_by_name(NULL, "pmic_mp5424");
	if (!np) {
		printk("Could not find pmic dts node\n");
		for(;;);
	}

	if (of_property_read_u32(np, "reg", &i2c_addr) != 0) {
		printk("Coult not find pmic address\n");
		for(;;);
	}

	i2c_node = of_get_parent(np);
	i2c_bus = of_alias_get_id(i2c_node, "i2c");
	i2c_adap1 = i2c_get_adapter(i2c_bus);
	i2c_dev = i2c_new_device(i2c_adap1, &pmic_info);
	if (NULL == i2c_dev) {
		printk("No pmic devices found\n");
		for(;;);
	}

	i2c_dev->addr = i2c_addr;
	i2c_smbus_write_byte_data(i2c_dev, 0x22, 0x00);
	for(;;);
}

void ambarella_power_off_prepare(void)
{

}

static int __init ambarella_init_pm(void)
{
	pm_power_off = ambarella_power_off;
	pm_power_off_prepare = ambarella_power_off_prepare;

	return 0;
}

arch_initcall(ambarella_init_pm);
