// SPDX-License-Identifier: GPL-2.0-only
/*
 * Power off driver for Oclea EFM8 MCU device.
 *
 * Copyright (c) 2023, Resideo Technologies Inc.  All rights reserved.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#define OCLEA_PMIC_REG_SHUTDOWN		0x30
#define OCLEA_PMIC_REG_WAKE_TIME	0x32
#define OCLEA_PMIC_REG_WAKE_REASON	0x34

#define OCLEA_PMIC_SHUTDOWN_VALUE	0xDEAD
#define OCLEA_PMIC_REBOOT_VALUE	    0xFEED

struct efm8_poweroff_data {
    struct i2c_client          *client;
};

static struct efm8_poweroff_data *pm_ctx;

static int reboot_handler(struct notifier_block *nb, unsigned long action, void *data)
{
    if (action == SYS_RESTART){
        i2c_smbus_write_word_data(pm_ctx->client, OCLEA_PMIC_REG_SHUTDOWN, OCLEA_PMIC_REBOOT_VALUE);
    }

    return NOTIFY_OK;
}

static struct notifier_block reboot_notifier = {
    .notifier_call = reboot_handler,
};

void efm8_poweroff_power_off(void)
{
    if (pm_ctx && pm_ctx->client) {
        i2c_smbus_write_word_data(pm_ctx->client, OCLEA_PMIC_REG_SHUTDOWN, OCLEA_PMIC_SHUTDOWN_VALUE);
    }

    for(;;);
}

static int efm8_poweroff_probe(struct i2c_client *client,
             const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct efm8_poweroff_data *dev_data;
    int status;

    dev_data = devm_kzalloc(dev, sizeof(struct efm8_poweroff_data), GFP_KERNEL);
    if (!dev_data) {
        return -ENOMEM;
    }

    dev_data->client = client;

    status = i2c_smbus_read_byte_data(client, OCLEA_PMIC_REG_WAKE_REASON);
    if (status < 0) {
        dev_err(dev, "Can't read pm, error: %d\n", status);
        return status;
    }

    i2c_set_clientdata(client, dev_data);
    dev_set_drvdata(dev, dev_data);
    pm_ctx = dev_data;
    pm_power_off = efm8_poweroff_power_off;
    register_reboot_notifier(&reboot_notifier);

    dev_info(dev, "%s: pm '%s'\n", dev_name(dev), client->name);

    return 0;
}

static int efm8_poweroff_remove(struct i2c_client *client)
{
    if (pm_power_off == efm8_poweroff_power_off) {
        pm_power_off = NULL;
    }

    unregister_reboot_notifier(&reboot_notifier);
    pm_ctx = NULL;

    return 0;
}

static const struct i2c_device_id efm8_poweroff_id[] = {
    {"efm8-poweroff", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, efm8_poweroff_id);

static const struct of_device_id efm8_poweroff_of_match[] = {
    {
        .compatible = "oclea,efm8-poweroff",
        .data = &efm8_poweroff_id[0]
    },
    {}
};
MODULE_DEVICE_TABLE(of, efm8_poweroff_of_match);

static struct i2c_driver efm8_poweroff_driver = {
    .probe = efm8_poweroff_probe,
    .remove = efm8_poweroff_remove,
    .id_table = efm8_poweroff_id,
    .driver = {
           .name = "efm8-poweroff",
           .of_match_table = of_match_ptr(efm8_poweroff_of_match)
    },
};

module_i2c_driver(efm8_poweroff_driver);

MODULE_DESCRIPTION("Power off driver for Oclea EFM8 PMIC Device");
MODULE_ALIAS("platform:efm8-power-off");
MODULE_AUTHOR("Caleb J");
MODULE_LICENSE("GPL v2");
