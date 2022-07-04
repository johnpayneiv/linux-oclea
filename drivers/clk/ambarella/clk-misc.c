/*
 * arch/arm/mach-ambarella/clk.c
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
#include <linux/list.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/uaccess.h>
#include <plat/rct.h>

static const char *gclk_names[] = {
	"pll_out_core", "pll_out_sd", "pll_out_hdmi", "pll_out_vo2", "pll_out_enet",
	"pll_out_video_a", "pll_out_video_b", "gclk_cortex", "gclk_cortex0", "gclk_cortex1",
	"gclk_axi", "smp_twd", "gclk_ddr", "gclk_ddr0", "gclk_ddr1", "gclk_core", "gclk_ahb",
	"gclk_apb", "gclk_idsp", "gclk_idspv", "gclk_so", "gclk_so2", "gclk_vo2", "gclk_vo",
	"gclk_vo_a", "gclk_vo_b", "gclk_nand", "gclk_sdxc", "gclk_sdio", "gclk_sd", "gclk_sd0",
	"gclk_sd1", "gclk_sd2", "gclk_uart", "gclk_uart0", "gclk_uart1", "gclk_uart2",
	"gclk_uart3", "gclk_uart4", "gclk_uart5", "gclk_uart6", "gclk_audio",
	"gclk_audio_aux", "gclk_ir", "gclk_adc", "gclk_ssi", "gclk_ssi2", "gclk_ssi3",
	"gclk_pwm", "gclk_stereo", "gclk_vision", "gclk_fex", "pll_out_slvsec",
};

/* simple guard to check clk freq */
struct safefreq_limitor {
	const char *name;
	unsigned long max; /* Hz */
	unsigned long min;
};

static struct safefreq_limitor gclk_safefreq_limitor[] = {
	[0] = {
		.name = "gclk_cortex",
		.max = 0,
		.min = 0,
	},

	[1] = {
		.name = "gclk_core",
		.max = 0,
		.min = 0,
	},
};

static void cortex_core_clk_limitor_init(void)
{
	struct device_node *np;
	struct property *prop;
	const __be32 *reg;
	u32 value, i = 0;

	np = of_find_compatible_node(NULL, NULL, "ambarella,cpufreq");
	if (!np)
		return;

	of_property_for_each_u32(np, "clocks-frequency-cortex-core",
			prop, reg, value) {
		if (gclk_safefreq_limitor[i % 2].min) {
			if (gclk_safefreq_limitor[i % 2].min > value)
				gclk_safefreq_limitor[i % 2].min = value;
			if (gclk_safefreq_limitor[i % 2].max < value)
				gclk_safefreq_limitor[i % 2].max = value;
		} else {
			gclk_safefreq_limitor[i % 2].min = value;
			gclk_safefreq_limitor[i % 2].max = value;
		}

		i++;
	}

	gclk_safefreq_limitor[0].min *= 1000;
	gclk_safefreq_limitor[0].max *= 1000;
	gclk_safefreq_limitor[1].min *= 1000;
	gclk_safefreq_limitor[1].max *= 1000;
}

static unsigned long ambarella_safefreq_check(const char *name, unsigned long freq)
{
	int i ,limit, match = 0;

	limit = sizeof(gclk_safefreq_limitor)/sizeof(gclk_safefreq_limitor[0]);

	for (i = 0; i < limit; i++) {
		if (!gclk_safefreq_limitor[i].name)
			continue;

		if (strcmp(name, gclk_safefreq_limitor[i].name))
			continue;

		match = 1;
		break;
	}

	if (!match) /* doesn't care */
		return freq;

	if (!gclk_safefreq_limitor[i].min || !gclk_safefreq_limitor[i].max)
		return freq; /* no rule to check */

	if ((freq >= gclk_safefreq_limitor[i].min) && (freq <= gclk_safefreq_limitor[i].max))
		return freq; /* pass */
	else
		return 0; /* freq = 0, invalid */
}

static void ambarella_safefreq_limitor_init(void)
{
	cortex_core_clk_limitor_init();
}

static int ambarella_clock_proc_show(struct seq_file *m, void *v)
{
	struct clk *gclk;
	int i;

	seq_printf(m, "\nClock Information:\n");
	for (i = 0; i < ARRAY_SIZE(gclk_names); i++) {
		gclk = clk_get_sys(NULL, gclk_names[i]);
		if (IS_ERR(gclk))
			continue;

		seq_printf(m, "\t%s:\t%ld Hz\n",
			__clk_get_name(gclk), clk_get_rate(gclk));

		clk_put(gclk);
	}

	return 0;
}

static ssize_t ambarella_clock_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *ppos)
{
	struct clk *gclk;
	char *buf, clk_name[32];
	unsigned long freq;
	int rval = count;

	pr_warn("!!!DANGEROUS!!! You must know what you are doing!\n");

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		rval = -EFAULT;
		goto exit;
	}

	sscanf(buf, "%s %ld", clk_name, &freq);
	if (!freq)
		goto exit;

	if (!strcmp(clk_name, "gclk_ddr"))
		goto exit;

	gclk = clk_get_sys(NULL, clk_name);
	if (IS_ERR(gclk)) {
		pr_err("Invalid clk name\n");
		rval = -EINVAL;
		goto exit;
	}

	freq = ambarella_safefreq_check(clk_name, freq);
	if (freq)
		clk_set_rate(gclk, freq);

	clk_put(gclk);
exit:
	kfree(buf);
	return rval;
}

static int ambarella_clock_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ambarella_clock_proc_show, PDE_DATA(inode));
}

static const struct file_operations proc_clock_fops = {
	.owner = THIS_MODULE,
	.open = ambarella_clock_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = ambarella_clock_proc_write,
	.release = single_release,
};

static int __init ambarella_init_clk(void)
{
	proc_create_data("clock", S_IRUGO, get_ambarella_proc_dir(),
		&proc_clock_fops, NULL);

	ambarella_safefreq_limitor_init();

	return 0;
}

late_initcall(ambarella_init_clk);

