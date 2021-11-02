// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 */
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <linux/of_reserved_mem.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

struct ambarella_rpmsg_dev {
	struct virtio_device vdev;
	struct virtqueue *vq[2];
	void *shmem;
	struct timer_list vbus_timer;
	struct work_struct work;
	struct regmap *reg_scr;
	int irq;
	int slave_hwirq;
};
/* ---------------------------------------------- */
static struct ambarella_rpmsg_dev *rpmsg_priv;

static u64 amba_rpmsg_get_features(struct virtio_device *vdev)
{
	return 1 << 0;
}
static int amba_rpmsg_finalize_features(struct virtio_device *vdev)
{
	return 0;
}

static void amba_rpmsg_del_vqs(struct virtio_device *vdev)
{
}

static bool amba_rpmsg_notify(struct virtqueue *vq)
{
	/* Finish a send operation at HOST side. */

	/* Send AXI software IRQ to slave */
	regmap_write(rpmsg_priv->reg_scr, 0x64, 1 << 0);
	return true;
}

static struct virtqueue *rp_find_vq(struct virtio_device *vdev,
		unsigned int id,
		void (*callback)(struct virtqueue *vq),
		const char *name, bool ctx)
{
	struct virtqueue *vq;
	vq = vring_new_virtqueue(id, 32, 64,
			vdev, true, ctx, rpmsg_priv->shmem + 0x100000 * id,
			amba_rpmsg_notify, callback, name);

	vq->priv = rpmsg_priv;
	rpmsg_priv->vq[id] = vq;

	return vq;
}

static int amba_rpmsg_find_vqs(struct virtio_device *vdev, unsigned nvqs,
		struct virtqueue *vqs[], vq_callback_t *callbacks[],
		const char * const names[], const bool *ctx,
		struct irq_affinity *desc)
{

	int i, ret, queue_idx = 0;

	for (i = 0; i < nvqs; ++i) {
		if (!names[i]) {
			vqs[i] = NULL;
			continue;
		}

		vqs[i] = rp_find_vq(vdev, queue_idx++, callbacks[i], names[i],
				ctx ? ctx[i] : false);
		if (IS_ERR(vqs[i])) {
			ret = PTR_ERR(vqs[i]);
			goto error;
		}
	}

	return 0;

error:
	amba_rpmsg_del_vqs(vdev);
	return ret;

	return 0;
}
static void amba_rpmsg_reset(struct virtio_device *vdev)
{
}
static u8 amba_rpmsg_get_status(struct virtio_device *vdev)
{
	return 0;
}
static void amba_rpmsg_set_status(struct virtio_device *vdev, u8 status)
{
}

static void ambarella_rpmsg_dev_release(struct device *dev)
{
}

static struct virtio_config_ops ambarella_rpmsg_config = {
	.get_features	= amba_rpmsg_get_features,
	.finalize_features = amba_rpmsg_finalize_features,
	.find_vqs	= amba_rpmsg_find_vqs,
	.del_vqs	= amba_rpmsg_del_vqs,
	.reset		= amba_rpmsg_reset,
	.set_status	= amba_rpmsg_set_status,
	.get_status	= amba_rpmsg_get_status,
};

static irqreturn_t ambarella_rpmsg_isr(int irq, void *data)
{
	struct ambarella_rpmsg_dev *priv = (struct ambarella_rpmsg_dev *)data;
	int hwirq = irqd_to_hwirq(irq_get_irq_data(irq));
	int value;

	regmap_read(priv->reg_scr, 0x64,  &value);

	/* ACK AXI software IRQ */
	regmap_write(priv->reg_scr, 0x68, 1 << (hwirq - priv->slave_hwirq));
	schedule_work(&priv->work);

	return IRQ_HANDLED;
}

static void ambarella_rpmsg_work_handler(struct work_struct *work)
{
	struct ambarella_rpmsg_dev *priv =
		container_of(work, struct ambarella_rpmsg_dev, work);

	vring_interrupt(0, priv->vq[0]);
}

static int amba_rpmsg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	struct irq_data *irqdata;
	int rval;

	rpmsg_priv = kzalloc(sizeof(struct ambarella_rpmsg_dev), GFP_KERNEL);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	rpmsg_priv->shmem = ioremap_wc(res->start, resource_size(res));
	if (!rpmsg_priv->shmem)
		return -ENODEV;

	memset_io(rpmsg_priv->shmem, 0, resource_size(res));

	rpmsg_priv->irq = platform_get_irq_byname(pdev, "host");
	if (rpmsg_priv->irq < 0) {
		dev_err(&pdev->dev, "get 'host' irq error\n");
		return -ENODEV;
	}

	rval = platform_get_irq_byname(pdev, "slave");
	if (rval < 0) {
		dev_err(&pdev->dev, "get 'slave' irq error\n");
		return -ENODEV;
	}

	irqdata = irq_get_irq_data(rval);
	if (IS_ERR_OR_NULL(irqdata)) {
		dev_err(&pdev->dev, "get irq data error\n");
		return -EINVAL;
	}
	rpmsg_priv->slave_hwirq = irqd_to_hwirq(irqdata);

	INIT_WORK(&rpmsg_priv->work, ambarella_rpmsg_work_handler);

	rval = devm_request_irq(&pdev->dev, rpmsg_priv->irq,
			ambarella_rpmsg_isr, 0,
			dev_name(&pdev->dev), rpmsg_priv);
	if (rval) {
		dev_err(&pdev->dev, "failed to request irq\n");
		return -ENODEV;
	}

	rval = of_reserved_mem_device_init_by_idx(dev, np, 0);
	if (rval) {
		dev_err(dev, "Can't associate reserved memory\n");
		return rval;
	}

	rpmsg_priv->reg_scr = syscon_regmap_lookup_by_phandle(np, "amb,scr-regmap");
	if (IS_ERR(rpmsg_priv->reg_scr)) {
		dev_err(&pdev->dev, "Can't find scr regmap\n");
		return -ENODEV;
	}

	rpmsg_priv->vdev.id.device = VIRTIO_ID_RPMSG;
	rpmsg_priv->vdev.config = &ambarella_rpmsg_config;
	rpmsg_priv->vdev.dev.parent = &pdev->dev;
	rpmsg_priv->vdev.dev.release = ambarella_rpmsg_dev_release;

	get_device(&pdev->dev);


	rval = register_virtio_device(&rpmsg_priv->vdev);
	if (rval)
		pr_err("register_virtio_device error %d\n", rval);

	/* Send AXI software IRQ to slave */
	regmap_write(rpmsg_priv->reg_scr, 0x64, 1 << 0);

	return rval;
}


static const struct of_device_id ambarella_rpmsg_dt_ids[] = {
	{ .compatible = "ambarella,rpmsg",},
	{ /* sentinel */ }
};

static struct platform_driver amba_rpmsg_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ambarella-rpmsg",
		.of_match_table = ambarella_rpmsg_dt_ids,
	},
	.probe = amba_rpmsg_probe,
};
static int __init amba_rpmsg_init(void)
{
	int ret;
	ret = platform_driver_register(&amba_rpmsg_driver);
	if (ret)
		pr_err("Unable to initialize rpmsg driver\n");
	else
		pr_info("Ambarella rpmsg driver is registered.\n");
	return ret;
}
MODULE_DESCRIPTION("Ambarella remote processor messaging virtio device");
MODULE_LICENSE("GPL v2");
subsys_initcall(amba_rpmsg_init);
