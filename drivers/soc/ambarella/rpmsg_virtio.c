// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020-2048, Ambarella, Inc.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/rpmsg.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <linux/remoteproc.h>
#include <linux/of_reserved_mem.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define AMBSP_SET_INTERRUPT_REG		0x64
#define AMBSP_CLR_INTERRUPT_REG		0x68

#define AMBSP_AXI_SWIRQ_SLAVE		BIT(0)	/* host to slave */
#define AMBSP_AXI_SWIRQ_HOST		BIT(1)	/* slave to host */

struct shared_resource_table {
	/* rpmsg vdev entry */
	struct fw_rsc_hdr hdr;
	struct fw_rsc_vdev rsc;
};

struct ambarella_rpmsg_dev {
	struct virtio_device vdev;
	struct virtqueue *vq[2];
	struct regmap *reg_scr;
	void *vring_mem;
	struct shared_resource_table *rsrc_tbl;
	void *vq_mem[2];
	struct work_struct work;
};

#define to_ambarella_rpmsg(d)	container_of(d, struct ambarella_rpmsg_dev, vdev)

/* ---------------------------------------------- */

static void ambarella_rpmsg_work_handler(struct work_struct *work)
{
	struct ambarella_rpmsg_dev *rpmsg_dev =
			container_of(work, struct ambarella_rpmsg_dev, work);

	vring_interrupt(0, rpmsg_dev->vq[0]);
}

static irqreturn_t ambarella_rpmsg_isr(int irq, void *data)
{
	struct ambarella_rpmsg_dev *rpmsg_dev = (struct ambarella_rpmsg_dev *)data;

	/* ACK AXI software IRQ */
	regmap_write(rpmsg_dev->reg_scr, AMBSP_CLR_INTERRUPT_REG, AMBSP_AXI_SWIRQ_HOST);

	schedule_work(&rpmsg_dev->work);

	return IRQ_HANDLED;
}

static bool ambarella_rpmsg_notify(struct virtqueue *vq)
{
	struct ambarella_rpmsg_dev *rpmsg_dev = vq->priv;

	/* Send AXI software IRQ to slave */
	regmap_write(rpmsg_dev->reg_scr, AMBSP_SET_INTERRUPT_REG, AMBSP_AXI_SWIRQ_SLAVE);

	return true;
}

static void ambarella_rpmsg_del_vqs(struct virtio_device *vdev)
{
	struct ambarella_rpmsg_dev *rpmsg_dev = to_ambarella_rpmsg(vdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(rpmsg_dev->vq); i++) {
		if (rpmsg_dev->vq[i]) {
			vring_del_virtqueue(rpmsg_dev->vq[i]);
			rpmsg_dev->vq[i] = NULL;
		}
	}
}

static int ambarella_rpmsg_find_vqs(struct virtio_device *vdev, unsigned nvqs,
		struct virtqueue *vqs[], vq_callback_t *callbacks[],
		const char * const names[], const bool *ctx,
		struct irq_affinity *desc)
{
	struct ambarella_rpmsg_dev *rpmsg_dev = to_ambarella_rpmsg(vdev);
	struct virtqueue *vq;
	u32 i;

	for (i = 0; i < nvqs; i++) {
		if (!names[i] || i >= ARRAY_SIZE(rpmsg_dev->vq)) {
			vqs[i] = NULL;
			continue;
		}

		memset_io(rpmsg_dev->vq_mem[i], 0, vring_size(32, 64));

		vq = vring_new_virtqueue(i, 32, 64, vdev, false, false,
				rpmsg_dev->vq_mem[i], ambarella_rpmsg_notify,
				callbacks[i], names[i]);
		if (!vq) {
			ambarella_rpmsg_del_vqs(vdev);
			return -ENOMEM;
		}

		vq->priv = rpmsg_dev;

		vqs[i] = rpmsg_dev->vq[i] = vq;
	}

	return 0;
}

static void ambarella_rpmsg_reset(struct virtio_device *vdev)
{
	struct ambarella_rpmsg_dev *rpmsg_dev = to_ambarella_rpmsg(vdev);

	rpmsg_dev->rsrc_tbl->rsc.status = 0;
}

static u8 ambarella_rpmsg_get_status(struct virtio_device *vdev)
{
	struct ambarella_rpmsg_dev *rpmsg_dev = to_ambarella_rpmsg(vdev);

	return rpmsg_dev->rsrc_tbl->rsc.status;
}

static void ambarella_rpmsg_set_status(struct virtio_device *vdev, u8 status)
{
	struct ambarella_rpmsg_dev *rpmsg_dev = to_ambarella_rpmsg(vdev);

	rpmsg_dev->rsrc_tbl->rsc.status = status;
}

static u64 ambarella_rpmsg_get_features(struct virtio_device *vdev)
{
	struct ambarella_rpmsg_dev *rpmsg_dev = to_ambarella_rpmsg(vdev);

	return rpmsg_dev->rsrc_tbl->rsc.dfeatures;
}

static int ambarella_rpmsg_finalize_features(struct virtio_device *vdev)
{
	struct ambarella_rpmsg_dev *rpmsg_dev = to_ambarella_rpmsg(vdev);

	/* Give virtio_ring a chance to accept features. */
	vring_transport_features(vdev);

	/*
	 * Remember the finalized features of our vdev, and provide it
	 * to the remote processor once it is powered on.
	 */
	rpmsg_dev->rsrc_tbl->rsc.gfeatures = vdev->features;

	return 0;
}

static void ambarella_rpmsg_dev_release(struct device *dev)
{
}

static struct virtio_config_ops ambarella_rpmsg_config = {
	.get_features		= ambarella_rpmsg_get_features,
	.finalize_features	= ambarella_rpmsg_finalize_features,
	.find_vqs		= ambarella_rpmsg_find_vqs,
	.del_vqs		= ambarella_rpmsg_del_vqs,
	.reset			= ambarella_rpmsg_reset,
	.set_status		= ambarella_rpmsg_set_status,
	.get_status		= ambarella_rpmsg_get_status,
};

static int ambarella_rpmsg_probe(struct platform_device *pdev)
{
	struct ambarella_rpmsg_dev *rpmsg_dev;
	struct device *dev = &pdev->dev;
	struct resource *res;
	size_t size = vring_size(32, 64);
	int irq, rval;

	rpmsg_dev = devm_kzalloc(dev, sizeof(*rpmsg_dev), GFP_KERNEL);
	if (!rpmsg_dev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no resource\n");
		return -ENOMEM;
	}

	if (sizeof(struct shared_resource_table) + size * 2 > resource_size(res)) {
		dev_err(dev, "vring size is not enough\n");
		return -ENOMEM;
	}

	rpmsg_dev->vring_mem = devm_ioremap_wc(dev, res->start, resource_size(res));
	if (!rpmsg_dev->vring_mem) {
		dev_err(dev, "ioremap failed\n");
		return -ENOMEM;
	}

	rpmsg_dev->vq_mem[0] = rpmsg_dev->vring_mem;
	rpmsg_dev->vq_mem[1] = rpmsg_dev->vring_mem + size;
	rpmsg_dev->rsrc_tbl = rpmsg_dev->vring_mem + size * 2;

	rpmsg_dev->reg_scr = syscon_regmap_lookup_by_phandle(dev->of_node, "amb,scr-regmap");
	if (IS_ERR(rpmsg_dev->reg_scr)) {
		dev_err(dev, "Can't find scr regmap\n");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "get 'host' irq error\n");
		return -ENXIO;
	}

	rval = devm_request_irq(dev, irq, ambarella_rpmsg_isr, 0,
				dev_name(dev), rpmsg_dev);
	if (rval) {
		dev_err(dev, "failed to request irq\n");
		return rval;
	}

	rval = of_reserved_mem_device_init_by_idx(dev, dev->of_node, 0);
	if (rval) {
		dev_err(dev, "Can't associate reserved memory\n");
		return rval;
	}

	INIT_WORK(&rpmsg_dev->work, ambarella_rpmsg_work_handler);

	rpmsg_dev->vdev.id.device = VIRTIO_ID_RPMSG;
	rpmsg_dev->vdev.config = &ambarella_rpmsg_config;
	rpmsg_dev->vdev.dev.parent = dev;
	rpmsg_dev->vdev.dev.release = ambarella_rpmsg_dev_release;

	rval = register_virtio_device(&rpmsg_dev->vdev);
	if (rval)
		pr_err("register_virtio_device error %d\n", rval);

	return rval;
}


static const struct of_device_id ambarella_rpmsg_dt_ids[] = {
	{ .compatible = "ambarella,rpmsg",},
	{ /* sentinel */ }
};

static struct platform_driver ambarella_rpmsg_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ambarella-rpmsg",
		.of_match_table = ambarella_rpmsg_dt_ids,
	},
	.probe = ambarella_rpmsg_probe,
};

static int __init ambarella_rpmsg_init(void)
{
	int rval;

	rval = platform_driver_register(&ambarella_rpmsg_driver);
	if (rval) {
		pr_err("Unable to initialize rpmsg driver\n");
		return rval;
	}

	pr_info("Ambarella rpmsg driver is registered.\n");

	return 0;
}
subsys_initcall(ambarella_rpmsg_init);

MODULE_DESCRIPTION("Ambarella remote processor messaging virtio device");
MODULE_LICENSE("GPL v2");

