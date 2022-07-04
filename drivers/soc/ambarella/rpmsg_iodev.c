// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2048, Ambarella, Inc.
 * Author: Cao Rongrong <rrcao@ambarella.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/rpmsg.h>
#include <linux/mtd/mtd.h>

#define	IODEV_PACKET_MAGIC	 	0x494f4445

enum {
	IODEV_PKT_MSGID_OWNER = 0x1000,
	IODEV_PKT_MSGID_OPERATION,
};

enum {
	IODEV_PKT_TYPE_REQUEST = 0,
	IODEV_PKT_TYPE_RESPONSE,
};

enum {
	IODEV_NAND = 0,
	IODEV_NUM,
};

enum {
	IODEV_HOST_NOT_READY = 0,
	IODEV_HOST_READY,
};

struct iodev_pkt_header {
	int magic;
	int type;		/* request or response */
	int msgid;
	int devid;
	int length;		/* packet length including header */
	int ack_status;		/* used for response only */
	int reserved[2];
};

struct iodev_pkt_nand {
	struct iodev_pkt_header hdr;
	char part_name[8];
	unsigned int op;	/* not used yet, currently support read only */
	unsigned int offset;
	unsigned int len;
	unsigned int rsvd;
	unsigned long buf;
};

struct iodev_rpmsg_ambarella {
	struct rpmsg_endpoint *ept;
	struct device *dev;
	struct notifier_block nb;
	struct completion ack[IODEV_NUM];
	struct work_struct work[IODEV_NUM];
	void *work_data[IODEV_NUM];
	int dev_status[IODEV_NUM];
};

static void iodev_pkt_init_header(struct iodev_pkt_header *hdr, int request, int msgid, int devid)
{
	hdr->magic = IODEV_PACKET_MAGIC;
	hdr->type = request ? IODEV_PKT_TYPE_REQUEST : IODEV_PKT_TYPE_RESPONSE;
	hdr->msgid = msgid;
	hdr->devid = devid;
	hdr->length = sizeof(struct iodev_pkt_header);
	hdr->ack_status = -1;
}

static int iodev_pkt_check_header(struct iodev_pkt_header *hdr)
{
	if (hdr->magic != IODEV_PACKET_MAGIC
		|| (hdr->msgid != IODEV_PKT_MSGID_OWNER &&
		    hdr->msgid != IODEV_PKT_MSGID_OPERATION)
		|| hdr->devid >= IODEV_NUM
		|| hdr->length < sizeof(struct iodev_pkt_header))
		return -1;

	return 0;
}

static void iodev_nand_worker(struct work_struct *work)
{
	struct iodev_rpmsg_ambarella *iodev;
	struct iodev_pkt_nand *pkt;
	struct mtd_info *mtd = NULL;
	void *buf = NULL;
	size_t retlen;
	int devid = IODEV_NAND, rval = -1;

	iodev = container_of(work, struct iodev_rpmsg_ambarella, work[devid]);
	pkt = iodev->work_data[devid];

	if (pkt->hdr.length != sizeof(struct iodev_pkt_nand)) {
		dev_err(iodev->dev, "invalid nand packet request\n");
		goto exit;
	}

	buf = memremap(pkt->buf, pkt->len, MEMREMAP_WC);
	if (!buf) {
		dev_err(iodev->dev, "memremap failed\n");
		goto exit;
	}

	while (1) {
		mtd = get_mtd_device_nm(pkt->part_name);
		if (!IS_ERR_OR_NULL(mtd))
			break;

		msleep_interruptible(10);
	}

	while (1) {
		rval = mtd_block_isbad(mtd, pkt->offset);
		if (rval < 0)
			goto exit;
		else if (rval == 0)
			break;

		pkt->offset += mtd->erasesize;
	}

	rval = mtd_read(mtd, pkt->offset, pkt->len, &retlen, buf);
	if (rval < 0) {
		if (mtd_is_bitflip(rval)) /* Ignore corrected ECC errors */
			rval = 0;
	}

exit:
	iodev_pkt_init_header(&pkt->hdr, false, pkt->hdr.msgid, devid);
	pkt->hdr.ack_status = rval < 0 ? rval : retlen;	/* tell remote if everything is ok or not */
	rpmsg_send(iodev->ept, pkt, pkt->hdr.length);

	if (buf)
		memunmap(buf);
	if (mtd)
		put_mtd_device(mtd);
}

static work_func_t iodev_work_handler[IODEV_NUM] = {
	iodev_nand_worker,
};

static int iodev_switch_io_owner(struct iodev_rpmsg_ambarella *iodev, int devid)
{
	struct iodev_pkt_header hdr;
	int rval;

	iodev_pkt_init_header(&hdr, true, IODEV_PKT_MSGID_OWNER, devid);

	rval = rpmsg_send(iodev->ept, &hdr, hdr.length);
	if (rval) {
		dev_err(iodev->dev, "rpmsg_send failed: %d\n", rval);
		return rval;
	}

	rval = wait_for_completion_timeout(&iodev->ack[devid], 5 * HZ);
	if (rval) {
		iodev->dev_status[devid] = IODEV_HOST_READY;
		rval = 0;
	} else {
		iodev->dev_status[devid] = IODEV_HOST_NOT_READY;
		rval = -ETIMEDOUT;
	}

	return rval;
}

static int iodev_bus_notifier_call(struct notifier_block *nb,
			      unsigned long event, void *data)
{
	struct iodev_rpmsg_ambarella *iodev;
	struct device *dev = data;
	int devid, rval = 0;

	if (of_device_is_compatible(dev->of_node, "ambarella,nand"))
		devid = IODEV_NAND;
	else
		return NOTIFY_DONE;

	iodev = container_of(nb, struct iodev_rpmsg_ambarella, nb);

	switch (event) {
	case BUS_NOTIFY_BIND_DRIVER:
		rval = iodev_switch_io_owner(iodev, devid);
		break;
	default:
		break;
	}

	return rval < 0 ? NOTIFY_BAD : NOTIFY_OK;
}

static int ambarella_rpmsg_iodev_callback(struct rpmsg_device *rpdev,
				 void *data, int count, void *priv, u32 addr)
{
	struct iodev_rpmsg_ambarella *iodev = dev_get_drvdata(&rpdev->dev);
	struct iodev_pkt_header *pkt_hdr = data;

	if (iodev_pkt_check_header(pkt_hdr) < 0) {
		dev_err(iodev->dev, "invalid request/response\n");
		return 0;
	}

	if (pkt_hdr->msgid == IODEV_PKT_MSGID_OPERATION) {
		iodev->work_data[pkt_hdr->devid] = data;
		schedule_work(&iodev->work[pkt_hdr->devid]);
	} else if (pkt_hdr->msgid == IODEV_PKT_MSGID_OWNER) {
		complete(&iodev->ack[pkt_hdr->devid]);
	}

	return 0;
}

static int ambarella_rpmsg_iodev_probe(struct rpmsg_device *rpdev)
{
	struct iodev_rpmsg_ambarella *iodev;
	int i;

	iodev = devm_kzalloc(&rpdev->dev, sizeof(*iodev), GFP_KERNEL);
	if (!iodev)
		return -ENOMEM;

	dev_set_drvdata(&rpdev->dev, iodev);

	iodev->dev = &rpdev->dev;
	iodev->ept = rpdev->ept;

	for (i = 0; i < IODEV_NUM; i++) {
		if (!iodev_work_handler[i]) {
			dev_err(iodev->dev, "No worker for %d\n", i);
			return -ENXIO;
		}

		INIT_WORK(&iodev->work[i], iodev_work_handler[i]);
		init_completion(&iodev->ack[i]);

		iodev->dev_status[i] = IODEV_HOST_NOT_READY;
	}

	iodev->nb.notifier_call = iodev_bus_notifier_call;
	bus_register_notifier(&platform_bus_type, &iodev->nb);

	rpdev->announce = true;

	dev_info(iodev->dev, "probed!\n");

	return 0;
}

static void ambarella_rpmsg_iodev_remove(struct rpmsg_device *rpdev)
{
	struct iodev_rpmsg_ambarella *iodev = dev_get_drvdata(&rpdev->dev);

	bus_unregister_notifier(&platform_bus_type, &iodev->nb);
}

static const struct rpmsg_device_id ambarella_rpmsg_iodev_match[] = {
	{ "rpmsg_iodev" },
	{}
};

static struct rpmsg_driver ambarella_rpmsg_iodev_driver = {
	.probe = ambarella_rpmsg_iodev_probe,
	.remove = ambarella_rpmsg_iodev_remove,
	.callback = ambarella_rpmsg_iodev_callback,
	.id_table = ambarella_rpmsg_iodev_match,
	.drv  = {
		.name  = "rpmsg_iodev",
	},
};

static int __init ambarella_rpmsg_iodev_init(void)
{
	return register_rpmsg_driver(&ambarella_rpmsg_iodev_driver);
}
subsys_initcall(ambarella_rpmsg_iodev_init);

static void __exit ambarella_rpmsg_iodev_exit(void)
{
	unregister_rpmsg_driver(&ambarella_rpmsg_iodev_driver);
}
module_exit(ambarella_rpmsg_iodev_exit);

MODULE_DESCRIPTION("Ambarella RPMSG Nand device");
MODULE_LICENSE("GPL v2");

