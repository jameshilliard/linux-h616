// SPDX-License-Identifier: GPL-2.0

#include <linux/device.h>
#include <linux/export.h>
#include <linux/mtd/mtd.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "mtddevice.h"

static int mtd_cls_suspend(struct device *dev)
{
	struct mtd_info *mtd = dev_get_drvdata(dev);

	return mtd ? mtd_suspend(mtd) : 0;
}

static int mtd_cls_resume(struct device *dev)
{
	struct mtd_info *mtd = dev_get_drvdata(dev);

	if (mtd)
		mtd_resume(mtd);
	return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(mtd_cls_pm_ops, mtd_cls_suspend, mtd_cls_resume);

struct class mtd_class = {
	.name = "mtd",
	.pm = pm_sleep_ptr(&mtd_cls_pm_ops),
};
EXPORT_SYMBOL_GPL(mtd_class);

static const struct device_type mtd_devtype = {
	.name = "mtd",
};

/* Device references may outlive both the controller and the MTD module. */
static void mtd_class_device_release(struct device *dev)
{
	of_node_put(dev->of_node);
	kfree(dev);
}

struct device *mtd_class_device_alloc(struct device_node *np)
{
	struct device *dev = kzalloc_obj(*dev);

	if (!dev)
		return NULL;

	device_initialize(dev);
	dev->release = mtd_class_device_release;
	dev->type = &mtd_devtype;
	dev->class = &mtd_class;
	dev->of_node = of_node_get(np);
	return dev;
}
EXPORT_SYMBOL_GPL(mtd_class_device_alloc);
