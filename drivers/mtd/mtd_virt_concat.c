// SPDX-License-Identifier: GPL-2.0+
/*
 * Virtual concat MTD device driver
 *
 * Copyright (C) 2018 Bernhard Frauendienst
 * Author: Bernhard Frauendienst <kernel@nospam.obeliks.de>
 */

#include <linux/device.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/concat.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "mtdcore.h"

#define CONCAT_PROP "part-concat-next"

struct mtd_virt_concat_part {
	struct device_node *node;
	struct mtd_info *mtd;
	bool ready;
};

struct mtd_virt_concat_device {
	struct list_head node;
	struct mtd_info *mtd;
};

struct mtd_virt_concat_node {
	struct list_head node;
	struct mtd_virt_concat_device *device;
	unsigned int count;
	struct mtd_virt_concat_part parts[] __counted_by(count);
};

static LIST_HEAD(concat_node_list);
static DEFINE_MUTEX(concat_lock);
static struct lock_class_key concat_partitions_lock_key;

static bool mtd_is_part_concat(struct device_node *node)
{
	struct mtd_virt_concat_node *item;
	unsigned int i;

	list_for_each_entry(item, &concat_node_list, node)
		for (i = 0; i < item->count; i++)
			if (item->parts[i].node == node)
				return true;
	return false;
}

/* Hidden partitions have no registered device to own their storage. */
static void mtd_virt_concat_part_release(struct kref *ref)
{
	struct mtd_info *mtd = container_of(ref, struct mtd_info, refcnt);
	struct mtd_info *master = mtd->ref_master;
	struct mtd_info *parent = mtd->parent;

	of_node_put(mtd_get_of_node(mtd));
	release_mtd_partition(mtd);
	if (master) {
		if (parent != master)
			mtd_put_data(parent);
		mtd_put_lifetime(master);
	}
}

static void mtd_virt_concat_part_put(struct mtd_info *mtd)
{
	kref_put(&mtd->refcnt, mtd_virt_concat_part_release);
}

static void mtd_virt_concat_release(struct mtd_info *mtd)
{
	struct mtd_concat *concat = container_of(mtd, struct mtd_concat, mtd);
	int i;

	for (i = 0; i < concat->num_subdev; i++)
		if (concat->subdev[i])
			mtd_virt_concat_part_put(concat->subdev[i]);
	kfree(mtd->priv);
	kfree(mtd->name);
	mtd_concat_destroy(mtd);
}

static void mtd_virt_concat_unregister(struct list_head *devices)
{
	struct mtd_virt_concat_device *device, *next;

	list_for_each_entry_safe(device, next, devices, node) {
		struct mtd_concat *concat = container_of(device->mtd, struct mtd_concat, mtd);
		int i;

		list_del(&device->node);
		mtd_device_unregister(device->mtd);
		for (i = 0; i < concat->num_subdev; i++) {
			mtd_virt_concat_part_put(concat->subdev[i]);
			concat->subdev[i] = NULL;
		}
		mtd_put_master(device->mtd);
	}
}

static bool mtd_virt_concat_contains(struct mtd_info *part, struct mtd_info *mtd)
{
	for (; part; part = part->parent)
		if (part == mtd)
			return true;
	return false;
}

int mtd_virt_concat_destroy(struct mtd_info *mtd)
{
	struct mtd_virt_concat_node *item;
	LIST_HEAD(devices);
	unsigned int i;

	/* A virtual master cannot itself be a DT concatenation component. */
	if (mtd_get_master(mtd)->master.release == mtd_virt_concat_release)
		return 0;

	mutex_lock(&concat_lock);
	list_for_each_entry(item, &concat_node_list, node) {
		for (i = 0; i < item->count; i++) {
			struct mtd_virt_concat_part *part = &item->parts[i];

			if (!mtd_virt_concat_contains(part->mtd, mtd))
				continue;
			if (item->device) {
				mtd_concat_disconnect(item->device->mtd);
				list_add_tail(&item->device->node, &devices);
				item->device = NULL;
			}
			mtd_virt_concat_part_put(part->mtd);
			part->mtd = NULL;
			part->ready = false;
		}
	}
	mutex_unlock(&concat_lock);

	/* Unregistration takes the virtual master's partition lock. */
	mtd_virt_concat_unregister(&devices);
	return 0;
}

static void mtd_virt_concat_free_item(struct mtd_virt_concat_node *item)
{
	unsigned int i;

	for (i = 0; i < item->count; i++)
		of_node_put(item->parts[i].node);
	kfree(item);
}

static int mtd_virt_concat_create_item(struct device_node *node, unsigned int count)
{
	struct mtd_virt_concat_node *item;
	unsigned int i, j;

	item = kzalloc_flex(*item, parts, count, GFP_KERNEL);
	if (!item)
		return -ENOMEM;
	item->count = count;
	item->parts[0].node = of_node_get(node);

	for (i = 1; i < count; i++) {
		struct device_node *part = of_parse_phandle(node, CONCAT_PROP, i - 1);

		item->parts[i].node = part;
		if (!part || mtd_is_part_concat(part))
			goto out;
		for (j = 0; j < i; j++)
			if (item->parts[j].node == part)
				goto out;
	}
	list_add_tail(&item->node, &concat_node_list);
	return 0;
out:
	mtd_virt_concat_free_item(item);
	return -EINVAL;
}

void mtd_virt_concat_destroy_joins(void)
{
	struct mtd_virt_concat_node *item;
	LIST_HEAD(devices);
	unsigned int i;

	mutex_lock(&concat_lock);
	list_for_each_entry(item, &concat_node_list, node) {
		if (item->device) {
			mtd_concat_disconnect(item->device->mtd);
			list_add_tail(&item->device->node, &devices);
			item->device = NULL;
		}
		for (i = 0; i < item->count; i++) {
			if (item->parts[i].mtd)
				mtd_virt_concat_part_put(item->parts[i].mtd);
			item->parts[i].mtd = NULL;
			item->parts[i].ready = false;
		}
	}
	mutex_unlock(&concat_lock);
	mtd_virt_concat_unregister(&devices);
}

void mtd_virt_concat_destroy_items(void)
{
	struct mtd_virt_concat_node *item, *next;

	guard(mutex)(&concat_lock);
	list_for_each_entry_safe(item, next, &concat_node_list, node) {
		list_del(&item->node);
		mtd_virt_concat_free_item(item);
	}
}

int mtd_virt_concat_add(struct mtd_info *mtd)
{
	struct mtd_info *master = mtd_get_master(mtd);
	struct mtd_virt_concat_node *item;
	unsigned int i;

	guard(mutex)(&concat_lock);
	list_for_each_entry(item, &concat_node_list, node) {
		for (i = 0; i < item->count; i++) {
			struct mtd_virt_concat_part *part = &item->parts[i];

			if (part->node != mtd_get_of_node(mtd))
				continue;
			if (part->mtd)
				return -EBUSY;
			kref_init(&mtd->refcnt);
			/* The master need not have a registered device. */
			if (master->master.release) {
				mtd_get_lifetime(master);
				mtd->ref_master = master;
				if (mtd->parent != master)
					mtd_get_data(mtd->parent);
			}
			of_node_get(mtd_get_of_node(mtd));
			part->mtd = mtd;
			return 1;
		}
	}
	return 0;
}

int mtd_virt_concat_node_create(void)
{
	struct device_node *node;
	int ret, count;

	guard(mutex)(&concat_lock);
	for_each_node_with_property(node, CONCAT_PROP) {
		if (!of_device_is_available(node) || mtd_is_part_concat(node))
			continue;
		count = of_count_phandle_with_args(node, CONCAT_PROP, NULL);
		if (count < 1)
			continue;
		ret = mtd_virt_concat_create_item(node, count + 1);
		if (ret) {
			of_node_put(node);
			return ret;
		}
	}
	return 0;
}

static int mtd_virt_concat_register(struct mtd_virt_concat_node *item)
{
	struct mtd_virt_concat_device *device;
	struct mtd_info **parts, *mtd;
	size_t name_size = sizeof("concat");
	unsigned int i;
	char *name, *p;
	int ret;

	parts = kmalloc_array(item->count, sizeof(*parts), GFP_KERNEL);
	if (!parts)
		return -ENOMEM;
	for (i = 0; i < item->count; i++) {
		parts[i] = item->parts[i].mtd;
		name_size += strlen(parts[i]->name) + 1;
	}
	name = kmalloc(name_size, GFP_KERNEL);
	if (!name) {
		ret = -ENOMEM;
		goto out_parts;
	}
	p = name;
	for (i = 0; i < item->count; i++)
		p += sprintf(p, "%s-", parts[i]->name);
	strscpy(p, "concat", name_size - (p - name));
	device = kzalloc_obj(*device);
	if (!device) {
		ret = -ENOMEM;
		goto out_name;
	}
	mtd = mtd_concat_create(parts, item->count, name);
	if (!mtd) {
		ret = -ENXIO;
		goto out_device;
	}
	for (i = 0; i < item->count; i++)
		kref_get(&parts[i]->refcnt);
	mtd->parent_dev = parts[0]->parent_dev;
	mtd_set_of_node(mtd, mtd_get_of_node(parts[0]));
	mtd->owner = THIS_MODULE;
	mtd->priv = device;
	mtd_set_dev_defaults(mtd);
	lockdep_set_class(&mtd->master.partitions_lock, &concat_partitions_lock_key);
	mtd_set_release(mtd, mtd_virt_concat_release);
	device->mtd = mtd;
	ret = add_mtd_device(mtd);
	if (ret)
		mtd_put_master(mtd);
	else
		item->device = device;
	kfree(parts);
	return ret;

out_device:
	kfree(device);
out_name:
	kfree(name);
out_parts:
	kfree(parts);
	return ret;
}

int mtd_virt_concat_create_join(struct mtd_info *master)
{
	struct mtd_virt_concat_node *item;
	unsigned int i;
	int ret;

	guard(mutex)(&concat_lock);
	list_for_each_entry(item, &concat_node_list, node) {
		bool ready = true, includes_master = false;

		for (i = 0; i < item->count; i++) {
			struct mtd_virt_concat_part *part = &item->parts[i];

			if (part->mtd && mtd_get_master(part->mtd) == master) {
				part->ready = true;
				includes_master = true;
			}
			ready &= part->ready;
		}
		if (!ready || !includes_master || item->device)
			continue;
		ret = mtd_virt_concat_register(item);
		if (ret)
			return ret;
	}
	return 0;
}
