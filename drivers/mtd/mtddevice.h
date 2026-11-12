/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MTD_DEVICE_H
#define __MTD_DEVICE_H

struct device;
struct device_node;
struct class;

extern struct class mtd_class;
struct device *mtd_class_device_alloc(struct device_node *np);

#endif
