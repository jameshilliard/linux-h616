/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MTD device concatenation layer definitions
 *
 * Copyright © 2002      Robert Kaiser <rkaiser@sysgo.de>
 */

#ifndef MTD_CONCAT_H
#define MTD_CONCAT_H

#include <linux/srcu.h>

/*
 * Our storage structure:
 * Subdev points to an array of pointers to struct mtd_info objects
 * which is allocated along with this structure
 *
 */
struct mtd_concat {
	struct mtd_info mtd;
	struct srcu_struct srcu;
	bool disconnected;
	int num_subdev;
	struct mtd_info *subdev[];
};

struct mtd_info *mtd_concat_create(
    struct mtd_info *subdev[],  /* subdevices to concatenate */
    int num_devs,               /* number of subdevices      */
    const char *name);          /* name for the new device   */

void mtd_concat_destroy(struct mtd_info *mtd);
void mtd_concat_disconnect(struct mtd_info *mtd);

/**
 * mtd_virt_concat_node_create - Create a component for concatenation
 *
 * Returns a positive number representing the no. of devices found for
 * concatenation, or a negative error code.
 *
 * List all the devices for concatenations found in DT and create a
 * component for concatenation.
 */
int mtd_virt_concat_node_create(void);

/**
 * mtd_virt_concat_add - add mtd_info object to the list of subdevices for concatenation
 * @mtd: pointer to new MTD device info structure
 *
 * Returns 1 if added, 0 if not a component, or a negative error code.
 *
 * The mtd_info object is added to the list of subdevices for concatenation.
 * A duplicate component is rejected; the caller retains ownership on error
 * or when the partition does not match a concatenation.
 */
int mtd_virt_concat_add(struct mtd_info *mtd);

/**
 * mtd_virt_concat_create_join - Create and register the concatenated MTD device
 * @master: master whose partition registration has completed
 *
 * Returns 0 on succes, or a negative error code.
 *
 * Creates and registers the concatenated MTD device
 */
int mtd_virt_concat_create_join(struct mtd_info *master);

/**
 * mtd_virt_concat_destroy - Remove the concat that includes a specific mtd device
 *                           as one of its components.
 * @mtd: pointer to MTD device info structure.
 *
 * Returns 0 on succes, or a negative error code.
 *
 * Unregister concatenations containing this device or its partitions and
 * release its staged components. Components of other masters remain staged
 * for a later rebind. Acquired handles retain disconnected concat metadata.
 *
 */
int mtd_virt_concat_destroy(struct mtd_info *mtd);

void mtd_virt_concat_destroy_joins(void);
void mtd_virt_concat_destroy_items(void);

#endif
