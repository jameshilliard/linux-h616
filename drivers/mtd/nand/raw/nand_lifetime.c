// SPDX-License-Identifier: GPL-2.0-only
/* Storage lifetime for disconnected NAND controllers and chips. */

#include <linux/device.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/mtd/rawnand.h>
#include <linux/slab.h>

struct nand_lifetime {
	void *allocation;
	struct nand_storage *storage;
	unsigned int ecc_sections;
	unsigned int free_sections;
	int ecc_end;
	int free_end;
	struct mtd_oob_region regions[];
};

static void nand_storage_release(struct kref *ref)
{
	struct nand_storage *storage = container_of(ref, struct nand_storage, refcnt);

	kfree(storage->allocation);
}

/**
 * nand_storage_init - initialize shared allocation ownership
 * @storage: reference count embedded in the allocation
 * @allocation: allocation to free with kfree() after the final put
 */
void nand_storage_init(struct nand_storage *storage, void *allocation)
{
	kref_init(&storage->refcnt);
	storage->allocation = allocation;
}
EXPORT_SYMBOL_GPL(nand_storage_init);

void nand_storage_get(struct nand_storage *storage)
{
	kref_get(&storage->refcnt);
}
EXPORT_SYMBOL_GPL(nand_storage_get);

void nand_storage_put(void *data)
{
	struct nand_storage *storage = data;

	kref_put(&storage->refcnt, nand_storage_release);
}
EXPORT_SYMBOL_GPL(nand_storage_put);

/**
 * devm_nand_storage_init - retain an allocation through managed teardown
 * @dev: device whose managed resources use the allocation
 * @storage: reference count embedded in the allocation
 * @allocation: allocation to free with kfree() after the final put
 *
 * Call before acquiring resources that use the allocation. Takes ownership
 * even on failure. Chips can hold additional references after driver unbind.
 */
int devm_nand_storage_init(struct device *dev, struct nand_storage *storage,
			   void *allocation)
{
	nand_storage_init(storage, allocation);
	return devm_add_action_or_reset(dev, nand_storage_put, storage);
}
EXPORT_SYMBOL_GPL(devm_nand_storage_init);

/**
 * devm_nand_controller_init - initialize retained controller storage
 * @dev: controller device
 * @controller: controller embedded in non-devm storage
 * @allocation: allocation containing the controller, freed with kfree()
 *
 * Call before acquiring managed hardware resources. Device teardown drops its
 * reference after releasing those resources. Takes ownership even on failure.
 */
int devm_nand_controller_init(struct device *dev, struct nand_controller *controller,
			      void *allocation)
{
	nand_controller_init(controller);
	return devm_nand_storage_init(dev, &controller->storage, allocation);
}
EXPORT_SYMBOL_GPL(devm_nand_controller_init);

static int nand_mtd_get_device(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd_get_master(mtd));

	guard(mutex)(&chip->lock);
	return chip->disconnected ? -ENODEV : 0;
}

static int nand_retained_ooblayout_ecc(struct mtd_info *mtd, int section,
				     struct mtd_oob_region *region)
{
	struct nand_lifetime *lifetime = mtd_to_nand(mtd)->lifetime;

	if (section >= lifetime->ecc_sections)
		return lifetime->ecc_end;
	*region = lifetime->regions[section];
	return 0;
}

static int nand_retained_ooblayout_free(struct mtd_info *mtd, int section,
				      struct mtd_oob_region *region)
{
	struct nand_lifetime *lifetime = mtd_to_nand(mtd)->lifetime;

	if (section >= lifetime->free_sections)
		return lifetime->free_end;
	*region = lifetime->regions[lifetime->ecc_sections + section];
	return 0;
}

static const struct mtd_ooblayout_ops nand_retained_ooblayout = {
	.ecc = nand_retained_ooblayout_ecc,
	.free = nand_retained_ooblayout_free,
};

static void nand_mtd_release(struct mtd_info *mtd)
{
	struct nand_lifetime *lifetime = mtd_to_nand(mtd)->lifetime;
	struct nand_storage *storage = lifetime->storage;
	struct device *parent = mtd->parent_dev;

	of_node_put(mtd_get_of_node(mtd));
	kfree(mtd->name);
	kfree(lifetime->allocation);
	kfree(lifetime);
	if (storage)
		nand_storage_put(storage);
	put_device(parent);
}

static int nand_prepare_lifetime(struct nand_chip *chip, void *allocation,
				 struct nand_storage *storage)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct nand_lifetime *lifetime;
	struct mtd_oob_region region;
	unsigned int ecc_sections, free_sections;
	const char *name = mtd->name;
	int ecc_end, free_end, ret;
	char *copy;

	/* Keep layout queries independent of controller-owned code and data. */
	for (ecc_sections = 0; ; ecc_sections++) {
		ecc_end = mtd_ooblayout_ecc(mtd, ecc_sections, &region);
		if (ecc_end)
			break;
	}
	if (ecc_end != -ERANGE && ecc_end != -ENOTSUPP)
		return ecc_end;
	for (free_sections = 0; ; free_sections++) {
		free_end = mtd_ooblayout_free(mtd, free_sections, &region);
		if (free_end)
			break;
	}
	if (free_end != -ERANGE && free_end != -ENOTSUPP)
		return free_end;

	lifetime = kzalloc(struct_size(lifetime, regions, ecc_sections + free_sections),
			   GFP_KERNEL);
	if (!lifetime)
		return -ENOMEM;
	for (unsigned int i = 0; i < ecc_sections; i++) {
		ret = mtd_ooblayout_ecc(mtd, i, &lifetime->regions[i]);
		if (ret)
			goto free_lifetime;
	}
	for (unsigned int i = 0; i < free_sections; i++) {
		ret = mtd_ooblayout_free(mtd, i, &lifetime->regions[ecc_sections + i]);
		if (ret)
			goto free_lifetime;
	}

	if (!name)
		name = dev_name(mtd->parent_dev);
	copy = kstrdup(name, GFP_KERNEL);
	if (!copy) {
		ret = -ENOMEM;
		goto free_lifetime;
	}

	lifetime->allocation = allocation;
	lifetime->storage = storage;
	lifetime->ecc_sections = ecc_sections;
	lifetime->free_sections = free_sections;
	lifetime->ecc_end = ecc_end;
	lifetime->free_end = free_end;
	chip->lifetime = lifetime;
	mtd->name = copy;
	mtd->owner = THIS_MODULE;
	mtd->_get_device = nand_mtd_get_device;
	mtd_set_ooblayout(mtd, &nand_retained_ooblayout);
	get_device(mtd->parent_dev);
	of_node_get(nand_get_flash_node(chip));
	if (storage)
		nand_storage_get(storage);
	mtd_set_release(mtd, nand_mtd_release);

	return 0;

free_lifetime:
	kfree(lifetime);
	return ret;
}

/**
 * nand_set_lifetime - retain a scanned chip until its final MTD reference
 * @chip: successfully scanned, unpublished NAND chip
 * @allocation: chip allocation, or NULL if embedded in retained controller storage
 *
 * Copies the name and OOB layout and retains the parent, flash node and managed
 * controller allocation. On success, unregister MTD, call nand_disconnect() to
 * drain operations and run NAND cleanup, then call mtd_put_master(). Use the
 * same disconnect/put sequence on registration failure. Final release only
 * frees data in the NAND core; it never calls back into the controller driver.
 *
 * Return: zero on success or a negative error, leaving ownership unchanged.
 */
int nand_set_lifetime(struct nand_chip *chip, void *allocation)
{
	struct nand_storage *storage = &chip->controller->storage;

	if (!storage->allocation)
		storage = NULL;
	return nand_prepare_lifetime(chip, allocation, storage);
}
EXPORT_SYMBOL_GPL(nand_set_lifetime);

/**
 * nand_set_shared_lifetime - retain a chip embedded in a shared allocation
 * @chip: successfully scanned, unpublished NAND chip
 * @storage: shared allocation containing the chip
 *
 * Like nand_set_lifetime(), but for a chip whose allocation is independent of
 * its controller, and must also outlive other device-managed resources.
 */
int nand_set_shared_lifetime(struct nand_chip *chip, struct nand_storage *storage)
{
	return nand_prepare_lifetime(chip, NULL, storage);
}
EXPORT_SYMBOL_GPL(nand_set_shared_lifetime);
