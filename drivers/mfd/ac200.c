// SPDX-License-Identifier: GPL-2.0-only
/*
 * MFD core driver for the X-Powers AC200
 *
 * Copyright (C) 2026 James Hilliard <james.hilliard1@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <dt-bindings/mfd/x-powers,ac200.h>

#define AC200_SYS_VERSION_REG			0x0000
#define AC200_SYS_VERSION_PACKAGE_MASK		GENMASK(15, 14)
#define AC200_SYS_VERSION_CHIP_MASK		GENMASK(11, 0)

#define AC200_SYS_CONTROL_REG			0x0002
#define AC200_SYS_CONTROL_CHIP_RESET_DEASSERT	BIT(0)

#define AC200_SYS_IRQ_ENABLE_REG			0x0004
#define AC200_SYS_IRQ_INTB_ENABLE		BIT(15)
#define AC200_SYS_IRQ_INTB_ACTIVE_HIGH		BIT(14)
#define AC200_SYS_IRQ_RTC_ENABLE			BIT(12)
#define AC200_SYS_IRQ_EPHY_ENABLE		BIT(8)
#define AC200_SYS_IRQ_TVE_ENABLE			BIT(4)

#define AC200_SYS_IRQ_STATUS_REG			0x0006

#define AC200_SYS_BG_CTL_REG			0x0050
/*
 * The vendor initialization sequence composes this as SID data | BIT(15) |
 * (0xa << 6). The public datasheet marks the whole register reserved and
 * does not name the fixed fields.
 */
#define AC200_SYS_BG_CTL_FIXED_BITS		(BIT(15) | (0xa << 6))

/* Interface register accessible from every register page. */
#define AC200_TWI_REG_ADDR_H	0x00fe
#define AC200_MAX_REG		0xa1f2

struct ac200 {
	struct regmap *regmap;
};

static const char * const ac200_supplies[] = {
	"ac-ldoin",
	"ephy-vcc",
	"rtc-vcc",
	"tv-vcc",
};

static const struct regmap_range_cfg ac200_range_cfg[] = {
	{
		.range_max = AC200_MAX_REG,
		.selector_reg = AC200_TWI_REG_ADDR_H,
		.selector_mask = 0xff,
		.window_len = 256,
	},
};

/*
 * Each AC200 sub-block can reset independently, invalidating its register
 * contents without regmap's knowledge. Cache only the common page selector;
 * this avoids a selector read-modify-write for every access on the same page
 * without ever returning stale functional-register values.
 */
static bool ac200_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg != AC200_TWI_REG_ADDR_H;
}

static const struct regmap_config ac200_regmap_config = {
	.name = "ac200",
	.reg_bits = 8,
	.reg_stride = 2,
	.val_bits = 16,
	.ranges = ac200_range_cfg,
	.num_ranges = ARRAY_SIZE(ac200_range_cfg),
	.max_register = AC200_MAX_REG,
	.volatile_reg = ac200_volatile_reg,
	.cache_type = REGCACHE_MAPLE,
};

static const struct regmap_irq ac200_irqs[] = {
	REGMAP_IRQ_REG(AC200_IRQ_TVE, 0, AC200_SYS_IRQ_TVE_ENABLE),
	REGMAP_IRQ_REG(AC200_IRQ_EPHY, 0, AC200_SYS_IRQ_EPHY_ENABLE),
	REGMAP_IRQ_REG(AC200_IRQ_RTC, 0, AC200_SYS_IRQ_RTC_ENABLE),
};

/*
 * SYS_IRQ_ENABLE is an enable register rather than a mask register, hence
 * unmask_base. The INTB output enable and polarity bits share the register,
 * but regmap-irq only updates the bits described by ac200_irqs.
 *
 * SYS_IRQ_STATUS follows the state of each source block and is read-only.
 * Interrupts must therefore be acknowledged in the source block itself.
 */
static const struct regmap_irq_chip ac200_irq_chip = {
	.name = "ac200",
	.status_base = AC200_SYS_IRQ_STATUS_REG,
	.unmask_base = AC200_SYS_IRQ_ENABLE_REG,
	.num_regs = 1,
	.irqs = ac200_irqs,
	.num_irqs = ARRAY_SIZE(ac200_irqs),
};

static const struct mfd_cell ac200_cells[] = {
	{
		.name = "ac200-ephy-ctl",
		.of_compatible = "x-powers,ac200-ephy-ctl",
	},
};

static int ac200_disable(struct ac200 *ac200)
{
	return regmap_write(ac200->regmap, AC200_SYS_CONTROL_REG, 0);
}

static void ac200_disable_action(void *data)
{
	ac200_disable(data);
}

static int ac200_init_irq(struct ac200 *ac200, int irq)
{
	struct device *dev = regmap_get_device(ac200->regmap);
	struct regmap_irq_chip_data *irq_data;
	u16 value = AC200_SYS_IRQ_INTB_ENABLE;
	u32 trigger;
	int ret;

	trigger = irq_get_trigger_type(irq);
	switch (trigger) {
	case IRQ_TYPE_LEVEL_HIGH:
		value |= AC200_SYS_IRQ_INTB_ACTIVE_HIGH;
		break;
	case IRQ_TYPE_NONE:
	case IRQ_TYPE_LEVEL_LOW:
		break;
	default:
		return dev_err_probe(dev, -EINVAL,
				     "INTB requires a level interrupt, not type %u\n",
				     trigger);
	}

	ret = regmap_update_bits(ac200->regmap, AC200_SYS_IRQ_ENABLE_REG,
				 AC200_SYS_IRQ_INTB_ENABLE |
				 AC200_SYS_IRQ_INTB_ACTIVE_HIGH, value);
	if (ret)
		return ret;

	ret = devm_regmap_add_irq_chip(dev, ac200->regmap, irq, IRQF_ONESHOT,
				       0, &ac200_irq_chip, &irq_data);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add interrupt controller\n");

	return 0;
}

static int ac200_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *ephy_node __free(device_node) = NULL;
	struct ac200 *ac200;
	struct clk *clk;
	unsigned int version;
	u16 bandgap_calibration = 0;
	int ret;

	ac200 = devm_kzalloc(dev, sizeof(*ac200), GFP_KERNEL);
	if (!ac200)
		return -ENOMEM;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(ac200_supplies),
					     ac200_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable input clock\n");

	ret = devm_clk_rate_exclusive_get(dev, clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to lock clock rate\n");

	ac200->regmap = devm_regmap_init_i2c(client, &ac200_regmap_config);
	if (IS_ERR(ac200->regmap))
		return dev_err_probe(dev, PTR_ERR(ac200->regmap),
				     "failed to initialize regmap\n");

	i2c_set_clientdata(client, ac200);

	if (device_property_present(dev, "nvmem-cells")) {
		ret = nvmem_cell_read_u16(dev, "bandgap",
					  &bandgap_calibration);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to read bandgap data\n");
	}

	/*
	 * No minimum delay is documented. Match the vendor driver's 40 ms delay
	 * before its first AC200 register access after enabling the input clock.
	 */
	msleep(40);

	ret = regmap_read(ac200->regmap, AC200_SYS_VERSION_REG, &version);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read chip version\n");

	dev_info(dev, "AC200 revision %#lx in package %lu\n",
		 FIELD_GET(AC200_SYS_VERSION_CHIP_MASK, version),
		 FIELD_GET(AC200_SYS_VERSION_PACKAGE_MASK, version));

	/* Run after the MFD children have been removed. */
	ret = devm_add_action_or_reset(dev, ac200_disable_action, ac200);
	if (ret)
		return ret;

	ret = regmap_write(ac200->regmap, AC200_SYS_CONTROL_REG, 0);
	if (ret)
		return ret;

	ret = regmap_write(ac200->regmap, AC200_SYS_CONTROL_REG,
			   AC200_SYS_CONTROL_CHIP_RESET_DEASSERT);
	if (ret)
		return ret;

	if (bandgap_calibration) {
		ret = regmap_write(ac200->regmap, AC200_SYS_BG_CTL_REG,
				   AC200_SYS_BG_CTL_FIXED_BITS |
				   bandgap_calibration);
		if (ret)
			return ret;
	}

	if (client->irq > 0) {
		ret = ac200_init_irq(ac200, client->irq);
		if (ret)
			return ret;
	}

	/* Neither the AC200 nor its child devices can perform DMA. */
	dev->coherent_dma_mask = 0;
	dev->dma_mask = &dev->coherent_dma_mask;
	ephy_node = of_get_compatible_child(dev->of_node,
					    "x-powers,ac200-ephy-ctl");
	if (!ephy_node)
		return 0;

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, ac200_cells,
				    ARRAY_SIZE(ac200_cells), NULL, 0, NULL);
}

static void ac200_shutdown(struct i2c_client *client)
{
	struct ac200 *ac200 = i2c_get_clientdata(client);

	ac200_disable(ac200);
}

static const struct of_device_id ac200_of_match[] = {
	{ .compatible = "x-powers,ac200" },
	{ }
};
MODULE_DEVICE_TABLE(of, ac200_of_match);

static const struct i2c_device_id ac200_i2c_ids[] = {
	{ "ac200" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ac200_i2c_ids);

static struct i2c_driver ac200_driver = {
	.driver = {
		.name = "ac200",
		.of_match_table = ac200_of_match,
	},
	.probe = ac200_probe,
	.shutdown = ac200_shutdown,
	.id_table = ac200_i2c_ids,
};
module_i2c_driver(ac200_driver);

MODULE_AUTHOR("James Hilliard <james.hilliard1@gmail.com>");
MODULE_DESCRIPTION("X-Powers AC200 MFD core driver");
MODULE_LICENSE("GPL");
