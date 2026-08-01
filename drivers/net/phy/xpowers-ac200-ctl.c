// SPDX-License-Identifier: GPL-2.0-only
/*
 * X-Powers AC200 Ethernet PHY control driver
 *
 * Copyright (C) 2026 James Hilliard <james.hilliard1@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "xpowers-acx00.h"

#define AC200_EPHY_BPS_EFFUSE_OFFSET	3

#define AC200_SYS_EPHY_CTL0_REG			0x0014
#define AC200_EPHY_RESET_DEASSERT		BIT(0)
#define AC200_EPHY_SYSCLK_ENABLE			BIT(1)

#define AC200_SYS_EPHY_CTL1_REG			0x0016
#define AC200_EPHY_MII_IO_ENABLE			BIT(0)
#define AC200_EPHY_LINK_LED_IO_ENABLE		BIT(1)
#define AC200_EPHY_SPEED_LED_IO_ENABLE		BIT(2)
#define AC200_EPHY_DUPLEX_LED_IO_ENABLE		BIT(3)
#define AC200_EPHY_LED_IO_ENABLE_MASK \
	(AC200_EPHY_LINK_LED_IO_ENABLE | AC200_EPHY_SPEED_LED_IO_ENABLE | \
	 AC200_EPHY_DUPLEX_LED_IO_ENABLE)

/* AC200-internal copy of the Ethernet PHY calibration eFuse. */
#define AC200_EFUSE_EPHY_REG			0x8004

#define AC200_EPHY_CTL_REG			0x6000
#define AC200_EPHY_SHUTDOWN			BIT(0)
#define AC200_EPHY_LED_ACTIVE_LOW		BIT(1)
#define AC200_EPHY_CLK_SEL_24_MHZ		BIT(2)
#define AC200_EPHY_PHY_ADDR_MASK			GENMASK(8, 4)
#define AC200_EPHY_RMII_SEL			BIT(11)
#define AC200_EPHY_BPS_EFFUSE_MASK		GENMASK(15, 12)

struct ac200_ephy_ctl {
	struct acx00_ephy_control control;
	struct regmap *regmap;
	struct mutex lock; /* Serializes power sequencing and state. */
	u16 ephy_ctl;
	u16 led_io_enable;
	unsigned int phy_addr;
	bool powered;
};

static int
ac200_ephy_ctl_set_led_outputs(struct acx00_ephy_control *control,
			       unsigned long outputs)
{
	struct ac200_ephy_ctl *priv =
		container_of(control, struct ac200_ephy_ctl, control);
	u16 led_io_enable;
	int ret = 0;

	if (outputs & ~GENMASK(ACX00_EPHY_LED_COUNT - 1, 0))
		return -EINVAL;

	led_io_enable = (outputs << 1) & AC200_EPHY_LED_IO_ENABLE_MASK;

	mutex_lock(&priv->lock);
	if (priv->powered)
		ret = regmap_update_bits(priv->regmap, AC200_SYS_EPHY_CTL1_REG,
					 AC200_EPHY_LED_IO_ENABLE_MASK,
					 led_io_enable);
	if (!ret)
		priv->led_io_enable = led_io_enable;
	mutex_unlock(&priv->lock);

	return ret;
}

static int
ac200_ephy_ctl_set_led_polarity(struct acx00_ephy_control *control,
				bool active_low)
{
	struct ac200_ephy_ctl *priv =
		container_of(control, struct ac200_ephy_ctl, control);
	u16 value = active_low ? AC200_EPHY_LED_ACTIVE_LOW : 0;
	int ret = 0;

	mutex_lock(&priv->lock);
	if (priv->powered)
		ret = regmap_update_bits(priv->regmap, AC200_EPHY_CTL_REG,
					 AC200_EPHY_LED_ACTIVE_LOW, value);
	if (!ret) {
		priv->ephy_ctl &= ~AC200_EPHY_LED_ACTIVE_LOW;
		priv->ephy_ctl |= value;
	}
	mutex_unlock(&priv->lock);

	return ret;
}

static int ac200_ephy_ctl_power_off_locked(struct ac200_ephy_ctl *priv)
{
	int err;
	int ret;

	if (!priv->powered)
		return 0;

	ret = regmap_write(priv->regmap, AC200_EPHY_CTL_REG,
			   priv->ephy_ctl |
			   FIELD_PREP(AC200_EPHY_PHY_ADDR_MASK,
				      priv->phy_addr) |
			   AC200_EPHY_SHUTDOWN);
	err = regmap_write(priv->regmap, AC200_SYS_EPHY_CTL1_REG, 0);
	if (!ret)
		ret = err;
	err = regmap_write(priv->regmap, AC200_SYS_EPHY_CTL0_REG, 0);
	if (!ret)
		ret = err;

	priv->powered = false;

	return ret;
}

static int ac200_ephy_ctl_power_off(struct acx00_ephy_control *control)
{
	struct ac200_ephy_ctl *priv =
		container_of(control, struct ac200_ephy_ctl, control);
	int ret;

	mutex_lock(&priv->lock);
	ret = ac200_ephy_ctl_power_off_locked(priv);
	mutex_unlock(&priv->lock);

	return ret;
}

static int ac200_ephy_ctl_power_on(struct acx00_ephy_control *control,
				   unsigned int phy_addr)
{
	struct ac200_ephy_ctl *priv =
		container_of(control, struct ac200_ephy_ctl, control);
	u16 ephy_ctl;
	int ret;

	if (phy_addr > FIELD_MAX(AC200_EPHY_PHY_ADDR_MASK))
		return -EINVAL;

	mutex_lock(&priv->lock);
	if (priv->powered && priv->phy_addr == phy_addr) {
		ret = 0;
		goto out_unlock;
	}
	if (priv->powered) {
		ret = ac200_ephy_ctl_power_off_locked(priv);
		if (ret)
			goto out_unlock;
	}

	ephy_ctl = priv->ephy_ctl |
		FIELD_PREP(AC200_EPHY_PHY_ADDR_MASK, phy_addr);

	/* Start from a disabled state before applying the configuration. */
	ret = regmap_write(priv->regmap, AC200_SYS_EPHY_CTL0_REG, 0);
	if (ret)
		goto err_disable;

	ret = regmap_write(priv->regmap, AC200_SYS_EPHY_CTL1_REG,
			   AC200_EPHY_MII_IO_ENABLE |
			   priv->led_io_enable);
	if (ret)
		goto err_disable;

	ret = regmap_write(priv->regmap, AC200_EPHY_CTL_REG,
			   ephy_ctl | AC200_EPHY_SHUTDOWN);
	if (ret)
		goto err_disable;

	ret = regmap_write(priv->regmap, AC200_SYS_EPHY_CTL0_REG,
			   AC200_EPHY_RESET_DEASSERT |
			   AC200_EPHY_SYSCLK_ENABLE);
	if (ret)
		goto err_disable;

	fsleep(10000);

	ret = regmap_write(priv->regmap, AC200_EPHY_CTL_REG, ephy_ctl);
	if (ret)
		goto err_disable;

	priv->phy_addr = phy_addr;
	priv->powered = true;
	goto out_unlock;

err_disable:
	/* Mark it active so the cleanup sequence is always attempted. */
	priv->phy_addr = phy_addr;
	priv->powered = true;
	ac200_ephy_ctl_power_off_locked(priv);
out_unlock:
	mutex_unlock(&priv->lock);

	return ret;
}

static int ac200_ephy_ctl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ac200_ephy_ctl *priv;
	phy_interface_t interface;
	struct clk *clk;
	unsigned long clk_rate;
	unsigned int calibration;
	u8 nvmem_calibration;
	u8 bps_effuse_code;
	u16 ephy_ctl;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	mutex_init(&priv->lock);

	priv->regmap = dev_get_regmap(dev->parent, NULL);
	if (!priv->regmap)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "parent regmap is not ready\n");

	if (device_property_present(dev, "nvmem-cells")) {
		ret = nvmem_cell_read_u8(dev, "calibration",
					 &nvmem_calibration);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to read calibration data\n");
		calibration = nvmem_calibration;
	} else {
		ret = regmap_read(priv->regmap, AC200_EFUSE_EPHY_REG,
				  &calibration);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to read on-chip calibration data\n");
	}

	ret = device_get_phy_mode(dev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read PHY mode\n");
	interface = ret;

	/* The vendor driver supplies no transfer function beyond this offset. */
	bps_effuse_code = (calibration + AC200_EPHY_BPS_EFFUSE_OFFSET) &
			   FIELD_MAX(AC200_EPHY_BPS_EFFUSE_MASK);
	ephy_ctl = FIELD_PREP(AC200_EPHY_BPS_EFFUSE_MASK, bps_effuse_code) |
		   AC200_EPHY_LED_ACTIVE_LOW;
	/* EPHY_MODE and BIST_CLK_EN stay clear for normal operation. */

	switch (interface) {
	case PHY_INTERFACE_MODE_MII:
		break;
	case PHY_INTERFACE_MODE_RMII:
		ephy_ctl |= AC200_EPHY_RMII_SEL;
		break;
	default:
		return dev_err_probe(dev, -EINVAL,
				     "unsupported PHY mode %s\n",
				     phy_modes(interface));
	}

	clk = clk_get(dev->parent, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to get input clock\n");

	clk_rate = clk_get_rate(clk);
	clk_put(clk);

	switch (clk_rate) {
	case 24000000:
		ephy_ctl |= AC200_EPHY_CLK_SEL_24_MHZ;
		break;
	case 27000000:
		break;
	default:
		return dev_err_probe(dev, -EINVAL,
				     "unsupported input clock rate %lu Hz\n",
				     clk_rate);
	}

	priv->control.power_on = ac200_ephy_ctl_power_on;
	priv->control.power_off = ac200_ephy_ctl_power_off;
	priv->control.set_led_outputs = ac200_ephy_ctl_set_led_outputs;
	priv->control.set_led_polarity = ac200_ephy_ctl_set_led_polarity;
	priv->control.interface = interface;
	priv->ephy_ctl = ephy_ctl;
	priv->led_io_enable = AC200_EPHY_LED_IO_ENABLE_MASK;
	platform_set_drvdata(pdev, &priv->control);
	return 0;
}

static void ac200_ephy_ctl_remove(struct platform_device *pdev)
{
	struct acx00_ephy_control *control = platform_get_drvdata(pdev);

	control->power_off(control);
}

static const struct of_device_id ac200_ephy_ctl_of_match[] = {
	{ .compatible = "x-powers,ac200-ephy-ctl" },
	{ }
};
MODULE_DEVICE_TABLE(of, ac200_ephy_ctl_of_match);

static struct platform_driver ac200_ephy_ctl_driver = {
	.probe = ac200_ephy_ctl_probe,
	.remove = ac200_ephy_ctl_remove,
	.shutdown = ac200_ephy_ctl_remove,
	.driver = {
		.name = "ac200-ephy-ctl",
		.of_match_table = ac200_ephy_ctl_of_match,
	},
};
module_platform_driver(ac200_ephy_ctl_driver);

MODULE_AUTHOR("James Hilliard <james.hilliard1@gmail.com>");
MODULE_DESCRIPTION("X-Powers AC200 Ethernet PHY control driver");
MODULE_LICENSE("GPL");
