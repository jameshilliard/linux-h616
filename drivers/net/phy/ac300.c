// SPDX-License-Identifier: GPL-2.0-only
/*
 * AC300 driver
 *
 * Copyright (C) 2025 Bootlin, Richard Genoud <richard.genoud@bootlin.com>
 *
 * Description:
 * The purpose of this driver is enabling and configuring the embedded
 * AC200-compatible PHY.
 *
 */

#include <linux/of_mdio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/mii.h>

#include <linux/device.h>
#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/of.h>

#define AC300_CTRL_PHY_ID	0xc0000000
#define AC300_CTRL_PHY_ID_MASK	0xfffffff0

static int ac300_config_init(struct phy_device *phydev)
{
	phy_write(phydev, 0x00, 0x1f40);
	phy_write(phydev, 0x00, 0x1f43);
	phy_write(phydev, 0x00, 0x1fb7);
	phy_write(phydev, 0x05, 0xa81f);
	fsleep(10000);
	phy_write(phydev, 0x06, 0x5811);
	fsleep(10000);
	phy_write(phydev, 0x06, 0x5810);

	return 0;
}

static int ac300_ephy_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct clk *clk;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Failed to request clock\n");

	clk_set_rate_exclusive(clk, 24000000);

	return 0;
}

static struct phy_driver ac300_control_driver[] = {
	{
		.phy_id		= AC300_CTRL_PHY_ID,
		.phy_id_mask	= AC300_CTRL_PHY_ID_MASK,
		.name		= "X-Powers AC300 PHY control",
		.config_init	= ac300_config_init,
		.probe		= ac300_ephy_probe,
	}
};
module_phy_driver(ac300_control_driver);

MODULE_AUTHOR("Richard Genoud <richard.genoud@bootlin.com>");
MODULE_DESCRIPTION("AC300 PHY control driver");
MODULE_LICENSE("GPL");

static const struct mdio_device_id __maybe_unused ac300_control_phy_tbl[] = {
	{ AC300_CTRL_PHY_ID, AC300_CTRL_PHY_ID_MASK },
	{ }
};
MODULE_DEVICE_TABLE(mdio, ac300_control_phy_tbl);
