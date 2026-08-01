// SPDX-License-Identifier: GPL-2.0-only
/*
 * X-Powers AC200/AC300 Ethernet PHY driver
 *
 * Copyright (C) 2026 James Hilliard <james.hilliard1@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/leds.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "xpowers-acx00.h"

#define ACX00_EPHY_ID			0x00441400

#define ACX00_EPHY_CONFIG_VARIANT_AC300		BIT(0)
#define ACX00_EPHY_CONFIG_CALIBRATION_LOW	BIT(1)

#define ACX00_PAGE_SELECT_REG		0x1f
#define ACX00_PAGE_SELECT_MASK		GENMASK(12, 8)
#define ACX00_PAGE_0			0
#define ACX00_PAGE_1			1
#define ACX00_PAGE_2			2
#define ACX00_PAGE_6			6
#define ACX00_PAGE_8			8
#define ACX00_PAGE_9			9

#define ACX00_PAGE0_INTERRUPT_STATUS_REG		0x10
#define ACX00_PAGE0_INTERRUPT_MASK_REG			0x11
#define ACX00_PAGE0_INTERRUPT_LINK_CHANGE		BIT(15)
#define ACX00_PAGE0_INTERRUPT_MAGIC_PACKET		BIT(14)
#define ACX00_PAGE0_INTERRUPT_MANAGED_EVENTS \
	(ACX00_PAGE0_INTERRUPT_LINK_CHANGE | \
	 ACX00_PAGE0_INTERRUPT_MAGIC_PACKET)
#define ACX00_PAGE0_GLOBAL_CONFIG_REG			0x13
#define ACX00_PAGE0_XMII_RX_CLOCK_INVERT		BIT(12)
#define ACX00_PAGE0_MAGIC_PACKET_WAKE_ENABLE		BIT(10)
#define ACX00_PAGE0_MAGIC_PACKET_BROADCAST_ENABLE	BIT(8)
#define ACX00_PAGE0_MAGIC_PACKET_PASSWORD_ENABLE	BIT(7)
#define ACX00_PAGE0_MDI_MODE_MASK			GENMASK(1, 0)
#define ACX00_PAGE0_MDI_MODE_MDI			0
#define ACX00_PAGE0_MDI_MODE_MDIX			1
#define ACX00_PAGE0_MDI_MODE_AUTO			2
#define ACX00_PAGE0_STATUS_REG				0x19
#define ACX00_PAGE0_STATUS_MDIX				BIT(7)
#define ACX00_PAGE0_MAGIC_PACKET_MAC_REG(_index)	(0x16 + (_index))

#define ACX00_PAGE1_APS_CONTROL_REG			0x12
#define ACX00_PAGE1_APS_DISABLED_4S_VALUE		0x4824
#define ACX00_PAGE1_UAPS_CONTROL_REG			0x13
#define ACX00_PAGE1_UAPS_ENABLE				BIT(15)
#define ACX00_PAGE1_INTELLIGENT_EEE_CONTROL_REG		0x17
#define ACX00_PAGE1_INTELLIGENT_EEE_ENABLE		BIT(3)

#define ACX00_PAGE2_TX_DATA_CONTROL_REG			0x18
#define ACX00_PAGE2_10BT_FIR_SELECT_MASK			GENMASK(14, 12)
#define ACX00_PAGE2_10BT_FIR_SELECT_DEFAULT		0

#define ACX00_PAGE6_ADC_CONTROL_REG			0x10
#define ACX00_PAGE6_ADC_CONTROL_LOW_CAL_VALUE		0x5523
#define ACX00_PAGE6_AFE_RX_CONTROL_REG			0x13
#define ACX00_PAGE6_AFE_RX_CONTROL_VALUE			0xf000
#define ACX00_PAGE6_AFE_EQ_RX_DETECT_CONTROL_REG		0x14
#define ACX00_PAGE6_AFE_EQ_RX_DETECT_VALUE		0x708b
#define ACX00_PAGE6_AFE_EQ_RX_DETECT_LOW_CAL_VALUE	0x7809
#define ACX00_PAGE6_TX_LEVEL_REG			0x15
#define ACX00_PAGE6_TX_LEVEL_100M_MASK		GENMASK(15, 8)
#define ACX00_PAGE6_TX_LEVEL_10M_MASK		GENMASK(7, 0)
#define ACX00_PAGE6_TX_LEVEL_VALUE(_100m, _10m) \
	(FIELD_PREP(ACX00_PAGE6_TX_LEVEL_100M_MASK, (_100m)) | \
	 FIELD_PREP(ACX00_PAGE6_TX_LEVEL_10M_MASK, (_10m)))
#define ACX00_PAGE6_TX_LEVEL_100M_DEFAULT	0x15
#define ACX00_PAGE6_TX_LEVEL_10M_DEFAULT		0x30
#define ACX00_PAGE6_TX_LEVEL_100M_LOW_CAL	0x35
#define ACX00_PAGE6_TX_LEVEL_10M_LOW_CAL		0x33
#define ACX00_PAGE6_TX_LEVEL_DEFAULT_VALUE \
	ACX00_PAGE6_TX_LEVEL_VALUE(ACX00_PAGE6_TX_LEVEL_100M_DEFAULT, \
				       ACX00_PAGE6_TX_LEVEL_10M_DEFAULT)
#define ACX00_PAGE6_TX_LEVEL_LOW_CAL_VALUE \
	ACX00_PAGE6_TX_LEVEL_VALUE(ACX00_PAGE6_TX_LEVEL_100M_LOW_CAL, \
				       ACX00_PAGE6_TX_LEVEL_10M_LOW_CAL)

#define ACX00_PAGE8_AFE_CONTROL_REG			0x18
#define ACX00_PAGE8_AFE_CONTROL_VALUE			0x00bc
#define ACX00_PAGE8_AUTO_CAL_CONTROL_REG			0x1d
#define ACX00_PAGE8_AUTO_CAL_TX_LEVEL_ADJUST_BYPASS	BIT(11)
#define ACX00_PAGE8_AUTO_CAL_LOW_CAL_OPAQUE_BITS		0x0044
#define ACX00_PAGE8_AUTO_CAL_LOW_VALUE \
	(ACX00_PAGE8_AUTO_CAL_TX_LEVEL_ADJUST_BYPASS | \
	 ACX00_PAGE8_AUTO_CAL_LOW_CAL_OPAQUE_BITS)

#define ACX00_PAGE9_EPGC_COMMAND_REG		0x10
#define ACX00_PAGE9_EPC_CLEAR_COUNTERS		BIT(3)
#define ACX00_PAGE9_EPG_CLEAR_COUNTERS		BIT(2)
#define ACX00_PAGE9_RX_BYTES_HIGH_REG		0x19
#define ACX00_PAGE9_RX_BYTES_LOW_REG		0x1a
#define ACX00_PAGE9_RX_PACKETS_HIGH_REG		0x1b
#define ACX00_PAGE9_RX_PACKETS_LOW_REG		0x1c
#define ACX00_PAGE9_RX_CRC_ERRORS_HIGH_REG	0x1d
#define ACX00_PAGE9_RX_CRC_ERRORS_LOW_REG	0x1e

enum acx00_ephy_stat {
	ACX00_STAT_RX_BYTES,
	ACX00_STAT_RX_PACKETS,
	ACX00_STAT_RX_CRC_ERRORS,
	ACX00_STAT_COUNT,
};

static const char acx00_ephy_stat_names[][ETH_GSTRING_LEN] = {
	[ACX00_STAT_RX_BYTES] = "phy_rx_bytes",
	[ACX00_STAT_RX_PACKETS] = "phy_rx_packets",
	[ACX00_STAT_RX_CRC_ERRORS] = "phy_rx_crc_errors",
};

/*
 * Another integration of this exact-ID PHY documents its digital vendor
 * register map, which also matches the observed ACx00 reset values. The ACx00
 * analog-page field encodings remain unpublished, so keep those as opaque
 * vendor initialization values instead of inventing bit definitions.
 */

struct acx00_ephy_priv {
	struct acx00_ephy_control *control;
	bool is_ac300;
	bool use_low_calibration_tuning;
	bool xmii_rx_clock_inverted;
	bool eee_initialized;
	bool uaps_enabled;
	bool stats_valid;
	bool wol_irq_enabled;
	bool wol_suspended;
	unsigned long led_outputs;
	int led_polarity_mode;
	u32 wolopts;
	u32 stats_last[ACX00_STAT_COUNT];
	u64 stats[ACX00_STAT_COUNT];
};

#if IS_ENABLED(CONFIG_OF_DYNAMIC)
static int acx00_enable_node(struct device *dev, struct device_node *node)
{
	struct of_changeset ocs;
	const char *status;
	int revert_ret;
	int ret;

	ret = of_property_read_string(node, "status", &status);
	if (ret || strcmp(status, "fail-needs-probe"))
		return dev_err_probe(dev, -EINVAL,
				     "%pOF must use status fail-needs-probe\n",
				     node);

	of_changeset_init(&ocs);
	ret = of_changeset_update_prop_string(&ocs, node, "status", "okay");
	if (!ret) {
		ret = of_changeset_apply(&ocs);
		if (ret && of_device_is_available(node)) {
			revert_ret = of_changeset_revert(&ocs);
			if (revert_ret)
				dev_err(dev,
					"failed to restore %pOF after enable error: %pe\n",
					node, ERR_PTR(revert_ret));
		}
	}
	of_changeset_destroy(&ocs);

	return ret;
}
#endif

static int acx00_prepare_node(struct device *dev, struct device_node *node,
			      bool runtime_selection)
{
	if (of_device_is_available(node))
		return 0;

	if (!runtime_selection)
		return dev_err_probe(dev, -ENODEV,
				     "%pOF must be enabled for static control selection\n",
				     node);

#if IS_ENABLED(CONFIG_OF_DYNAMIC)
	return acx00_enable_node(dev, node);
#else
	return dev_err_probe(dev, -EOPNOTSUPP,
			     "enabling %pOF requires CONFIG_OF_DYNAMIC\n", node);
#endif
}

static struct device_node *
acx00_ac200_get_enable_node(struct device *dev, struct device_node *control)
{
	struct device_node *parent;
	struct device_node *enable_node;

	if (!of_device_is_available(control)) {
		dev_err(dev, "%pOF must be enabled\n", control);
		return ERR_PTR(-ENODEV);
	}

	parent = of_get_parent(control);
	if (!parent)
		return ERR_PTR(-EINVAL);

	if (!of_device_is_compatible(parent, "x-powers,ac200")) {
		dev_err(dev, "%pOF is not an AC200 control function\n", control);
		of_node_put(parent);
		return ERR_PTR(-EINVAL);
	}
	if (!of_device_is_available(parent)) {
		dev_err(dev, "%pOF must be enabled\n", parent);
		of_node_put(parent);
		return ERR_PTR(-ENODEV);
	}

	enable_node = of_get_parent(parent);
	of_node_put(parent);
	if (!enable_node)
		return ERR_PTR(-EINVAL);

	return enable_node;
}

static struct device *
acx00_find_supplier(struct device_node *control, bool is_ac300)
{
	struct platform_device *pdev;
	struct mdio_device *mdiodev;

	if (is_ac300) {
		mdiodev = of_mdio_find_device(control);
		return mdiodev ? &mdiodev->dev : NULL;
	}

	pdev = of_find_device_by_node(control);
	return pdev ? &pdev->dev : NULL;
}

static int acx00_select_control(struct phy_device *phydev, bool is_ac300,
				bool runtime_selection,
				unsigned long led_outputs,
				bool led_active_low,
				struct acx00_ephy_control **selected_control)
{
	struct acx00_ephy_control *ephy_control;
	struct device *dev = &phydev->mdio.dev;
	struct device_node *control;
	struct device_node *enable_node;
	struct device *supplier;
	struct device_link *link;
	const char *control_compatible;
	const char *control_property;
	u32 configured_addr;
	int ret;

	control_property = is_ac300 ? "x-powers,ac300-control" :
				      "x-powers,ac200-control";
	control_compatible = is_ac300 ? "x-powers,ac300-ephy-ctl" :
					 "x-powers,ac200-ephy-ctl";
	control = of_parse_phandle(dev->of_node, control_property, 0);
	if (!control)
		return dev_err_probe(dev, -EINVAL,
				     "missing %s phandle\n", control_property);
	if (!of_device_is_compatible(control, control_compatible)) {
		ret = dev_err_probe(dev, -EINVAL,
				    "%s does not reference a %s device\n",
				    control_property, control_compatible);
		goto out_put_control;
	}

	if (is_ac300) {
		ret = of_property_read_u32(control, "reg", &configured_addr);
		if (ret || configured_addr != phydev->mdio.addr +
				       AC300_EPHY_CONTROL_ADDR_OFFSET) {
			ret = dev_err_probe(dev, -EINVAL,
					    "AC300 control address does not match PHY address\n");
			goto out_put_control;
		}

		enable_node = of_node_get(control);
	} else {
		enable_node = acx00_ac200_get_enable_node(dev, control);
		if (IS_ERR(enable_node)) {
			ret = PTR_ERR(enable_node);
			goto out_put_control;
		}
	}

	ret = acx00_prepare_node(dev, enable_node, runtime_selection);
	of_node_put(enable_node);
	if (ret)
		goto out_put_control;

	supplier = acx00_find_supplier(control, is_ac300);
	if (!supplier) {
		ret = -EPROBE_DEFER;
		goto out_put_control;
	}

	link = device_link_add(dev, supplier, DL_FLAG_AUTOPROBE_CONSUMER);
	if (!link) {
		ret = -EINVAL;
	} else {
		device_lock(supplier);
		if (!device_is_bound(supplier)) {
			ret = -EPROBE_DEFER;
		} else {
			ephy_control = dev_get_drvdata(supplier);
			if (!ephy_control || !ephy_control->power_on ||
			    !ephy_control->power_off ||
			    !ephy_control->set_led_outputs ||
			    !ephy_control->set_led_polarity) {
				ret = -EINVAL;
			} else {
				ret = ephy_control->set_led_polarity(ephy_control,
								 led_active_low);
				if (!ret)
					ret = ephy_control->set_led_outputs(ephy_control,
									led_outputs);
				if (!ret)
					ret = ephy_control->power_on(ephy_control,
								     phydev->mdio.addr);
				if (!ret) {
					*selected_control = ephy_control;
				} else {
					ephy_control->set_led_outputs(ephy_control, 0);
					ephy_control->set_led_polarity(ephy_control,
									false);
				}
			}
		}
		device_unlock(supplier);
	}

	put_device(supplier);
	if (ret && ret != -EPROBE_DEFER)
		ret = dev_err_probe(dev, ret,
				    "failed to enable %s control\n",
				    is_ac300 ? "AC300" : "AC200");

out_put_control:
	of_node_put(control);
	return ret;
}

static int acx00_ephy_parse_leds(struct phy_device *phydev,
				 unsigned long *led_outputs,
				 int *led_polarity_mode)
{
	struct device_node *leds;
	u32 index;
	int ret = 0;

	/* Match the vendor fallback when firmware provides no LED topology. */
	*led_outputs = GENMASK(ACX00_EPHY_LED_COUNT - 1, 0);
	*led_polarity_mode = 1;

	leds = of_get_child_by_name(phydev->mdio.dev.of_node, "leds");
	if (!leds)
		return 0;

	/* An explicit container replaces the fallback package configuration. */
	*led_outputs = 0;
	*led_polarity_mode = -1;

	for_each_available_child_of_node_scoped(leds, led) {
		bool active_high = of_property_read_bool(led, "active-high");
		bool active_low = of_property_read_bool(led, "active-low");
		int polarity_mode;

		ret = of_property_read_u32(led, "reg", &index);
		if (ret)
			break;
		if (index >= ACX00_EPHY_LED_COUNT ||
		    (*led_outputs & BIT(index))) {
			ret = -EINVAL;
			break;
		}
		*led_outputs |= BIT(index);

		if (active_high && active_low) {
			ret = -EINVAL;
			break;
		}
		if (of_property_read_bool(led, "inactive-high-impedance")) {
			ret = -EOPNOTSUPP;
			break;
		}
		if (!active_high && !active_low)
			continue;

		polarity_mode = active_low;
		if (*led_polarity_mode >= 0 &&
		    *led_polarity_mode != polarity_mode) {
			ret = -EINVAL;
			break;
		}
		*led_polarity_mode = polarity_mode;
	}

	of_node_put(leds);
	if (ret)
		return dev_err_probe(&phydev->mdio.dev, ret,
				     "invalid EPHY LED description\n");
	if (*led_polarity_mode < 0)
		*led_polarity_mode = 1;

	return 0;
}

static void acx00_ephy_power_off(void *data)
{
	struct phy_device *phydev = data;
	struct acx00_ephy_priv *priv = phydev->priv;
	int ret;

	ret = priv->control->set_led_outputs(priv->control, 0);
	if (ret)
		phydev_warn(phydev, "failed to disable LED outputs: %pe\n",
			    ERR_PTR(ret));

	ret = priv->control->set_led_polarity(priv->control, false);
	if (ret)
		phydev_warn(phydev, "failed to restore LED polarity: %pe\n",
			    ERR_PTR(ret));

	ret = priv->control->power_off(priv->control);
	if (ret)
		phydev_warn(phydev, "failed to power off control block: %pe\n",
			    ERR_PTR(ret));
}

static int acx00_ephy_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct acx00_ephy_priv *priv;
	bool has_ac200_control;
	bool has_ac300_control;
	bool runtime_selection;
	u32 configuration = 0;
	int led_polarity_mode;
	unsigned long led_outputs;
	int ret;

	has_ac200_control = of_property_present(dev->of_node,
						"x-powers,ac200-control");
	has_ac300_control = of_property_present(dev->of_node,
						"x-powers,ac300-control");
	if (!has_ac200_control && !has_ac300_control)
		return -ENODEV;
	runtime_selection = has_ac200_control && has_ac300_control;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = acx00_ephy_parse_leds(phydev, &led_outputs,
				    &led_polarity_mode);
	if (ret)
		return ret;

	if (has_ac300_control &&
	    (runtime_selection ||
	     of_property_present(dev->of_node, "nvmem-cells"))) {
		ret = nvmem_cell_read_variable_le_u32(dev, "configuration",
						      &configuration);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to read PHY configuration\n");
	}

	if (runtime_selection) {
		priv->is_ac300 = configuration &
				 ACX00_EPHY_CONFIG_VARIANT_AC300;
	} else {
		priv->is_ac300 = has_ac300_control;
	}
	if (priv->is_ac300)
		priv->use_low_calibration_tuning = configuration &
						   ACX00_EPHY_CONFIG_CALIBRATION_LOW;
	priv->xmii_rx_clock_inverted =
		device_property_read_bool(dev,
					  "x-powers,xmii-rx-clock-inverted");

	ret = acx00_select_control(phydev, priv->is_ac300,
				   runtime_selection,
				   led_outputs, led_polarity_mode > 0,
				   &priv->control);
	if (ret)
		return ret;

	priv->led_outputs = led_outputs;
	priv->led_polarity_mode = led_polarity_mode;
	phydev->priv = priv;
	ret = devm_add_action_or_reset(dev, acx00_ephy_power_off, phydev);
	if (ret)
		return ret;

	phydev->mdix_ctrl = ETH_TP_MDI_AUTO;

	/*
	 * The Wake-on-LAN events use the normal PHY interrupt. Only advertise
	 * wake support when firmware describes a routed interrupt.
	 */
	if (device_property_read_bool(dev, "wakeup-source") &&
	    phy_interrupt_is_valid(phydev))
		device_set_wakeup_capable(dev, true);

	return 0;
}

static int acx00_ephy_led_rules(u8 index, unsigned long *rules)
{
	switch (index) {
	case ACX00_EPHY_LED_LINK_ACTIVITY:
		*rules = BIT(TRIGGER_NETDEV_LINK) | BIT(TRIGGER_NETDEV_TX) |
			 BIT(TRIGGER_NETDEV_RX);
		break;
	case ACX00_EPHY_LED_SPEED:
		*rules = BIT(TRIGGER_NETDEV_LINK_100);
		break;
	case ACX00_EPHY_LED_DUPLEX:
		/* Half-duplex collision flashing is fixed and has no rule bit. */
		*rules = BIT(TRIGGER_NETDEV_FULL_DUPLEX);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int acx00_ephy_led_hw_is_supported(struct phy_device *phydev, u8 index,
					  unsigned long rules)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	unsigned long supported_rules;
	int ret;

	if (index >= ACX00_EPHY_LED_COUNT ||
	    !test_bit(index, &priv->led_outputs))
		return -EINVAL;

	ret = acx00_ephy_led_rules(index, &supported_rules);
	if (ret)
		return ret;

	return rules == supported_rules ? 0 : -EOPNOTSUPP;
}

static int acx00_ephy_led_hw_control_set(struct phy_device *phydev, u8 index,
					 unsigned long rules)
{
	return acx00_ephy_led_hw_is_supported(phydev, index, rules);
}

static int acx00_ephy_led_hw_control_get(struct phy_device *phydev, u8 index,
					 unsigned long *rules)
{
	struct acx00_ephy_priv *priv = phydev->priv;

	if (index >= ACX00_EPHY_LED_COUNT ||
	    !test_bit(index, &priv->led_outputs))
		return -EINVAL;

	return acx00_ephy_led_rules(index, rules);
}

static int acx00_ephy_led_polarity_set(struct phy_device *phydev, int index,
				       unsigned long modes)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	bool active_high;
	bool active_low;
	int polarity_mode;
	int ret;

	if (index < 0 || index >= ACX00_EPHY_LED_COUNT ||
	    !test_bit(index, &priv->led_outputs))
		return -EINVAL;

	active_high = test_bit(PHY_LED_ACTIVE_HIGH, &modes);
	active_low = test_bit(PHY_LED_ACTIVE_LOW, &modes);
	if (modes & ~(BIT(PHY_LED_ACTIVE_HIGH) |
		      BIT(PHY_LED_ACTIVE_LOW)))
		return -EOPNOTSUPP;
	if (active_high == active_low)
		return -EINVAL;

	polarity_mode = active_low;
	if (priv->led_polarity_mode != polarity_mode) {
		phydev_err(phydev,
			   "LED polarity is global and must match for all outputs\n");
		return -EINVAL;
	}

	ret = priv->control->set_led_polarity(priv->control, active_low);

	return ret;
}

static int acx00_ephy_read_counter(struct phy_device *phydev, u32 high_reg,
				   u32 low_reg, u32 *counter)
{
	int high;
	int high_check;
	int low;

	do {
		high = __phy_read(phydev, high_reg);
		if (high < 0)
			return high;

		low = __phy_read(phydev, low_reg);
		if (low < 0)
			return low;

		high_check = __phy_read(phydev, high_reg);
		if (high_check < 0)
			return high_check;
	} while (high != high_check);

	*counter = (u32)high << 16 | low;

	return 0;
}

static int acx00_ephy_update_stats(struct phy_device *phydev)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	u32 counters[ACX00_STAT_COUNT];
	int oldpage;
	int ret;
	int i;

	oldpage = phy_select_page(phydev, ACX00_PAGE_9);
	if (oldpage < 0)
		return phy_restore_page(phydev, oldpage, 0);

	ret = acx00_ephy_read_counter(phydev,
				      ACX00_PAGE9_RX_BYTES_HIGH_REG,
				      ACX00_PAGE9_RX_BYTES_LOW_REG,
				      &counters[ACX00_STAT_RX_BYTES]);
	if (ret)
		goto out_restore_page;

	ret = acx00_ephy_read_counter(phydev,
				      ACX00_PAGE9_RX_PACKETS_HIGH_REG,
				      ACX00_PAGE9_RX_PACKETS_LOW_REG,
				      &counters[ACX00_STAT_RX_PACKETS]);
	if (ret)
		goto out_restore_page;

	ret = acx00_ephy_read_counter(phydev,
				      ACX00_PAGE9_RX_CRC_ERRORS_HIGH_REG,
				      ACX00_PAGE9_RX_CRC_ERRORS_LOW_REG,
				      &counters[ACX00_STAT_RX_CRC_ERRORS]);

out_restore_page:
	ret = phy_restore_page(phydev, oldpage, ret);
	if (ret)
		return ret;

	if (priv->stats_valid) {
		for (i = 0; i < ACX00_STAT_COUNT; i++)
			priv->stats[i] += counters[i] - priv->stats_last[i];
	}

	memcpy(priv->stats_last, counters, sizeof(counters));
	priv->stats_valid = true;

	return 0;
}

static void acx00_ephy_snapshot_and_invalidate_stats(struct phy_device *phydev)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	int ret;

	if (!priv->stats_valid)
		return;

	ret = acx00_ephy_update_stats(phydev);
	if (ret)
		phydev_dbg(phydev, "failed to snapshot statistics: %pe\n",
			   ERR_PTR(ret));
	priv->stats_valid = false;
}

static int acx00_ephy_validate_interface(struct phy_device *phydev)
{
	struct acx00_ephy_priv *priv = phydev->priv;

	if (phydev->interface == priv->control->interface)
		return 0;

	phydev_err(phydev, "MAC uses %s but control device is configured for %s\n",
		   phy_modes(phydev->interface),
		   phy_modes(priv->control->interface));

	return -EINVAL;
}

static int acx00_ephy_soft_reset(struct phy_device *phydev)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	int ret;

	ret = acx00_ephy_validate_interface(phydev);
	if (ret)
		return ret;

	/* The control sequence below may reset the hardware counters. */
	acx00_ephy_snapshot_and_invalidate_stats(phydev);

	ret = priv->control->power_on(priv->control, phydev->mdio.addr);
	if (ret)
		return ret;

	/*
	 * ACx00 acknowledges reset while powered down but does not restart.
	 * This can happen when reattaching a previously suspended PHY.
	 */
	ret = genphy_resume(phydev);
	if (ret)
		return ret;

	return genphy_soft_reset(phydev);
}

static int acx00_ephy_read_page(struct phy_device *phydev)
{
	int ret;

	ret = __phy_read(phydev, ACX00_PAGE_SELECT_REG);
	if (ret < 0)
		return ret;

	return FIELD_GET(ACX00_PAGE_SELECT_MASK, ret);
}

static int acx00_ephy_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, ACX00_PAGE_SELECT_REG,
			   FIELD_PREP(ACX00_PAGE_SELECT_MASK, page));
}

static int __acx00_ephy_ack_interrupt(struct phy_device *phydev)
{
	int status;

	status = __phy_read(phydev, ACX00_PAGE0_INTERRUPT_STATUS_REG);
	if (status < 0)
		return status;

	/* All interrupt status bits are write-one-to-clear. */
	return __phy_write(phydev, ACX00_PAGE0_INTERRUPT_STATUS_REG, status);
}

static int __acx00_ephy_config_interrupts(struct phy_device *phydev,
					  u32 wolopts, bool wake_only)
{
	u16 disable;
	u16 mask = 0;
	int ret;

	if ((!wake_only && phydev->interrupts == PHY_INTERRUPT_ENABLED) ||
	    (wolopts & WAKE_PHY))
		mask |= ACX00_PAGE0_INTERRUPT_LINK_CHANGE;
	if (wolopts & WAKE_MAGIC)
		mask |= ACX00_PAGE0_INTERRUPT_MAGIC_PACKET;

	/* Mask sources being removed before clearing their latched status. */
	disable = ACX00_PAGE0_INTERRUPT_MANAGED_EVENTS & ~mask;
	if (disable) {
		ret = __phy_modify(phydev, ACX00_PAGE0_INTERRUPT_MASK_REG,
				   disable, 0);
		if (ret)
			return ret;
	}

	ret = __acx00_ephy_ack_interrupt(phydev);
	if (ret)
		return ret;

	return __phy_modify(phydev, ACX00_PAGE0_INTERRUPT_MASK_REG,
			    ACX00_PAGE0_INTERRUPT_MANAGED_EVENTS, mask);
}

static int acx00_ephy_config_interrupts(struct phy_device *phydev,
					u32 wolopts, bool wake_only)
{
	int oldpage;
	int ret;

	oldpage = phy_select_page(phydev, ACX00_PAGE_0);
	if (oldpage < 0)
		return phy_restore_page(phydev, oldpage, 0);

	ret = __acx00_ephy_config_interrupts(phydev, wolopts, wake_only);

	return phy_restore_page(phydev, oldpage, ret);
}

static int acx00_ephy_config_intr(struct phy_device *phydev)
{
	struct acx00_ephy_priv *priv = phydev->priv;

	return acx00_ephy_config_interrupts(phydev, priv->wolopts,
					    priv->wol_suspended);
}

static irqreturn_t acx00_ephy_handle_interrupt(struct phy_device *phydev)
{
	unsigned int active = 0;
	int oldpage;
	int enabled;
	int status;
	int ret = 0;

	oldpage = phy_select_page(phydev, ACX00_PAGE_0);
	if (oldpage < 0) {
		ret = phy_restore_page(phydev, oldpage, 0);
		goto out_error;
	}

	enabled = __phy_read(phydev, ACX00_PAGE0_INTERRUPT_MASK_REG);
	if (enabled < 0) {
		ret = enabled;
		goto out_restore_page;
	}

	status = __phy_read(phydev, ACX00_PAGE0_INTERRUPT_STATUS_REG);
	if (status < 0) {
		ret = status;
		goto out_restore_page;
	}

	active = status & enabled;
	if (active)
		/* Clear every event latched in the write-one-to-clear register. */
		ret = __phy_write(phydev, ACX00_PAGE0_INTERRUPT_STATUS_REG,
				  status);

out_restore_page:
	ret = phy_restore_page(phydev, oldpage, ret);
	if (ret)
		goto out_error;

	if (!active)
		return IRQ_NONE;

	if (active & ACX00_PAGE0_INTERRUPT_LINK_CHANGE)
		phy_trigger_machine(phydev);

	return IRQ_HANDLED;

out_error:
	phy_error(phydev);
	return IRQ_NONE;
}

/*
 * Arm the nested interrupt as a wake source immediately. The AC200 is on an
 * I2C bus, so deferring this until the noirq suspend phase could require an
 * I2C register update after the controller has already been suspended.
 */
static int acx00_ephy_set_wake_irq(struct phy_device *phydev, bool enable)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	int ret;

	if (priv->wol_irq_enabled == enable)
		return 0;
	if (!phy_interrupt_is_valid(phydev))
		return enable ? -EOPNOTSUPP : 0;

	if (enable)
		ret = enable_irq_wake(phydev->irq);
	else
		ret = disable_irq_wake(phydev->irq);
	if (ret)
		return ret;

	priv->wol_irq_enabled = enable;

	return 0;
}

static void acx00_ephy_get_wol(struct phy_device *phydev,
			       struct ethtool_wolinfo *wol)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;

	wol->supported = 0;
	wol->wolopts = 0;

	if (!device_can_wakeup(dev) || !phy_interrupt_is_valid(phydev))
		return;

	wol->supported = WAKE_MAGIC | WAKE_PHY;
	wol->wolopts = priv->wolopts;
}

static int acx00_ephy_config_wol(struct phy_device *phydev, u32 wolopts)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	struct net_device *ndev = phydev->attached_dev;
	const u8 *mac;
	int oldpage;
	int ret = 0;
	int i;

	if (wolopts & WAKE_MAGIC && !ndev)
		return -ENODEV;

	oldpage = phy_select_page(phydev, ACX00_PAGE_0);
	if (oldpage < 0)
		return phy_restore_page(phydev, oldpage, 0);

	if (wolopts & WAKE_MAGIC) {
		/* The detector stores the MAC address big-endian in three registers. */
		mac = ndev->dev_addr;
		for (i = 0; i < ETH_ALEN / 2; i++) {
			ret = __phy_write(phydev,
					  ACX00_PAGE0_MAGIC_PACKET_MAC_REG(i),
					  mac[2 * i] << 8 | mac[2 * i + 1]);
			if (ret)
				goto out_restore_page;
		}
	}

	ret = __phy_modify(phydev, ACX00_PAGE0_GLOBAL_CONFIG_REG,
			   ACX00_PAGE0_MAGIC_PACKET_WAKE_ENABLE |
			   ACX00_PAGE0_MAGIC_PACKET_BROADCAST_ENABLE |
			   ACX00_PAGE0_MAGIC_PACKET_PASSWORD_ENABLE,
			   wolopts & WAKE_MAGIC ?
			   ACX00_PAGE0_MAGIC_PACKET_WAKE_ENABLE |
			   ACX00_PAGE0_MAGIC_PACKET_BROADCAST_ENABLE : 0);
	if (ret)
		goto out_restore_page;

	ret = __acx00_ephy_config_interrupts(phydev, wolopts,
					     priv->wol_suspended);

out_restore_page:
	return phy_restore_page(phydev, oldpage, ret);
}

static int acx00_ephy_set_wol(struct phy_device *phydev,
			      struct ethtool_wolinfo *wol)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	u32 old_wolopts = priv->wolopts;
	bool enable = !!wol->wolopts;
	bool old_enable = !!old_wolopts;
	int rollback_ret;
	int ret;

	if (wol->wolopts & ~(WAKE_MAGIC | WAKE_PHY))
		return -EOPNOTSUPP;
	if (enable && (!device_can_wakeup(dev) ||
		       !phy_interrupt_is_valid(phydev)))
		return -EOPNOTSUPP;

	ret = acx00_ephy_config_wol(phydev, wol->wolopts);
	if (ret)
		goto out_rollback;

	ret = acx00_ephy_set_wake_irq(phydev, enable);
	if (ret)
		goto out_rollback;

	ret = device_set_wakeup_enable(dev, enable);
	if (ret)
		goto out_rollback;

	priv->wolopts = wol->wolopts;

	return 0;

out_rollback:
	rollback_ret = acx00_ephy_config_wol(phydev, old_wolopts);
	if (rollback_ret)
		phydev_warn(phydev,
			    "failed to restore Wake-on-LAN configuration: %pe\n",
			    ERR_PTR(rollback_ret));

	rollback_ret = acx00_ephy_set_wake_irq(phydev, old_enable);
	if (rollback_ret)
		phydev_warn(phydev,
			    "failed to restore wake IRQ state: %pe\n",
			    ERR_PTR(rollback_ret));

	rollback_ret = device_set_wakeup_enable(dev, old_enable);
	if (rollback_ret)
		phydev_warn(phydev,
			    "failed to restore device wake state: %pe\n",
			    ERR_PTR(rollback_ret));

	return ret;
}

static int acx00_ephy_config_init(struct phy_device *phydev)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	bool use_low_calibration_tuning = priv->use_low_calibration_tuning;
	bool is_ac300 = priv->is_ac300;
	u16 afe_eq_rx_detect = ACX00_PAGE6_AFE_EQ_RX_DETECT_VALUE;
	u16 tx_level_value = ACX00_PAGE6_TX_LEVEL_DEFAULT_VALUE;
	int oldpage;
	int ret;
	bool tx_lpi_enabled;

	/* Configuration clears the hardware counters below. */
	acx00_ephy_snapshot_and_invalidate_stats(phydev);

	if (!priv->eee_initialized) {
		/* The vendor configuration starts with both EEE modes disabled. */
		phydev->eee_cfg.eee_enabled = false;
		phydev->eee_cfg.tx_lpi_enabled = false;
		priv->eee_initialized = true;
	}
	tx_lpi_enabled = phydev->eee_cfg.tx_lpi_enabled &&
			 !phydev->autonomous_eee_disabled;

	ret = phy_modify_paged(phydev, ACX00_PAGE_0,
			       ACX00_PAGE0_GLOBAL_CONFIG_REG,
			       ACX00_PAGE0_XMII_RX_CLOCK_INVERT,
			       priv->xmii_rx_clock_inverted ?
			       ACX00_PAGE0_XMII_RX_CLOCK_INVERT : 0);
	if (ret)
		return ret;

	if (is_ac300) {
		if (use_low_calibration_tuning) {
			afe_eq_rx_detect =
				ACX00_PAGE6_AFE_EQ_RX_DETECT_LOW_CAL_VALUE;
			tx_level_value = ACX00_PAGE6_TX_LEVEL_LOW_CAL_VALUE;
		}
	}

	oldpage = phy_select_page(phydev, ACX00_PAGE_1);
	if (oldpage < 0)
		return phy_restore_page(phydev, oldpage, 0);

	ret = __phy_write(phydev, ACX00_PAGE1_APS_CONTROL_REG,
			  ACX00_PAGE1_APS_DISABLED_4S_VALUE);
	if (ret)
		goto out_restore_page;
	ret = __phy_modify(phydev, ACX00_PAGE1_UAPS_CONTROL_REG,
			   ACX00_PAGE1_UAPS_ENABLE,
			   priv->uaps_enabled ? ACX00_PAGE1_UAPS_ENABLE : 0);
	if (ret)
		goto out_restore_page;
	/* Intelligent EEE lets the PHY generate LPI without MAC support. */
	ret = __phy_modify(phydev, ACX00_PAGE1_INTELLIGENT_EEE_CONTROL_REG,
			   ACX00_PAGE1_INTELLIGENT_EEE_ENABLE,
			   tx_lpi_enabled ?
			   ACX00_PAGE1_INTELLIGENT_EEE_ENABLE : 0);
	if (ret)
		goto out_restore_page;

	ret = acx00_ephy_write_page(phydev, ACX00_PAGE_2);
	if (ret)
		goto out_restore_page;
	ret = __phy_modify(phydev, ACX00_PAGE2_TX_DATA_CONTROL_REG,
			   ACX00_PAGE2_10BT_FIR_SELECT_MASK,
			   FIELD_PREP(ACX00_PAGE2_10BT_FIR_SELECT_MASK,
				      ACX00_PAGE2_10BT_FIR_SELECT_DEFAULT));
	if (ret)
		goto out_restore_page;

	ret = acx00_ephy_write_page(phydev, ACX00_PAGE_6);
	if (ret)
		goto out_restore_page;
	ret = __phy_write(phydev, ACX00_PAGE6_AFE_EQ_RX_DETECT_CONTROL_REG,
			  afe_eq_rx_detect);
	if (ret)
		goto out_restore_page;
	ret = __phy_write(phydev, ACX00_PAGE6_AFE_RX_CONTROL_REG,
			  ACX00_PAGE6_AFE_RX_CONTROL_VALUE);
	if (ret)
		goto out_restore_page;
	if (is_ac300 && use_low_calibration_tuning) {
		ret = __phy_write(phydev, ACX00_PAGE6_ADC_CONTROL_REG,
				  ACX00_PAGE6_ADC_CONTROL_LOW_CAL_VALUE);
		if (ret)
			goto out_restore_page;
	}
	ret = __phy_write(phydev, ACX00_PAGE6_TX_LEVEL_REG, tx_level_value);
	if (ret)
		goto out_restore_page;

	ret = acx00_ephy_write_page(phydev, ACX00_PAGE_8);
	if (ret)
		goto out_restore_page;
	if (is_ac300 && use_low_calibration_tuning) {
		ret = __phy_write(phydev, ACX00_PAGE8_AUTO_CAL_CONTROL_REG,
				  ACX00_PAGE8_AUTO_CAL_LOW_VALUE);
		if (ret)
			goto out_restore_page;
	}
	ret = __phy_write(phydev, ACX00_PAGE8_AFE_CONTROL_REG,
			  ACX00_PAGE8_AFE_CONTROL_VALUE);
	if (ret)
		goto out_restore_page;

	ret = acx00_ephy_write_page(phydev, ACX00_PAGE_9);
	if (ret)
		goto out_restore_page;
	ret = __phy_write(phydev, ACX00_PAGE9_EPGC_COMMAND_REG,
			  ACX00_PAGE9_EPC_CLEAR_COUNTERS |
			  ACX00_PAGE9_EPG_CLEAR_COUNTERS);
	if (!ret) {
		memset(priv->stats_last, 0, sizeof(priv->stats_last));
		priv->stats_valid = true;
	}

out_restore_page:
	ret = phy_restore_page(phydev, oldpage, ret);
	if (ret)
		return ret;

	/* Apply the retained Linux EEE policy after every hardware reset. */
	ret = genphy_c45_an_config_eee_aneg(phydev);
	if (ret < 0)
		return ret;

	/* A soft reset also clears the retained Wake-on-LAN configuration. */
	return acx00_ephy_config_wol(phydev, priv->wolopts);
}

static int acx00_ephy_set_intelligent_eee(struct phy_device *phydev,
					  bool enable)
{
	return phy_modify_paged(phydev, ACX00_PAGE_1,
				ACX00_PAGE1_INTELLIGENT_EEE_CONTROL_REG,
				ACX00_PAGE1_INTELLIGENT_EEE_ENABLE,
				enable ? ACX00_PAGE1_INTELLIGENT_EEE_ENABLE : 0);
}

static int acx00_ephy_set_tx_lpi(struct phy_device *phydev,
				 const struct eee_config *config)
{
	if (config->tx_lpi_enabled && config->tx_lpi_timer)
		return -EOPNOTSUPP;

	return acx00_ephy_set_intelligent_eee(phydev,
						 config->tx_lpi_enabled);
}

static int acx00_ephy_config_aneg(struct phy_device *phydev)
{
	u16 mode;
	int ret;

	switch (phydev->mdix_ctrl) {
	case ETH_TP_MDI:
		mode = ACX00_PAGE0_MDI_MODE_MDI;
		break;
	case ETH_TP_MDI_X:
		mode = ACX00_PAGE0_MDI_MODE_MDIX;
		break;
	case ETH_TP_MDI_AUTO:
		mode = ACX00_PAGE0_MDI_MODE_AUTO;
		break;
	default:
		return -EINVAL;
	}

	ret = phy_modify_paged_changed(phydev, ACX00_PAGE_0,
				       ACX00_PAGE0_GLOBAL_CONFIG_REG,
				       ACX00_PAGE0_MDI_MODE_MASK,
				       FIELD_PREP(ACX00_PAGE0_MDI_MODE_MASK,
						  mode));
	if (ret < 0)
		return ret;

	return __genphy_config_aneg(phydev, ret > 0);
}

static int acx00_ephy_read_status(struct phy_device *phydev)
{
	int ret;

	ret = genphy_read_status(phydev);
	if (ret)
		return ret;

	if (!phydev->link) {
		phydev->mdix = ETH_TP_MDI_INVALID;
		return 0;
	}

	ret = phy_read_paged(phydev, ACX00_PAGE_0, ACX00_PAGE0_STATUS_REG);
	if (ret < 0)
		return ret;

	phydev->mdix = ret & ACX00_PAGE0_STATUS_MDIX ? ETH_TP_MDI_X :
						       ETH_TP_MDI;

	return 0;
}

static int acx00_ephy_get_sset_count(struct phy_device *phydev)
{
	return ACX00_STAT_COUNT;
}

static void acx00_ephy_get_strings(struct phy_device *phydev, u8 *data)
{
	unsigned int i;

	for (i = 0; i < ACX00_STAT_COUNT; i++)
		ethtool_puts(&data, acx00_ephy_stat_names[i]);
}

static void acx00_ephy_get_stats(struct phy_device *phydev,
				 struct ethtool_stats *stats, u64 *data)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	unsigned int i;
	int ret;

	ret = acx00_ephy_update_stats(phydev);
	for (i = 0; i < ACX00_STAT_COUNT; i++)
		data[i] = ret ? U64_MAX : priv->stats[i];
}

static void acx00_ephy_get_phy_stats(struct phy_device *phydev,
				     struct ethtool_eth_phy_stats *eth_stats,
				     struct ethtool_phy_stats *stats)
{
	struct acx00_ephy_priv *priv = phydev->priv;

	stats->rx_errors = priv->stats[ACX00_STAT_RX_CRC_ERRORS];
}

static int acx00_ephy_get_tunable(struct phy_device *phydev,
				  struct ethtool_tunable *tunable, void *data)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	int ret;

	if (tunable->id != ETHTOOL_PHY_EDPD)
		return -EOPNOTSUPP;

	ret = phy_read_paged(phydev, ACX00_PAGE_1,
			     ACX00_PAGE1_UAPS_CONTROL_REG);
	if (ret < 0)
		return ret;
	priv->uaps_enabled = ret & ACX00_PAGE1_UAPS_ENABLE;

	*(u16 *)data = priv->uaps_enabled ? ETHTOOL_PHY_EDPD_NO_TX :
					      ETHTOOL_PHY_EDPD_DISABLE;

	return 0;
}

static int acx00_ephy_set_tunable(struct phy_device *phydev,
				  struct ethtool_tunable *tunable,
				  const void *data)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	bool enable;
	u16 edpd;
	int ret;

	if (tunable->id != ETHTOOL_PHY_EDPD)
		return -EOPNOTSUPP;

	edpd = *(const u16 *)data;
	switch (edpd) {
	case ETHTOOL_PHY_EDPD_DISABLE:
		enable = false;
		break;
	case ETHTOOL_PHY_EDPD_NO_TX:
		enable = true;
		break;
	default:
		return -EINVAL;
	}

	ret = phy_modify_paged(phydev, ACX00_PAGE_1,
			       ACX00_PAGE1_UAPS_CONTROL_REG,
			       ACX00_PAGE1_UAPS_ENABLE,
			       enable ? ACX00_PAGE1_UAPS_ENABLE : 0);
	if (!ret)
		priv->uaps_enabled = enable;

	return ret;
}

static int acx00_ephy_power_on_and_resume(struct phy_device *phydev)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	int ret;

	ret = priv->control->power_on(priv->control, phydev->mdio.addr);
	if (ret)
		return ret;

	ret = genphy_resume(phydev);
	if (ret)
		return ret;

	/* The control block loses the link-PHY vendor state while powered off. */
	return acx00_ephy_config_init(phydev);
}

static int acx00_ephy_resume(struct phy_device *phydev)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	int ret;

	ret = acx00_ephy_validate_interface(phydev);
	if (ret)
		return ret;

	if (priv->wol_suspended) {
		ret = acx00_ephy_config_interrupts(phydev, priv->wolopts,
						   false);
		if (ret)
			return ret;

		priv->wol_suspended = false;
		return 0;
	}

	return acx00_ephy_power_on_and_resume(phydev);
}

static int acx00_ephy_suspend(struct phy_device *phydev)
{
	struct acx00_ephy_priv *priv = phydev->priv;
	int resume_ret;
	int ret;

	mutex_lock(&phydev->lock);
	if (priv->stats_valid)
		acx00_ephy_update_stats(phydev);
	mutex_unlock(&phydev->lock);

	if (phydev->wol_enabled) {
		ret = acx00_ephy_config_interrupts(phydev, priv->wolopts,
						   true);
		if (!ret)
			priv->wol_suspended = true;

		return ret;
	}

	ret = genphy_suspend(phydev);
	if (ret)
		return ret;

	ret = priv->control->power_off(priv->control);

	mutex_lock(&phydev->lock);
	priv->stats_valid = false;
	mutex_unlock(&phydev->lock);
	if (ret) {
		resume_ret = acx00_ephy_power_on_and_resume(phydev);
		if (resume_ret)
			phydev_warn(phydev,
				    "failed to recover from suspend error: %pe\n",
				    ERR_PTR(resume_ret));
	}

	return ret;
}

static void acx00_ephy_remove(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	int ret;

	ret = acx00_ephy_set_wake_irq(phydev, false);
	if (ret)
		phydev_warn(phydev, "failed to disarm wake IRQ: %pe\n",
			    ERR_PTR(ret));

	device_set_wakeup_enable(dev, false);
	device_set_wakeup_capable(dev, false);
}

static int acx00_ephy_match_phy_device(struct phy_device *phydev,
				       const struct phy_driver *phydrv)
{
	struct device_node *node = phydev->mdio.dev.of_node;

	if (!genphy_match_phy_device(phydev, phydrv) || !node)
		return 0;

	/* RK630 reports the same PHY ID, so require an X-Powers control link. */
	return of_property_present(node, "x-powers,ac200-control") ||
	       of_property_present(node, "x-powers,ac300-control");
}

static struct phy_driver acx00_ephy_driver[] = {
	{
		PHY_ID_MATCH_MODEL(ACX00_EPHY_ID),
		.name = "X-Powers AC200/AC300 EPHY",
		.flags = PHY_ALWAYS_CALL_SUSPEND,
		.match_phy_device = acx00_ephy_match_phy_device,
		.probe = acx00_ephy_probe,
		.read_page = acx00_ephy_read_page,
		.write_page = acx00_ephy_write_page,
		.soft_reset = acx00_ephy_soft_reset,
		.config_init = acx00_ephy_config_init,
		.config_aneg = acx00_ephy_config_aneg,
		.read_status = acx00_ephy_read_status,
		.config_intr = acx00_ephy_config_intr,
		.handle_interrupt = acx00_ephy_handle_interrupt,
		.get_wol = acx00_ephy_get_wol,
		.set_wol = acx00_ephy_set_wol,
		.remove = acx00_ephy_remove,
		.set_tx_lpi = acx00_ephy_set_tx_lpi,
		.get_sset_count = acx00_ephy_get_sset_count,
		.get_strings = acx00_ephy_get_strings,
		.get_stats = acx00_ephy_get_stats,
		.get_phy_stats = acx00_ephy_get_phy_stats,
		.update_stats = acx00_ephy_update_stats,
		.get_tunable = acx00_ephy_get_tunable,
		.set_tunable = acx00_ephy_set_tunable,
		.led_hw_is_supported = acx00_ephy_led_hw_is_supported,
		.led_hw_control_set = acx00_ephy_led_hw_control_set,
		.led_hw_control_get = acx00_ephy_led_hw_control_get,
		.led_polarity_set = acx00_ephy_led_polarity_set,
		.suspend = acx00_ephy_suspend,
		.resume = acx00_ephy_resume,
	},
};
module_phy_driver(acx00_ephy_driver);

static const struct mdio_device_id __maybe_unused acx00_ephy_tbl[] = {
	{ PHY_ID_MATCH_MODEL(ACX00_EPHY_ID) },
	{ }
};
MODULE_DEVICE_TABLE(mdio, acx00_ephy_tbl);

MODULE_AUTHOR("James Hilliard <james.hilliard1@gmail.com>");
MODULE_DESCRIPTION("X-Powers AC200/AC300 Ethernet PHY driver");
MODULE_LICENSE("GPL");
