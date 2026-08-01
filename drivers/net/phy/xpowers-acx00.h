/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __DRIVERS_NET_PHY_XPOWERS_ACX00_H
#define __DRIVERS_NET_PHY_XPOWERS_ACX00_H

#include <linux/phy.h>

#define AC300_EPHY_CONTROL_ADDR_OFFSET	16

enum acx00_ephy_led {
	ACX00_EPHY_LED_LINK_ACTIVITY,
	ACX00_EPHY_LED_SPEED,
	ACX00_EPHY_LED_DUPLEX,
	ACX00_EPHY_LED_COUNT,
};

struct acx00_ephy_control {
	phy_interface_t interface;
	int (*power_on)(struct acx00_ephy_control *control,
			unsigned int phy_addr);
	int (*power_off)(struct acx00_ephy_control *control);
	int (*set_led_outputs)(struct acx00_ephy_control *control,
			       unsigned long outputs);
	int (*set_led_polarity)(struct acx00_ephy_control *control,
				bool active_low);
};

#endif
