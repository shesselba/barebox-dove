/*
 * ChipIdea USB2 Dual Role controller driver
 *
 * Sascha Hauer <s.hauer@pengutronix.de>
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __CI_DRC_H__
#define __CI_DRC_H__

#include <usb/ehci.h>
#include <usb/fsl_usb2.h>
#include <usb/usb.h>

struct device_d;
struct regulator;

struct ci_usb {
	struct ehci_data ehci;
	struct device_d *dev;
	void __iomem *base;
	struct regulator *vbus;
	enum usb_dr_mode mode;
	enum usb_phy_interface phyintf;
	u32 caps;
#define CAPS_HOST	BIT(0)
#define CAPS_DEVICE	BIT(1)
	void *priv;
};

struct ci_usb *chipidea_usb2_alloc(struct device_d *dev,
				   void __iomem *base, void *priv);
int chipidea_usb2_register(struct ci_usb *usb);

#endif
