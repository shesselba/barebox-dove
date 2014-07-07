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

#include <common.h>
#include <init.h>
#include <io.h>
#include <linux/err.h>
#include <regulator.h>

#include "ci-drc.h"

#define CI_ID			0x000
#define  CONTROLLER_NID_SHIFT	8
#define  CONTROLLER_NID_MASK	(0x3f << CONTROLLER_NID_SHIFT)
#define  CONTROLLER_ID_MASK	0x3f
#define CI_HWGENERAL		0x004
#define CI_HWHOST		0x008
#define  VUSB_HS_NUM_PORT_SHIFT	1
#define  VUSB_HS_NUM_PORT_MASK	(0x7 << VUSB_HS_NUM_PORT_SHIFT)
#define  VUSB_HS_HOST		BIT(0)
#define CI_HWDEVICE		0x00c
#define  VUSB_HS_DEV_EP_SHIFT	1
#define  VUSB_HS_DEV_EP_MASK	(0x1f << VUSB_HS_DEV_EP_SHIFT)
#define  VUSB_HS_DEVICE		BIT(0)
#define EHCI_REGS_OFFSET	0x100

struct ci_usb *chipidea_usb2_alloc(struct device_d *dev, void __iomem *base, void *priv)
{
	struct ci_usb *usb;

	usb = xzalloc(sizeof(*usb));
	usb->base = base;
	usb->vbus = regulator_get(dev, "vbus");
	usb->dev = dev;
	usb->mode = USB_DR_MODE_UNKNOWN;
	usb->phyintf = USBPHY_INTERFACE_MODE_UNKNOWN;
	usb->ehci.hccr = usb->base + EHCI_REGS_OFFSET;
	usb->ehci.flags = EHCI_HAS_TT;
	usb->ehci.drvdata = usb;
	usb->priv = priv;

	if (IS_ENABLED(CONFIG_OFDEVICE) && dev->device_node) {
		usb->mode = of_usb_get_dr_mode(dev->device_node, NULL);
		usb->phyintf = of_usb_get_phy_mode(dev->device_node, NULL);
	}

	return usb;
}

int chipidea_usb2_register(struct ci_usb *usb)
{
	u32 reg;
	int ret;

	reg = readl(usb->base + CI_HWHOST);
	if (reg & VUSB_HS_HOST)
		usb->caps |= CAPS_HOST;
	reg = readl(usb->base + CI_HWDEVICE);
	if (reg & VUSB_HS_DEVICE)
		usb->caps |= CAPS_DEVICE;

	dev_dbg(usb->dev, "ChipIdea USB %s controller\n",
		(usb->caps & (CAPS_HOST | CAPS_DEVICE)) ? "dual-role" :
		(usb->caps & CAPS_HOST) ? "host" :
		(usb->caps & CAPS_DEVICE) ? "device" : "unknown");

	if (usb->mode == USB_DR_MODE_HOST &&
	    usb->caps & CAPS_HOST &&
	    IS_ENABLED(CONFIG_USB_CHIPIDEA_HOST)) {
		ret = regulator_enable(usb->vbus);
		if (ret)
			return ret;
		ret = ehci_register(usb->dev, &usb->ehci);
		if (ret)
			regulator_disable(usb->vbus);
	} else if (usb->mode == USB_DR_MODE_PERIPHERAL &&
		   usb->caps & CAPS_DEVICE &&
		   IS_ENABLED(CONFIG_USB_CHIPIDEA_DEVICE)) {
		ret = regulator_disable(usb->vbus);
		if (ret)
			return ret;
		ret = ci_udc_register(usb->dev, usb->base);
	} else {
		dev_err(usb->dev, "Unsupported USB role\n");
		ret = -ENODEV;
	}

	return ret;
}

#if defined(CONFIG_USB_CHIPIDEA_GENERIC)
static int chipidea_usb2_probe(struct device_d *dev)
{
	struct ci_usb *usb;
	void __iomem *base;

	base = dev_request_mem_region(dev, 0);
	if (!base)
		return -ENOMEM;

	usb = chipidea_usb2_alloc(dev, base, NULL);
	if (IS_ERR(usb))
		return PTR_ERR(usb);

	return chipidea_usb2_register(usb);
}

static struct of_device_id chipidea_usb2_dt_ids[] = {
	{ .compatible = "chipidea,usb2-generic", },
	{ /* sentinel */ }
};

static struct driver_d chipidea_usb2_driver = {
	.name = "chipidea-usb2",
	.probe = chipidea_usb2_probe,
	.of_compatible	= chipidea_usb2_dt_ids,
};
device_platform_driver(chipidea_usb2_driver);
#endif
