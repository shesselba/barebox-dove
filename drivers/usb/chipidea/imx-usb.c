/*
 * Copyright (c) 2012 Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <common.h>
#include <init.h>
#include <io.h>
#include <linux/err.h>
#include <usb/chipidea-imx.h>
#include <usb/ulpi.h>

#include "ci-drc.h"

struct imx_usb2 {
	int portno;
	unsigned long flags;
	enum imx_usb_mode mode;
};

static int imx_usb2_port_init(void *drvdata)
{
	struct ci_usb *usb = drvdata;
	struct imx_usb2 *priv = usb->priv;
	int ret;

	if ((priv->flags & MXC_EHCI_PORTSC_MASK) == MXC_EHCI_MODE_ULPI) {
		dev_dbg(usb->dev, "using ULPI phy\n");
		if (IS_ENABLED(CONFIG_USB_ULPI)) {
			ret = ulpi_setup(usb->base + 0x170, 1);
			if (ret)
				dev_err(usb->dev, "ULPI setup failed with %s\n",
					strerror(-ret));
			mdelay(20);
		} else {
			dev_err(usb->dev, "no ULPI support available\n");
			ret = -ENODEV;
		}

		if (ret)
			return ret;
	}

	ret = imx_usbmisc_port_init(priv->portno, priv->flags);
	if (ret)
		dev_err(usb->dev, "misc init failed: %s\n", strerror(-ret));

	return ret;
}

static int imx_usb2_port_post_init(void *drvdata)
{
	struct ci_usb *usb = drvdata;
	struct imx_usb2 *priv = usb->priv;
	int ret;

	ret = imx_usbmisc_port_post_init(priv->portno, priv->flags);
	if (ret)
		dev_err(usb->dev, "post misc init failed: %s\n", strerror(-ret));

	return ret;
}

static int imx_usb2_probe_dt(struct ci_usb *usb)
{
	struct imxusb_platformdata *pdata;
	struct device_node *np = usb->dev->device_node;
	struct of_phandle_args out_args;

	if (usb->dev->platform_data)
		return 0;

	pdata = xzalloc(sizeof(*pdata));
	usb->dev->platform_data = pdata;

	if (of_parse_phandle_with_args(np,
		       "fsl,usbmisc", "#index-cells", 0, &out_args))
		return -ENODEV;
	usb->dev->id = out_args.args[0];

	pdata->flags = MXC_EHCI_MODE_UTMI_8BIT;
	switch (usb->mode) {
	case USB_DR_MODE_HOST:
	default:
		pdata->mode = IMX_USB_MODE_HOST;
		break;
	case USB_DR_MODE_PERIPHERAL:
		pdata->mode = IMX_USB_MODE_DEVICE;
		break;
	}

	switch (usb->phyintf) {
	case USBPHY_INTERFACE_MODE_UTMI:
		pdata->flags = MXC_EHCI_MODE_UTMI_8BIT;
		break;
	case USBPHY_INTERFACE_MODE_UTMIW:
		pdata->flags = MXC_EHCI_MODE_UTMI_16_BIT;
		break;
	case USBPHY_INTERFACE_MODE_ULPI:
		pdata->flags = MXC_EHCI_MODE_ULPI;
		break;
	case USBPHY_INTERFACE_MODE_SERIAL:
		pdata->flags = MXC_EHCI_MODE_SERIAL;
		break;
	case USBPHY_INTERFACE_MODE_HSIC:
		pdata->flags = MXC_EHCI_MODE_HSIC;
		break;
	default:
		dev_dbg(usb->dev, "no phy_type setting. Relying on reset default\n");
	}

	if (of_find_property(np, "disable-over-current", NULL))
		pdata->flags |= MXC_EHCI_DISABLE_OVERCURRENT;

	return 0;
}

static int imx_usb2_probe(struct device_d *dev)
{
	struct imxusb_platformdata *pdata;
	struct imx_usb2 *priv;
	struct ci_usb *usb;
	void __iomem *base;
	int ret;

	priv = xzalloc(sizeof(*priv));
	base = dev_request_mem_region(dev, 0);
	if (!base)
		return -ENODEV;

	usb = chipidea_usb2_alloc(dev, base, priv);
	if (IS_ERR(usb))
		return PTR_ERR(usb);

	if (IS_ENABLED(CONFIG_OFDEVICE) && dev->device_node) {
		ret = imx_usb2_probe_dt(usb);
		if (ret)
			return ret;
	}

	pdata = dev->platform_data;
	if (!pdata) {
		dev_err(dev, "no pdata!\n");
		return -EINVAL;
	}

	priv->portno = dev->id;
	priv->flags = pdata->flags;
	priv->mode = pdata->mode;
	usb->ehci.init = imx_usb2_port_init;
	usb->ehci.post_init = imx_usb2_port_post_init;

	if ((priv->flags & MXC_EHCI_PORTSC_MASK) == MXC_EHCI_MODE_HSIC)
		imx_usb2_port_init(usb);

	if (usb->phyintf != USBPHY_INTERFACE_MODE_UNKNOWN) {
		u32 portsc = readl(usb->base + 0x184);
		portsc &= ~MXC_EHCI_PORTSC_MASK;
		portsc |= priv->flags & MXC_EHCI_PORTSC_MASK;
		writel(portsc, usb->base + 0x184);
	}

	return chipidea_usb2_register(usb);
};

static __maybe_unused struct of_device_id imx_usb2_dt_ids[] = {
	{ .compatible = "fsl,imx27-usb", },
	{ /* sentinel */ }
};

static struct driver_d imx_usb2_driver = {
	.name   = "imx-usb2",
	.probe  = imx_usb2_probe,
	.of_compatible = DRV_OF_COMPAT(imx_usb2_dt_ids),
};
device_platform_driver(imx_usb2_driver);
