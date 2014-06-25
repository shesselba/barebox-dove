/*
 * Marvell MVEBU USB PHY driver
 *
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 *
 * Based on BSP code (C) Marvell International Ltd.
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <common.h>
#include <init.h>
#include <io.h>
#include <linux/clk.h>
#include <linux/mbus.h>
#include <mach/socid.h>
#include <regulator.h>
#include <usb/ehci.h>
#include <usb/fsl_usb2.h>
#include <usb/usb.h>

#define EHCI_REGS_OFFSET	0x100

#define BRIDGE_CTRL		0x300
#define BRIDGE_INTR_CAUSE	0x310
#define BRIDGE_INTR_MASK	0x314
#define BRIDGE_ERR_ACCESS	0x31c
#define WINDOW_CTRL(i)		(0x320 + ((i) << 4))
#define WINDOW_BASE(i)		(0x324 + ((i) << 4))
#define BRIDGE_IPG		0x360
#define  START_IPG(x)		((x) << 0)
#define  START_IPG_MASK		START_IPG(0x3f)
#define  NON_START_IPG(x)	((x) << 8)
#define  NON_START_IPG_MASK	NON_START_IPG(0x3f)

struct mvebu_usb {
	struct ehci_data ehci;
	struct device_d *dev;
	void __iomem *base;
	struct clk *clk;
	struct regulator *vbus;
	u16 devid;
	u16 revid;
	enum usb_dr_mode mode;
};

static void mvebu_usb_mbus_setup(struct mvebu_usb *usb)
{
	const struct mbus_dram_target_info *dram = mvebu_mbus_dram_info();
	int n;

	for (n = 0; n < 4; n++) {
		writel(0, usb->base + WINDOW_CTRL(n));
		writel(0, usb->base + WINDOW_BASE(n));
	}

	for (n = 0; n < dram->num_cs; n++) {
		const struct mbus_dram_window *w = &dram->cs[n];
		u32 reg;

		writel(w->base, usb->base + WINDOW_BASE(n));
		reg = ((w->size - 1) & 0xffff0000) | (w->mbus_attr << 8) |
			(dram->mbus_dram_target_id << 4) | 1;
		writel(reg, usb->base + WINDOW_CTRL(n));
	}
}

static void mvebu_usb_ipg_setup(struct mvebu_usb *usb)
{
	u32 reg;

	/* IPG Metal fix register not available on below SoCs */
	if ((usb->devid == DEVID_F5180 && usb->revid <= REVID_F5180N_B1) ||
	    (usb->devid == DEVID_F5181 && usb->revid <= REVID_F5181_B1) ||
	    (usb->devid == DEVID_F5181 && usb->revid == REVID_F5181L) ||
	    (usb->devid == DEVID_F5182 && usb->revid <= REVID_F5182_A1))
		return;

	reg = readl(usb->base + BRIDGE_IPG);
	/* Change reserved bits [31:30] from 1 to 0 */
	reg &= ~(BIT(31) | BIT(30));
	/* Change NON_START_IPG to 0xd */
	reg &= ~NON_START_IPG_MASK;
	reg |= NON_START_IPG(0xd);
	writel(reg, usb->base + BRIDGE_IPG);
}

static struct of_device_id mvebu_usb_dt_ids[] = {
	{ .compatible = "marvell,mvebu-usb", },
};

static int mvebu_usb_probe(struct device_d *dev)
{
	struct mvebu_usb *usb;
	int ret;

	usb = xzalloc(sizeof(*usb));

	usb->base = dev_request_mem_region(dev, 0);
	if (!usb->base)
		return -ENOMEM;

	usb->clk = clk_get(dev, NULL);
	if (IS_ERR(usb->clk))
		return PTR_ERR(usb->clk);

	usb->vbus = regulator_get(dev, "vbus");
	if (IS_ERR(usb->vbus))
		return PTR_ERR(usb->vbus);

	usb->dev = dev;
	usb->devid = mvebu_get_soc_devid();
	usb->revid = mvebu_get_soc_revid();
	usb->mode = of_usb_get_dr_mode(dev->device_node, NULL);
	if (usb->mode == USB_DR_MODE_UNKNOWN)
		usb->mode = USB_DR_MODE_HOST;

	usb->ehci.hccr = usb->base + EHCI_REGS_OFFSET;
	usb->ehci.flags = EHCI_HAS_TT;

	clk_enable(usb->clk);

	mvebu_usb_ipg_setup(usb);
	mvebu_usb_mbus_setup(usb);

	if (usb->mode == USB_DR_MODE_HOST &&
	    IS_ENABLED(CONFIG_USB_MVEBU_HOST)) {
		ret = regulator_enable(usb->vbus);
		if (ret)
			return ret;
		ret = ehci_register(dev, &usb->ehci);
		if (ret)
			regulator_disable(usb->vbus);
	} else if (usb->mode == USB_DR_MODE_PERIPHERAL &&
		   IS_ENABLED(CONFIG_USB_MVEBU_DEVICE)) {
		ret = regulator_disable(usb->vbus);
		if (ret)
			return ret;
		ret = ci_udc_register(dev, usb->base);
	} else {
		dev_err(dev, "Unsupported USB role\n");
		ret = -ENODEV;
	}

	return ret;
}

static struct driver_d mvebu_usb_driver = {
	.name = "mvebu-usb",
	.probe = mvebu_usb_probe,
	.of_compatible	= mvebu_usb_dt_ids,
};
device_platform_driver(mvebu_usb_driver);
