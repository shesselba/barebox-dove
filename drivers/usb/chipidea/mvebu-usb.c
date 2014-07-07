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

#include "ci-drc.h"

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

static void mvebu_usb2_mbus_setup(void __iomem *base)
{
	const struct mbus_dram_target_info *dram = mvebu_mbus_dram_info();
	int n;

	for (n = 0; n < 4; n++) {
		writel(0, base + WINDOW_CTRL(n));
		writel(0, base + WINDOW_BASE(n));
	}

	for (n = 0; n < dram->num_cs; n++) {
		const struct mbus_dram_window *w = &dram->cs[n];
		u32 reg;

		writel(w->base, base + WINDOW_BASE(n));
		reg = ((w->size - 1) & 0xffff0000) | (w->mbus_attr << 8) |
			(dram->mbus_dram_target_id << 4) | 1;
		writel(reg, base + WINDOW_CTRL(n));
	}
}

static void mvebu_usb2_ipg_setup(void __iomem *base)
{
	u16 devid = mvebu_get_soc_devid();
	u16 revid = mvebu_get_soc_revid();
	u32 reg;

	/* IPG Metal fix register not available on below SoCs */
	if ((devid == DEVID_F5180 && revid <= REVID_F5180N_B1) ||
	    (devid == DEVID_F5181 && revid <= REVID_F5181_B1) ||
	    (devid == DEVID_F5181 && revid == REVID_F5181L) ||
	    (devid == DEVID_F5182 && revid <= REVID_F5182_A1))
		return;

	reg = readl(base + BRIDGE_IPG);
	/* Change reserved bits [31:30] from 1 to 0 */
	reg &= ~(BIT(31) | BIT(30));
	/* Change NON_START_IPG to 0xd */
	reg &= ~NON_START_IPG_MASK;
	reg |= NON_START_IPG(0xd);
	writel(reg, base + BRIDGE_IPG);
}

static struct of_device_id mvebu_usb2_dt_ids[] = {
	{ .compatible = "marvell,mvebu-usb2", },
	{ /* sentinel */ }
};

static int mvebu_usb2_probe(struct device_d *dev)
{
	struct ci_usb *usb;
	void __iomem *base;
	struct clk *clk;

	base = dev_request_mem_region(dev, 0);
	if (!base)
		return -ENOMEM;

	clk = clk_get(dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	clk_enable(clk);

	usb = chipidea_usb2_alloc(dev, base, NULL);
	if (IS_ERR(usb))
		return PTR_ERR(usb);

	mvebu_usb2_ipg_setup(base);
	mvebu_usb2_mbus_setup(base);

	return chipidea_usb2_register(usb);
}

static struct driver_d mvebu_usb2_driver = {
	.name = "mvebu-usb2",
	.probe = mvebu_usb2_probe,
	.of_compatible	= mvebu_usb2_dt_ids,
};
device_platform_driver(mvebu_usb2_driver);
