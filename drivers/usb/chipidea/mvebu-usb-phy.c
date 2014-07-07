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
#include <mach/socid.h>

/* 40nm USB PHY registers */
#define PHY40N_PLL_REG(x)	(0x00 + ((x) * 0x04))
#define  PHY40N_PLL_POWERUP	BIT(9)
#define  PHY40N_VCO_CALIBRATE	BIT(21)
#define PHY40N_CHANNEL_REG(c,x)	(0x40 + ((c) * 0x40) + ((x) * 0x04))
#define  PHY40N_CH_RECALIBRATE	BIT(12)

/* 65nm+ USB PHY registers */
#define PHY_POWER_CTRL		0x00
#define PHY_PLL_CTRL		0x10
#define  KVCO_EXT		BIT(22)
#define  VCO_CALIBRATE		BIT(21)
#define  ICP(x)			((x) << 12)
#define  ICP_MASK		ICP(0x7)
#define PHY_TX_CTRL		0x20
#define  HS_STRESS_CTRL		BIT(31)
#define  TX_BLOCK_EN		BIT(21)
#define  IMP_CAL_VTH(x)		((x) << 14)
#define  IMP_CAL_VTH_MASK	IMP_CAL_VTH(0x7)
#define  TX_CALIBRATE		BIT(12)
#define  LOWVDD_EN		BIT(11)
#define  TX_AMP(x)		((x) << 0)
#define  TX_AMP_MASK		TX_AMP(0x7)
#define PHY_RX_CTRL		0x30
#define  EDGE_DET(x)		((x) << 26)
#define  EDGE_DET_1T		EDGE_DET(0x0)
#define  EDGE_DET_2T		EDGE_DET(0x1)
#define  EDGE_DET_3T		EDGE_DET(0x2)
#define  EDGE_DET_4T		EDGE_DET(0x3)
#define  EDGE_DET_MASK		EDGE_DET(0x3)
#define  CDR_FASTLOCK_EN	BIT(21)
#define  SQ_LENGTH(x)		((x) << 15)
#define  SQ_LENGTH_MASK		SQ_LENGTH(0x3)
#define  SQ_THRESH(x)		((x) << 4)
#define  SQ_THRESH_MASK		SQ_THRESH(0xf)
#define  LPF_COEFF(x)		((x) << 2)
#define  LPF_COEFF_1_8		LPF_COEFF(0x0)
#define  LPF_COEFF_1_4		LPF_COEFF(0x1)
#define  LPF_COEFF_1_2		LPF_COEFF(0x2)
#define  LPF_COEFF_1_1		LPF_COEFF(0x3)
#define  LPF_COEFF_MASK		LPF_COEFF(0x3)
#define PHY_IVREF_CTRL		0x440
#define  TXVDD12(x)		((x) << 8)
#define  TXVDD12_VDD		TXVDD12(0x0)
#define  TXVDD12_1V2		TXVDD12(0x1)
#define  TXVDD12_1V3		TXVDD12(0x2)
#define  TXVDD12_1V4		TXVDD12(0x3)
#define  TXVDD12_MASK		TXVDD12(0x3)
#define PHY_TESTGRP0_CTRL	0x50
#define  FIFO_SQ_RST		BIT(15)
#define PHY_TESTGRP1_CTRL	0x54
#define PHY_TESTGRP2_CTRL	0x58
#define PHY_TESTGRP3_CTRL	0x5c

struct mvebu_usb2_phy {
	struct device_d *dev;
	void __iomem *base;
	struct clk *clk;
	u16 devid;
	u16 revid;
	int (*setup)(struct mvebu_usb2_phy *phy);
};

static __maybe_unused int mvebu_usb2_phy_setup_40nm(struct mvebu_usb2_phy *phy)
{
	struct device_node *cnp;
	u32 reg;

	/* Set USB PLL REF frequency to 25MHz */
	reg = readl(phy->base + PHY40N_PLL_REG(1));
	reg &= ~0x3ff;
	reg |= 0x605;
	writel(reg, phy->base + PHY40N_PLL_REG(1));

	/* Power up PLL and PHY channel */
	reg = readl(phy->base + PHY40N_PLL_REG(2));
	reg |= PHY40N_PLL_POWERUP;
	writel(reg, phy->base + PHY40N_PLL_REG(2));

	/* Calibrate VCO */
	reg = readl(phy->base + PHY40N_PLL_REG(1));
	reg |= PHY40N_VCO_CALIBRATE;
	writel(reg, phy->base + PHY40N_PLL_REG(1));
	udelay(1000);

	/* Setup all individual PHYs */
	for_each_child_of_node(phy->dev->device_node, cnp) {
		u32 n;

		if (of_property_read_u32(cnp, "reg", &n))
			continue;

		reg = readl(phy->base + PHY40N_CHANNEL_REG(n, 3));
		reg |= BIT(15);
		writel(reg, phy->base + PHY40N_CHANNEL_REG(n, 3));

		reg = readl(phy->base + PHY40N_CHANNEL_REG(n, 1));
		reg |= PHY40N_CH_RECALIBRATE;
		writel(reg, phy->base + PHY40N_CHANNEL_REG(n, 1));

		udelay(40);

		reg = readl(phy->base + PHY40N_CHANNEL_REG(n, 1));
		reg &= ~PHY40N_CH_RECALIBRATE;
		writel(reg, phy->base + PHY40N_CHANNEL_REG(n, 1));

		switch (phy->devid) {
		case DEVID_F6707:
		case DEVID_F6710:
			writel(0x20000131, phy->base + PHY40N_CHANNEL_REG(n, 4));
			break;
		}
	}

	return 0;
}

static __maybe_unused int mvebu_usb2_phy_setup_65nm(struct mvebu_usb2_phy *phy)
{
	u32 reg;

	/* USB PHY PLL */
	reg = readl(phy->base + PHY_PLL_CTRL);
	writel(reg | VCO_CALIBRATE, phy->base + PHY_PLL_CTRL);
	udelay(100);
	writel(reg & ~VCO_CALIBRATE, phy->base + PHY_PLL_CTRL);

	/* USB PHY Tx */
	reg = readl(phy->base + PHY_TX_CTRL);
	reg &= ~TX_CALIBRATE;
	writel(reg | TX_CALIBRATE, phy->base + PHY_TX_CTRL);
	udelay(100);
	writel(reg & ~TX_CALIBRATE, phy->base + PHY_TX_CTRL);

	switch (phy->devid) {
	case DEVID_AP510:
	case DEVID_F6781:
		reg &= ~(TX_BLOCK_EN | HS_STRESS_CTRL);
		reg |= LOWVDD_EN;
		break;
	}

	switch (phy->devid) {
	case DEVID_AP510:
	case DEVID_F6280:
		reg = (reg & ~IMP_CAL_VTH_MASK) | IMP_CAL_VTH(0x5);
		break;
	}

	reg &= ~TX_AMP_MASK;
	switch (phy->devid) {
	case DEVID_F6321:
	case DEVID_F6322:
	case DEVID_F6323:
	case DEVID_MV76100:
	case DEVID_MV78100:
	case DEVID_MV78200:
		reg |= TX_AMP(0x4);
		break;
	default:
		reg |= TX_AMP(0x3);
		break;
	}
	writel(reg, phy->base + PHY_TX_CTRL);

	/* USB PHY Rx */
	reg = readl(phy->base + PHY_RX_CTRL);

	reg = (reg & ~LPF_COEFF_MASK) | LPF_COEFF_1_4;

	reg &= ~SQ_THRESH_MASK;
	switch (phy->devid) {
	case DEVID_AP510:
	case DEVID_F6282:
		reg |= SQ_THRESH(0xc);
		break;
	case DEVID_F6781:
		reg |= SQ_THRESH(0x7);
		break;
	default:
		reg |= SQ_THRESH(0x8);
		break;
	}

	if (phy->devid == DEVID_AP510 ||
	    phy->devid == DEVID_F6781) {
		reg = (reg & ~SQ_LENGTH_MASK) | SQ_LENGTH(0x1);
		reg = (reg & ~EDGE_DET_MASK) | EDGE_DET_1T;
		reg &= ~CDR_FASTLOCK_EN;
	}
	writel(reg, phy->base + PHY_RX_CTRL);

	/* USB PHY IVREF */
	reg = readl(phy->base + PHY_IVREF_CTRL);
	reg &= ~TXVDD12_MASK;
	switch (phy->devid) {
	case DEVID_AP510:
	case DEVID_F6180:
	case DEVID_F6190:
	case DEVID_F6192:
	case DEVID_F6280:
	case DEVID_F6281:
	case DEVID_F6282:
	case DEVID_F6781:
		reg |= TXVDD12_1V4;
		break;
	default:
		reg |= TXVDD12_1V2;
		break;
	}
	writel(reg, phy->base + PHY_IVREF_CTRL);

	/* USB PHY Test Group */
	reg = readl(phy->base + PHY_TESTGRP0_CTRL);
	if (phy->devid == DEVID_AP510 ||
	    phy->devid == DEVID_F6781)
		reg &= ~FIFO_SQ_RST;
	writel(reg, phy->base + PHY_TESTGRP0_CTRL);

	return 0;
}

static __maybe_unused int mvebu_usb2_phy_setup_90nm(struct mvebu_usb2_phy *phy)
{
	return -ENODEV;
}

static __maybe_unused int mvebu_usb2_phy_setup_150nm(struct mvebu_usb2_phy *phy)
{
	return -ENODEV;
}

static struct of_device_id mvebu_usb2_phy_dt_ids[] = {
#if defined(CONFIG_USB_MVEBU_PHY_40NM)
	{ .compatible = "marvell,mvebu-usb2-phy-40nm",
	  .data = (u32)mvebu_usb2_phy_setup_40nm },
#endif
#if defined(CONFIG_USB_MVEBU_PHY_65NM)
	{ .compatible = "marvell,mvebu-usb2-phy-65nm",
	  .data = (u32)mvebu_usb2_phy_setup_65nm },
#endif
#if defined(CONFIG_USB_MVEBU_PHY_90NM)
	{ .compatible = "marvell,mvebu-usb2-phy-90nm",
	  .data = (u32)mvebu_usb2_phy_setup_90nm },
#endif
#if defined(CONFIG_USB_MVEBU_PHY_150NM)
	{ .compatible = "marvell,mvebu-usb2-phy-150nm",
	  .data = (u32)mvebu_usb2_phy_setup_150nm },
	{},
#endif
};

static int mvebu_usb2_phy_probe(struct device_d *dev)
{
	struct mvebu_usb2_phy *phy;
	const struct of_device_id *match =
		of_match_node(mvebu_usb2_phy_dt_ids, dev->device_node);

	phy = xzalloc(sizeof(*phy));
	phy->base = dev_request_mem_region(dev, 0);
	if (!phy->base)
		return -ENOMEM;

	phy->clk = clk_get(dev, NULL);
	if (IS_ERR(phy->clk))
		return PTR_ERR(phy->clk);

	phy->dev = dev;
	phy->devid = mvebu_get_soc_devid();
	phy->revid = mvebu_get_soc_revid();
	phy->setup = (void *)match->data;

	clk_enable(phy->clk);
	return phy->setup(phy);
}

static struct driver_d mvebu_usb2_phy_driver = {
	.name = "mvebu-usb2-phy",
	.probe = mvebu_usb2_phy_probe,
	.of_compatible	= mvebu_usb2_phy_dt_ids,
};
device_platform_driver(mvebu_usb2_phy_driver);
