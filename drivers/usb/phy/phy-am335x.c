// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/usb/otg.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/usb/of.h>

#include "phy-am335x-control.h"
#include "phy-generic.h"

struct am335x_phy {
	struct usb_phy_generic usb_phy_gen;
	struct phy_control *phy_ctrl;
	int id;
	enum usb_dr_mode dr_mode;
};

static int am335x_init(struct usb_phy *phy)	//Invoked by musb_core.c:musb_init_controller calling usb_phy_init.
{
//printk("drivers/usb/phy/phy-am335x.c:am335x_init START: %s\n", current->comm);
	struct am335x_phy *am_phy = dev_get_drvdata(phy->dev);
	phy_ctrl_power(am_phy->phy_ctrl, am_phy->id, am_phy->dr_mode, true);
//printk("drivers/usb/phy/phy-am335x.c:am335x_init END: %s\n", current->comm);
	return 0;
}

static void am335x_shutdown(struct usb_phy *phy)
{
	struct am335x_phy *am_phy = dev_get_drvdata(phy->dev);
//printk("drivers/usb/phy/phy-am335x.c:am335x_shutdown\n");
	phy_ctrl_power(am_phy->phy_ctrl, am_phy->id, am_phy->dr_mode, false);
}

static int am335x_phy_probe(struct platform_device *pdev)	//Invoked twice, once for each USB PHY. Named according to DTB node.
{
	struct am335x_phy *am_phy;
	struct device *dev = &pdev->dev;
	int ret;
//printk("drivers/usb/phy/phy-am335x.c:am335x_phy_probe START: %s, platform device name: %s\n", current->comm, pdev->name);

	am_phy = devm_kzalloc(dev, sizeof(*am_phy), GFP_KERNEL);				//Allocates memory for am335x_phy struct.
	if (!am_phy) {
//printk("drivers/usb/phy/phy-am335x.c:am335x_phy_probe END: %s, ENOMEM\n", current->comm);
		return -ENOMEM;
}

	am_phy->phy_ctrl = am335x_get_phy_control(dev);
	if (!am_phy->phy_ctrl) {
//printk("drivers/usb/phy/phy-am335x.c:am335x_phy_probe END: %s, EPROBE_DEFER\n", current->comm);
		return -EPROBE_DEFER;
}

	//Get the suffix number of the alias of phy0 or phy1 (0 or 1), as defined by
	//aliases {..., phy0 = &usb0_phy; phy1 = &usb1_phy; ...} in am33xx.dtsi.
	am_phy->id = of_alias_get_id(pdev->dev.of_node, "phy");
//printk("drivers/usb/phy/phy-am335x.c:am335x_phy_probe: %s, device tree node name %s\n", current->comm, pdev->dev.of_node->name);
	if (am_phy->id < 0) {
		dev_err(&pdev->dev, "Missing PHY id: %d\n", am_phy->id);
//printk("drivers/usb/phy/phy-am335x.c:am335x_phy_probe END: %s, Missing PHY\n", current->comm);
		return am_phy->id;
	}
	//Obtains "otg" from the device tree from the phy's parent usb device tree node field dr_mode = "otg";
	am_phy->dr_mode = of_usb_get_dr_mode_by_phy(pdev->dev.of_node, -1);
	ret = usb_phy_gen_create_phy(dev, &am_phy->usb_phy_gen);	//Initializes PHY and OTG fields.
	if (ret) {
//printk("drivers/usb/phy/phy-am335x.c:am335x_phy_probe END OTG %s\n", current->comm);
		return ret;
}

	am_phy->usb_phy_gen.phy.init = am335x_init;			//Initialization of PHY by invoking am335x_phy_power.
	am_phy->usb_phy_gen.phy.shutdown = am335x_shutdown;	//Shutting down PHY by invoking am335x_phy_wkup.
	platform_set_drvdata(pdev, am_phy);					//Driver specific data is the PHY structure.
	device_init_wakeup(dev, true);						//USB can wake up the system.

	/*
	 * If we leave PHY wakeup enabled then AM33XX wakes up
	 * immediately from DS0. To avoid this we mark dev->power.can_wakeup
	 * to false. The same is checked in suspend routine to decide
	 * on whether to enable PHY wakeup or not.
	 * PHY wakeup works fine in standby mode, there by allowing us to
	 * handle remote wakeup, wakeup on disconnect and connect.
	 */
	device_set_wakeup_enable(dev, false);				//Wake up disable.
	phy_ctrl_power(am_phy->phy_ctrl, am_phy->id, am_phy->dr_mode, false);	//Power-down with false when calling am335x_phy_power.
	//"[...]coordinate the activities of drivers for host and peripheral
	// controllers, and in some cases for VBUS current regulation."
	ret = usb_add_phy_dev(&am_phy->usb_phy_gen.phy);
//printk("drivers/usb/phy/phy-am335x.c:am335x_phy_probe END %s\n", current->comm);
	return ret;
}

static int am335x_phy_remove(struct platform_device *pdev)
{
	struct am335x_phy *am_phy = platform_get_drvdata(pdev);
//printk("drivers/usb/phy/phy-am335x.c:am335x_phy_remove\n");
	usb_remove_phy(&am_phy->usb_phy_gen.phy);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int am335x_phy_suspend(struct device *dev)
{
	struct am335x_phy *am_phy = dev_get_drvdata(dev);
//printk("drivers/usb/phy/phy-am335x.c:am335x_phy_suspend\n");
	/*
	 * Enable phy wakeup only if dev->power.can_wakeup is true.
	 * Make sure to enable wakeup to support remote wakeup	in
	 * standby mode ( same is not supported in OFF(DS0) mode).
	 * Enable it by doing
	 * echo enabled > /sys/bus/platform/devices/<usb-phy-id>/power/wakeup
	 */

	if (device_may_wakeup(dev))
		phy_ctrl_wkup(am_phy->phy_ctrl, am_phy->id, true);

	phy_ctrl_power(am_phy->phy_ctrl, am_phy->id, am_phy->dr_mode, false);

	return 0;
}

static int am335x_phy_resume(struct device *dev)
{
	struct am335x_phy	*am_phy = dev_get_drvdata(dev);
//printk("drivers/usb/phy/phy-am335x.c:am335x_phy_resume\n");
	phy_ctrl_power(am_phy->phy_ctrl, am_phy->id, am_phy->dr_mode, true);

	if (device_may_wakeup(dev))
		phy_ctrl_wkup(am_phy->phy_ctrl, am_phy->id, false);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(am335x_pm_ops, am335x_phy_suspend, am335x_phy_resume);

static const struct of_device_id am335x_phy_ids[] = {
	{ .compatible = "ti,am335x-usb-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, am335x_phy_ids);

static struct platform_driver am335x_phy_driver = {
	.probe          = am335x_phy_probe,
	.remove         = am335x_phy_remove,
	.driver         = {
		.name   = "am335x-phy-driver",
		.pm = &am335x_pm_ops,
		.of_match_table = am335x_phy_ids,
	},
};

module_platform_driver(am335x_phy_driver);
MODULE_LICENSE("GPL v2");
