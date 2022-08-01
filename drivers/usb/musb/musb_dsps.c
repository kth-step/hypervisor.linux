// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments DSPS platforms "glue layer"
 *
 * Copyright (C) 2012, by Texas Instruments
 *
 * Based on the am35x "glue layer" code.
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * musb_dsps.c will be a common file for all the TI DSPS platforms
 * such as dm64x, dm36x, dm35x, da8x, am35x and ti81x.
 * For now only ti81x is using this and in future davinci.c, am35x.c
 * da8xx.c would be merged to this file after testing.
 */

#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/module.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/platform_data/usb-omap.h>
#include <linux/sizes.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/usb/of.h>

#include <linux/debugfs.h>

#include "musb_core.h"

static const struct of_device_id musb_dsps_of_match[];

/*
 * DSPS musb wrapper register offset.
 * FIXME: This should be expanded to have all the wrapper registers from TI DSPS
 * musb ips.
 */
struct dsps_musb_wrapper {
	u16	revision;
	u16	control;
	u16	status;
	u16	epintr_set;
	u16	epintr_clear;
	u16	epintr_status;
	u16	coreintr_set;
	u16	coreintr_clear;
	u16	coreintr_status;
	u16	phy_utmi;
	u16	mode;
	u16	tx_mode;
	u16	rx_mode;

	/* bit positions for control */
	unsigned	reset:5;

	/* bit positions for interrupt */
	unsigned	usb_shift:5;
	u32		usb_mask;
	u32		usb_bitmap;
	unsigned	drvvbus:5;

	unsigned	txep_shift:5;
	u32		txep_mask;
	u32		txep_bitmap;

	unsigned	rxep_shift:5;
	u32		rxep_mask;
	u32		rxep_bitmap;

	/* bit positions for phy_utmi */
	unsigned	otg_disable:5;

	/* bit positions for mode */
	unsigned	iddig:5;
	unsigned	iddig_mux:5;
	/* miscellaneous stuff */
	unsigned	poll_timeout;
};

/*
 * register shadow for suspend
 */
struct dsps_context {
	u32 control;
	u32 epintr;
	u32 coreintr;
	u32 phy_utmi;
	u32 mode;
	u32 tx_mode;
	u32 rx_mode;
};

/*
 * DSPS glue structure.
 */
struct dsps_glue {
	struct device *dev;
	struct platform_device *musb;	/* child musb pdev */
	const struct dsps_musb_wrapper *wrp; /* wrapper register offsets */
	int vbus_irq;			/* optional vbus irq */
	unsigned long last_timer;    /* last timer data for each instance */
	bool sw_babble_enabled;
	void __iomem *usbss_base;

	struct dsps_context context;
	struct debugfs_regset32 regset;
	struct dentry *dbgfs_root;
};

static const struct debugfs_reg32 dsps_musb_regs[] = {
	{ "revision",		0x00 },
	{ "control",		0x14 },
	{ "status",		0x18 },
	{ "eoi",		0x24 },
	{ "intr0_stat",		0x30 },
	{ "intr1_stat",		0x34 },
	{ "intr0_set",		0x38 },
	{ "intr1_set",		0x3c },
	{ "txmode",		0x70 },
	{ "rxmode",		0x74 },
	{ "autoreq",		0xd0 },
	{ "srpfixtime",		0xd4 },
	{ "tdown",		0xd8 },
	{ "phy_utmi",		0xe0 },
	{ "mode",		0xe8 },
};

//Converts 2ms to jiffies (see poll_timeout field of am33xx_driver_data) and
//sets the timeout to the corresponding number of jiffies.
static void dsps_mod_timer(struct dsps_glue *glue, int wait_ms)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_mod_timer\n");
	struct musb *musb = platform_get_drvdata(glue->musb);
	int wait;

	if (wait_ms < 0)
		wait = msecs_to_jiffies(glue->wrp->poll_timeout);
	else
		wait = msecs_to_jiffies(wait_ms);

	mod_timer(&musb->dev_timer, jiffies + wait);
}

/*
 * If no vbus irq from the PMIC is configured, we need to poll VBUS status.
 */
static void dsps_mod_timer_optional(struct dsps_glue *glue)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_mod_timer_optional\n");
	if (glue->vbus_irq)
		return;

	//Converts 2ms to jiffies (see poll_timeout field of am33xx_driver_data) and
	//sets the timeout to the corresponding number of jiffies.
	dsps_mod_timer(glue, -1);
}

/* USBSS  / USB AM335x */
#define USBSS_IRQ_STATUS	0x28
#define USBSS_IRQ_ENABLER	0x2c
#define USBSS_IRQ_CLEARR	0x30

#define USBSS_IRQ_PD_COMP	(1 << 2)

/*
 * dsps_musb_enable - enable interrupts
 */
//Enable MUSB interrupts via USB0/1 TRM registers.
static void dsps_musb_enable(struct musb *musb)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_musb_enable\n");
	struct device *dev = musb->controller;
	struct dsps_glue *glue = dev_get_drvdata(dev->parent);
	const struct dsps_musb_wrapper *wrp = glue->wrp;
	void __iomem *reg_base = musb->ctrl_base;
	u32 epmask, coremask;

	/* Workaround: setup IRQs through both register sets. */
	epmask = ((musb->epmask & wrp->txep_mask) << wrp->txep_shift) |
	       ((musb->epmask & wrp->rxep_mask) << wrp->rxep_shift);
	coremask = (wrp->usb_bitmap & ~MUSB_INTR_SOF);

	musb_writel(reg_base, wrp->epintr_set, epmask);		//USB0/1IRQENABLESET0. Interrupt enable for TX/RX endpoints.
	musb_writel(reg_base, wrp->coreintr_set, coremask);	//USB0/1IRQENABLESET1. Interrupt enable for TX FIFO endpoints and USB interrupts.
	/*
	 * start polling for runtime PM active and idle,
	 * and for ID change in dual-role idle mode.
	 */
	if (musb->xceiv->otg->state == OTG_STATE_B_IDLE)
		//Converts 2ms to jiffies (see poll_timeout field of am33xx_driver_data)
		//and sets the timeout to the corresponding number of jiffies.
		dsps_mod_timer(glue, -1);
}

/*
 * dsps_musb_disable - disable HDRC and flush interrupts
 */
//Disables interrupts and removes NAK timer?
static void dsps_musb_disable(struct musb *musb)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_musb_disable START\n");
	struct device *dev = musb->controller;
	struct dsps_glue *glue = dev_get_drvdata(dev->parent);
	const struct dsps_musb_wrapper *wrp = glue->wrp;
	void __iomem *reg_base = musb->ctrl_base;						//USB0/1 TRM registers.

	musb_writel(reg_base, wrp->coreintr_clear, wrp->usb_bitmap);	//USB0/1IRQENABLECLR1: Disable interrupts for TX FIFOs and USB interrupts.
	musb_writel(reg_base, wrp->epintr_clear,						//USB0/1IRQENABLECLR0: Disable interrupts for TX/RX endpoints.
			 wrp->txep_bitmap | wrp->rxep_bitmap);
	del_timer_sync(&musb->dev_timer);								//Remove timer used for NAKs?
//printk("drivers/usb/musb/musb_dsps.c:dsps_musb_disable END\n");
}

/* Caller must take musb->lock */
//Checks status of OTG protocol.
static int dsps_check_status(struct musb *musb, void *unused)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_check_status\n");
	void __iomem *mregs = musb->mregs;
	struct device *dev = musb->controller;
	struct dsps_glue *glue = dev_get_drvdata(dev->parent);
	const struct dsps_musb_wrapper *wrp = glue->wrp;
	u8 devctl;
	int skip_session = 0;

	if (glue->vbus_irq)
		del_timer(&musb->dev_timer);

	/*
	 * We poll because DSPS IP's won't expose several OTG-critical
	 * status change events (from the transceiver) otherwise.
	 */
	devctl = musb_readb(mregs, MUSB_DEVCTL);
	dev_dbg(musb->controller, "Poll devctl %02x (%s)\n", devctl,
				usb_otg_state_string(musb->xceiv->otg->state));

	switch (musb->xceiv->otg->state) {
	case OTG_STATE_A_WAIT_VRISE:
		if (musb->port_mode == MUSB_HOST) {
			musb->xceiv->otg->state = OTG_STATE_A_WAIT_BCON;
			dsps_mod_timer_optional(glue);
			break;
		}
		fallthrough;

	case OTG_STATE_A_WAIT_BCON:
		/* keep VBUS on for host-only mode */
		if (musb->port_mode == MUSB_HOST) {
			dsps_mod_timer_optional(glue);
			break;
		}
		musb_writeb(musb->mregs, MUSB_DEVCTL, 0);
		skip_session = 1;
		fallthrough;

	case OTG_STATE_A_IDLE:
	case OTG_STATE_B_IDLE:
		if (!glue->vbus_irq) {
			if (devctl & MUSB_DEVCTL_BDEVICE) {
				musb->xceiv->otg->state = OTG_STATE_B_IDLE;
				MUSB_DEV_MODE(musb);
			} else {
				musb->xceiv->otg->state = OTG_STATE_A_IDLE;
				MUSB_HST_MODE(musb);
			}

			if (musb->port_mode == MUSB_PERIPHERAL)
				skip_session = 1;

			if (!(devctl & MUSB_DEVCTL_SESSION) && !skip_session)
				musb_writeb(mregs, MUSB_DEVCTL,
					    MUSB_DEVCTL_SESSION);
		}
		dsps_mod_timer_optional(glue);
		break;
	case OTG_STATE_A_WAIT_VFALL:
		musb->xceiv->otg->state = OTG_STATE_A_WAIT_VRISE;
		musb_writel(musb->ctrl_base, wrp->coreintr_set,
			    MUSB_INTR_VBUSERROR << wrp->usb_shift);
		break;
	default:
		break;
	}

	return 0;
}

//Set as a callback for timeout during initialization.
//Adds USB0/1 controller to resume operation queue.
static void otg_timer(struct timer_list *t)
{
//printk("drivers/usb/musb/musb_dsps.c:otg_timer\n");
	struct musb *musb = from_timer(musb, t, dev_timer);
	struct device *dev = musb->controller;	//Linux device structure of the USB0/1 controller.
	unsigned long flags;
	int err;

	err = pm_runtime_get(dev);				//Add USB0/1 controller to resume queue.
	if ((err != -EINPROGRESS) && err < 0) {
		dev_err(dev, "Poll could not pm_runtime_get: %i\n", err);
		pm_runtime_put_noidle(dev);

		return;
	}

	spin_lock_irqsave(&musb->lock, flags);
	//"run work if device is active or else queue the work to happen on resume."
	err = musb_queue_resume_work(musb, dsps_check_status, NULL);
	if (err < 0)
		dev_err(dev, "%s resume work: %i\n", __func__, err);
	spin_unlock_irqrestore(&musb->lock, flags);
	pm_runtime_mark_last_busy(dev);		//"Update the last access time of a device."
	pm_runtime_put_autosuspend(dev);	//"Drop device usage counter and queue autosuspend if 0."
}

//Clears TI TRM USB0/1IRQSTAT0 status for given RX endpoint.
static void dsps_musb_clear_ep_rxintr(struct musb *musb, int epnum)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_musb_clear_ep_rxintr\n");
	u32 epintr;
	struct dsps_glue *glue = dev_get_drvdata(musb->controller->parent);
	const struct dsps_musb_wrapper *wrp = glue->wrp;

	/* musb->lock might already been held */
	epintr = (1 << epnum) << wrp->rxep_shift;
	musb_writel(musb->ctrl_base, wrp->epintr_status, epintr);	//USB0/1IRQSTAT0. Clear status for given RX endpoint.
}

//Reads status of interrupts and clears them.
//
//Updates the session request protocol OTG state, and if necessary sets the
//timer to pull for VBUS status if there is no VBUS interrupt from the power
//management chip.
//
//Handles interrupts of TX/RX endpoints, and USB interrupts related to device
//resume, SRP, VBUS, suspend, device connection/disconnection, and reset.
static irqreturn_t dsps_interrupt(int irq, void *hci)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_interrupt\n");
	struct musb  *musb = hci;
	void __iomem *reg_base = musb->ctrl_base;						//USB0/1 TRM registers.
	struct device *dev = musb->controller;
	struct dsps_glue *glue = dev_get_drvdata(dev->parent);
	const struct dsps_musb_wrapper *wrp = glue->wrp;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	u32 epintr, usbintr;

	spin_lock_irqsave(&musb->lock, flags);

	/* Get endpoint interrupts */
	epintr = musb_readl(reg_base, wrp->epintr_status);				//USB0/1IRQSTAT0. Interrupt status for TX/RX endpoints.
	musb->int_rx = (epintr & wrp->rxep_bitmap) >> wrp->rxep_shift;	//RX endpoint interrupt. rxep_bitmap = (0xfffe << 16).
	musb->int_tx = (epintr & wrp->txep_bitmap) >> wrp->txep_shift;	//TX endpoint interrupt. txep_bitmap = (0xffff << 0).

	if (epintr)														//If asserted interrupts,
		musb_writel(reg_base, wrp->epintr_status, epintr);			//write bits to clear asserted interrupts.

	/* Get usb core interrupts */
	usbintr = musb_readl(reg_base, wrp->coreintr_status);			//Read USB0IRQSTAT1: Get interrupt status of TX FIFOs and USB.
	if (!usbintr && !epintr)										//No interrupt asserted: Return.
		goto out;

	//Chip specific offsets of interrupts? See documentation of int_usb.
	//wrp->usb_shift shifts bit fields depending on chip?
	//Get interrupts of MUSB except generic interrupt flag, which corresponds to
	//the INTRUSB register of MUSB.
	musb->int_usb =	(usbintr & wrp->usb_bitmap) >> wrp->usb_shift;
	if (usbintr)													//If asserted interrupts,
		musb_writel(reg_base, wrp->coreintr_status, usbintr);		//write bits to clear asserted interrupts.

	dev_dbg(musb->controller, "usbintr (%x) epintr(%x)\n",
			usbintr, epintr);

	//TRM 16.2.1:
	//"The USB controller drives the USB_DRVVBUS signal high when it assumes the
	// role of a host while the controller is in session."
	//
	//Wikipedia:
	//"Session Request Protocol (SRP): Allows both communicating devices to
	// control when the link's power session is active; in standard USB, only
	// the host is capable of doing so."
	if (usbintr & ((1 << wrp->drvvbus) << wrp->usb_shift)) {		//drvvbus indicates bit position of driving VBUS interrupt?
		int drvvbus = musb_readl(reg_base, wrp->status);			//wrp->status identifies offset of interrupt status register for VBUS?
		void __iomem *mregs = musb->mregs;							//Undocumented mentor graphics registers for MUSB0/1.
		u8 devctl = musb_readb(mregs, MUSB_DEVCTL);					//MUSB_DEVCTL is offset of MUSB0/1 control register.
		int err;
		//Session end interrupt error? None or both host and peripheral driving VBUS? None or both providing power?
		err = musb->int_usb & MUSB_INTR_VBUSERROR;
		if (err) {
			/*
			 * The Mentor core doesn't debounce VBUS as needed
			 * to cope with device connect current spikes. This
			 * means it's not uncommon for bus-powered devices
			 * to get VBUS errors during enumeration.
			 *
			 * This is a workaround, but newer RTL from Mentor
			 * seems to allow a better one: "re"-starting sessions
			 * without waiting for VBUS to stop registering in
			 * devctl.
			 */
			musb->int_usb &= ~MUSB_INTR_VBUSERROR;					//Clear VBUS error interrupt.
			musb->xceiv->otg->state = OTG_STATE_A_WAIT_VFALL;		//Wait for VBUS to get low?
			//Sets timer to pull for VBUS status if there is no VBUS interrupt from the power management chip.
			dsps_mod_timer_optional(glue);
			WARNING("VBUS error workaround (delay coming)\n");
		} else if (drvvbus) {		//Host driver power.
			MUSB_HST_MODE(musb);	//MUSB0/1 is set to be host.
			musb->xceiv->otg->state = OTG_STATE_A_WAIT_VRISE;		//Wait for VBUS to get high?
			//Sets timer to pull for VBUS status if there is no VBUS interrupt from the power management chip.
			dsps_mod_timer_optional(glue);
		} else {					//Host does not drive power.
			musb->is_active = 0;
			MUSB_DEV_MODE(musb);	//MUSB0/1 is set to be peripheral.
			musb->xceiv->otg->state = OTG_STATE_B_IDLE;				//Go idle on bus? Release bus? Just go idle as controller?
		}

		/* NOTE: this must complete power-on within 100 ms. */
		dev_dbg(musb->controller, "VBUS %s (%s)%s, devctl %02x\n",
				drvvbus ? "on" : "off",
				usb_otg_state_string(musb->xceiv->otg->state),
				err ? " ERROR" : "",
				devctl);
		ret = IRQ_HANDLED;											//"interrupt was handled by this device"
	}

	if (musb->int_tx || musb->int_rx || musb->int_usb)				//TX, RX, or USB interrupt requires additional handling.
		//Handles interrupts of TX/RX endpoints, and USB interrupts related to
		//device resume, SRP, VBUS, suspend, device connection/disconnection,
		//and reset.
		ret |= musb_interrupt(musb);

	/* Poll for ID change and connect */
	switch (musb->xceiv->otg->state) {
	case OTG_STATE_B_IDLE:
	case OTG_STATE_A_WAIT_BCON:
		//Sets timer to pull for VBUS status if there is no VBUS interrupt from the power management chip.
		dsps_mod_timer_optional(glue);
		break;
	default:
		break;
	}

out:
	spin_unlock_irqrestore(&musb->lock, flags);

	return ret;
}

//Creates debug file for printing register values.
static int dsps_musb_dbg_init(struct musb *musb, struct dsps_glue *glue)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_musb_dbg_init\n");
	struct dentry *root;
	char buf[128];

	sprintf(buf, "%s.dsps", dev_name(musb->controller));
	root = debugfs_create_dir(buf, usb_debug_root);
	glue->dbgfs_root = root;

	glue->regset.regs = dsps_musb_regs;
	glue->regset.nregs = ARRAY_SIZE(dsps_musb_regs);
	glue->regset.base = musb->ctrl_base;
	//Creates debug file for printing register values, where &glue->regset is a
	//pointer to a structure with register definitions initialized a few lines
	//above to dsps_musb_regs with a base register being USB0/1.
	debugfs_create_regset32("regdump", S_IRUGO, root, &glue->regset);
	return 0;
}

//Allocates data structures, initializes timer, resets hardware, and initializes
//data structure fields.
static int dsps_musb_init(struct musb *musb)	//Invoked by drivers/usb/musb/musb_core.h:musb_platform_init.
{
//printk("drivers/usb/musb/musb_dsps.c:dspsf_musb_init\n");
	struct device *dev = musb->controller;		//Linux device structure for USB controller. Set by musb_core.c:allocate_instance.
	//USB0/1 controller musb device linux structure's parent (the complete USB device) set by
	//dsps_create_musb_pdev (musb->dev.parent) and musb_core.c:allocate_instance.
	struct dsps_glue *glue = dev_get_drvdata(dev->parent);					//Driver data of complete USB controller (including USB0/1).
	struct platform_device *parent = to_platform_device(dev->parent);		//USB device (not controller) platform device.
	const struct dsps_musb_wrapper *wrp = glue->wrp;						//Initialized by dsps_probe.
	void __iomem *reg_base;
	struct resource *r;
	u32 rev, val;
	int ret;

	r = platform_get_resource_byname(parent, IORESOURCE_MEM, "control");	//USB0 and USB1 registers in TRM.
	reg_base = devm_ioremap_resource(dev, r);								//Maps the virtual addresses to the physical addresses of USB0/1.
	if (IS_ERR(reg_base))
		return PTR_ERR(reg_base);
	//Virtual addresses of USB0/USB1 for this USB controller (0/1). Overwrites
	//assignment by musb_core.c:allocate_instance.
	musb->ctrl_base = reg_base;

	/* NOP driver needs change if supporting dual instance */
	//Gets pointer to a struct usb_phy for the property "phys" in dts.
	//USB0 gets usb-phy@1300 in dts and USB1 gets usb-phy@1b00.
	musb->xceiv = devm_usb_get_phy_by_phandle(dev->parent, "phys", 0);

	if (IS_ERR(musb->xceiv))	//FALSE
		return PTR_ERR(musb->xceiv);

	//Obtains struct phy including a pointer to its Linux device structure for
	//the PHY. No such PHY in the device tree and musb->phy is ERROR
	musb->phy = devm_phy_get(dev->parent, "usb2-phy");

	/* Returns zero if e.g. not clocked */
	rev = musb_readl(reg_base, wrp->revision);	//USB0REV or USB1REV register.
	if (!rev)
		return -ENODEV;

	if (IS_ERR(musb->phy))  {	//TRUE!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		musb->phy = NULL;
	} else {
		ret = phy_init(musb->phy);
		if (ret < 0)
			return ret;
		ret = phy_power_on(musb->phy);
		if (ret) {
			phy_exit(musb->phy);
			return ret;
		}
	}

	//Initializes a timer with otg_timer being the function called when the
	//timer expires. musb.dev_timer is a struct in musb. No timer flags are set.
	timer_setup(&musb->dev_timer, otg_timer, 0);

	/* Reset the musb */
	//Writes USB0/1CTRL register field SOFT_RESET which initializes software
	//reset.
	musb_writel(reg_base, wrp->control, (1 << wrp->reset));

	musb->isr = dsps_interrupt;

	/* reset the otgdisable bit, needed for host mode to work */
	val = musb_readl(reg_base, wrp->phy_utmi);	//Reads the USB0/1UTMI register
	val &= ~(1 << wrp->otg_disable);	//Clears the OTGDISABLE bit of the USB0/1UTMI register
	//Writes the value of the register OTGDISABLE to USB0/1 CORE. The actual
	//register is undocumented.
	musb_writel(musb->ctrl_base, wrp->phy_utmi, val);

	/*
	 *  Check whether the dsps version has babble control enabled.
	 * In latest silicon revision the babble control logic is enabled.
	 * If MUSB_BABBLE_CTL returns 0x4 then we have the babble control
	 * logic enabled.
	 */
	val = musb_readb(musb->mregs, MUSB_BABBLE_CTL);	//Reads USB0/1CORE undocumented register.
	if (val & MUSB_BABBLE_RCV_DISABLE) {
		glue->sw_babble_enabled = true;
		val |= MUSB_BABBLE_SW_SESSION_CTRL;
		musb_writeb(musb->mregs, MUSB_BABBLE_CTL, val);
	}

	//Converts 2ms to jiffies (see poll_timeout field of am33xx_driver_data) and
	//sets the timeout to the corresponding number of jiffies.
	dsps_mod_timer(glue, -1);

	return dsps_musb_dbg_init(musb, glue);	//Initializes debugging data structures and debugging with the kernel.
}

//Turns off PHY.
static int dsps_musb_exit(struct musb *musb)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_musb_exit\n");
	struct device *dev = musb->controller;
	struct dsps_glue *glue = dev_get_drvdata(dev->parent);

	del_timer_sync(&musb->dev_timer);
	phy_power_off(musb->phy);					//Turns off PHY.
	phy_exit(musb->phy);						//Power management related to PHY?
	debugfs_remove_recursive(glue->dbgfs_root);

	return 0;
}

//Sets the (M)USB0/1 mode to host, peripheral, OTG.
static int dsps_musb_set_mode(struct musb *musb, u8 mode)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_musb_set_mode\n");
	struct device *dev = musb->controller;							//Device structure of MUSB0/1 controller.
	struct dsps_glue *glue = dev_get_drvdata(dev->parent);
	const struct dsps_musb_wrapper *wrp = glue->wrp;
	void __iomem *ctrl_base = musb->ctrl_base;						//MUSB0/1 registers
	u32 reg;

	reg = musb_readl(ctrl_base, wrp->mode);

	switch (mode) {
	case MUSB_HOST:
		reg &= ~(1 << wrp->iddig);

		/*
		 * if we're setting mode to host-only or device-only, we're
		 * going to ignore whatever the PHY sends us and just force
		 * ID pin status by SW
		 */
		reg |= (1 << wrp->iddig_mux);

		musb_writel(ctrl_base, wrp->mode, reg);
		musb_writel(ctrl_base, wrp->phy_utmi, 0x02);
		break;
	case MUSB_PERIPHERAL:
		reg |= (1 << wrp->iddig);

		/*
		 * if we're setting mode to host-only or device-only, we're
		 * going to ignore whatever the PHY sends us and just force
		 * ID pin status by SW
		 */
		reg |= (1 << wrp->iddig_mux);

		musb_writel(ctrl_base, wrp->mode, reg);
		break;
	case MUSB_OTG:
		musb_writel(ctrl_base, wrp->phy_utmi, 0x02);
		break;
	default:
		dev_err(glue->dev, "unsupported mode %d\n", mode);
		return -EINVAL;
	}

	return 0;
}

//Accesses undocumented registers.
//"A babble error occurs when the hots computer or the device receives more data
// than the specific maximum packet size."
static bool dsps_sw_babble_control(struct musb *musb)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_sw_babble_control\n");
	u8 babble_ctl;
	bool session_restart =  false;

	babble_ctl = musb_readb(musb->mregs, MUSB_BABBLE_CTL);				//MISC register.
	dev_dbg(musb->controller, "babble: MUSB_BABBLE_CTL value %x\n",
		babble_ctl);
	/*
	 * check line monitor flag to check whether babble is
	 * due to noise
	 */
	dev_dbg(musb->controller, "STUCK_J is %s\n",
		babble_ctl & MUSB_BABBLE_STUCK_J ? "set" : "reset");			//_J stands for connect? See musb_virthub.c:musb_port_suspend.

	if (babble_ctl & MUSB_BABBLE_STUCK_J) {
		int timeout = 10;

		/*
		 * babble is due to noise, then set transmit idle (d7 bit)
		 * to resume normal operation
		 */
		babble_ctl = musb_readb(musb->mregs, MUSB_BABBLE_CTL);
		babble_ctl |= MUSB_BABBLE_FORCE_TXIDLE;
		musb_writeb(musb->mregs, MUSB_BABBLE_CTL, babble_ctl);

		/* wait till line monitor flag cleared */
		dev_dbg(musb->controller, "Set TXIDLE, wait J to clear\n");
		do {
			babble_ctl = musb_readb(musb->mregs, MUSB_BABBLE_CTL);
			udelay(1);
		} while ((babble_ctl & MUSB_BABBLE_STUCK_J) && timeout--);

		/* check whether stuck_at_j bit cleared */
		if (babble_ctl & MUSB_BABBLE_STUCK_J) {
			/*
			 * real babble condition has occurred
			 * restart the controller to start the
			 * session again
			 */
			dev_dbg(musb->controller, "J not cleared, misc (%x)\n",
				babble_ctl);
			session_restart = true;
		}
	} else {
		session_restart = true;
	}

	return session_restart;
}

//Recovers from data transfer exceeding size limit?
static int dsps_musb_recover(struct musb *musb)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_musb_recover\n");
	struct device *dev = musb->controller;
	struct dsps_glue *glue = dev_get_drvdata(dev->parent);
	int session_restart = 0;

	if (glue->sw_babble_enabled)
		//"A babble error occurs when the hots computer or the device receives
		// more data than the specific maximum packet size."
		session_restart = dsps_sw_babble_control(musb);
	else
		session_restart = 1;

	return session_restart ? 0 : -EPIPE;
}

/* Similar to am35x, dm81xx support only 32-bit read operation */
static void dsps_read_fifo32(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_read_fifo32\n");
	void __iomem *fifo = hw_ep->fifo;

	if (len >= 4) {
		ioread32_rep(fifo, dst, len >> 2);
		dst += len & ~0x03;
		len &= 0x03;
	}

	/* Read any remaining 1 to 3 bytes */
	if (len > 0) {
		u32 val = musb_readl(fifo, 0);
		memcpy(dst, &val, len);
	}
}

#ifdef CONFIG_USB_TI_CPPI41_DMA
//Reads the status of the PD_CMP flag after a packet is completed, and
//sets/enables the corresponding bit/interrupt in the IRQENABLER regiser.
static void dsps_dma_controller_callback(struct dma_controller *c)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_dma_controller_callback\n");
	struct musb *musb = c->musb;
	struct dsps_glue *glue = dev_get_drvdata(musb->controller->parent);
	void __iomem *usbss_base = glue->usbss_base;
	u32 status;

	status = musb_readl(usbss_base, USBSS_IRQ_STATUS);
	if (status & USBSS_IRQ_PD_COMP)
		musb_writel(usbss_base, USBSS_IRQ_STATUS, USBSS_IRQ_PD_COMP);
}
//Called by musb_core.c:musb_init_controller.
//Allocates memory for the USB0/1 DMA controller, including DMA channel
//structures, timers, and functions to call, and enables packet completion
//interrupts.
static struct dma_controller *
dsps_dma_controller_create(struct musb *musb, void __iomem *base)	//base is Virtual memory addresses of USB0/1 CORE.
{
printk("drivers/usb/musb/musb_dsps.c:dsps_dma_controller_create START\n");
	struct dma_controller *controller;
	struct dsps_glue *glue = dev_get_drvdata(musb->controller->parent);	////Driver data of complete USB controller (including USB0/1).
	void __iomem *usbss_base = glue->usbss_base;						//USBSS physical base address at 0x4740_0000.
	//Allocates memory for the USB0/1 DMA controller, sets timers and assigns
	//functions for the DMA controller driver. Fails first time before DMA
	//controller has been created by /drivers/dma/cppi41.c:cppi41_dma_probe.
	controller = cppi41_dma_controller_create(musb, base);
	if (IS_ERR_OR_NULL(controller))
		return controller;
	//Sets the PD_CMP_FLAG of the IRQENABLER register described in section 16.4.1.5:
	//"Interrupt enable when the packet is completed, the differ bits is set,
	// and the Packet Descriptor is pushed into the queue manager"
	musb_writel(usbss_base, USBSS_IRQ_ENABLER, USBSS_IRQ_PD_COMP);
	//"dma_callback: invoked on DMA completion, useful to run platform code such
	// IRQ acknowledgment."
	controller->dma_callback = dsps_dma_controller_callback;
printk("drivers/usb/musb/musb_dsps.c:dsps_dma_controller_create END\n");
	return controller;
}

#ifdef CONFIG_PM_SLEEP
static void dsps_dma_controller_suspend(struct dsps_glue *glue)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_dma_controller_suspend\n");
	void __iomem *usbss_base = glue->usbss_base;

	musb_writel(usbss_base, USBSS_IRQ_CLEARR, USBSS_IRQ_PD_COMP);
}

static void dsps_dma_controller_resume(struct dsps_glue *glue)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_dma_controller_resume\n");
	void __iomem *usbss_base = glue->usbss_base;

	musb_writel(usbss_base, USBSS_IRQ_ENABLER, USBSS_IRQ_PD_COMP);
}
#endif
#else /* CONFIG_USB_TI_CPPI41_DMA */
#ifdef CONFIG_PM_SLEEP
static void dsps_dma_controller_suspend(struct dsps_glue *glue) {
//printk("drivers/usb/musb/musb_dsps.c:dsps_dma_controller_suspend\n");}
static void dsps_dma_controller_resume(struct dsps_glue *glue) {
//printk("drivers/usb/musb/musb_dsps.c:dsps_dma_controller_resume\n");}
#endif
#endif /* CONFIG_USB_TI_CPPI41_DMA */

static struct musb_platform_ops dsps_ops = {
	.quirks		= MUSB_DMA_CPPI41 | MUSB_INDEXED_EP,
	.init		= dsps_musb_init,
	.exit		= dsps_musb_exit,

#ifdef CONFIG_USB_TI_CPPI41_DMA
	.dma_init	= dsps_dma_controller_create,
	.dma_exit	= cppi41_dma_controller_destroy,
#endif
	.enable		= dsps_musb_enable,
	.disable	= dsps_musb_disable,

	.set_mode	= dsps_musb_set_mode,
	.recover	= dsps_musb_recover,
	.clear_ep_rxintr = dsps_musb_clear_ep_rxintr,
};

static u64 musb_dmamask = DMA_BIT_MASK(32);

//Get property field of device node..
static int get_int_prop(struct device_node *dn, const char *s)
{
//printk("drivers/usb/musb/musb_dsps.c:get_int_prop\n");
	int ret;
	u32 val;

	ret = of_property_read_u32(dn, s, &val);
	if (ret)
		return 0;
	return val;
}

//Creates and initializes data structures.
static int dsps_create_musb_pdev(struct dsps_glue *glue,
		struct platform_device *parent)	//Creates platform device for a USB controller (0 or 1)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_create_musb_pdev\n");
	struct musb_hdrc_platform_data pdata;	//Structure telling whether USB controller is host/peripheral/OTG, power managment, etc
	struct resource	resources[2];			//IO port resource in tree structure specifying the start and end physical addresses.
	struct resource	*res;
	struct device *dev = &parent->dev;
	struct musb_hdrc_config	*config;
	struct platform_device *musb;
	struct device_node *dn = parent->dev.of_node;
	int ret, val;

	memset(resources, 0, sizeof(resources));	//Clears resources as they are allocated on the stack which is not initialized.
	res = platform_get_resource_byname(parent, IORESOURCE_MEM, "mc");	//Gets the memory resource named "mc" of the platform.
	if (!res) {
		dev_err(dev, "failed to get memory.\n");
		return -EINVAL;
	}
	resources[0] = *res;

	res = platform_get_resource_byname(parent, IORESOURCE_IRQ, "mc");	//Gets the interrupt resource named "mc" of the platform.
	if (!res) {
		dev_err(dev, "failed to get irq.\n");
		return -EINVAL;
	}
	resources[1] = *res;

	/* allocate the child platform device */
	//AM335x contains two USB controllers built around the Mentor Graphics USB OTG controller musbmhdrc
	//dts shows that usb0/usb1 is at offset 1400/1800.
	//dsps_probe is invoked for each usb0 and usb1, and this resource is created for both.
	musb = platform_device_alloc("musb-hdrc",
			(resources[0].start & 0xFFF) == 0x400 ? 0 : 1);
	if (!musb) {
		dev_err(dev, "failed to allocate musb device\n");
		return -ENOMEM;
	}
	//Initializes the USB controller children.
	musb->dev.parent		= dev;							//The Linux device structure for USB0/1's parent is USB device.
	musb->dev.dma_mask		= &musb_dmamask;
	musb->dev.coherent_dma_mask	= musb_dmamask;
	device_set_of_node_from_dev(&musb->dev, &parent->dev);	//Copies device tree node of parent to child USB controller.

	glue->musb = musb;

	ret = platform_device_add_resources(musb, resources,
			ARRAY_SIZE(resources));	//Adds the memory/register and irq resources to the USB controller.
	if (ret) {
		dev_err(dev, "failed to add resources\n");
		goto err;
	}
	//Kernel memory initialized to zero for configuration structure, storing
	//among other things number of endpoints, speed.
	config = devm_kzalloc(&parent->dev, sizeof(*config), GFP_KERNEL);
	if (!config) {
		ret = -ENOMEM;
		goto err;
	}
	pdata.config = config;
	pdata.platform_ops = &dsps_ops;

	config->num_eps = get_int_prop(dn, "mentor,num-eps");	//Number of endpoints.
	config->ram_bits = get_int_prop(dn, "mentor,ram-bits");	//RAM address size: 12 bits ~ 0x4000 locations, excludes queue manager.

	config->host_port_deassert_reset_at_resume = 1;			//Necessary to reset port after resume (wake up?).
	pdata.mode = musb_get_mode(dev);	//Reads device tree node to check OTG/peripheral (usb0)/host (usb1) mode of USB controller.
	/* DT keeps this entry in mA, musb expects it as per USB spec */
	pdata.power = get_int_prop(dn, "mentor,power") / 2;		//Sets power of USB controller platform data structure.

	ret = of_property_read_u32(dn, "mentor,multipoint", &val);
	if (!ret && val)										//FALSE
		config->multipoint = true;							//config pointer is returned from devm_kzalloc, meaning false initialized.

	config->maximum_speed = usb_get_maximum_speed(&parent->dev);	//Reads device tree node to find "maximum-speed".
	switch (config->maximum_speed) {
	case USB_SPEED_LOW:
	case USB_SPEED_FULL:
		break;
	case USB_SPEED_SUPER:
		dev_warn(dev, "ignore incorrect maximum_speed "
				"(super-speed) setting in dts");
		fallthrough;
	default:														//config->max_speed is unknown.
		config->maximum_speed = USB_SPEED_HIGH;
	}
	//Add pdata to USB controller device of USB controller platform device.
	ret = platform_device_add_data(musb, &pdata, sizeof(pdata));
	if (ret) {
		dev_err(dev, "failed to add platform_data\n");
		goto err;
	}

	ret = platform_device_add(musb);	//Adds the new platform usb controller (0/1) to the Linux kernel device hierarchy.
	if (ret) {
		dev_err(dev, "failed to register musb device\n");
		goto err;
	}
	return 0;

err:
	platform_device_put(musb);
	return ret;
}

//Sets the timer to timeout now.
static irqreturn_t dsps_vbus_threaded_irq(int irq, void *priv)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_vbus_threaded_irq\n");
	struct dsps_glue *glue = priv;
	struct musb *musb = platform_get_drvdata(glue->musb);

	if (!musb)
		return IRQ_NONE;

	dev_dbg(glue->dev, "VBUS interrupt\n");
	dsps_mod_timer(glue, 0);	//modify a timer's timeout to 0 (now).

	return IRQ_HANDLED;
}

//Sets the interrupt handler for VBUS interrupts and the timer to timeout at the
//time of invocation of the interrupt handler.
static int dsps_setup_optional_vbus_irq(struct platform_device *pdev,
					struct dsps_glue *glue)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_setup_optional_vbus_irq\n");
	int error;

	glue->vbus_irq = platform_get_irq_byname(pdev, "vbus");	//"get an IRQ for a device by name"; in dts probably hw irq 18, mapped to 59.
	if (glue->vbus_irq == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (glue->vbus_irq <= 0) {
		glue->vbus_irq = 0;
		return 0;
	}
	//"allocate an interrupt line for a managed device"
	//The device is glue->dev, interrupt line is glue->vbus_irq, with
	//dsps_vbus_threaded_irq called in a threaded interrupt context.
	//IRQF_ONESHOT:
	//"Interrupt is not reenabled after the hardirq handler finished. Used by
	// threaded interrupts which need to keep the irq line disabled until the
	// threaded handler has been run."
	//The handler sets the timer to timeout now.
	error = devm_request_threaded_irq(glue->dev, glue->vbus_irq,
					  NULL, dsps_vbus_threaded_irq,
					  IRQF_ONESHOT,
					  "vbus", glue);
	if (error) {
		glue->vbus_irq = 0;
		return error;
	}
	dev_dbg(glue->dev, "VBUS irq %i configured\n", glue->vbus_irq);

	return 0;
}

//Checks whether this device driver matches the device represented by the
//platform_device argument. Allocates data strutures, performs virtual memory
//maps of the registers, and enables power management.
static int dsps_probe(struct platform_device *pdev)	//Entry point to DMA driver
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_probe START\n");
	const struct of_device_id *match;
	const struct dsps_musb_wrapper *wrp;
	struct dsps_glue *glue;
	int ret;

	if (!strcmp(pdev->name, "musb-hdrc"))
		return -ENODEV;

	match = of_match_node(musb_dsps_of_match, pdev->dev.of_node);	//Check device tree node: ti,musb-am33xx
	if (!match) {
		dev_err(&pdev->dev, "fail to get matching of_match struct\n");
		return -EINVAL;
	}
	wrp = match->data;	//Probably am33xx_driver_data as can probably be verified by musb_dsps_of_match and their initialized data fields.

	if (of_device_is_compatible(pdev->dev.of_node, "ti,musb-dm816"))	//FALSE
		dsps_ops.read_fifo = dsps_read_fifo32;

	/* allocate glue */
	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);	//Allocate zero initialized kernel memory of size struct dsps_glue.
	if (!glue)
		return -ENOMEM;

	glue->dev = &pdev->dev;										//GLUE and PLATFORM device points
	glue->wrp = wrp;
	glue->usbss_base = of_iomap(pdev->dev.parent->of_node, 0);	//usbss_base is physical address of block USBSS at 0x4740_0000.
	if (!glue->usbss_base)
		return -ENXIO;

	platform_set_drvdata(pdev, glue);							//Sets the driver data of device of platform devie to glue
	pm_runtime_enable(&pdev->dev);								//Enables runtime power management of the USB device.
	ret = dsps_create_musb_pdev(glue, pdev);					//Creates platform device for USB controller 0 or 1.
	if (ret)
		goto err;

	if (usb_get_dr_mode(&pdev->dev) == USB_DR_MODE_PERIPHERAL) {	//Checks of node of device whether it has dual role: True.
		ret = dsps_setup_optional_vbus_irq(pdev, glue);
		if (ret)
			goto unregister_pdev;
	}

	return 0;

unregister_pdev:
	platform_device_unregister(glue->musb);
err:
	pm_runtime_disable(&pdev->dev);
	iounmap(glue->usbss_base);
	return ret;
}

static int dsps_remove(struct platform_device *pdev)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_remove\n");
	struct dsps_glue *glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);

	pm_runtime_disable(&pdev->dev);
	iounmap(glue->usbss_base);

	return 0;
}

static const struct dsps_musb_wrapper am33xx_driver_data = {
	.revision		= 0x00,
	.control		= 0x14,
	.status			= 0x18,
	.epintr_set		= 0x38,
	.epintr_clear		= 0x40,
	.epintr_status		= 0x30,
	.coreintr_set		= 0x3c,
	.coreintr_clear		= 0x44,
	.coreintr_status	= 0x34,
	.phy_utmi		= 0xe0,
	.mode			= 0xe8,
	.tx_mode		= 0x70,
	.rx_mode		= 0x74,
	.reset			= 0,
	.otg_disable		= 21,
	.iddig			= 8,
	.iddig_mux		= 7,
	.usb_shift		= 0,
	.usb_mask		= 0x1ff,
	.usb_bitmap		= (0x1ff << 0),
	.drvvbus		= 8,
	.txep_shift		= 0,
	.txep_mask		= 0xffff,
	.txep_bitmap		= (0xffff << 0),
	.rxep_shift		= 16,
	.rxep_mask		= 0xfffe,
	.rxep_bitmap		= (0xfffe << 16),
	.poll_timeout		= 2000, /* ms */
};

static const struct of_device_id musb_dsps_of_match[] = {
	{ .compatible = "ti,musb-am33xx",
		.data = &am33xx_driver_data, },
	{ .compatible = "ti,musb-dm816",
		.data = &am33xx_driver_data, },
	{  },
};
MODULE_DEVICE_TABLE(of, musb_dsps_of_match);

#ifdef CONFIG_PM_SLEEP
static int dsps_suspend(struct device *dev)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_suspend\n");
	struct dsps_glue *glue = dev_get_drvdata(dev);
	const struct dsps_musb_wrapper *wrp = glue->wrp;
	struct musb *musb = platform_get_drvdata(glue->musb);
	void __iomem *mbase;
	int ret;

	if (!musb)
		/* This can happen if the musb device is in -EPROBE_DEFER */
		return 0;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		return ret;
	}

	del_timer_sync(&musb->dev_timer);

	mbase = musb->ctrl_base;
	glue->context.control = musb_readl(mbase, wrp->control);
	glue->context.epintr = musb_readl(mbase, wrp->epintr_set);
	glue->context.coreintr = musb_readl(mbase, wrp->coreintr_set);
	glue->context.phy_utmi = musb_readl(mbase, wrp->phy_utmi);
	glue->context.mode = musb_readl(mbase, wrp->mode);
	glue->context.tx_mode = musb_readl(mbase, wrp->tx_mode);
	glue->context.rx_mode = musb_readl(mbase, wrp->rx_mode);

	dsps_dma_controller_suspend(glue);

	return 0;
}

static int dsps_resume(struct device *dev)
{
//printk("drivers/usb/musb/musb_dsps.c:dsps_resume\n");
	struct dsps_glue *glue = dev_get_drvdata(dev);
	const struct dsps_musb_wrapper *wrp = glue->wrp;
	struct musb *musb = platform_get_drvdata(glue->musb);
	void __iomem *mbase;

	if (!musb)
		return 0;

	dsps_dma_controller_resume(glue);

	mbase = musb->ctrl_base;
	musb_writel(mbase, wrp->control, glue->context.control);
	musb_writel(mbase, wrp->epintr_set, glue->context.epintr);
	musb_writel(mbase, wrp->coreintr_set, glue->context.coreintr);
	musb_writel(mbase, wrp->phy_utmi, glue->context.phy_utmi);
	musb_writel(mbase, wrp->mode, glue->context.mode);
	musb_writel(mbase, wrp->tx_mode, glue->context.tx_mode);
	musb_writel(mbase, wrp->rx_mode, glue->context.rx_mode);
	if (musb->xceiv->otg->state == OTG_STATE_B_IDLE &&
	    musb->port_mode == MUSB_OTG)
		dsps_mod_timer(glue, -1);

	pm_runtime_put(dev);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(dsps_pm_ops, dsps_suspend, dsps_resume);

static struct platform_driver dsps_usbss_driver = {
	.probe		= dsps_probe,
	.remove         = dsps_remove,
	.driver         = {
		.name   = "musb-dsps",
		.pm	= &dsps_pm_ops,
		.of_match_table	= musb_dsps_of_match,
	},
};

MODULE_DESCRIPTION("TI DSPS MUSB Glue Layer");
MODULE_AUTHOR("Ravi B <ravibabu@ti.com>");
MODULE_AUTHOR("Ajay Kumar Gupta <ajay.gupta@ti.com>");
MODULE_LICENSE("GPL v2");

module_platform_driver(dsps_usbss_driver);
