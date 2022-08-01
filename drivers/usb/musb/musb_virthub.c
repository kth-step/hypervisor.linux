// SPDX-License-Identifier: GPL-2.0
/*
 * MUSB OTG driver virtual root hub support
 *
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006-2007 Nokia Corporation
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/timer.h>

#include <asm/unaligned.h>

#include "musb_core.h"

//Clears resume bit of POWER register, which should be done after 20ms of being
//set. Updates the port status and PHY OTG state, and notifies kernel of resume
//event.
void musb_host_finish_resume(struct work_struct *work)
{
	struct musb *musb;
	unsigned long flags;
	u8 power;

	musb = container_of(work, struct musb, finish_resume_work.work);

	spin_lock_irqsave(&musb->lock, flags);

	power = musb_readb(musb->mregs, MUSB_POWER);
	power &= ~MUSB_POWER_RESUME;
	musb_dbg(musb, "root port resume stopped, power %02x", power);
	musb_writeb(musb->mregs, MUSB_POWER, power);

	/*
	 * ISSUE:  DaVinci (RTL 1.300) disconnects after
	 * resume of high speed peripherals (but not full
	 * speed ones).
	 */
	musb->is_active = 1;
	musb->port1_status &= ~(USB_PORT_STAT_SUSPEND | MUSB_PORT_STAT_RESUME);
	musb->port1_status |= USB_PORT_STAT_C_SUSPEND << 16;
	usb_hcd_poll_rh_status(musb->hcd);						//Kernel is notified of resume event.
	/* NOTE: it might really be A_WAIT_BCON ... */
	musb->xceiv->otg->state = OTG_STATE_A_HOST;

	spin_unlock_irqrestore(&musb->lock, flags);
}

//Suspends or wakes up device depending on @do_suspend argument.
int musb_port_suspend(struct musb *musb, bool do_suspend)
{
	struct usb_otg	*otg = musb->xceiv->otg;
	u8		power;
	void __iomem	*mbase = musb->mregs;

	if (!is_host_active(musb))						//Not in host mode.
		return 0;
													//Host mode.
	/* NOTE:  this doesn't necessarily put PHY into low power mode,
	 * turning off its clock; that's a function of PHY integration and
	 * MUSB_POWER_ENSUSPEND.  PHY may need a clock (sigh) to detect
	 * SE0 changing to connect (J) or wakeup (K) states.
	 */
	power = musb_readb(mbase, MUSB_POWER);
	if (do_suspend) {								//Suspend.
		int retries = 10000;

		if (power & MUSB_POWER_RESUME)				//"Set by the CPU to generate Resume signaling when the device is in Suspend mode."
			return -EBUSY;							//Already suspended?

		if (!(power & MUSB_POWER_SUSPENDM)) {		//"In Host mode, this bit is set by the CPU to enter Suspend mode."
			power |= MUSB_POWER_SUSPENDM;
			musb_writeb(mbase, MUSB_POWER, power);	//Enters suspend mode.

			/* Needed for OPT A tests */
			power = musb_readb(mbase, MUSB_POWER);
			while (power & MUSB_POWER_SUSPENDM) {	//"It is cleared when the CPU reads the interrupt register, or sets the Resume bit above."
				power = musb_readb(mbase, MUSB_POWER);	//Does not read interrupt register. Does not make sense.
				if (retries-- < 1)
					break;
			}
		}

		musb_dbg(musb, "Root port suspended, power %02x", power);

		musb->port1_status |= USB_PORT_STAT_SUSPEND;			//Sets USB port in suspend mode.
		switch (musb->xceiv->otg->state) {						//Check PHY OTG state.
		case OTG_STATE_A_HOST:
			musb->xceiv->otg->state = OTG_STATE_A_SUSPEND;		//Set PHY OTG in suspend mode.
			//host is parent bus, and b_hnp_enable is "OTG: did A-Host enable HNP?"
			//Host Negotiation Protocol: "Allows the two devices to exchange their host/peripheral roles".
			musb->is_active = otg->host->b_hnp_enable;
			if (musb->is_active)
				mod_timer(&musb->otg_timer, jiffies				//Set timer to 200msec.
					+ msecs_to_jiffies(
						OTG_TIME_A_AIDL_BDIS));
			musb_platform_try_idle(musb, 0);					//Empty function.
			break;
		case OTG_STATE_B_HOST:
			musb->xceiv->otg->state = OTG_STATE_B_WAIT_ACON;	
			musb->is_active = otg->host->b_hnp_enable;
			musb_platform_try_idle(musb, 0);					//Empty function.
			break;
		default:
			musb_dbg(musb, "bogus rh suspend? %s",
				usb_otg_state_string(musb->xceiv->otg->state));
		}
	} else if (power & MUSB_POWER_SUSPENDM) {					//In suspend mode.
		power &= ~MUSB_POWER_SUSPENDM;							//Remove suspend mode.
		power |= MUSB_POWER_RESUME;								//Set resume mode. Should be cleared after 20ms.
		musb_writeb(mbase, MUSB_POWER, power);

		musb_dbg(musb, "Root port resuming, power %02x", power);

		musb->port1_status |= MUSB_PORT_STAT_RESUME;			//USB port in resume state.
		//Invokes musb_host_finish_resume after 40ms.
		//Clears resume bit of POWER register, which should be done after 20ms
		//of being set. Updates the port status and PHY OTG state, and notifies
		//kernel of resume event.
		schedule_delayed_work(&musb->finish_resume_work,		
				      msecs_to_jiffies(USB_RESUME_TIMEOUT));
	}
	return 0;
}

//Resets, or stops reset and notifies kernel of resumption, depending on the
//@do_reset argument.
void musb_port_reset(struct musb *musb, bool do_reset)
{
	u8		power;
	void __iomem	*mbase = musb->mregs;

	if (musb->xceiv->otg->state == OTG_STATE_B_IDLE) {
		musb_dbg(musb, "HNP: Returning from HNP; no hub reset from b_idle");
		musb->port1_status &= ~USB_PORT_STAT_RESET;
		return;
	}

	if (!is_host_active(musb))										//Not in host mode.
		return;

	/* NOTE:  caller guarantees it will turn off the reset when
	 * the appropriate amount of time has passed
	 */
	power = musb_readb(mbase, MUSB_POWER);
	if (do_reset) {													//Perform reset.
		/*
		 * If RESUME is set, we must make sure it stays minimum 20 ms.
		 * Then we must clear RESUME and wait a bit to let musb start
		 * generating SOFs. If we don't do this, OPT HS A 6.8 tests
		 * fail with "Error! Did not receive an SOF before suspend
		 * detected".
		 */
		if (power &  MUSB_POWER_RESUME) {							//Resume flag has been set, should be cleared after 20ms.
			long remain = (unsigned long) musb->rh_timer - jiffies;	//Remaining time for resume timeout.

			if (musb->rh_timer > 0 && remain > 0) {					//Timeout shall be made and remaining timeout is greater than zero.
				/* take into account the minimum delay after resume */
				schedule_delayed_work(
					&musb->deassert_reset_work, remain);			//Schedule this function via musb_deassert_reset, which then do not reset.
				return;
			}

			musb_writeb(mbase, MUSB_POWER,							//Timeout has occurred.
				    power & ~MUSB_POWER_RESUME);					//Clears Resume bit, which should be done after 20ms.

			/* Give the core 1 ms to clear MUSB_POWER_RESUME */
			schedule_delayed_work(&musb->deassert_reset_work,		//Schedule this function via musb_deassert_reset, which then do not reset.
					      msecs_to_jiffies(1));
			return;
		}

		power &= 0xf0;
		musb_writeb(mbase, MUSB_POWER,								//Sets reset, and clears resume, suspend and enable suspend bits.
				power | MUSB_POWER_RESET);

		musb->port1_status |= USB_PORT_STAT_RESET;					//Status is reset
		musb->port1_status &= ~USB_PORT_STAT_ENABLE;				//and not enable.
		schedule_delayed_work(&musb->deassert_reset_work,			//Schedule this function via musb_deassert_reset, which then do not reset.
				      msecs_to_jiffies(50));
	} else {														//Do not reset (stop reset).
		musb_dbg(musb, "root port reset stopped");
		musb_platform_pre_root_reset_end(musb);						//Empty function.
		musb_writeb(mbase, MUSB_POWER,
				power & ~MUSB_POWER_RESET);							//Stops reset.
		musb_platform_post_root_reset_end(musb);					//Empty function.

		power = musb_readb(mbase, MUSB_POWER);
		if (power & MUSB_POWER_HSMODE) {//"When set, this read-only bit indicates High-speed mode successfully negotiated during USB reset."
			musb_dbg(musb, "high-speed device connected");
			musb->port1_status |= USB_PORT_STAT_HIGH_SPEED;
		}

		musb->port1_status &= ~USB_PORT_STAT_RESET;					//Not in reset state.
		musb->port1_status |= USB_PORT_STAT_ENABLE					//Enables (usable) state.
					| (USB_PORT_STAT_C_RESET << 16)
					| (USB_PORT_STAT_C_ENABLE << 16);
		usb_hcd_poll_rh_status(musb->hcd);							//Kernel is notified of resume event.

		musb->vbuserr_retry = VBUSERR_RETRY_COUNT;					//3 retries.
	}
}

//Updates OTG state and allows the Linux kernel to respond to an event?
void musb_root_disconnect(struct musb *musb)
{
	struct usb_otg	*otg = musb->xceiv->otg;

	musb->port1_status = USB_PORT_STAT_POWER
			| (USB_PORT_STAT_C_CONNECTION << 16);

	//"Root Hub interrupt transfers are polled using a timer if the driver
	// requests it; otherwise the driver is responsible for calling
	// usb_hcd_poll_rh_status() when an event occurs."
	usb_hcd_poll_rh_status(musb->hcd);
	musb->is_active = 0;

	switch (musb->xceiv->otg->state) {
	case OTG_STATE_A_SUSPEND:
		if (otg->host->b_hnp_enable) {
			musb->xceiv->otg->state = OTG_STATE_A_PERIPHERAL;	//PHY is in state peripheral.
			musb->g.is_a_peripheral = 1;
			break;
		}
		fallthrough;
	case OTG_STATE_A_HOST:
		musb->xceiv->otg->state = OTG_STATE_A_WAIT_BCON;
		musb->is_active = 0;
		break;
	case OTG_STATE_A_WAIT_VFALL:
		musb->xceiv->otg->state = OTG_STATE_B_IDLE;
		break;
	default:
		musb_dbg(musb, "host disconnect (%s)",
			usb_otg_state_string(musb->xceiv->otg->state));
	}
}
EXPORT_SYMBOL_GPL(musb_root_disconnect);


/*---------------------------------------------------------------------*/

/* Caller may or may not hold musb->lock */
int musb_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct musb	*musb = hcd_to_musb(hcd);
	int		retval = 0;

	/* called in_irq() via usb_hcd_poll_rh_status() */
	if (musb->port1_status & 0xffff0000) {
		*buf = 0x02;
		retval = 1;
	}
	return retval;
}

static int musb_has_gadget(struct musb *musb)
{
	/*
	 * In host-only mode we start a connection right away. In OTG mode
	 * we have to wait until we loaded a gadget. We don't really need a
	 * gadget if we operate as a host but we should not start a session
	 * as a device without a gadget or else we explode.
	 */
#ifdef CONFIG_USB_MUSB_HOST
	return 1;
#else
	return musb->port_mode == MUSB_HOST;
#endif
}

int musb_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength)
{
	struct musb	*musb = hcd_to_musb(hcd);
	u32		temp;
	int		retval = 0;
	unsigned long	flags;
	bool		start_musb = false;

	spin_lock_irqsave(&musb->lock, flags);

	if (unlikely(!HCD_HW_ACCESSIBLE(hcd))) {
		spin_unlock_irqrestore(&musb->lock, flags);
		return -ESHUTDOWN;
	}

	/* hub features:  always zero, setting is a NOP
	 * port features: reported, sometimes updated when host is active
	 * no indicators
	 */
	switch (typeReq) {
	case ClearHubFeature:
	case SetHubFeature:
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			goto error;
		}
		break;
	case ClearPortFeature:
		if ((wIndex & 0xff) != 1)
			goto error;

		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			break;
		case USB_PORT_FEAT_SUSPEND:
			musb_port_suspend(musb, false);
			break;
		case USB_PORT_FEAT_POWER:
			if (!hcd->self.is_b_host)
				musb_platform_set_vbus(musb, 0);
			break;
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_RESET:
		case USB_PORT_FEAT_C_SUSPEND:
			break;
		default:
			goto error;
		}
		musb_dbg(musb, "clear feature %d", wValue);
		musb->port1_status &= ~(1 << wValue);
		break;
	case GetHubDescriptor:
		{
		struct usb_hub_descriptor *desc = (void *)buf;

		desc->bDescLength = 9;
		desc->bDescriptorType = USB_DT_HUB;
		desc->bNbrPorts = 1;
		desc->wHubCharacteristics = cpu_to_le16(
			HUB_CHAR_INDV_PORT_LPSM /* per-port power switching */
			| HUB_CHAR_NO_OCPM	/* no overcurrent reporting */
			);
		desc->bPwrOn2PwrGood = 5;	/* msec/2 */
		desc->bHubContrCurrent = 0;

		/* workaround bogus struct definition */
		desc->u.hs.DeviceRemovable[0] = 0x02;	/* port 1 */
		desc->u.hs.DeviceRemovable[1] = 0xff;
		}
		break;
	case GetHubStatus:
		temp = 0;
		*(__le32 *) buf = cpu_to_le32(temp);
		break;
	case GetPortStatus:
		if (wIndex != 1)
			goto error;

		put_unaligned(cpu_to_le32(musb->port1_status
					& ~MUSB_PORT_STAT_RESUME),
				(__le32 *) buf);

		/* port change status is more interesting */
		musb_dbg(musb, "port status %08x", musb->port1_status);
		break;
	case SetPortFeature:
		if ((wIndex & 0xff) != 1)
			goto error;

		switch (wValue) {
		case USB_PORT_FEAT_POWER:
			/* NOTE: this controller has a strange state machine
			 * that involves "requesting sessions" according to
			 * magic side effects from incompletely-described
			 * rules about startup...
			 *
			 * This call is what really starts the host mode; be
			 * very careful about side effects if you reorder any
			 * initialization logic, e.g. for OTG, or change any
			 * logic relating to VBUS power-up.
			 */
			if (!hcd->self.is_b_host && musb_has_gadget(musb))
				start_musb = true;
			break;
		case USB_PORT_FEAT_RESET:
			musb_port_reset(musb, true);
			break;
		case USB_PORT_FEAT_SUSPEND:
			musb_port_suspend(musb, true);
			break;
		case USB_PORT_FEAT_TEST:
			if (unlikely(is_host_active(musb)))
				goto error;

			wIndex >>= 8;
			switch (wIndex) {
			case USB_TEST_J:
				pr_debug("USB_TEST_J\n");
				temp = MUSB_TEST_J;
				break;
			case USB_TEST_K:
				pr_debug("USB_TEST_K\n");
				temp = MUSB_TEST_K;
				break;
			case USB_TEST_SE0_NAK:
				pr_debug("USB_TEST_SE0_NAK\n");
				temp = MUSB_TEST_SE0_NAK;
				break;
			case USB_TEST_PACKET:
				pr_debug("USB_TEST_PACKET\n");
				temp = MUSB_TEST_PACKET;
				musb_load_testpacket(musb);
				break;
			case USB_TEST_FORCE_ENABLE:
				pr_debug("USB_TEST_FORCE_ENABLE\n");
				temp = MUSB_TEST_FORCE_HOST
					| MUSB_TEST_FORCE_HS;

				musb_writeb(musb->mregs, MUSB_DEVCTL,
						MUSB_DEVCTL_SESSION);
				break;
			case 6:
				pr_debug("TEST_FIFO_ACCESS\n");
				temp = MUSB_TEST_FIFO_ACCESS;
				break;
			default:
				goto error;
			}
			musb_writeb(musb->mregs, MUSB_TESTMODE, temp);
			break;
		default:
			goto error;
		}
		musb_dbg(musb, "set feature %d", wValue);
		musb->port1_status |= 1 << wValue;
		break;

	default:
error:
		/* "protocol stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&musb->lock, flags);

	if (start_musb)
		musb_start(musb);

	return retval;
}