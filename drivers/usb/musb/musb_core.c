// SPDX-License-Identifier: GPL-2.0
/*
 * MUSB OTG driver core code
 *
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006-2007 Nokia Corporation
 */

/*
 * Inventra (Multipoint) Dual-Role Controller Driver for Linux.
 *
 * This consists of a Host Controller Driver (HCD) and a peripheral
 * controller driver implementing the "Gadget" API; OTG support is
 * in the works.  These are normal Linux-USB controller drivers which
 * use IRQs and have no dedicated thread.
 *
 * This version of the driver has only been used with products from
 * Texas Instruments.  Those products integrate the Inventra logic
 * with other DMA, IRQ, and bus modules, as well as other logic that
 * needs to be reflected in this driver.
 *
 *
 * NOTE:  the original Mentor code here was pretty much a collection
 * of mechanisms that don't seem to have been fully integrated/working
 * for *any* Linux kernel version.  This version aims at Linux 2.6.now,
 * Key open issues include:
 *
 *  - Lack of host-side transaction scheduling, for all transfer types.
 *    The hardware doesn't do it; instead, software must.
 *
 *    This is not an issue for OTG devices that don't support external
 *    hubs, but for more "normal" USB hosts it's a user issue that the
 *    "multipoint" support doesn't scale in the expected ways.  That
 *    includes DaVinci EVM in a common non-OTG mode.
 *
 *      * Control and bulk use dedicated endpoints, and there's as
 *        yet no mechanism to either (a) reclaim the hardware when
 *        peripherals are NAKing, which gets complicated with bulk
 *        endpoints, or (b) use more than a single bulk endpoint in
 *        each direction.
 *
 *        RESULT:  one device may be perceived as blocking another one.
 *
 *      * Interrupt and isochronous will dynamically allocate endpoint
 *        hardware, but (a) there's no record keeping for bandwidth;
 *        (b) in the common case that few endpoints are available, there
 *        is no mechanism to reuse endpoints to talk to multiple devices.
 *
 *        RESULT:  At one extreme, bandwidth can be overcommitted in
 *        some hardware configurations, no faults will be reported.
 *        At the other extreme, the bandwidth capabilities which do
 *        exist tend to be severely undercommitted.  You can't yet hook
 *        up both a keyboard and a mouse to an external USB hub.
 */

/*
 * This gets many kinds of configuration information:
 *	- Kconfig for everything user-configurable
 *	- platform_device for addressing, irq, and platform_data
 *	- platform_data is mostly for board-specific information
 *	  (plus recentrly, SOC or family details)
 *
 * Most of the conditional compilation will (someday) vanish.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/kobject.h>
#include <linux/prefetch.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/dma-mapping.h>
#include <linux/usb.h>
#include <linux/usb/of.h>

#include "musb_core.h"
#include "musb_trace.h"

#define TA_WAIT_BCON(m) max_t(int, (m)->a_wait_bcon, OTG_TIME_A_WAIT_BCON)


#define DRIVER_AUTHOR "Mentor Graphics, Texas Instruments, Nokia"
#define DRIVER_DESC "Inventra Dual-Role USB Controller Driver"

#define MUSB_VERSION "6.0"

#define DRIVER_INFO DRIVER_DESC ", v" MUSB_VERSION

#define MUSB_DRIVER_NAME "musb-hdrc"
const char musb_driver_name[] = MUSB_DRIVER_NAME;

MODULE_DESCRIPTION(DRIVER_INFO);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" MUSB_DRIVER_NAME);


/*-------------------------------------------------------------------------*/

//Gets the MUSB0/1 from the Linux device structure of the MUSB0/1.
static inline struct musb *dev_to_musb(struct device *dev)
{
//printk("drivers/usb/musb/musb_core.c:dev_to_musb\n");
	return dev_get_drvdata(dev);
}

//Reads device tree node to check OTG/peripheral (usb0)/host (usb1) mode of USB controller.
enum musb_mode musb_get_mode(struct device *dev)
{
//printk("drivers/usb/musb/musb_core.c:musb_get_mode\n");
	enum usb_dr_mode mode;

	mode = usb_get_dr_mode(dev);
	switch (mode) {
	case USB_DR_MODE_HOST:
		return MUSB_HOST;
	case USB_DR_MODE_PERIPHERAL:
		return MUSB_PERIPHERAL;
	case USB_DR_MODE_OTG:
	case USB_DR_MODE_UNKNOWN:
	default:
		return MUSB_OTG;
	}
}
EXPORT_SYMBOL_GPL(musb_get_mode);

/*-------------------------------------------------------------------------*/

static int musb_ulpi_read(struct usb_phy *phy, u32 reg)
{
//printk("drivers/usb/musb/musb_core.c:musb_ulpi_read\n");
	void __iomem *addr = phy->io_priv;
	int	i = 0;
	u8	r;
	u8	power;
	int	ret;

	pm_runtime_get_sync(phy->io_dev);

	/* Make sure the transceiver is not in low power mode */
	power = musb_readb(addr, MUSB_POWER);
	power &= ~MUSB_POWER_SUSPENDM;
	musb_writeb(addr, MUSB_POWER, power);

	/* REVISIT: musbhdrc_ulpi_an.pdf recommends setting the
	 * ULPICarKitControlDisableUTMI after clearing POWER_SUSPENDM.
	 */

	musb_writeb(addr, MUSB_ULPI_REG_ADDR, (u8)reg);
	musb_writeb(addr, MUSB_ULPI_REG_CONTROL,
			MUSB_ULPI_REG_REQ | MUSB_ULPI_RDN_WR);

	while (!(musb_readb(addr, MUSB_ULPI_REG_CONTROL)
				& MUSB_ULPI_REG_CMPLT)) {
		i++;
		if (i == 10000) {
			ret = -ETIMEDOUT;
			goto out;
		}

	}
	r = musb_readb(addr, MUSB_ULPI_REG_CONTROL);
	r &= ~MUSB_ULPI_REG_CMPLT;
	musb_writeb(addr, MUSB_ULPI_REG_CONTROL, r);

	ret = musb_readb(addr, MUSB_ULPI_REG_DATA);

out:
	pm_runtime_put(phy->io_dev);

	return ret;
}

static int musb_ulpi_write(struct usb_phy *phy, u32 val, u32 reg)
{
//printk("drivers/usb/musb/musb_core.c:musb_ulpi_write\n");
	void __iomem *addr = phy->io_priv;
	int	i = 0;
	u8	r = 0;
	u8	power;
	int	ret = 0;

	pm_runtime_get_sync(phy->io_dev);

	/* Make sure the transceiver is not in low power mode */
	power = musb_readb(addr, MUSB_POWER);
	power &= ~MUSB_POWER_SUSPENDM;
	musb_writeb(addr, MUSB_POWER, power);

	musb_writeb(addr, MUSB_ULPI_REG_ADDR, (u8)reg);
	musb_writeb(addr, MUSB_ULPI_REG_DATA, (u8)val);
	musb_writeb(addr, MUSB_ULPI_REG_CONTROL, MUSB_ULPI_REG_REQ);

	while (!(musb_readb(addr, MUSB_ULPI_REG_CONTROL)
				& MUSB_ULPI_REG_CMPLT)) {
		i++;
		if (i == 10000) {
			ret = -ETIMEDOUT;
			goto out;
		}
	}

	r = musb_readb(addr, MUSB_ULPI_REG_CONTROL);
	r &= ~MUSB_ULPI_REG_CMPLT;
	musb_writeb(addr, MUSB_ULPI_REG_CONTROL, r);

out:
	pm_runtime_put(phy->io_dev);

	return ret;
}

static struct usb_phy_io_ops musb_ulpi_access = {
	.read = musb_ulpi_read,
	.write = musb_ulpi_write,
};

/*-------------------------------------------------------------------------*/

//The offset of the FIFO size 8-bit register (stored in 32-bits?) for given
//endpoint number/ID (1-15).
static u32 musb_default_fifo_offset(u8 epnum)
{
//printk("drivers/usb/musb/musb_core.c:musb_default_fifo_offset\n");
	return 0x20 + (epnum * 4);
}

/* "flat" mapping: each endpoint has its own i/o address */
static void musb_flat_ep_select(void __iomem *mbase, u8 epnum)
{
//printk("drivers/usb/musb/musb_core.c:musb_flat_ep_select\n");
}

static u32 musb_flat_ep_offset(u8 epnum, u16 offset)
{
//printk("drivers/usb/musb/musb_core.c:musb_flat_ep_offset\n");
	return 0x100 + (0x10 * epnum) + offset;
}

/* "indexed" mapping: INDEX register controls register bank select */
static void musb_indexed_ep_select(void __iomem *mbase, u8 epnum)
{
//printk("drivers/usb/musb/musb_core.c:musb_indexed_ep_select\n");
	musb_writeb(mbase, MUSB_INDEX, epnum);
}

//Returns offset address for given endpoint number. Is used to return the offset
//of the indexed CSR register.
static u32 musb_indexed_ep_offset(u8 epnum, u16 offset)
{
//printk("drivers/usb/musb/musb_core.c:musb_indexed_ep_offset\n");
	return 0x10 + offset;
}

//Returns register addresses related to target address registers.
//"The MUSBMHDRC has the facility, when operating in Host mode, to act as the
// host to a range of USB peripheral devices – high-speed, full-speed or
// low-speed – where these devices are connected to the MUSBMHDRC via a USB
// hub."
static u32 musb_default_busctl_offset(u8 epnum, u16 offset)
{
//printk("drivers/usb/musb/musb_core.c:musb_default_busctl_offset\n");
	return 0x80 + (0x08 * epnum) + offset;
}

//Read one byte from addr + offset.
static u8 musb_default_readb(void __iomem *addr, u32 offset)
{
//printk("drivers/usb/musb/musb_core.c:musb_default_readb\n");
	u8 data =  __raw_readb(addr + offset);

	trace_musb_readb(__builtin_return_address(0), addr, offset, data);
	return data;
}

//Write one byte to addr + offset.
static void musb_default_writeb(void __iomem *addr, u32 offset, u8 data)
{
//printk("drivers/usb/musb/musb_core.c:musb_default_writeb\n");
	trace_musb_writeb(__builtin_return_address(0), addr, offset, data);
	__raw_writeb(data, addr + offset);
}

//Reads 2 bytes from addr + offset.
static u16 musb_default_readw(void __iomem *addr, u32 offset)
{
//printk("drivers/usb/musb/musb_core.c:musb_default_readw\n");
	u16 data = __raw_readw(addr + offset);

	trace_musb_readw(__builtin_return_address(0), addr, offset, data);
	return data;
}

//Writes 2 bytes from addr + offset.
static void musb_default_writew(void __iomem *addr, u32 offset, u16 data)
{
//printk("drivers/usb/musb/musb_core.c:musb_default_writew\n");
	trace_musb_writew(__builtin_return_address(0), addr, offset, data);
	__raw_writew(data, addr + offset);
}

//State of TX endpoint or RX endpoint 0 data toggle.
static u16 musb_default_get_toggle(struct musb_qh *qh, int is_out)
{
//printk("drivers/usb/musb/musb_core.c:musb_default_get_toggle\n");
	void __iomem *epio = qh->hw_ep->regs;
	u16 csr;

	if (is_out)
		//"When read, this bit indicates the current state of the TX Endpoint
		// data toggle."
		csr = musb_readw(epio, MUSB_TXCSR) & MUSB_TXCSR_H_DATATOGGLE;
	else
		//"When read, this bit indicates the current state of the Endpoint 0
		// data toggle."
		csr = musb_readw(epio, MUSB_RXCSR) & MUSB_RXCSR_H_DATATOGGLE;

	return csr;
}

//Configures toggling of endpoint of given qh.
static u16 musb_default_set_toggle(struct musb_qh *qh, int is_out,
				   struct urb *urb)
{
//printk("drivers/usb/musb/musb_core.c:musb_default_set_toggle\n");
	u16 csr;
	u16 toggle;

	//Toggles of data packets flip bits for synchronization?
	//"An alternating DATA0/DATA1 is used as a part of the error control
	// protocol to (or from) a particular endpoint."
	toggle = usb_gettoggle(urb->dev, qh->epnum, is_out);

	if (is_out)										//Transmit.
		csr = toggle ? (MUSB_TXCSR_H_WR_DATATOGGLE	//"enable the current state of the TX Endpoint data toggle to be written"
				| MUSB_TXCSR_H_DATATOGGLE)			//"written with the required setting of the data toggle"
				: MUSB_TXCSR_CLRDATATOG;			//"reset the endpoint data toggle to 0"
	else											//Receive.
		csr = toggle ? (MUSB_RXCSR_H_WR_DATATOGGLE	//"enable the current state of the Endpoint 0 data toggle to be written"
				| MUSB_RXCSR_H_DATATOGGLE) : 0;		//"written with the required setting of the data toggle"

	return csr;
}

/*
 * Load an endpoint's FIFO
 */
//Arguments are hardware endpoint structure, bytes left to transfer from memory
//to FIFO, next memory location to read.
//PIO write function from memory to FIFO.
static void musb_default_write_fifo(struct musb_hw_ep *hw_ep, u16 len,
				    const u8 *src)
{
//printk("drivers/usb/musb/musb_core.c:musb_default_write_fifo\n");
	struct musb *musb = hw_ep->musb;
	void __iomem *fifo = hw_ep->fifo;	//FIFO start address for this endpoint.

	if (unlikely(len == 0))
		return;

	prefetch((u8 *)src);				//Prefetch byte at address by casting from constant pointer to pointer.

	dev_dbg(musb->controller, "%cX ep%d fifo %p count %d buf %p\n",
			'T', hw_ep->epnum, fifo, len, src);

	/* we can't assume unaligned reads work */
	if (likely((0x01 & (unsigned long) src) == 0)) {			//Half-word aligned memory source address.
		u16	index = 0;

		/* best case is 32bit-aligned source address */
		if ((0x02 & (unsigned long) src) == 0) {				//32-bit word aligned memory source address.
			if (len >= 4) {										//At least 4 bytes to transfer from memory to FIFO.
				iowrite32_rep(fifo, src + index, len >> 2);		//Write as many 4-byte words as possible from memory to FIFO.
				index += len & ~0x03;							//Increment index by number of bytes transferred.
			}
			if (len & 0x02) {									//At least 2 more bytes to transfer.
				__raw_writew(*(u16 *)&src[index], fifo);		//Write half-word to FIFO.	
				index += 2;										//Increment index by number of bytes transferred.
			}
		} else {												//Half-word aligned source address.
			if (len >= 2) {										//At least two bytes to transfer.
				iowrite16_rep(fifo, src + index, len >> 1);		//Transfer as many half-words as possible.
				index += len & ~0x01;							//Increment index by number of bytes transferred.
			}
		}
		if (len & 0x01)											//If a byte remains to be transferred,
			__raw_writeb(src[index], fifo);						//transfer that byte from memory to FIFO.
	} else  {
		/* byte aligned */
		iowrite8_rep(fifo, src, len);							//Write all bytes.
	}
}

/*
 * Unload an endpoint's FIFO
 */
//Arguments are hardware endpoint structure, bytes left to transfer from FIFO
//to memory, next memory to write.
//PIO read function from FIFO to memory.
static void musb_default_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
//printk("drivers/usb/musb/musb_core.c:musb_default_read_fifo\n");
	struct musb *musb = hw_ep->musb;
	void __iomem *fifo = hw_ep->fifo;	//FIFO start address for this endpoint.

	if (unlikely(len == 0))
		return;

	dev_dbg(musb->controller, "%cX ep%d fifo %p count %d buf %p\n",
			'R', hw_ep->epnum, fifo, len, dst);

	/* we can't assume unaligned writes work */
	if (likely((0x01 & (unsigned long) dst) == 0)) {	//Memory destination is half-word aligned.
		u16	index = 0;

		/* best case is 32bit-aligned destination address */
		if ((0x02 & (unsigned long) dst) == 0) {		//Memory destination word aligned.
			if (len >= 4) {								//At least 4 bytes to read from FIFO and store to memory.
				ioread32_rep(fifo, dst, len >> 2);		//Read endpoint FIFO from start address and store at dst.
				index = len & ~0x03;					//Number of bytes written to memory at dst.
			}
			if (len & 0x02) {							//At least 2 more bytes to read.
				*(u16 *)&dst[index] = __raw_readw(fifo);	//Read 2 bytes from FIFO and store in memory at dst.
				index += 2;								//2 additional bytes written to memory.
			}
		} else {										//Memory destination is not word aligned.
			if (len >= 2) {								//At least 2 bytes left to read.
				ioread16_rep(fifo, dst, len >> 1);	//Divide number of bytes to read by 2 to find out the number of half-words to read.
				index = len & ~0x01;					//Number of bytes written to memory at dst.
			}
		}
		if (len & 0x01)									//One byte left to write to memory.
			dst[index] = __raw_readb(fifo);				//Read last byte from FIFO and write that byte to memory.
	} else  {
		/* byte aligned */
		ioread8_rep(fifo, dst, len);					//Transfer len bytes from FIFO to memory.
	}
}

/*
 * Old style IO functions
 */
u8 (*musb_readb)(void __iomem *addr, u32 offset);
EXPORT_SYMBOL_GPL(musb_readb);

void (*musb_writeb)(void __iomem *addr, u32 offset, u8 data);
EXPORT_SYMBOL_GPL(musb_writeb);

u8 (*musb_clearb)(void __iomem *addr, u32 offset);
EXPORT_SYMBOL_GPL(musb_clearb);

u16 (*musb_readw)(void __iomem *addr, u32 offset);
EXPORT_SYMBOL_GPL(musb_readw);

void (*musb_writew)(void __iomem *addr, u32 offset, u16 data);
EXPORT_SYMBOL_GPL(musb_writew);

u16 (*musb_clearw)(void __iomem *addr, u32 offset);
EXPORT_SYMBOL_GPL(musb_clearw);

//Reads 32 bits from addr + offset.
u32 musb_readl(void __iomem *addr, u32 offset)
{
//printk("drivers/usb/musb/musb_core.c:musb_readl\n");
	u32 data = __raw_readl(addr + offset);

	trace_musb_readl(__builtin_return_address(0), addr, offset, data);
	return data;
}
EXPORT_SYMBOL_GPL(musb_readl);

//Writes 32 bits from addr + offset.
void musb_writel(void __iomem *addr, u32 offset, u32 data)
{
//printk("drivers/usb/musb/musb_core.c:musb_writel\n");
	trace_musb_writel(__builtin_return_address(0), addr, offset, data);
	__raw_writel(data, addr + offset);
}
EXPORT_SYMBOL_GPL(musb_writel);

#ifndef CONFIG_MUSB_PIO_ONLY
struct dma_controller *
(*musb_dma_controller_create)(struct musb *musb, void __iomem *base);
EXPORT_SYMBOL(musb_dma_controller_create);

void (*musb_dma_controller_destroy)(struct dma_controller *c);
EXPORT_SYMBOL(musb_dma_controller_destroy);
#endif

/*
 * New style IO functions
 */
//Calls musb_default_read_fifo, which is a PIO read function from FIFO to
//memory.
void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
//printk("drivers/usb/musb/musb_core.c:musb_read_fifo\n");
	return hw_ep->musb->io.read_fifo(hw_ep, len, dst);
}

//Calls musb_default_write_fifo which is a PIO write function from memory to
//FIFO.
void musb_write_fifo(struct musb_hw_ep *hw_ep, u16 len, const u8 *src)
{
//printk("drivers/usb/musb/musb_core.c:musb_write_fifo\n");
	return hw_ep->musb->io.write_fifo(hw_ep, len, src);
}

static u8 musb_read_devctl(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_read_devctl\n");
	return musb_readb(musb->mregs, MUSB_DEVCTL);
}

/**
 * musb_set_host - set and initialize host mode
 * @musb: musb controller driver data
 *
 * At least some musb revisions need to enable devctl session bit in
 * peripheral mode to switch to host mode. Initializes things to host
 * mode and sets A_IDLE. SoC glue needs to advance state further
 * based on phy provided VBUS state.
 *
 * Note that the SoC glue code may need to wait for musb to settle
 * on enable before calling this to avoid babble.
 */
int musb_set_host(struct musb *musb)
{
printk("drivers/usb/musb/musb_core.c:musb_set_host START\n");
	int error = 0;
	u8 devctl;

	if (!musb)
		return -EINVAL;

	devctl = musb_read_devctl(musb);
	if (!(devctl & MUSB_DEVCTL_BDEVICE)) {
		trace_musb_state(musb, devctl, "Already in host mode");
		goto init_data;
	}

	devctl |= MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	error = readx_poll_timeout(musb_read_devctl, musb, devctl,
				   !(devctl & MUSB_DEVCTL_BDEVICE), 5000,
				   1000000);
	if (error) {
		dev_err(musb->controller, "%s: could not set host: %02x\n",
			__func__, devctl);

		return error;
	}

	devctl = musb_read_devctl(musb);
	trace_musb_state(musb, devctl, "Host mode set");

init_data:
	musb->is_active = 1;
	musb->xceiv->otg->state = OTG_STATE_A_IDLE;
	MUSB_HST_MODE(musb);
printk("drivers/usb/musb/musb_core.c:musb_set_host END\n");
	return error;
}
EXPORT_SYMBOL_GPL(musb_set_host);

/**
 * musb_set_peripheral - set and initialize peripheral mode
 * @musb: musb controller driver data
 *
 * Clears devctl session bit and initializes things for peripheral
 * mode and sets B_IDLE. SoC glue needs to advance state further
 * based on phy provided VBUS state.
 */
int musb_set_peripheral(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_set_peripheral\n");
	int error = 0;
	u8 devctl;

	if (!musb)
		return -EINVAL;

	devctl = musb_read_devctl(musb);
	if (devctl & MUSB_DEVCTL_BDEVICE) {
		trace_musb_state(musb, devctl, "Already in peripheral mode");
		goto init_data;
	}

	devctl &= ~MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	error = readx_poll_timeout(musb_read_devctl, musb, devctl,
				   devctl & MUSB_DEVCTL_BDEVICE, 5000,
				   1000000);
	if (error) {
		dev_err(musb->controller, "%s: could not set peripheral: %02x\n",
			__func__, devctl);

		return error;
	}

	devctl = musb_read_devctl(musb);
	trace_musb_state(musb, devctl, "Peripheral mode set");

init_data:
	musb->is_active = 0;
	musb->xceiv->otg->state = OTG_STATE_B_IDLE;
	MUSB_DEV_MODE(musb);

	return error;
}
EXPORT_SYMBOL_GPL(musb_set_peripheral);

/*-------------------------------------------------------------------------*/

/* for high speed test mode; see USB 2.0 spec 7.1.20 */
static const u8 musb_test_packet[53] = {
	/* implicit SYNC then DATA0 to start */

	/* JKJKJKJK x9 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* JJKKJJKK x8 */
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	/* JJJJKKKK x8 */
	0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
	/* JJJJJJJKKKKKKK x8 */
	0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* JJJJJJJK x8 */
	0x7f, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd,
	/* JKKKKKKK x10, JK */
	0xfc, 0x7e, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd, 0x7e

	/* implicit CRC16 then EOP to end */
};

void musb_load_testpacket(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_load_testpacket\n");
	void __iomem	*regs = musb->endpoints[0].regs;

	musb_ep_select(musb->mregs, 0);
	musb_write_fifo(musb->control_ep,
			sizeof(musb_test_packet), musb_test_packet);
	musb_writew(regs, MUSB_CSR0, MUSB_CSR0_TXPKTRDY);
}

/*-------------------------------------------------------------------------*/

/*
 * Handles OTG hnp timeouts, such as b_ase0_brst
 */
static void musb_otg_timer_func(struct timer_list *t)
{
//printk("drivers/usb/musb/musb_core.c:musb_otg_timer_func\n");
	struct musb	*musb = from_timer(musb, t, otg_timer);
	unsigned long	flags;

	spin_lock_irqsave(&musb->lock, flags);
	switch (musb->xceiv->otg->state) {	//OTG state during reset.
	case OTG_STATE_B_WAIT_ACON:
		musb_dbg(musb,
			"HNP: b_wait_acon timeout; back to b_peripheral");
		musb_g_disconnect(musb);
		musb->xceiv->otg->state = OTG_STATE_B_PERIPHERAL;
		musb->is_active = 0;
		break;
	case OTG_STATE_A_SUSPEND:
	case OTG_STATE_A_WAIT_BCON:
		musb_dbg(musb, "HNP: %s timeout",
			usb_otg_state_string(musb->xceiv->otg->state));
		musb_platform_set_vbus(musb, 0);
		musb->xceiv->otg->state = OTG_STATE_A_WAIT_VFALL;
		break;
	default:
		musb_dbg(musb, "HNP: Unhandled mode %s",
			usb_otg_state_string(musb->xceiv->otg->state));
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}

/*
 * Stops the HNP transition. Caller must take care of locking.
 */
//Sets MUSB in suspend mode.
void musb_hnp_stop(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_hnp_stop\n");
	struct usb_hcd	*hcd = musb->hcd;
	void __iomem	*mbase = musb->mregs;
	u8	reg;

	musb_dbg(musb, "HNP: stop from %s",
			usb_otg_state_string(musb->xceiv->otg->state));

	switch (musb->xceiv->otg->state) {
	case OTG_STATE_A_PERIPHERAL:
		musb_g_disconnect(musb);							//Empty function.
		musb_dbg(musb, "HNP: back to %s",
			usb_otg_state_string(musb->xceiv->otg->state));
		break;
	case OTG_STATE_B_HOST:
		musb_dbg(musb, "HNP: Disabling HR");
		if (hcd)
			hcd->self.is_b_host = 0;
		musb->xceiv->otg->state = OTG_STATE_B_PERIPHERAL;
		MUSB_DEV_MODE(musb);								//MUSB in peripheral mode.
		reg = musb_readb(mbase, MUSB_POWER);
		reg |= MUSB_POWER_SUSPENDM;							//Sets MUSB in suspend mode.
		musb_writeb(mbase, MUSB_POWER, reg);
		/* REVISIT: Start SESSION_REQUEST here? */
		break;
	default:
		musb_dbg(musb, "HNP: Stopping in unknown state %s",
			usb_otg_state_string(musb->xceiv->otg->state));
	}

	/*
	 * When returning to A state after HNP, avoid hub_port_rebounce(),
	 * which cause occasional OPT A "Did not receive reset after connect"
	 * errors.
	 */
	musb->port1_status &= ~(USB_PORT_STAT_C_CONNECTION << 16);
}

static void musb_recover_from_babble(struct musb *musb);

//Handle resume after suspend mode.
static void musb_handle_intr_resume(struct musb *musb, u8 devctl)
{
//printk("drivers/usb/musb/musb_core.c:musb_handle_intr_resume\n");
	musb_dbg(musb, "RESUME (%s)",
			usb_otg_state_string(musb->xceiv->otg->state));

	if (devctl & MUSB_DEVCTL_HM) {
		//Follows the state transition diagram in On-The-Go Supplement to the
		//USB 2.0 Specification on pages 38 and 42.
		switch (musb->xceiv->otg->state) {
		//Wait for the B device to start a session (B-device requests A-device to turn on VBUS and start a session).
		case OTG_STATE_A_SUSPEND:
			/* remote wakeup? */
			musb->port1_status |=
					(USB_PORT_STAT_C_SUSPEND << 16)
					| MUSB_PORT_STAT_RESUME;
			//"Every Host controller driver should drive the resume signalling
			// on the bus for the amount of time defined by this macro
			// [USB_RESUME_TIMEOUT]."
			musb->rh_timer = jiffies
				+ msecs_to_jiffies(USB_RESUME_TIMEOUT);
			musb->xceiv->otg->state = OTG_STATE_A_HOST;
			musb->is_active = 1;
			//Calls usb_hcd_resume_root_hub: "called by HCD to resume its root
			//hub". Different root hubs connected to the USB controller for
			//managing the speed (1.1, 2.0 and 3.0).
			musb_host_resume_root_hub(musb);
			schedule_delayed_work(&musb->finish_resume_work,	//More computational heavy work is delayed to kernel thread.
				msecs_to_jiffies(USB_RESUME_TIMEOUT));
			break;
		case OTG_STATE_B_WAIT_ACON:
			musb->xceiv->otg->state = OTG_STATE_B_PERIPHERAL;
			musb->is_active = 1;								//Active device connected?
			MUSB_DEV_MODE(musb);								//USB controller has a peripheral role.
			break;
		default:
			WARNING("bogus %s RESUME (%s)\n",
				"host",
				usb_otg_state_string(musb->xceiv->otg->state));
		}
	} else {
		switch (musb->xceiv->otg->state) {
		case OTG_STATE_A_SUSPEND:
			/* possibly DISCONNECT is upcoming */
			musb->xceiv->otg->state = OTG_STATE_A_HOST;
			musb_host_resume_root_hub(musb);					//Calls usb_hcd_resume_root_hub: "called by HCD to resume its root hub"
			break;
		case OTG_STATE_B_WAIT_ACON:
		case OTG_STATE_B_PERIPHERAL:
			/* disconnect while suspended?  we may
			 * not get a disconnect irq...
			 */
			if ((devctl & MUSB_DEVCTL_VBUS)						//VBus valid
					!= (3 << MUSB_DEVCTL_VBUS_SHIFT)
					) {
				musb->int_usb |= MUSB_INTR_DISCONNECT;			//"Set in Host mode when a device disconnect is detected."
				musb->int_usb &= ~MUSB_INTR_SUSPEND;			//Clear suspend interrupt.
				break;
			}
			//Empty function since
			//'IS_ENABLED(CONFIG_USB_MUSB_GADGET) || IS_ENABLED(CONFIG_USB_MUSB_DUAL_ROLE)'
			//is false in musb_gadget.h which is included by musb_core.h.
			musb_g_resume(musb);
			break;
		case OTG_STATE_B_IDLE:
			musb->int_usb &= ~MUSB_INTR_SUSPEND;
			break;
		default:
			WARNING("bogus %s RESUME (%s)\n",
				"peripheral",
				usb_otg_state_string(musb->xceiv->otg->state));
		}
	}
}

/* return IRQ_HANDLED to tell the caller to return immediately */
//Starts a session and updates mode to host if A device.
static irqreturn_t musb_handle_intr_sessreq(struct musb *musb, u8 devctl)
{
//printk("drivers/usb/musb/musb_core.c:musb_handle_intr_sessreq\n");
	void __iomem *mbase = musb->mregs;

	if ((devctl & MUSB_DEVCTL_VBUS) == MUSB_DEVCTL_VBUS		//VBus valid.
			&& (devctl & MUSB_DEVCTL_BDEVICE)) {			//MUSB operating in B device mode.
		musb_dbg(musb, "SessReq while on B state");
		return IRQ_HANDLED;
	}

	musb_dbg(musb, "SESSION_REQUEST (%s)",
		usb_otg_state_string(musb->xceiv->otg->state));

	/* IRQ arrives from ID pin sense or (later, if VBUS power
	 * is removed) SRP.  responses are time critical:
	 *  - turn on VBUS (with silicon-specific mechanism)
	 *  - go through A_WAIT_VRISE
	 *  - ... to A_WAIT_BCON.
	 * a_wait_vrise_tmout triggers VBUS_ERROR transitions
	 */
	//"When operating as an ‘A’ device, this bit is set or cleared by the CPU to start or end a session."
	musb_writeb(mbase, MUSB_DEVCTL, MUSB_DEVCTL_SESSION);
	musb->ep0_stage = MUSB_EP0_START;						//"expect ack of setup"
	musb->xceiv->otg->state = OTG_STATE_A_IDLE;
	MUSB_HST_MODE(musb);									//MUSB is in host mode.
	musb_platform_set_vbus(musb, 1);						//Empty function?

	return IRQ_NONE;
}

//Restarts a session.
static void musb_handle_intr_vbuserr(struct musb *musb, u8 devctl)
{
//printk("drivers/usb/musb/musb_core.c:musb_handle_intr_vbuserr\n");
	int	ignore = 0;

	/* During connection as an A-Device, we may see a short
	 * current spikes causing voltage drop, because of cable
	 * and peripheral capacitance combined with vbus draw.
	 * (So: less common with truly self-powered devices, where
	 * vbus doesn't act like a power supply.)
	 *
	 * Such spikes are short; usually less than ~500 usec, max
	 * of ~2 msec.  That is, they're not sustained overcurrent
	 * errors, though they're reported using VBUSERROR irqs.
	 *
	 * Workarounds:  (a) hardware: use self powered devices.
	 * (b) software:  ignore non-repeated VBUS errors.
	 *
	 * REVISIT:  do delays from lots of DEBUG_KERNEL checks
	 * make trouble here, keeping VBUS < 4.4V ?
	 */
	switch (musb->xceiv->otg->state) {
	case OTG_STATE_A_HOST:
		/* recovery is dicey once we've gotten past the
		 * initial stages of enumeration, but if VBUS
		 * stayed ok at the other end of the link, and
		 * another reset is due (at least for high speed,
		 * to redo the chirp etc), it might work OK...
		 */
	case OTG_STATE_A_WAIT_BCON:
	case OTG_STATE_A_WAIT_VRISE:
		if (musb->vbuserr_retry) {
			void __iomem *mbase = musb->mregs;

			musb->vbuserr_retry--;
			ignore = 1;
			devctl |= MUSB_DEVCTL_SESSION;				//Start a session.
			musb_writeb(mbase, MUSB_DEVCTL, devctl);
		} else {
			musb->port1_status |=
				  USB_PORT_STAT_OVERCURRENT
				| (USB_PORT_STAT_C_OVERCURRENT << 16);
		}
		break;
	default:
		break;
	}

	dev_printk(ignore ? KERN_DEBUG : KERN_ERR, musb->controller,
			"VBUS_ERROR in %s (%02x, %s), retry #%d, port1 %08x\n",
			usb_otg_state_string(musb->xceiv->otg->state),
			devctl,
			({ char *s;
			switch (devctl & MUSB_DEVCTL_VBUS) {
			case 0 << MUSB_DEVCTL_VBUS_SHIFT:
				s = "<SessEnd"; break;
			case 1 << MUSB_DEVCTL_VBUS_SHIFT:
				s = "<AValid"; break;
			case 2 << MUSB_DEVCTL_VBUS_SHIFT:
				s = "<VBusValid"; break;
			/* case 3 << MUSB_DEVCTL_VBUS_SHIFT: */
			default:
				s = "VALID"; break;
			} s; }),
			VBUSERR_RETRY_COUNT - musb->vbuserr_retry,
			musb->port1_status);

	/* go through A_WAIT_VFALL then start a new session */
	if (!ignore)
		musb_platform_set_vbus(musb, 0);	//Empty function?
}

//Sets MUSB in suspend mode, updates PHY OTG state and communicates this to the
//Linux kernel.
static void musb_handle_intr_suspend(struct musb *musb, u8 devctl)
{
//printk("drivers/usb/musb/musb_core.c:musb_handle_intr_suspend\n");
	musb_dbg(musb, "SUSPEND (%s) devctl %02x",
		usb_otg_state_string(musb->xceiv->otg->state), devctl);

	switch (musb->xceiv->otg->state) {
	case OTG_STATE_A_PERIPHERAL:
		/* We also come here if the cable is removed, since
		 * this silicon doesn't report ID-no-longer-grounded.
		 *
		 * We depend on T(a_wait_bcon) to shut us down, and
		 * hope users don't do anything dicey during this
		 * undesired detour through A_WAIT_BCON.
		 */
		musb_hnp_stop(musb);									//Sets MUSB in suspend mode.
		musb_host_resume_root_hub(musb);						//"submits a workqueue request to resume the root hub"
		musb_root_disconnect(musb);					//musb_virthub.c: Updates OTG state and allows the Linux kernel to respond to an event?
		musb_platform_try_idle(musb, jiffies					//Empty function?
				+ msecs_to_jiffies(musb->a_wait_bcon
					? : OTG_TIME_A_WAIT_BCON));

		break;
	case OTG_STATE_B_IDLE:
		if (!musb->is_active)
			break;
		fallthrough;
	case OTG_STATE_B_PERIPHERAL:
		musb_g_suspend(musb);									//Empty function.
		musb->is_active = musb->g.b_hnp_enable;
		if (musb->is_active) {
			musb->xceiv->otg->state = OTG_STATE_B_WAIT_ACON;	//Update PHY OTG state?
			musb_dbg(musb, "HNP: Setting timer for b_ase0_brst");
			mod_timer(&musb->otg_timer, jiffies					//Modify timeout for NAKs?
				+ msecs_to_jiffies(
						OTG_TIME_B_ASE0_BRST));
		}
		break;
	case OTG_STATE_A_WAIT_BCON:
		if (musb->a_wait_bcon != 0)
			musb_platform_try_idle(musb, jiffies				//Empty function?
				+ msecs_to_jiffies(musb->a_wait_bcon));
		break;
	case OTG_STATE_A_HOST:
		musb->xceiv->otg->state = OTG_STATE_A_SUSPEND;			//Update PHY OTG state?
		musb->is_active = musb->hcd->self.b_hnp_enable;
		break;
	case OTG_STATE_B_HOST:
		/* Transition to B_PERIPHERAL, see 6.8.2.6 p 44 */
		musb_dbg(musb, "REVISIT: SUSPEND as B_HOST");
		break;
	default:
		/* "should not happen" */
		musb->is_active = 0;
		break;
	}
}

//Enables TX/RX endpoint and USB interrupts and updates OTG state.
static void musb_handle_intr_connect(struct musb *musb, u8 devctl, u8 int_usb)
{
//printk("drivers/usb/musb/musb_core.c:musb_handle_intr_connect\n");
	struct usb_hcd *hcd = musb->hcd;

	musb->is_active = 1;
	musb->ep0_stage = MUSB_EP0_START;

	musb->intrtxe = musb->epmask;
	musb_writew(musb->mregs, MUSB_INTRTXE, musb->intrtxe);	//"interrupt enable bits for the interrupts" of TX endpoints.
	musb->intrrxe = musb->epmask & 0xfffe;
	musb_writew(musb->mregs, MUSB_INTRRXE, musb->intrrxe);	//"interrupt enable bits for the interrupts" of RX endpoints.
	musb_writeb(musb->mregs, MUSB_INTRUSBE, 0xf7);			//Enables all USB interrupts except suspend.
	musb->port1_status &= ~(USB_PORT_STAT_LOW_SPEED
				|USB_PORT_STAT_HIGH_SPEED
				|USB_PORT_STAT_ENABLE
				);
	musb->port1_status |= USB_PORT_STAT_CONNECTION
				|(USB_PORT_STAT_C_CONNECTION << 16);

	/* high vs full speed is just a guess until after reset */
	//"This Read-only bit is set when a low-speed device has been detected being
	// connected to the port. Only valid in Host mode."
	if (devctl & MUSB_DEVCTL_LSDEV)
		musb->port1_status |= USB_PORT_STAT_LOW_SPEED;

	/* indicate new connection to OTG machine */
	switch (musb->xceiv->otg->state) {
	case OTG_STATE_B_PERIPHERAL:
		if (int_usb & MUSB_INTR_SUSPEND) {
			musb_dbg(musb, "HNP: SUSPEND+CONNECT, now b_host");
			int_usb &= ~MUSB_INTR_SUSPEND;
			goto b_host;
		} else
			musb_dbg(musb, "CONNECT as b_peripheral???");
		break;
	case OTG_STATE_B_WAIT_ACON:
		musb_dbg(musb, "HNP: CONNECT, now b_host");
b_host:
		musb->xceiv->otg->state = OTG_STATE_B_HOST;
		if (musb->hcd)
			musb->hcd->self.is_b_host = 1;
		del_timer(&musb->otg_timer);
		break;
	default:
		if ((devctl & MUSB_DEVCTL_VBUS)
				== (3 << MUSB_DEVCTL_VBUS_SHIFT)) {
			musb->xceiv->otg->state = OTG_STATE_A_HOST;
			if (hcd)
				hcd->self.is_b_host = 0;
		}
		break;
	}

	musb_host_poke_root_hub(musb);	//Calls the root hub due to an event if pending URB, otherwise resumes root hub.

	musb_dbg(musb, "CONNECT (%s) devctl %02x",
			usb_otg_state_string(musb->xceiv->otg->state), devctl);
}

//Sets USB in disconnected/unplugged state?
static void musb_handle_intr_disconnect(struct musb *musb, u8 devctl)
{
//printk("drivers/usb/musb/musb_core.c:musb_handle_intr_disconnect\n");
	musb_dbg(musb, "DISCONNECT (%s) as %s, devctl %02x",
			usb_otg_state_string(musb->xceiv->otg->state),
			MUSB_MODE(musb), devctl);

	switch (musb->xceiv->otg->state) {
	case OTG_STATE_A_HOST:
	case OTG_STATE_A_SUSPEND:
		musb_host_resume_root_hub(musb);					//"submits a workqueue request to resume the root hub"
		musb_root_disconnect(musb);					//musb_virthub.c: Updates OTG state and allows the Linux kernel to respond to an event?
		if (musb->a_wait_bcon != 0)
			musb_platform_try_idle(musb, jiffies			//Empty function?
				+ msecs_to_jiffies(musb->a_wait_bcon));
		break;
	case OTG_STATE_B_HOST:
		/* REVISIT this behaves for "real disconnect"
		 * cases; make sure the other transitions from
		 * from B_HOST act right too.  The B_HOST code
		 * in hnp_stop() is currently not used...
		 */
		musb_root_disconnect(musb);					//musb_virthub.c: Updates OTG state and allows the Linux kernel to respond to an event?
		if (musb->hcd)
			musb->hcd->self.is_b_host = 0;
		musb->xceiv->otg->state = OTG_STATE_B_PERIPHERAL;	//Updates PHY OTG state?
		MUSB_DEV_MODE(musb);								//USB controller in peripheral mode.
		musb_g_disconnect(musb);							//Empty function.
		break;
	case OTG_STATE_A_PERIPHERAL:
		musb_hnp_stop(musb);								//Sets MUSB in suspend mode.
		musb_root_disconnect(musb);					//musb_virthub.c: Updates OTG state and allows the Linux kernel to respond to an event?
		fallthrough;
	case OTG_STATE_B_WAIT_ACON:
	case OTG_STATE_B_PERIPHERAL:
	case OTG_STATE_B_IDLE:
		musb_g_disconnect(musb);							//Empty function.
		break;
	default:
		WARNING("unhandled DISCONNECT transition (%s)\n",
			usb_otg_state_string(musb->xceiv->otg->state));
		break;
	}
}

/*
 * mentor saves a bit: bus reset and babble share the same irq.
 * only host sees babble; only peripheral sees bus reset.
 */
//Recovers from transfer size exceeding limit(?), sets OTG timer(?), and sets
//PHY OTG state(?).
static void musb_handle_intr_reset(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_handle_intr_reset\n");
	if (is_host_active(musb)) {	//Host mode?
		/*
		 * When BABBLE happens what we can depends on which
		 * platform MUSB is running, because some platforms
		 * implemented proprietary means for 'recovering' from
		 * Babble conditions. One such platform is AM335x. In
		 * most cases, however, the only thing we can do is
		 * drop the session.
		 */
		dev_err(musb->controller, "Babble\n");
		//Recovers from data transfer exceeding size limit(?), enables
		//interrupts, notifies the Linux kernel of an event, configures FIFOs,
		//and sets OTG state.
		musb_recover_from_babble(musb);
	} else {
		musb_dbg(musb, "BUS RESET as %s",
			usb_otg_state_string(musb->xceiv->otg->state));
		switch (musb->xceiv->otg->state) {
		case OTG_STATE_A_SUSPEND:
			musb_g_reset(musb);									//Empty function.
			fallthrough;
		case OTG_STATE_A_WAIT_BCON:	/* OPT TD.4.7-900ms */
			/* never use invalid T(a_wait_bcon) */
			musb_dbg(musb, "HNP: in %s, %d msec timeout",
				usb_otg_state_string(musb->xceiv->otg->state),
				TA_WAIT_BCON(musb));
			mod_timer(&musb->otg_timer, jiffies					//Sets timer to 1 min.
				+ msecs_to_jiffies(TA_WAIT_BCON(musb)));
			break;
		case OTG_STATE_A_PERIPHERAL:
			del_timer(&musb->otg_timer);						//Removes OTG state timer?
			musb_g_reset(musb);									//Empty function.
			break;
		case OTG_STATE_B_WAIT_ACON:
			musb_dbg(musb, "HNP: RESET (%s), to b_peripheral",
				usb_otg_state_string(musb->xceiv->otg->state));
			musb->xceiv->otg->state = OTG_STATE_B_PERIPHERAL;	//Sets PHY OTG state?
			musb_g_reset(musb);									//Empty function.
			break;
		case OTG_STATE_B_IDLE:
			musb->xceiv->otg->state = OTG_STATE_B_PERIPHERAL;	//Sets PHY OTG state?
			fallthrough;
		case OTG_STATE_B_PERIPHERAL:
			musb_g_reset(musb);									//Empty function.
			break;
		default:
			musb_dbg(musb, "Unhandled BUS RESET as %s",
				usb_otg_state_string(musb->xceiv->otg->state));
		}
	}
}

/*
 * Interrupt Service Routine to record USB "global" interrupts.
 * Since these do not happen often and signify things of
 * paramount importance, it seems OK to check them individually;
 * the order of the tests is specified in the manual
 *
 * @param musb instance pointer
 * @param int_usb register contents
 * @param devctl
 */
//Most of these functions deal with the OTG Session Request Protocol.
//See On-The-Go Supplement to the USB 2.0 Specification sections 6.8.1 and 6.8.2.
//Handles interrupts related to device resume, SRP, VBUS, suspend, device
//connection/disconnection, and reset.
static irqreturn_t musb_stage0_irq(struct musb *musb, u8 int_usb, //int_usb=interrupts of MUSB except generic interrupt flag, which is INTRUSB
				u8 devctl)											//devctl = OTG device control register.
{
//printk("drivers/usb/musb/musb_core.c:musb_stage0_irq\n");
	irqreturn_t handled = IRQ_NONE;

	musb_dbg(musb, "<== DevCtl=%02x, int_usb=0x%x", devctl, int_usb);

	/* in host mode, the peripheral may issue remote wakeup.
	 * in peripheral mode, the host may resume the link.
	 * spurious RESUME irqs happen too, paired with SUSPEND.
	 */
	if (int_usb & MUSB_INTR_RESUME) {			//"Set when Resume signaling is detected on the bus while the MUSBMHDRC is in Suspend mode."
		musb_handle_intr_resume(musb, devctl);	//Handle resume after suspend mode.
		handled = IRQ_HANDLED;
	}

	/* see manual for the order of the tests */
	if (int_usb & MUSB_INTR_SESSREQ) {		//"Set when Session Request signaling has been detected. Only valid when MUSBMHDRC is ‘A’ device."
		if (musb_handle_intr_sessreq(musb, devctl))	//Starts a session and updates mode to host if A device.
			return IRQ_HANDLED;
		handled = IRQ_HANDLED;
	}

	if (int_usb & MUSB_INTR_VBUSERROR) {
		//"Set when VBus drops below the VBus Valid threshold during a session. Only valid when MUSBMHDRC is ‘A’ device."
		musb_handle_intr_vbuserr(musb, devctl);	//Restarts a session.
		handled = IRQ_HANDLED;
	}

	if (int_usb & MUSB_INTR_SUSPEND) {	//"Set when Suspend signaling is detected on the bus. Only valid in Peripheral mode."
		musb_handle_intr_suspend(musb, devctl);	//Sets MUSB in suspend mode, updates PHY OTG state and communicates this to the Linux kernel.
		handled = IRQ_HANDLED;
	}

	if (int_usb & MUSB_INTR_CONNECT) {	
		//"Set when a device connection is detected. Only valid in Host mode. Valid at all transaction speeds."
		musb_handle_intr_connect(musb, devctl, int_usb);	//Enables TX/RX endpoint and USB interrupts and updates OTG state.
		handled = IRQ_HANDLED;
	}

	if (int_usb & MUSB_INTR_DISCONNECT) {
		//"Set in Host mode when a device disconnect is detected. Set in Peripheral mode when a session ends. Valid at all transaction speeds"
		musb_handle_intr_disconnect(musb, devctl);			//Sets USB in disconnected/unplugged state?
		handled = IRQ_HANDLED;
	}

	if (int_usb & MUSB_INTR_RESET) {	//"Set in Peripheral mode when Reset signaling is detected on the bus."
		musb_handle_intr_reset(musb);	//Recovers from transfer size exceeding limit(?), sets OTG timer(?), and sets PHY OTG state(?).
		handled = IRQ_HANDLED;
	}

#if 0
/* REVISIT ... this would be for multiplexing periodic endpoints, or
 * supporting transfer phasing to prevent exceeding ISO bandwidth
 * limits of a given frame or microframe.
 *
 * It's not needed for peripheral side, which dedicates endpoints;
 * though it _might_ use SOF irqs for other purposes.
 *
 * And it's not currently needed for host side, which also dedicates
 * endpoints, relies on TX/RX interval registers, and isn't claimed
 * to support ISO transfers yet.
 */
	if (int_usb & MUSB_INTR_SOF) {
		void __iomem *mbase = musb->mregs;
		struct musb_hw_ep	*ep;
		u8 epnum;
		u16 frame;

		dev_dbg(musb->controller, "START_OF_FRAME\n");
		handled = IRQ_HANDLED;

		/* start any periodic Tx transfers waiting for current frame */
		frame = musb_readw(mbase, MUSB_FRAME);
		ep = musb->endpoints;
		for (epnum = 1; (epnum < musb->nr_endpoints)
					&& (musb->epmask >= (1 << epnum));
				epnum++, ep++) {
			/*
			 * FIXME handle framecounter wraps (12 bits)
			 * eliminate duplicated StartUrb logic
			 */
			if (ep->dwWaitFrame >= frame) {
				ep->dwWaitFrame = 0;
				pr_debug("SOF --> periodic TX%s on %d\n",
					ep->tx_channel ? " DMA" : "",
					epnum);
				if (!ep->tx_channel)
					musb_h_tx_start(musb, epnum);
				else
					cppi_hostdma_start(musb, epnum);
			}
		}		/* end of for loop */
	}
#endif

	//Work to be executed as part of a kernel thread rather than in the
	//interrupt handler to prevent slow response?
	schedule_delayed_work(&musb->irq_work, 0);

	return handled;
}

/*-------------------------------------------------------------------------*/
//Disables and clears interrupts, by writing MUSB (USB0/1CORE undocumented)
//register.
static void musb_disable_interrupts(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_disable_interrupts\n");
	void __iomem	*mbase = musb->mregs;

	/* disable interrupts */
	musb_writeb(mbase, MUSB_INTRUSBE, 0);	//Disables interrupts for USB controller.
	musb->intrtxe = 0;
	musb_writew(mbase, MUSB_INTRTXE, 0);	//Disables interrupts for TX endpoints.
	musb->intrrxe = 0;
	musb_writew(mbase, MUSB_INTRRXE, 0);	//Disables interrupts for RX endpoints.

	/*  flush pending interrupts */
	musb_clearb(mbase, MUSB_INTRUSB);	//Invokes musb_default_readb which clears active interrupts when reading.
	musb_clearw(mbase, MUSB_INTRTX);	//Invokes musb_default_readw which clears active interrupts when reading.
	musb_clearw(mbase, MUSB_INTRRX);	//Invokes musb_default_readw which clears active interrupts when reading.
}

//Enables TX/RX endpoints and common USB interrupts.
static void musb_enable_interrupts(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_enable_interrupts\n");
	void __iomem    *regs = musb->mregs;

	/*  Set INT enable registers, enable interrupts */
	musb->intrtxe = musb->epmask;					//TX endpoints whose interrupts shall be enabled.
	musb_writew(regs, MUSB_INTRTXE, musb->intrtxe);	//Writes enables flags to TX endpoints interrupt enable register.
	musb->intrrxe = musb->epmask & 0xfffe;			//RX endpoints whose interrupts shall be enabled.
	musb_writew(regs, MUSB_INTRRXE, musb->intrrxe);	//Writes enables flags to RX endpoints interrupt enable register.
	musb_writeb(regs, MUSB_INTRUSBE, 0xf7);		//Enable all interrupt for common USB controller interrupts, except for suspend interrupts.

}

/*
 * Program the HDRC to start (enable interrupts, dma, etc.).
 */
//Enables interrupts and sets the OTG state.
void musb_start(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_start\n");
	void __iomem    *regs = musb->mregs;
	u8              devctl = musb_readb(regs, MUSB_DEVCTL);
	u8		power;

	musb_dbg(musb, "<== devctl %02x", devctl);

	musb_enable_interrupts(musb);			//Enables TX/RX endpoints and common USB interrupts.
	//"Testmode is an 8-bit register that is primarily used to put the MUSBMHDRC
	// into one of the four test modes for High-speed operation described in the
	// USB 2.0 specification."
	//No test mode.
	musb_writeb(regs, MUSB_TESTMODE, 0);

	power = MUSB_POWER_ISOUPDATE;			//"MUSBMHDRC will wait for an SOF token from the time TxPktRdy is set before sending the packet"
	/*
	 * treating UNKNOWN as unspecified maximum speed, in which case
	 * we will default to high-speed.
	 */
	if (musb->config->maximum_speed == USB_SPEED_HIGH ||
			musb->config->maximum_speed == USB_SPEED_UNKNOWN)
		power |= MUSB_POWER_HSENAB;			//"the MUSBMHDRC will negotiate for High-speed mode when the device is reset by the hub"
	musb_writeb(regs, MUSB_POWER, power);	//Writes the power register which controls suspend and resume signaling and some other basics.

	musb->is_active = 0;					//MUSB is inactive by default.
	//Reads OTG device control register:
	//"used to select whether the MUSBMHDRC is operating in Peripheral mode or
	// in Host mode, and for controlling and monitoring the USB VBus line."
	devctl = musb_readb(regs, MUSB_DEVCTL);
	//"When operating as an ‘A’ device, this bit is set or cleared by the CPU to start or end a session."
	//"When operating as a ‘B’ device, this bit is set/cleared by the MUSBMHDRC when a session starts/ends."
	devctl &= ~MUSB_DEVCTL_SESSION;

	/* session started after:
	 * (a) ID-grounded irq, host mode;
	 * (b) vbus present/connect IRQ, peripheral mode;
	 * (c) peripheral initiates, using SRP
	 */
	//"Session Request Protocol (SRP): Allows both communicating devices to
	// control when the link's power session is active"
	if (musb->port_mode != MUSB_HOST &&
			musb->xceiv->otg->state != OTG_STATE_A_WAIT_BCON &&
			(devctl & MUSB_DEVCTL_VBUS) == MUSB_DEVCTL_VBUS) {
		musb->is_active = 1;	//MUSB is active.
	} else {
		devctl |= MUSB_DEVCTL_SESSION;
	}

	musb_platform_enable(musb);				//Invokes dsps_musb_enable which enables MUSB interrupts via USB0/1 TRM registers.
	musb_writeb(regs, MUSB_DEVCTL, devctl);	//Writes OTG device control register.
}

/*
 * Make the HDRC stop (disable interrupts, etc.);
 * reversible by musb_start
 * called on gadget driver unregister
 * with controller locked, irqs blocked
 * acts as a NOP unless some role activated the hardware
 */
void musb_stop(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_stop\n");
	/* stop IRQs, timers, ... */
	musb_platform_disable(musb);
	musb_disable_interrupts(musb);
	musb_writeb(musb->mregs, MUSB_DEVCTL, 0);

	/* FIXME
	 *  - mark host and/or peripheral drivers unusable/inactive
	 *  - disable DMA (and enable it in HdrcStart)
	 *  - make sure we can musb_start() after musb_stop(); with
	 *    OTG mode, gadget driver module rmmod/modprobe cycles that
	 *  - ...
	 */
	musb_platform_try_idle(musb, 0);
}

/*-------------------------------------------------------------------------*/

/*
 * The silicon either has hard-wired endpoint configurations, or else
 * "dynamic fifo" sizing.  The driver has support for both, though at this
 * writing only the dynamic sizing is very well tested.   Since we switched
 * away from compile-time hardware parameters, we can no longer rely on
 * dead code elimination to leave only the relevant one in the object file.
 *
 * We don't currently use dynamic fifo setup capability to do anything
 * more than selecting one of a bunch of predefined configurations.
 */
static ushort fifo_mode;

/* "modprobe ... fifo_mode=1" etc */
module_param(fifo_mode, ushort, 0);
MODULE_PARM_DESC(fifo_mode, "initial endpoint configuration");

/*
 * tables defining fifo_mode values.  define more if you like.
 * for host side, make sure both halves of ep1 are set up.
 */

/* mode 0 - fits in 2KB */
static struct musb_fifo_cfg mode_0_cfg[] = {
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_RXTX, .maxpacket = 512, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

/* mode 1 - fits in 4KB */
static struct musb_fifo_cfg mode_1_cfg[] = {
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 2, .style = FIFO_RXTX, .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

/* mode 2 - fits in 4KB */
static struct musb_fifo_cfg mode_2_cfg[] = {
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 960, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 1024, },
};

/* mode 3 - fits in 4KB */
static struct musb_fifo_cfg mode_3_cfg[] = {
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

/* mode 4 - fits in 16KB */
static struct musb_fifo_cfg mode_4_cfg[] = {
{ .hw_ep_num =  1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  3, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  3, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  4, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  4, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  5, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  5, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  6, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  6, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  7, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  7, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  8, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  8, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  9, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  9, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 10, .style = FIFO_TX,   .maxpacket = 256, },
{ .hw_ep_num = 10, .style = FIFO_RX,   .maxpacket = 64, },
{ .hw_ep_num = 11, .style = FIFO_TX,   .maxpacket = 256, },
{ .hw_ep_num = 11, .style = FIFO_RX,   .maxpacket = 64, },
{ .hw_ep_num = 12, .style = FIFO_TX,   .maxpacket = 256, },
{ .hw_ep_num = 12, .style = FIFO_RX,   .maxpacket = 64, },
{ .hw_ep_num = 13, .style = FIFO_RXTX, .maxpacket = 4096, },
{ .hw_ep_num = 14, .style = FIFO_RXTX, .maxpacket = 1024, },
{ .hw_ep_num = 15, .style = FIFO_RXTX, .maxpacket = 1024, },
};

/* mode 5 - fits in 8KB */
static struct musb_fifo_cfg mode_5_cfg[] = {
{ .hw_ep_num =  1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  3, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  3, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  4, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  4, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  5, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  5, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  6, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num =  6, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num =  7, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num =  7, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num =  8, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num =  8, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num =  9, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num =  9, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num = 10, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num = 10, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num = 11, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num = 11, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num = 12, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num = 12, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num = 13, .style = FIFO_RXTX, .maxpacket = 512, },
{ .hw_ep_num = 14, .style = FIFO_RXTX, .maxpacket = 1024, },
{ .hw_ep_num = 15, .style = FIFO_RXTX, .maxpacket = 1024, },
};

/*
 * configure a fifo; for non-shared endpoints, this may be called
 * once for a tx fifo and once for an rx fifo.
 *
 * returns negative errno or offset for next fifo.
 */
//Configures the USB packet FIFO for an endpoint.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//Offset is the start of the FIFO of the endpoint in the shared memory??????????
//See section 16.2.6 in TRM!Total ly 32KB in each USB0/1 Mentor graphics module!
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//Sets size and start offset of endpoint FIFO, records the maximum packet size
//(not larger than FIFO size) and whether the FIFO is shared between TX and RX
//(false).
static int
fifo_setup(struct musb *musb, struct musb_hw_ep  *hw_ep,
		const struct musb_fifo_cfg *cfg, u16 offset)	//Offset is offset of endpoint FIFO in USB0/1 FIFO RAM?????????????????????
{
//printk("drivers/usb/musb/musb_core.c:fifo_setup\n");
	void __iomem	*mbase = musb->mregs;				//Undocumented USB0/1CORE registers.
	int	size = 0;
	u16	maxpacket = cfg->maxpacket;						//64 for endpoint 0.
	u16	c_off = offset >> 3;							//FIFO offset is specified as multiples of 8 bytes?
	u8	c_size;

	//"The allocation of FIFO space to the different endpoints requires
	// programming for each Tx and Rx endpoint with the following information:
	// •The start address of the FIFO within the RAM block
	// •The maximum size of packet to be supported
	// •Whether double-buffering is required"
	// (These last two together define the amount of space that needs to be
	// allocated to the FIFO.)

	/* expect hw_ep has already been zero-initialized */
	//Finds LSB that is set. 1 means 2^(1-1) = 1 byte, 2 means 2^(2-1) = 2 bytes,
	//3 means 2^(3-1)=4 bytes.
	size = ffs(max(maxpacket, (u16) 8)) - 1;	//Recalculate power by start counting from 0.
	maxpacket = 1 << size;						//Max packet size is a power of 2?

	c_size = size - 3;							//FIFO size is one 8th as large as specified?
	//FALSE since mode 4 is not explicitly initialized, meaning that the enum is
	//0, and hence BUF_SINGLE.
	if (cfg->mode == BUF_DOUBLE) {	//FALSE
		if ((offset + (maxpacket << 1)) >
				(1 << (musb->config->ram_bits + 2)))
			return -EMSGSIZE;
		c_size |= MUSB_FIFOSZ_DPB;
	} else {						//TRUE
		if ((offset + maxpacket) > (1 << (musb->config->ram_bits + 2)))
			return -EMSGSIZE;
	}

	/* configure the FIFO */
	musb_writeb(mbase, MUSB_INDEX, hw_ep->epnum);	//Writes proxy register (the real register not directly accessible; see TRM 16.2.6).

	/* EP0 reserved endpoint for control, bidirectional;
	 * EP1 reserved for bulk, two unidirectional halves.
	 */
	if (hw_ep->epnum == 1)
		musb->bulk_ep = hw_ep;
	/* REVISIT error check:  be sure ep0 can both rx and tx ... */
	//Write size of endpoint FIFO.
	//Write start offset of endpoint FIFO.
	//Record for this endpoint the maximum packet size.
	//Record for this endpoint whether the FIFO is shared if the endpoint is for
	//both RX and TX. is_shared_fifo is initialized to false since the memory
	//for allocating a hardware endpoint is initialized to zero
	//(see musb = devm_kzalloc(dev, sizeof(*musb), GFP_KERNEL), where musb
	//contains struct musb_hw_ep endpoints[MUSB_C_NUM_EPS], where MUSB_C_NUM_EPS = 16).
	switch (cfg->style) {	//Initialized by mode_4_cfg array structure.
	case FIFO_TX:
		musb_writeb(mbase, MUSB_TXFIFOSZ, c_size);
		musb_writew(mbase, MUSB_TXFIFOADD, c_off);
		hw_ep->tx_double_buffered = !!(c_size & MUSB_FIFOSZ_DPB);
		hw_ep->max_packet_sz_tx = maxpacket;
		break;
	case FIFO_RX:
		musb_writeb(mbase, MUSB_RXFIFOSZ, c_size);
		musb_writew(mbase, MUSB_RXFIFOADD, c_off);
		hw_ep->rx_double_buffered = !!(c_size & MUSB_FIFOSZ_DPB);
		hw_ep->max_packet_sz_rx = maxpacket;
		break;
	case FIFO_RXTX:
		musb_writeb(mbase, MUSB_TXFIFOSZ, c_size);
		musb_writew(mbase, MUSB_TXFIFOADD, c_off);
		hw_ep->rx_double_buffered = !!(c_size & MUSB_FIFOSZ_DPB);
		hw_ep->max_packet_sz_rx = maxpacket;

		musb_writeb(mbase, MUSB_RXFIFOSZ, c_size);
		musb_writew(mbase, MUSB_RXFIFOADD, c_off);
		hw_ep->tx_double_buffered = hw_ep->rx_double_buffered;
		hw_ep->max_packet_sz_tx = maxpacket;

		hw_ep->is_shared_fifo = true;
		break;
	}

	/* NOTE rx and tx endpoint irqs aren't managed separately,
	 * which happens to be ok
	 */
	musb->epmask |= (1 << hw_ep->epnum);	//Sets interrupt mask?
	//Returns the start address of the hardware endpoint FIFO plus the size in
	//byte of that FIFO (as specified by mode_4_cfg). That is, The start
	//address of the next FIFO.
	return offset + (maxpacket << ((c_size & MUSB_FIFOSZ_DPB) ? 1 : 0));
}

static struct musb_fifo_cfg ep0_cfg = {
	.style = FIFO_RXTX, .maxpacket = 64,
};

//Configures the FIFOs for the endpoints for the DMA packets independent of the
//DMA.
static int ep_config_from_table(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:ep_config_from_table\n");
	const struct musb_fifo_cfg	*cfg;
	unsigned		i, n;
	int			offset;
	struct musb_hw_ep	*hw_ep = musb->endpoints;

	if (musb->config->fifo_cfg) {
		cfg = musb->config->fifo_cfg;
		n = musb->config->fifo_cfg_size;
		goto done;
	}

	switch (fifo_mode) {	//4.
	default:
		fifo_mode = 0;
		fallthrough;
	case 0:
		cfg = mode_0_cfg;
		n = ARRAY_SIZE(mode_0_cfg);
		break;
	case 1:
		cfg = mode_1_cfg;
		n = ARRAY_SIZE(mode_1_cfg);
		break;
	case 2:
		cfg = mode_2_cfg;
		n = ARRAY_SIZE(mode_2_cfg);
		break;
	case 3:
		cfg = mode_3_cfg;
		n = ARRAY_SIZE(mode_3_cfg);
		break;
	case 4:
		cfg = mode_4_cfg;			//Mode 4 array
		n = ARRAY_SIZE(mode_4_cfg);	//Number of entries. 27, where endpoints 13, 14 and 15 are both RX and TX.
		break;						//The first 9 endpoints has a maximum packet size/FIFO size(?) of 512 bytes.
	case 5:
		cfg = mode_5_cfg;
		n = ARRAY_SIZE(mode_5_cfg);
		break;
	}

	pr_debug("%s: setup fifo_mode %d\n", musb_driver_name, fifo_mode);


done:
	//MUSB0/1 data structure, pointer to array of endpoint structures, address
	//of endpoint 0 configuration (both RX and TX and 64 bytes max size packet).
	//Initializes the FIFO start address at 0 and size for endpoint 0 (64 bytes)
	//and returns the start address of the FIFO for the next endpoint.
	offset = fifo_setup(musb, hw_ep, &ep0_cfg, 0);
	/* assert(offset > 0) */

	/* NOTE:  for RTL versions >= 1.400 EPINFO and RAMINFO would
	 * be better than static musb->config->num_eps and DYN_FIFO_SIZE...
	 */

	for (i = 0; i < n; i++) {		//For all endpoint entries in mode_4_cfg get
		u8	epn = cfg->hw_ep_num;	//the endpoint number for the current entry of mode_4_cfg.

		if (epn >= musb->config->num_eps) {	//Invalid endpoint number
			pr_debug("%s: invalid ep %d\n",
					musb_driver_name, epn);
			return -EINVAL;
		}
		//Configure the FIFO for the endpoint, given its hw_ep structure, next
		//entry of mode_4_cfg and the start address immediately following the
		//end address of the FIFO of the previous endpoint.
		offset = fifo_setup(musb, hw_ep + epn, cfg++, offset);
		if (offset < 0) {
			pr_debug("%s: mem overrun, ep %d\n",
					musb_driver_name, epn);
			return offset;
		}
		//If the endpoint number found in mode_4_cfg implies more endpoints than
		//previously recorded, then that number is incremented.
		epn++;
		musb->nr_endpoints = max(epn, musb->nr_endpoints);
	}

	pr_debug("%s: %d/%d max ep, %d/%d memory\n",
			musb_driver_name,
			n + 1, musb->config->num_eps * 2 - 1,
			offset, (1 << (musb->config->ram_bits + 2)));

	if (!musb->bulk_ep) {
		pr_debug("%s: missing bulk\n", musb_driver_name);
		return -EINVAL;
	}

	return 0;
}


/*
 * ep_config_from_hw - when MUSB_C_DYNFIFO_DEF is false
 * @param musb the controller
 */
static int ep_config_from_hw(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:ep_config_from_hw\n");
	u8 epnum = 0;
	struct musb_hw_ep *hw_ep;
	void __iomem *mbase = musb->mregs;
	int ret = 0;

	musb_dbg(musb, "<== static silicon ep config");

	/* FIXME pick up ep0 maxpacket size */

	for (epnum = 1; epnum < musb->config->num_eps; epnum++) {
		musb_ep_select(mbase, epnum);
		hw_ep = musb->endpoints + epnum;

		ret = musb_read_fifosize(musb, hw_ep, epnum);
		if (ret < 0)
			break;

		/* FIXME set up hw_ep->{rx,tx}_double_buffered */

		/* pick an RX/TX endpoint for bulk */
		if (hw_ep->max_packet_sz_tx < 512
				|| hw_ep->max_packet_sz_rx < 512)
			continue;

		/* REVISIT:  this algorithm is lazy, we should at least
		 * try to pick a double buffered endpoint.
		 */
		if (musb->bulk_ep)
			continue;
		musb->bulk_ep = hw_ep;
	}

	if (!musb->bulk_ep) {
		pr_debug("%s: missing bulk\n", musb_driver_name);
		return -EINVAL;
	}

	return 0;
}

enum { MUSB_CONTROLLER_MHDRC, MUSB_CONTROLLER_HDRC, };

/* Initialize MUSB (M)HDRC part of the USB hardware subsystem;
 * configure endpoints, or take their config from silicon
 */
//Initializes FIFOs for the DMA packet buffers of the MUSB0/1 controller.
static int musb_core_init(u16 musb_type, struct musb *musb)	//HDRC = Highspeed Dual Role Controller is Mentor USB controller (not DMA).
{
//printk("drivers/usb/musb/musb_core.c:musb_core_init\n");
	u8 reg;
	char *type;
	char aInfo[90];
	void __iomem	*mbase = musb->mregs;					//Undocumented USB0/1CORE Mentor Graphics registers.
	int		status = 0;
	int		i;

	/* log core options (read using indexed model) */
	reg = musb_read_configdata(mbase);						//Writes and reads undocumented USB0/1CORE registers.

	strcpy(aInfo, (reg & MUSB_CONFIGDATA_UTMIDW) ? "UTMI-16" : "UTMI-8");	//String stored depending on read undocumented register.
	if (reg & MUSB_CONFIGDATA_DYNFIFO) {	//Endpoints 1-15 of the mentor graphics USB0/1 CORE has dynamically sizable FIFO. TRUE.
		strcat(aInfo, ", dyn FIFOs");
		musb->dyn_fifo = true;
	}
	if (reg & MUSB_CONFIGDATA_MPRXE) {
		strcat(aInfo, ", bulk combine");
		musb->bulk_combine = true;
	}
	if (reg & MUSB_CONFIGDATA_MPTXE) {
		strcat(aInfo, ", bulk split");
		musb->bulk_split = true;
	}
	if (reg & MUSB_CONFIGDATA_HBRXE) {
		strcat(aInfo, ", HB-ISO Rx");
		musb->hb_iso_rx = true;
	}
	if (reg & MUSB_CONFIGDATA_HBTXE) {
		strcat(aInfo, ", HB-ISO Tx");
		musb->hb_iso_tx = true;
	}
	if (reg & MUSB_CONFIGDATA_SOFTCONE)
		strcat(aInfo, ", SoftConn");

	pr_debug("%s: ConfigData=0x%02x (%s)\n", musb_driver_name, reg, aInfo);

	if (MUSB_CONTROLLER_MHDRC == musb_type) {		//False
		musb->is_multipoint = 1;
		type = "M";
	} else {										//True
		musb->is_multipoint = 0;
		type = "";
		if (IS_ENABLED(CONFIG_USB) &&
		    !IS_ENABLED(CONFIG_USB_OTG_DISABLE_EXTERNAL_HUB)) {	//True.
			pr_err("%s: kernel must disable external hubs, please fix the configuration\n",
			       musb_driver_name);
		}
	}

	/* log release info */
	musb->hwvers = musb_readw(mbase, MUSB_HWVERS);		//Reads undocumented USB0/1CORE registers.
	pr_debug("%s: %sHDRC RTL version %d.%d%s\n",
		 musb_driver_name, type, MUSB_HWVERS_MAJOR(musb->hwvers),
		 MUSB_HWVERS_MINOR(musb->hwvers),
		 (musb->hwvers & MUSB_HWVERS_RC) ? "RC" : "");

	/* configure ep0 */
	//Sets endpoint FIFO size to 64 bytes and flags that memory as shared between
	//TX and RX? See 16.2 and 16.2.6 in TRM.
	musb_configure_ep0(musb);

	/* discover endpoint configuration */
	musb->nr_endpoints = 1;	//At least one endpoint.
	musb->epmask = 1;

	if (musb->dyn_fifo)		//True.
		//Configures the FIFOs for the endpoints for the DMA packets independent
		//of the DMA. Returns 0.
		status = ep_config_from_table(musb);
	else
		status = ep_config_from_hw(musb);

	if (status < 0)
		return status;

	/* finish init, and print endpoint config */
	for (i = 0; i < musb->nr_endpoints; i++) {
		struct musb_hw_ep	*hw_ep = musb->endpoints + i;		//Get the endpoint hardware structure.
		//Calls musb_default_fifo_offset with mbase undocumented registers and
		//adding 16*endpoint_number + 0x100.
		hw_ep->fifo = musb->io.fifo_offset(i) + mbase;			//Start address of endpoint's FIFO for DMA packet storage.
#if IS_ENABLED(CONFIG_USB_MUSB_TUSB6010)	//FALSE
		if (musb->ops->quirks & MUSB_IN_TUSB) {
			hw_ep->fifo_async = musb->async + 0x400 +
				musb->io.fifo_offset(i);
			hw_ep->fifo_sync = musb->sync + 0x400 +
				musb->io.fifo_offset(i);
			hw_ep->fifo_sync_va =
				musb->sync_va + 0x400 + musb->io.fifo_offset(i);

			if (i == 0)
				hw_ep->conf = mbase - 0x400 + TUSB_EP0_CONF;
			else
				hw_ep->conf = mbase + 0x400 +
					(((i - 1) & 0xf) << 2);
		}
#endif
		//musb_indexed_ep_offset(i, 0) = 0x10 + 0, where mbase is undocumented USB0/1 CORE.
		//Undocumented addresses USB0/1 CORE + 0x10.
		//Calls musb_indexed_ep_offset, which returns 0x10 + undocumented register base.
		hw_ep->regs = musb->io.ep_offset(i, 0) + mbase;
		hw_ep->rx_reinit = 1;	//Reinitialize RX hardware endpoint.
		hw_ep->tx_reinit = 1;	//Reinitialize TX hardware endpoint.

		if (hw_ep->max_packet_sz_tx) {
			musb_dbg(musb, "%s: hw_ep %d%s, %smax %d",
				musb_driver_name, i,
				hw_ep->is_shared_fifo ? "shared" : "tx",
				hw_ep->tx_double_buffered
					? "doublebuffer, " : "",
				hw_ep->max_packet_sz_tx);
		}
		if (hw_ep->max_packet_sz_rx && !hw_ep->is_shared_fifo) {
			musb_dbg(musb, "%s: hw_ep %d%s, %smax %d",
				musb_driver_name, i,
				"rx",
				hw_ep->rx_double_buffered
					? "doublebuffer, " : "",
				hw_ep->max_packet_sz_rx);
		}
		if (!(hw_ep->max_packet_sz_tx || hw_ep->max_packet_sz_rx))
			musb_dbg(musb, "hw_ep %d not configured", i);
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

/*
 * handle all the irqs defined by the HDRC core. for now we expect:  other
 * irq sources (phy, dma, etc) will be handled first, musb->int_* values
 * will be assigned, and the irq will already have been acked.
 *
 * called in irq context with spinlock held, irqs blocked
 */
//Handles interrupts of TX/RX endpoints, and USB interrupts related to device
//resume, SRP, VBUS, suspend, device connection/disconnection, and reset.
irqreturn_t musb_interrupt(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_interrupt\n");
	irqreturn_t	retval = IRQ_NONE;
	unsigned long	status;
	unsigned long	epnum;
	u8		devctl;

	if (!musb->int_usb && !musb->int_tx && !musb->int_rx)	//No interrupt: IRQ_NONE = "interrupt was not from this device or was not handled"
		return IRQ_NONE;

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);			//Read OTG device control register.

	trace_musb_isr(musb);

	/**
	 * According to Mentor Graphics' documentation, flowchart on page 98,
	 * IRQ should be handled as follows:
	 *
	 * . Resume IRQ
	 * . Session Request IRQ
	 * . VBUS Error IRQ
	 * . Suspend IRQ
	 * . Connect IRQ
	 * . Disconnect IRQ
	 * . Reset/Babble IRQ
	 * . SOF IRQ (we're not using this one)
	 * . Endpoint 0 IRQ
	 * . TX Endpoints
	 * . RX Endpoints
	 *
	 * We will be following that flowchart in order to avoid any problems
	 * that might arise with internal Finite State Machine.
	 */

	if (musb->int_usb)										//Set by dsps_interrupt to interrupts of MUSB except generic interrupt flag.
		//Handles interrupts related to device resume, SRP, VBUS, suspend, device
		//connection/disconnection, and reset.
		retval |= musb_stage0_irq(musb, musb->int_usb, devctl);

	if (musb->int_tx & 1) {	//Handles Endpoint 0 IRQ for TX.
		if (is_host_active(musb))
			//Handles interrupt related to endpoint 0 which is the control
			//endpoint 0 of the USB controller as a whole. Completes the current
			//URB of the endpoint and setups transfer of its next URB?
			retval |= musb_h_ep0_irq(musb);
		else
			retval |= musb_g_ep0_irq(musb);

		/* we have just handled endpoint 0 IRQ, clear it */
		musb->int_tx &= ~BIT(0);
	}

	status = musb->int_tx;

	for_each_set_bit(epnum, &status, 16) {	//Handles TX endpoint interrupts.
		retval = IRQ_HANDLED;
		if (is_host_active(musb))
			musb_host_tx(musb, epnum);
		else
			musb_g_tx(musb, epnum);
	}

	status = musb->int_rx;

	for_each_set_bit(epnum, &status, 16) {	//Handles RX endpoint interrupts.
		retval = IRQ_HANDLED;
		if (is_host_active(musb))
			musb_host_rx(musb, epnum);
		else
			musb_g_rx(musb, epnum);
	}

	return retval;
}
EXPORT_SYMBOL_GPL(musb_interrupt);

#ifndef CONFIG_MUSB_PIO_ONLY	//CONFIG_MUSB_PIO_ONLY is undefined.
static bool use_dma = true;

/* "modprobe ... use_dma=0" etc */
module_param(use_dma, bool, 0644);
MODULE_PARM_DESC(use_dma, "enable/disable use of DMA");

//Services TX, TX availability and RX interrupts, and setups new DMA transfers
//for next URBs. 
void musb_dma_completion(struct musb *musb, u8 epnum, u8 transmit)
{
//printk("drivers/usb/musb/musb_core.c:musb_dma_completion\n");
	/* called with controller lock already held */

	if (!epnum) {							//Endpoint 0.
		if (!is_cppi_enabled(musb)) {		//is_cppi_enabled is true, and this condition is false, as quirks is set to MUSB_DMA_CPPI41.
			/* endpoint 0 */
			if (is_host_active(musb))
				//Handles interrupt related to endpoint 0 which is the control
				//endpoint 0 of the USB controller as a whole. Completes the
				//current URB of the endpoint and setups transfer of its next
				//URB?
				musb_h_ep0_irq(musb);
			else
				//Defined as empty function in musb_gadget.h, which is included
				//in musb_core.h, since CONFIG_USB_MUSB_GADGET and
				//CONFIG_USB_MUSB_DUAL_ROLE are not included in the configuration.
				musb_g_ep0_irq(musb);
		}
	} else {								//Not endpoint 0.
		/* endpoints 1..15 */
		if (transmit) {
			if (is_host_active(musb))		//USB0/1 MUSB controller is a host.
				//Services interrupts for non-zero transmit endpoints. Includes
				//stopping transactions on NAK timeouts, configures next
				//transaction and updates endpoint and DMA data structures.
				musb_host_tx(musb, epnum);	//Service an IRQ for transmit availability or DMA completion.
			else							//USB0/1 MUSB controller is a peripheral.
				musb_g_tx(musb, epnum);	//Empty function body since CONFIG_USB_MUSB_GADGET and CONFIG_USB_MUSB_DUAL_ROLE are undefined.
		} else {
			/* receive */
			if (is_host_active(musb))		//USB0/1 MUSB controller is a host.
				//If last transfer, stops current transaction, updates DMA
				//channel data structures and schedules the next transaction
				//URB. If fault, abort DMA.
				musb_host_rx(musb, epnum);	//"Service an RX interrupt for the given IN endpoint"
			else							//USB0/1 MUSB controller is a peripheral.
				musb_g_rx(musb, epnum);	//Empty function body since CONFIG_USB_MUSB_GADGET and CONFIG_USB_MUSB_DUAL_ROLE are undefined.
		}
	}
}
EXPORT_SYMBOL_GPL(musb_dma_completion);

#else
#define use_dma			0
#endif

static int (*musb_phy_callback)(enum musb_vbus_id_status status);

/*
 * musb_mailbox - optional phy notifier function
 * @status phy state change
 *
 * Optionally gets called from the USB PHY. Note that the USB PHY must be
 * disabled at the point the phy_callback is registered or unregistered.
 */
int musb_mailbox(enum musb_vbus_id_status status)
{
//printk("drivers/usb/musb/musb_core.c:musb_mailbox\n");
	if (musb_phy_callback)
		return musb_phy_callback(status);

	return -ENODEV;
};
EXPORT_SYMBOL_GPL(musb_mailbox);

/*-------------------------------------------------------------------------*/

static ssize_t
mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//printk("drivers/usb/musb/musb_core.c:mode_show\n");
	struct musb *musb = dev_to_musb(dev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&musb->lock, flags);
	ret = sprintf(buf, "%s\n", usb_otg_state_string(musb->xceiv->otg->state));
	spin_unlock_irqrestore(&musb->lock, flags);

	return ret;
}

static ssize_t
mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
//printk("drivers/usb/musb/musb_core.c:mode_store\n");
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;
	int		status;

	spin_lock_irqsave(&musb->lock, flags);
	if (sysfs_streq(buf, "host"))
		status = musb_platform_set_mode(musb, MUSB_HOST);
	else if (sysfs_streq(buf, "peripheral"))
		status = musb_platform_set_mode(musb, MUSB_PERIPHERAL);
	else if (sysfs_streq(buf, "otg"))
		status = musb_platform_set_mode(musb, MUSB_OTG);
	else
		status = -EINVAL;
	spin_unlock_irqrestore(&musb->lock, flags);

	return (status == 0) ? n : status;
}
static DEVICE_ATTR_RW(mode);

static ssize_t
vbus_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
//printk("drivers/usb/musb/musb_core.c:vbus_store\n");
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;
	unsigned long	val;

	if (sscanf(buf, "%lu", &val) < 1) {
		dev_err(dev, "Invalid VBUS timeout ms value\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&musb->lock, flags);
	/* force T(a_wait_bcon) to be zero/unlimited *OR* valid */
	musb->a_wait_bcon = val ? max_t(int, val, OTG_TIME_A_WAIT_BCON) : 0 ;
	if (musb->xceiv->otg->state == OTG_STATE_A_WAIT_BCON)
		musb->is_active = 0;
	musb_platform_try_idle(musb, jiffies + msecs_to_jiffies(val));
	spin_unlock_irqrestore(&musb->lock, flags);

	return n;
}

static ssize_t
vbus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//printk("drivers/usb/musb/musb_core.c:vbus_show\n");
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;
	unsigned long	val;
	int		vbus;
	u8		devctl;

	pm_runtime_get_sync(dev);
	spin_lock_irqsave(&musb->lock, flags);
	val = musb->a_wait_bcon;
	vbus = musb_platform_get_vbus_status(musb);
	if (vbus < 0) {
		/* Use default MUSB method by means of DEVCTL register */
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if ((devctl & MUSB_DEVCTL_VBUS)
				== (3 << MUSB_DEVCTL_VBUS_SHIFT))
			vbus = 1;
		else
			vbus = 0;
	}
	spin_unlock_irqrestore(&musb->lock, flags);
	pm_runtime_put_sync(dev);

	return sprintf(buf, "Vbus %s, timeout %lu msec\n",
			vbus ? "on" : "off", val);
}
static DEVICE_ATTR_RW(vbus);

/* Gadget drivers can't know that a host is connected so they might want
 * to start SRP, but users can.  This allows userspace to trigger SRP.
 */
static ssize_t srp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
//printk("drivers/usb/musb/musb_core.c:srp_store\n");
	struct musb	*musb = dev_to_musb(dev);
	unsigned short	srp;

	if (sscanf(buf, "%hu", &srp) != 1
			|| (srp != 1)) {
		dev_err(dev, "SRP: Value must be 1\n");
		return -EINVAL;
	}

	if (srp == 1)
		musb_g_wakeup(musb);

	return n;
}
static DEVICE_ATTR_WO(srp);

static struct attribute *musb_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_vbus.attr,
	&dev_attr_srp.attr,
	NULL
};
ATTRIBUTE_GROUPS(musb);

#define MUSB_QUIRK_B_INVALID_VBUS_91	(MUSB_DEVCTL_BDEVICE | \
					 (2 << MUSB_DEVCTL_VBUS_SHIFT) | \
					 MUSB_DEVCTL_SESSION)
#define MUSB_QUIRK_B_DISCONNECT_99	(MUSB_DEVCTL_BDEVICE | \
					 (3 << MUSB_DEVCTL_VBUS_SHIFT) | \
					 MUSB_DEVCTL_SESSION)
#define MUSB_QUIRK_A_DISCONNECT_19	((3 << MUSB_DEVCTL_VBUS_SHIFT) | \
					 MUSB_DEVCTL_SESSION)

//Schedules musb_irq_work if not all attempts have been made and flush_irq is
//false (all irq handlers in the work queue have been executed).
static bool musb_state_needs_recheck(struct musb *musb, u8 devctl,
				     const char *desc)
{
//printk("drivers/usb/musb/musb_core.c:musb_state_needs_recheck\n");
	if (musb->quirk_retries && !musb->flush_irq_work) {
		trace_musb_state(musb, devctl, desc);
		//musb_irq_work is to be executed as part of a kernel thread rather than
		//in the interrupt handler to prevent slow response?
		schedule_delayed_work(&musb->irq_work,
				      msecs_to_jiffies(1000));
		musb->quirk_retries--;

		return true;
	}

	return false;
}

/*
 * Check the musb devctl session bit to determine if we want to
 * allow PM runtime for the device. In general, we want to keep things
 * active when the session bit is set except after host disconnect.
 *
 * Only called from musb_irq_work. If this ever needs to get called
 * elsewhere, proper locking must be implemented for musb->session.
 */
//Schedules the IRQ handler if retries remains and all pending IRQ handler
//executions have been executed, and updates power management counters.
static void musb_pm_runtime_check_session(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_pm_runtime_check_session\n");
	u8 devctl, s;
	int error;

	//Device control register:
	//"MUSBMHDRC is operating in Peripheral mode or in Host mode, and for
	// controlling and monitoring the USB VBus line"
	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	/* Handle session status quirks first */
	//FSDev: "This Read-only bit is set when a full-speed or high-speed device
	// has been detected being connected to the port. (High-speed devices are
	// distinguished from full-speed by checking for high-speed chirps when the
	// device is reset.) Only valid in Host mode."
	//LSDev: "This Read-only bit is set when a low-speed device has been detected being connected to the port. Only valid in Host mode."
	//Host Req: "When set, the MUSBMHDRC will initiate the Host Negotiation when
	// Suspend mode is entered. It is cleared when Host Negotiation is completed."
	//
	//High-speed device, low-speed device, MUSB initiates host negotiation when
	//suspend mode is entered.
	s = MUSB_DEVCTL_FSDEV | MUSB_DEVCTL_LSDEV |
		MUSB_DEVCTL_HR;
	switch (devctl & ~s) {
	case MUSB_QUIRK_B_DISCONNECT_99:
		//Schedules musb_irq_work if not all attempts have been made and
		//flush_irq is false (all irq handlers in the work queue have been
		//executed).
		musb_state_needs_recheck(musb, devctl,
			"Poll devctl in case of suspend after disconnect");
		break;
	case MUSB_QUIRK_B_INVALID_VBUS_91:
		//Schedules musb_irq_work if not all attempts have been made and
		//flush_irq is false (all irq handlers in the work queue have been
		//executed).
		if (musb_state_needs_recheck(musb, devctl,
				"Poll devctl on invalid vbus, assume no session"))
			return;
		fallthrough;
	case MUSB_QUIRK_A_DISCONNECT_19:
		//Schedules musb_irq_work if not all attempts have been made and
		//flush_irq is false (all irq handlers in the work queue have been
		//executed).
		if (musb_state_needs_recheck(musb, devctl,
				"Poll devctl on possible host mode disconnect"))
			return;
		if (!musb->session)
			break;
		trace_musb_state(musb, devctl, "Allow PM on possible host mode disconnect");
		pm_runtime_mark_last_busy(musb->controller);	//"Update the last access time of a device."
		pm_runtime_put_autosuspend(musb->controller);	//"Drop device usage counter and queue autosuspend if 0."(add to autosuspend q if 0)
		musb->session = false;							//No session.
		return;
	default:
		break;
	}

	/* No need to do anything if session has not changed */
	s = devctl & MUSB_DEVCTL_SESSION;
	if (s == musb->session)								//Session status unchanged.
		return;

	/* Block PM or allow PM? */
	if (s) {											//Session is ongoing.
		trace_musb_state(musb, devctl, "Block PM on active session");
		error = pm_runtime_get_sync(musb->controller);	//"Bump up usage counter of a device and resume it [(let it perform its operations)]."
		if (error < 0)
			dev_err(musb->controller, "Could not enable: %i\n",
				error);
		musb->quirk_retries = 3;

		/*
		 * We can get a spurious MUSB_INTR_SESSREQ interrupt on start-up
		 * in B-peripheral mode with nothing connected and the session
		 * bit clears silently. Check status again in 3 seconds.
		 */
		if (devctl & MUSB_DEVCTL_BDEVICE)
			//musb_irq_work is to be executed as part of a kernel thread rather
			//than in the interrupt handler to prevent slow response?
			schedule_delayed_work(&musb->irq_work,
					      msecs_to_jiffies(3000));
	} else {
		trace_musb_state(musb, devctl, "Allow PM with no session");
		pm_runtime_mark_last_busy(musb->controller);	//"Update the last access time of a device."
		pm_runtime_put_autosuspend(musb->controller);	//"Drop device usage counter and queue autosuspend if 0."(add to autosuspend q if 0)
	}

	musb->session = s;									//Record current session status.
}

/* Only used to provide driver mode change events */
//Update power management handlers and schedules IRQ handler.
static void musb_irq_work(struct work_struct *data)
{
//printk("drivers/usb/musb/musb_core.c:musb_irq_work\n");
	struct musb *musb = container_of(data, struct musb, irq_work.work);
	int error;

	error = pm_runtime_resume_and_get(musb->controller);	//Bump up usage counter of a device and resume it.
	if (error < 0) {
		dev_err(musb->controller, "Could not enable: %i\n", error);

		return;
	}

	//Schedules the IRQ handler if retries remains and all pending IRQ handler
	//executions have been executed, and updates power management counters.
	musb_pm_runtime_check_session(musb);

	if (musb->xceiv->otg->state != musb->xceiv_old_state) {	//PHY OTG state is different from previously recorded state.
		musb->xceiv_old_state = musb->xceiv->otg->state;
		sysfs_notify(&musb->controller->kobj, NULL, "mode");
	}

	pm_runtime_mark_last_busy(musb->controller);		//"Update the last access time of a device."
	pm_runtime_put_autosuspend(musb->controller);		//"Drop device usage counter and queue autosuspend if 0."(add to autosuspend q if 0)
}

//Recovers from data transfer exceeding size limit(?), enables interrupts,
//notifies the Linux kernel of an event, configures FIFOs, and sets OTG state.
static void musb_recover_from_babble(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_recover_from_babble\n");
	int ret;
	u8 devctl;

	//Disables and clears interrupt flags.
	musb_disable_interrupts(musb);

	/*
	 * wait at least 320 cycles of 60MHz clock. That's 5.3us, we will give
	 * it some slack and wait for 10us.
	 */
	udelay(10);

	ret  = musb_platform_recover(musb);				//Recovers from data transfer exceeding size limit?
	if (ret) {
		musb_enable_interrupts(musb);				//Enables TX/RX endpoints and common USB interrupts.
		return;
	}

	/* drop session bit */
	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
	devctl &= ~MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);	//Ends session.

	/* tell usbcore about it */
	musb_root_disconnect(musb);						//musb_virthub.c: Updates OTG state and allows the Linux kernel to respond to an event?

	/*
	 * When a babble condition occurs, the musb controller
	 * removes the session bit and the endpoint config is lost.
	 */
	if (musb->dyn_fifo)								//TRUE
		//Configures the FIFOs for the endpoints for the DMA packets independent
		//of the DMA.
		ret = ep_config_from_table(musb);
	else											//FALSE
		ret = ep_config_from_hw(musb);

	/* restart session */
	if (ret == 0)
		musb_start(musb);							//Enables interrupts and sets the OTG state.
}

/* --------------------------------------------------------------------------
 * Init support
 */
//The Linux device structure of the USB controller (0/1) and config structure
//of USB controller (0/1) platform device initialized by musb_dsps.c:dsps_probe.
static struct musb *allocate_instance(struct device *dev,
		const struct musb_hdrc_config *config, void __iomem *mbase)
{
//printk("drivers/usb/musb/musb_core.c:allocate_instance\n");
	struct musb		*musb;
	struct musb_hw_ep	*ep;		//Struct describing the hardware of an endpoint.
	int			epnum;
	int			ret;

	musb = devm_kzalloc(dev, sizeof(*musb), GFP_KERNEL);	//Create a data structure for the driver for the USB controller.
	if (!musb)
		return NULL;
	//Initializes linked list heads to empty list.
	INIT_LIST_HEAD(&musb->control);
	INIT_LIST_HEAD(&musb->in_bulk);
	INIT_LIST_HEAD(&musb->out_bulk);
	INIT_LIST_HEAD(&musb->pending_list);

	musb->vbuserr_retry = VBUSERR_RETRY_COUNT;	//VBUS carry +5V supply on a USB wire.
	musb->a_wait_bcon = OTG_TIME_A_WAIT_BCON;	//"VBUS timeout in msecs".
	musb->mregs = mbase;						//Base registers of USB controller (USB0/1 CORE in TRM). Undocumented.
	musb->ctrl_base = mbase;					//Base registers of USB controller (USB0/1 CORE in TRM). Undocumented.
	musb->nIrq = -ENODEV;
	musb->config = config;
	BUG_ON(musb->config->num_eps > MUSB_C_NUM_EPS);
	for (epnum = 0, ep = musb->endpoints;		//16 endpoints in an array of type musb_hw_ep is part of the musb structure.
			epnum < musb->config->num_eps;
			epnum++, ep++) {
		ep->musb = musb;						//Each ep gets a pointer to the USB controller data structure.
		ep->epnum = epnum;						//Each ep gets its ID number: [0, 15].
	}

	musb->controller = dev;						//Pointer to the Linux kernel device structure for this USB controller (0/1).

	ret = musb_host_alloc(musb);	//Calls drivers/usb/musb/musb_host.c:musb_host_alloc to create host controller driver.
	if (ret < 0)
		goto err_free;
	//The Linux device structure of the USB controller gets its driver_data
	//pointer set to the driver internal structure for the USB controller.
	dev_set_drvdata(dev, musb);

	return musb;

err_free:
	return NULL;
}

//Disables power management wakeup for IRQ of USB controller and frees the
//associated host controller driver.
static void musb_free(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_free\n");
	/* this has multiple entry modes. it handles fault cleanup after
	 * probe(), where things may be partially set up, as well as rmmod
	 * cleanup after everything's been de-activated.
	 */

	if (musb->nIrq >= 0) {
		if (musb->irq_wake)
			disable_irq_wake(musb->nIrq);	//"disable power management wakeup" for irq
		free_irq(musb->nIrq, musb);
	}

	musb_host_free(musb);
}

struct musb_pending_work {
	int (*callback)(struct musb *musb, void *data);
	void *data;
	struct list_head node;
};

#ifdef CONFIG_PM	//TRUE
/*
 * Called from musb_runtime_resume(), musb_resume(), and
 * musb_queue_resume_work(). Callers must take musb->lock.
 */
//Checks status of OTG protocol.
static int musb_run_resume_work(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_run_resume_work\n");
	struct musb_pending_work *w, *_w;
	unsigned long flags;
	int error = 0;

	spin_lock_irqsave(&musb->list_lock, flags);
	list_for_each_entry_safe(w, _w, &musb->pending_list, node) {
		if (w->callback) {
			error = w->callback(musb, w->data);				//Calls dsps_check_status: Checks status of OTG protocol.
			if (error < 0) {
				dev_err(musb->controller,
					"resume callback %p failed: %i\n",
					w->callback, error);
			}
		}
		list_del(&w->node);
		devm_kfree(musb->controller, w);
	}
	spin_unlock_irqrestore(&musb->list_lock, flags);

	return error;
}
#endif

/*
 * Called to run work if device is active or else queue the work to happen
 * on resume. Caller must take musb->lock and must hold an RPM reference.
 *
 * Note that we cowardly refuse queuing work after musb PM runtime
 * resume is done calling musb_run_resume_work() and return -EINPROGRESS
 * instead.
 */
//Adds callback to work queue if suspended to check the status of OTG protocol.
//If not suspended calls the callback immediately.
int musb_queue_resume_work(struct musb *musb,
			   int (*callback)(struct musb *musb, void *data),
			   void *data)
{
//printk("drivers/usb/musb/musb_core.c:musb_queue_resume_work\n");
	struct musb_pending_work *w;
	unsigned long flags;
	bool is_suspended;
	int error;

	if (WARN_ON(!callback))
		return -EINVAL;

	spin_lock_irqsave(&musb->list_lock, flags);
	is_suspended = musb->is_runtime_suspended;

	if (is_suspended) {
		w = devm_kzalloc(musb->controller, sizeof(*w), GFP_ATOMIC);
		if (!w) {
			error = -ENOMEM;
			goto out_unlock;
		}

		w->callback = callback;	//callback is dsps_check_status, when invoked by musb_dsps.c:
		w->data = data;

		list_add_tail(&w->node, &musb->pending_list);	//Adds callback function with argument to pending work list.
		error = 0;
	}

out_unlock:
	spin_unlock_irqrestore(&musb->list_lock, flags);

	if (!is_suspended)
		error = callback(musb, data);

	return error;
}
EXPORT_SYMBOL_GPL(musb_queue_resume_work);

//Stops reset and notifies kernel of resumption.
static void musb_deassert_reset(struct work_struct *work)
{
//printk("drivers/usb/musb/musb_core.c:musb_deassert_reset\n");
	struct musb *musb;
	unsigned long flags;

	musb = container_of(work, struct musb, deassert_reset_work.work);

	spin_lock_irqsave(&musb->lock, flags);

	if (musb->port1_status & USB_PORT_STAT_RESET)
		//Stops reset and notifies kernel of resumption.
		musb_port_reset(musb, false);

	spin_unlock_irqrestore(&musb->lock, flags);
}

/*
 * Perform generic per-controller initialization.
 *
 * @dev: the controller (already clocked, etc)
 * @nIrq: IRQ number
 * @ctrl: virtual address of controller registers,
 *	not yet corrected for platform-specific offsets
 */
//Creates data structures for the driver for the USB0/1 controller and Host Controller Driver.
//
//Initializes data structure fields.
//
//obtains data structures for the PHY, resets the USB controller 0/1 and
//performs some other initializations regarding enabling/disabling OTG,
//debugging and power management.
//
//Initializes power management and PHY.
//
//Initializes USB packet FIFOs for the DMA packet buffers of the MUSB0/1
//controller.
//
//Prepare timer without specific flags, which on expiration calls
//musb_otg_timer_func, which updates the OTG state during reset.
//
//Adds musb->isr (dsps_interrupt) to interrupt number 18/19 (see "interrupts"
//property of usb@1400 and usb@1800 in dts and musb_probe). IRQF_SHARED allows
//the interrupt to be shared among several devices.
//
//Sets the (M)USB0/1 controller mode to host.
static int
musb_init_controller(struct device *dev, int nIrq, void __iomem *ctrl)	//dev is usb controller 0/1 device structure.
{
//printk("drivers/usb/musb/musb_core.c:musb_init_controller\n");
	int			status;
	struct musb		*musb;
	//Gets musb_hdrc_platform_data which is initialized by musb_dsps.c:dsps_probe
	struct musb_hdrc_platform_data *plat = dev_get_platdata(dev);

	/* The driver might handle more features than the board; OK.
	 * Fail when the board needs a feature that's not enabled.
	 */
	if (!plat) {
		dev_err(dev, "no platform_data?\n");
		status = -ENODEV;
		goto fail0;
	}

	/* allocate */
	//Creates data structures for the driver for the USB0/1 controller and Host Controller Driver.
	//Sets musb->controller = dev.
	musb = allocate_instance(dev, plat->config, ctrl);
	if (!musb) {
		status = -ENOMEM;
		goto fail0;
	}

	spin_lock_init(&musb->lock);
	spin_lock_init(&musb->list_lock);
	musb->board_set_power = plat->set_power;	//Function for powering on or off the device.
	musb->min_power = plat->min_power;			//Maximum poweqr consumed.
	musb->ops = plat->platform_ops;				//Platform operations which are initialized by musb_dsps.c:dsps_probe to dsps_ops.
	musb->port_mode = plat->mode;				//Host, peripheral or OTG.

	/*
	 * Initialize the default IO functions. At least omap2430 needs
	 * these early. We initialize the platform specific IO functions
	 * later on.
	 */
	musb_readb = musb_default_readb;			//Function for reading one byte.
	musb_writeb = musb_default_writeb;			//Function for writing one byte.
	musb_readw = musb_default_readw;			//Function for reading two bytes, a half 32-bit word.
	musb_writew = musb_default_writew;			//Function for writing two bytes, a half 32-bit word.

	/* The musb_platform_init() call:
	 *   - adjusts musb->mregs
	 *   - sets the musb->isr
	 *   - may initialize an integrated transceiver
	 *   - initializes musb->xceiv, usually by otg_get_phy()
	 *   - stops powering VBUS
	 *
	 * There are various transceiver configurations.
	 * DaVinci, TUSB60x0, and others integrate them.  OMAP3 uses
	 * external/discrete ones in various flavors (twl4030 family,
	 * isp1504, non-OTG, etc) mostly hooking up through ULPI.
	 */
	//Invokes musb_dsps.c:dsps_musb_init which maps registers into the virtual
	//address space, obtains data structures for the PHY, resets the USB
	//controller 0/1 and performs some other initializations regarding
	//enabling/disabling OTG, debugging and power management.
	status = musb_platform_init(musb);
	if (status < 0)
		goto fail1;

	if (!musb->isr) {
		status = -ENODEV;
		goto fail2;
	}

	/* Most devices use indexed offset or flat offset */
	//Initializes musb->io field which is part of the musb struct.
	if (musb->ops->quirks & MUSB_INDEXED_EP) {	//TRUE: musb-ops is set by dsps_probe to dsps_ops, with sets quirks to MUSB_INDEXED_EP.
		musb->io.ep_offset = musb_indexed_ep_offset;	//Function that given endpoint number and offset, returns register address.
		musb->io.ep_select = musb_indexed_ep_select;	//Function for addressing a register bank.
	} else {
		musb->io.ep_offset = musb_flat_ep_offset;
		musb->io.ep_select = musb_flat_ep_select;
	}

	if (musb->ops->quirks & MUSB_G_NO_SKB_RESERVE)	//Not set by dsps_ops.
		musb->g.quirk_avoids_skb_reserve = 1;

	/* At least tusb6010 has its own offsets */
	if (musb->ops->ep_offset)	//Not set by dsps_ops.
		musb->io.ep_offset = musb->ops->ep_offset;
	if (musb->ops->ep_select)	//Not set by dsps_ops.
		musb->io.ep_select = musb->ops->ep_select;

	if (musb->ops->fifo_mode)	//Not set by dsps_ops.
		fifo_mode = musb->ops->fifo_mode;
	else
		fifo_mode = 4;			//Probably means hard-wired fifo sizing.

	if (musb->ops->fifo_offset)	//Not set by dsps_ops.
		musb->io.fifo_offset = musb->ops->fifo_offset;
	else
		musb->io.fifo_offset = musb_default_fifo_offset;	//Probably for queue manager.

	if (musb->ops->busctl_offset)	//Not set by dsps_ops.
		musb->io.busctl_offset = musb->ops->busctl_offset;
	else
		musb->io.busctl_offset = musb_default_busctl_offset;

	if (musb->ops->readb)	//Not set by dsps_ops.
		musb_readb = musb->ops->readb;
	if (musb->ops->writeb)	//Not set by dsps_ops.
		musb_writeb = musb->ops->writeb;
	if (musb->ops->clearb)	//Not set by dsps_ops.
		musb_clearb = musb->ops->clearb;
	else
		musb_clearb = musb_readb;			//musb_readb is initialized to musb_readb = musb_default_readb by musb_init_controller.

	if (musb->ops->readw)	//Not set by dsps_ops.
		musb_readw = musb->ops->readw;
	if (musb->ops->writew)	//Not set by dsps_ops.
		musb_writew = musb->ops->writew;
	if (musb->ops->clearw)	//Not set by dsps_ops.
		musb_clearw = musb->ops->clearw;
	else
		musb_clearw = musb_readw;			//musb_readw is initialized to musb_readw = musb_default_readw by musb_init_controller.

#ifndef CONFIG_MUSB_PIO_ONLY				//Not set.
	if (!musb->ops->dma_init || !musb->ops->dma_exit) {
		dev_err(dev, "DMA controller not set\n");
		status = -ENODEV;
		goto fail2;
	}
	musb_dma_controller_create = musb->ops->dma_init;	//Set to dsps_dma_controller_create by dsps_ops.
	musb_dma_controller_destroy = musb->ops->dma_exit;	//Set to cppi41_dma_controller_destroy by dsps_ops.
#endif

	if (musb->ops->read_fifo)	//Not set by dsps_ops.
		musb->io.read_fifo = musb->ops->read_fifo;
	else
		//Unload a FIFO of an endpoint. Invoked by musb_core.c:musb_read_fifo,
		//which in turn is invoked by musb_host.c:musb_host_packet_rx and
		//musb_host.c:musb_h_ep0_continue.
		musb->io.read_fifo = musb_default_read_fifo;

	if (musb->ops->write_fifo)	//Not set by dsps_ops.
		musb->io.write_fifo = musb->ops->write_fifo;
	else
		//Load a FIFO of an endpoint. Invoked by musb_core.c:musb_write_fifo,
		//which in turn is invoked by:
		//must_core.c:musb_load_testpacket
		//musb_host.c:musb_ep_program
		//musb_host.c:musb_host_tx
		//musb_host.c:musb_h_ep0_continue
		musb->io.write_fifo = musb_default_write_fifo;

	if (musb->ops->get_toggle)	//Not set by dsps_ops.
		musb->io.get_toggle = musb->ops->get_toggle;
	else
		musb->io.get_toggle = musb_default_get_toggle;	//Invoked by musb_host.c:musb_advance_schedule and musb_host.c:musb_bulk_nak_timeout.

	if (musb->ops->set_toggle)	//Not set by dsps_ops.
		musb->io.set_toggle = musb->ops->set_toggle;
	else
		musb->io.set_toggle = musb_default_set_toggle;	//Invoked by musb_host.c:musb_ep_program and musb_host.c:musb_ep_program.

	if (!musb->xceiv->io_ops) {					//True as is not initialized by devm_usb_get_phy_by_phandle in musb_dsps.c:dsps_musb_init.
		musb->xceiv->io_dev = musb->controller;		//The Linux device structure of the USB0/1 controller.
		musb->xceiv->io_priv = musb->mregs;			//Virtual memory addresses of USB0/1 CORE.
		//musb_ulpi_read/musb_ulpi_write. "The PHY (physical interface circuitry)
		//that presents USB interfaces also has to interface to the host computer.
		//This is done using a UTMI interface ULPI is a lower pin-count version
		//of that internal interface."
		musb->xceiv->io_ops = &musb_ulpi_access;
	}

	if (musb->ops->phy_callback)	//Not set by dsps_ops.
		musb_phy_callback = musb->ops->phy_callback;

	/*
	 * We need musb_read/write functions initialized for PM.
	 * Note that at least 2430 glue needs autosuspend delay
	 * somewhere above 300 ms for the hardware to idle properly
	 * after disconnecting the cable in host mode. Let's use
	 * 500 ms for some margin.
	 */
	pm_runtime_use_autosuspend(musb->controller);				//"Allow autosuspend to be used for a device."
	//"Set a device's autosuspend_delay value."
	//"USB autosuspend is a power-saving feature in recent Linux kernels that
	// powers off USB devices if the kernel thinks that those devices aren't
	// needed right now."
	pm_runtime_set_autosuspend_delay(musb->controller, 500);
	//Enable power management of the USB0/1 controller.
	//"Power management is a feature that turns off the power or switches
	// system's components to a low-power state when inactive."
	pm_runtime_enable(musb->controller);
	pm_runtime_get_sync(musb->controller);	//"Bump up usage counter of a device and resume it [(let it perform its operations)]."

	status = usb_phy_init(musb->xceiv);		//Calls drivers/usb/phy/phy-am335x.c:am335x_init, which always returns 0.
	if (status < 0)
		goto err_usb_phy_init;

	if (use_dma && dev->dma_mask) {			//TRUE!
//printk("drivers/usb/musb/musb_core.c:musb_init_controller CALLS dsps_dma_controller_create!!!!!!!!!!!!!!!!!!!!!!!!\n");
		//Allocates memory for the USB0/1 DMA controller, including DMA channel
		//structures, timers, and functions to call, and enables packet completion
		//interrupts.
		musb->dma_controller =
			musb_dma_controller_create(musb, musb->mregs);	//Set to dsps_dma_controller_create by dsps_ops and above.
//printk("drivers/usb/musb/musb_core.c:musb_init_controller RETURNS FROM dsps_dma_controller_create!!!!!!!!!!!!!!!!!!!!!!!!\\n");
		if (IS_ERR(musb->dma_controller)) {	//True first time before cppi41_dma_probe has allocated memory for DMA channel structures.
			status = PTR_ERR(musb->dma_controller);
			goto fail2_5;
		}
	}

	/* be sure interrupts are disabled before connecting ISR */
	//Calls dsps_musb_disable as set by dsps_ops, and removes the timer that is
	//used (among other purposes???) for initialization.
	musb_platform_disable(musb);
	musb_disable_interrupts(musb);
	musb_writeb(musb->mregs, MUSB_DEVCTL, 0);	//Writes USB0/1CORE undocumented register.

	/* MUSB_POWER_SOFTCONN might be already set, JZ4740 does this. */
	musb_writeb(musb->mregs, MUSB_POWER, 0);	//Writes USB0/1CORE undocumented register.

	/* Init IRQ workqueue before request_irq */
	//Initialize work (executed by musb_irq_work, musb_deassert, and
	//musb_host_finish_resume, respectively, to be executed at given earliest
	//delay.
	INIT_DELAYED_WORK(&musb->irq_work, musb_irq_work);
	INIT_DELAYED_WORK(&musb->deassert_reset_work, musb_deassert_reset);
	INIT_DELAYED_WORK(&musb->finish_resume_work, musb_host_finish_resume);

	/* setup musb parts of the core (especially endpoints) */
	//Initializes FIFOs for the DMA packet buffers of the MUSB0/1 controller.
	status = musb_core_init(plat->config->multipoint		//plat->config->multipoint is initialized to false in dsps_create_musb_pdev
			? MUSB_CONTROLLER_MHDRC							//Hence, the argument is MUSB_CONTROLLER_HDRC.
			: MUSB_CONTROLLER_HDRC, musb);
	if (status < 0)
		goto fail3;

	//Prepare timer without specific flags, which on expiration calls
	//musb_otg_timer_func, which updates the OTG state during reset.
	timer_setup(&musb->otg_timer, musb_otg_timer_func, 0);

	/* attach to the IRQ */
	//Adds musb->isr (dsps_interrupt) to interrupt number 18/19 (see
	//"interrupts" property of usb@1400 and usb@1800 in dts and musb_probe).
	//IRQF_SHARED allows the interrupt to be shared among several devices.
	if (request_irq(nIrq, musb->isr, IRQF_SHARED, dev_name(dev), musb)) {
		dev_err(dev, "request_irq %d failed!\n", nIrq);
		status = -ENODEV;
		goto fail3;
	}
	musb->nIrq = nIrq;	//Store irq number for this MUSB0/1 (18/19).
	/* FIXME this handles wakeup irqs wrong */
	if (enable_irq_wake(nIrq) == 0) {						//enable_irq_wake enables this IRQ to wake up the system from sleep.
		musb->irq_wake = 1;
		device_init_wakeup(dev, 1);							//Device wakeup initialization: Enables the device two wake up the system.
	} else {
		musb->irq_wake = 0;
	}

	/* program PHY to use external vBus if required */
	if (plat->extvbus) {									//"program PHY for external Vbus"
		u8 busctl = musb_readb(musb->mregs, MUSB_ULPI_BUSCONTROL);
		busctl |= MUSB_ULPI_USE_EXTVBUS;
		musb_writeb(musb->mregs, MUSB_ULPI_BUSCONTROL, busctl);
	}

	MUSB_DEV_MODE(musb);									//USB0/1 controller set as device/peripheral.
	musb->xceiv->otg->state = OTG_STATE_B_IDLE;				//Sets state of PHY?

	switch (musb->port_mode) {								//USB0/1 role depends on connector used on board: Host, Peripheral, OTG, und
	case MUSB_HOST:											//(M)USB0/1 controller role is host.
		//plat-power = power budget = "(HOST or OTG) mA/2 power supplied on (default = 8mA)"
		//Finishes initialization of the host controller driver and power wake up source.
		status = musb_host_setup(musb, plat->power);
		if (status < 0)
			goto fail3;
		//Calls musb_dsps.c:dsps_musb_set_mode which sets the (M)USB0/1 mode to host.
		status = musb_platform_set_mode(musb, MUSB_HOST);
		break;
	case MUSB_PERIPHERAL:
		status = musb_gadget_setup(musb);	//Empty function since CONFIG_USB_MUSB_GADGET and CONFIG_USB_MUSB_DUAL_ROLE are undefined.
		if (status < 0)										//TRUE!!!!!!!!!!
			goto fail3;
		//Calls musb_dsps.c:dsps_musb_set_mode which sets the (M)USB0/1 mode to peripheral.
		status = musb_platform_set_mode(musb, MUSB_PERIPHERAL);
		break;
	case MUSB_OTG:											//FALSE!
		status = musb_host_setup(musb, plat->power);
		if (status < 0)
			goto fail3;
		status = musb_gadget_setup(musb);
		if (status) {
			musb_host_cleanup(musb);
			goto fail3;
		}
		status = musb_platform_set_mode(musb, MUSB_OTG);
		break;
	default:
		dev_err(dev, "unsupported port mode %d\n", musb->port_mode);
		break;
	}

	if (status < 0)
		goto fail3;

	musb_init_debugfs(musb);

	musb->is_initialized = 1;								//MUSB0/1 is initialized.
	pm_runtime_mark_last_busy(musb->controller);			//"Update the last access time of a device."
	pm_runtime_put_autosuspend(musb->controller);	//"Drop device usage counter and queue autosuspend if 0."(add to autosuspend q if 0)

	return 0;
//glöm inte kommentera vad funktionen gör.
fail3:
	cancel_delayed_work_sync(&musb->irq_work);
	cancel_delayed_work_sync(&musb->finish_resume_work);
	cancel_delayed_work_sync(&musb->deassert_reset_work);
	if (musb->dma_controller)
		musb_dma_controller_destroy(musb->dma_controller);

fail2_5:
	usb_phy_shutdown(musb->xceiv);

err_usb_phy_init:
	pm_runtime_dont_use_autosuspend(musb->controller);
	pm_runtime_put_sync(musb->controller);
	pm_runtime_disable(musb->controller);

fail2:
	if (musb->irq_wake)
		device_init_wakeup(dev, 0);
	musb_platform_exit(musb);

fail1:
	if (status != -EPROBE_DEFER)
		dev_err(musb->controller,
			"%s failed with status %d\n", __func__, status);

	musb_free(musb);

fail0:

	return status;

}

/*-------------------------------------------------------------------------*/

/* all implementations (PCI bridge to FPGA, VLYNQ, etc) should just
 * bridge to a platform device; this driver then suffices.
 */
//Invoked when a USB controller platform device has been added to the Linux
//kernel device hierarchy, which occurs by platform_device_add in
//musb_dsps.c:dsps_create_musb_pdev.
static int musb_probe(struct platform_device *pdev)
{
//printk("drivers/usb/musb/musb_core.c:musb_probe\n");
	struct device	*dev = &pdev->dev;
	//Gets interrupts property of usb controller, "mc" is value of
	//"interrupt-names" in device tree, and the index of the list of interrupts
	//in the device tree (one interrupt per usb controller). Device tree says
	//interrupts at 0x12 (18) and 0x13 (19), which agrees with ARM Cortex-A8
	//Interrupts in section 6.3 in the technical reference manual.
	int		irq = platform_get_irq_byname(pdev, "mc");
	void __iomem	*base;

	if (irq <= 0)
		return -ENODEV;
	//Virtual memory maps resource with index 0 (memory at 0, irq at 1) of the USB controller.
	base = devm_platform_ioremap_resource(pdev, 0);	//USB0 CORE and USB1 CORE in TRM at 0x47401400 and 0x47401C00.
	if (IS_ERR(base))
		return PTR_ERR(base);

	int ret = musb_init_controller(dev, irq, base);
//printk("drivers/usb/musb/musb_core.c:musb_probe RETURNS FROM musb_init_controller\n");
	return ret;
}

static int musb_remove(struct platform_device *pdev)
{
//printk("drivers/usb/musb/musb_core.c:musb_remove\n");
	struct device	*dev = &pdev->dev;
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;

	/* this gets called on rmmod.
	 *  - Host mode: host may still be active
	 *  - Peripheral mode: peripheral is deactivated (or never-activated)
	 *  - OTG mode: both roles are deactivated (or never-activated)
	 */
	musb_exit_debugfs(musb);

	cancel_delayed_work_sync(&musb->irq_work);
	cancel_delayed_work_sync(&musb->finish_resume_work);
	cancel_delayed_work_sync(&musb->deassert_reset_work);
	pm_runtime_get_sync(musb->controller);
	musb_host_cleanup(musb);
	musb_gadget_cleanup(musb);

	musb_platform_disable(musb);
	spin_lock_irqsave(&musb->lock, flags);
	musb_disable_interrupts(musb);
	musb_writeb(musb->mregs, MUSB_DEVCTL, 0);
	spin_unlock_irqrestore(&musb->lock, flags);
	musb_platform_exit(musb);

	pm_runtime_dont_use_autosuspend(musb->controller);
	pm_runtime_put_sync(musb->controller);
	pm_runtime_disable(musb->controller);
	musb_phy_callback = NULL;
	if (musb->dma_controller)
		musb_dma_controller_destroy(musb->dma_controller);
	usb_phy_shutdown(musb->xceiv);
	musb_free(musb);
	device_init_wakeup(dev, 0);
	return 0;
}

#ifdef	CONFIG_PM
//Stores MUSB0/1 registers related to testmode, power, interrupts, maximum
//packet size, control status registers, FIFO addresses, and FIFO sizes.
static void musb_save_context(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_save_context\n");
	int i;
	void __iomem *musb_base = musb->mregs;
	void __iomem *epio;

	musb->context.frame = musb_readw(musb_base, MUSB_FRAME);
	musb->context.testmode = musb_readb(musb_base, MUSB_TESTMODE);
	musb->context.busctl = musb_readb(musb_base, MUSB_ULPI_BUSCONTROL);
	musb->context.power = musb_readb(musb_base, MUSB_POWER);
	musb->context.intrusbe = musb_readb(musb_base, MUSB_INTRUSBE);
	musb->context.index = musb_readb(musb_base, MUSB_INDEX);
	musb->context.devctl = musb_readb(musb_base, MUSB_DEVCTL);

	for (i = 0; i < musb->config->num_eps; ++i) {
		struct musb_hw_ep	*hw_ep;

		hw_ep = &musb->endpoints[i];
		if (!hw_ep)
			continue;

		epio = hw_ep->regs;
		if (!epio)
			continue;

		musb_writeb(musb_base, MUSB_INDEX, i);
		musb->context.index_regs[i].txmaxp =
			musb_readw(epio, MUSB_TXMAXP);
		musb->context.index_regs[i].txcsr =
			musb_readw(epio, MUSB_TXCSR);
		musb->context.index_regs[i].rxmaxp =
			musb_readw(epio, MUSB_RXMAXP);
		musb->context.index_regs[i].rxcsr =
			musb_readw(epio, MUSB_RXCSR);

		if (musb->dyn_fifo) {
			musb->context.index_regs[i].txfifoadd =
					musb_readw(musb_base, MUSB_TXFIFOADD);
			musb->context.index_regs[i].rxfifoadd =
					musb_readw(musb_base, MUSB_RXFIFOADD);
			musb->context.index_regs[i].txfifosz =
					musb_readb(musb_base, MUSB_TXFIFOSZ);
			musb->context.index_regs[i].rxfifosz =
					musb_readb(musb_base, MUSB_RXFIFOSZ);
		}

		musb->context.index_regs[i].txtype =
			musb_readb(epio, MUSB_TXTYPE);
		musb->context.index_regs[i].txinterval =
			musb_readb(epio, MUSB_TXINTERVAL);
		musb->context.index_regs[i].rxtype =
			musb_readb(epio, MUSB_RXTYPE);
		musb->context.index_regs[i].rxinterval =
			musb_readb(epio, MUSB_RXINTERVAL);

		musb->context.index_regs[i].txfunaddr =
			musb_read_txfunaddr(musb, i);
		musb->context.index_regs[i].txhubaddr =
			musb_read_txhubaddr(musb, i);
		musb->context.index_regs[i].txhubport =
			musb_read_txhubport(musb, i);

		musb->context.index_regs[i].rxfunaddr =
			musb_read_rxfunaddr(musb, i);
		musb->context.index_regs[i].rxhubaddr =
			musb_read_rxhubaddr(musb, i);
		musb->context.index_regs[i].rxhubport =
			musb_read_rxhubport(musb, i);
	}
}

//Restores MUSB0/1 registers related to interrupt, maximum packet size, control
//registers, FIFO size and start address, transaction protocol, and polling
//interval.
static void musb_restore_context(struct musb *musb)
{
//printk("drivers/usb/musb/musb_core.c:musb_restore_context\n");
	int i;
	void __iomem *musb_base = musb->mregs;
	void __iomem *epio;
	u8 power;

	musb_writew(musb_base, MUSB_FRAME, musb->context.frame);					//Received frame number.
	//"8-bit register that is primarily used to put the MUSBMHDRC into one of the four test modes"
	musb_writeb(musb_base, MUSB_TESTMODE, musb->context.testmode);
	musb_writeb(musb_base, MUSB_ULPI_BUSCONTROL, musb->context.busctl);			//Undocumented.

	/* Don't affect SUSPENDM/RESUME bits in POWER reg */
	power = musb_readb(musb_base, MUSB_POWER);						//"8-bit register that is used for controlling Suspend and Resume"
	power &= MUSB_POWER_SUSPENDM | MUSB_POWER_RESUME;				//Preserve suspend and resume
	musb->context.power &= ~(MUSB_POWER_SUSPENDM | MUSB_POWER_RESUME);	//Get other old flags?
	power |= musb->context.power;									//Preserved suspend and resume, and other flags.
	musb_writeb(musb_base, MUSB_POWER, power);

	musb_writew(musb_base, MUSB_INTRTXE, musb->intrtxe);			//"interrupt enable bits for the interrupts in IntrTx"
	musb_writew(musb_base, MUSB_INTRRXE, musb->intrrxe);			//"interrupt enable bits for the interrupts in IntrRx"
	musb_writeb(musb_base, MUSB_INTRUSBE, musb->context.intrusbe);	//"indicates which USB interrupts are currently active"
	if (musb->context.devctl & MUSB_DEVCTL_SESSION)
		//"register that is used to select whether the MUSBMHDRC is operating in
		// Peripheral mode or in Host mode, and for controlling and monitoring
		// the USB VBus line"
		musb_writeb(musb_base, MUSB_DEVCTL, musb->context.devctl);	//OTG device control register.

	for (i = 0; i < musb->config->num_eps; ++i) {
		struct musb_hw_ep	*hw_ep;

		hw_ep = &musb->endpoints[i];
		if (!hw_ep)
			continue;

		epio = hw_ep->regs;
		if (!epio)
			continue;

		musb_writeb(musb_base, MUSB_INDEX, i);
		//"register defines the maximum amount of data that can be transferred through the selected TX endpoint in a single operation"
		musb_writew(epio, MUSB_TXMAXP,
			musb->context.index_regs[i].txmaxp);
		//"register that provides control and status bits for transfers through the currently-selected TX endpoint"
		musb_writew(epio, MUSB_TXCSR,
			musb->context.index_regs[i].txcsr);
		//"register defines the maximum amount of data that can be transferred through the selected Rx endpoint in a single operation"
		musb_writew(epio, MUSB_RXMAXP,
			musb->context.index_regs[i].rxmaxp);
		//"register that provides control and status bits for transfers through the currently-selected Rx endpoint"
		musb_writew(epio, MUSB_RXCSR,
			musb->context.index_regs[i].rxcsr);

		if (musb->dyn_fifo) {							//TRUE
			musb_writeb(musb_base, MUSB_TXFIFOSZ,		//"register which controls the size of the selected TX endpoint FIFO"
				musb->context.index_regs[i].txfifosz);
			musb_writeb(musb_base, MUSB_RXFIFOSZ,		//"register which controls the size of the selected Rx endpoint FIFO"
				musb->context.index_regs[i].rxfifosz);
			musb_writew(musb_base, MUSB_TXFIFOADD,		//"register which controls the start address of the selected Tx endpoint FIFO"
				musb->context.index_regs[i].txfifoadd);
			musb_writew(musb_base, MUSB_RXFIFOADD,		//"register which controls the start address of the selected Rx endpoint FIFO"
				musb->context.index_regs[i].rxfifoadd);
		}

		//"register that should be written with the endpoint number to be
		// targeted by the endpoint, the transaction protocol to use for the
		// currently-selected TX endpoint, and its operating speed"
		musb_writeb(epio, MUSB_TXTYPE,
				musb->context.index_regs[i].txtype);
		//"TxInterval is an 8-bit register that, for Interrupt and Isochronous
		// transfers, defines the polling interval for the currently-selected
		// TX endpoint. For Bulk endpoints, this register sets the number of
		// frames/microframes after which the endpoint should timeout on
		// receiving a stream of NAK responses."
		musb_writeb(epio, MUSB_TXINTERVAL,
				musb->context.index_regs[i].txinterval);
		//"register that should be written with the endpoint number to be
		// targeted by the endpoint, the transaction protocol to use for the
		// currently-selected Rx endpoint, and its operating speed."
		musb_writeb(epio, MUSB_RXTYPE,
				musb->context.index_regs[i].rxtype);
		//"RxInterval is an 8-bit register that, for Interrupt and Isochronous
		// transfers, defines the polling interval for the currently-selected Rx
		// endpoint. For Bulk endpoints, this register sets the number of
		// frames/microframes after which the endpoint should timeout on
		// receiving a stream of NAK responses."
		musb_writeb(epio, MUSB_RXINTERVAL,
				musb->context.index_regs[i].rxinterval);
		//"TxFuncAddr and RxFuncAddr are 7-bit read/write registers that record
		// the address of the target function that is to be accessed through the
		// associated endpoint (EPn)."
		//Target function seems to be related to synthesizing the USB controller
		//related to the register memory map addresses?
		musb_write_txfunaddr(musb, i,
				musb->context.index_regs[i].txfunaddr);
		//"transaction translation to convert between high-speed transmission and full-/low-speed transmission"
		musb_write_txhubaddr(musb, i,
				musb->context.index_regs[i].txhubaddr);
		//"TxHubPort and RxHubPort only need to be written where a full- or
		// low-speed device is connected to TX/Rx Endpoint EPn via a high-speed
		// USB 2.0 hub which carries out the necessary transaction translation."
		musb_write_txhubport(musb, i,
				musb->context.index_regs[i].txhubport);

		musb_write_rxfunaddr(musb, i,
				musb->context.index_regs[i].rxfunaddr);
		musb_write_rxhubaddr(musb, i,
				musb->context.index_regs[i].rxhubaddr);
		musb_write_rxhubport(musb, i,
				musb->context.index_regs[i].rxhubport);
	}
	musb_writeb(musb_base, MUSB_INDEX, musb->context.index);
}

static int musb_suspend(struct device *dev)
{
//printk("drivers/usb/musb/musb_core.c:musb_suspend\n");
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;
	int ret;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		return ret;
	}

	musb_platform_disable(musb);
	musb_disable_interrupts(musb);

	musb->flush_irq_work = true;
	while (flush_delayed_work(&musb->irq_work))
		;
	musb->flush_irq_work = false;

	if (!(musb->ops->quirks & MUSB_PRESERVE_SESSION))
		musb_writeb(musb->mregs, MUSB_DEVCTL, 0);

	WARN_ON(!list_empty(&musb->pending_list));

	spin_lock_irqsave(&musb->lock, flags);

	if (is_peripheral_active(musb)) {
		/* FIXME force disconnect unless we know USB will wake
		 * the system up quickly enough to respond ...
		 */
	} else if (is_host_active(musb)) {
		/* we know all the children are suspended; sometimes
		 * they will even be wakeup-enabled.
		 */
	}

	//Stores MUSB0/1 registers related to testmode, power, interrupts, maximum
	//packet size, control status registers, FIFO addresses, and FIFO sizes.
	musb_save_context(musb);

	spin_unlock_irqrestore(&musb->lock, flags);
	return 0;
}

static int musb_resume(struct device *dev)
{
//printk("drivers/usb/musb/musb_core.c:musb_resume\n");
	struct musb *musb = dev_to_musb(dev);
	unsigned long flags;
	int error;
	u8 devctl;
	u8 mask;

	/*
	 * For static cmos like DaVinci, register values were preserved
	 * unless for some reason the whole soc powered down or the USB
	 * module got reset through the PSC (vs just being disabled).
	 *
	 * For the DSPS glue layer though, a full register restore has to
	 * be done. As it shouldn't harm other platforms, we do it
	 * unconditionally.
	 */

	//Restores MUSB0/1 registers related to interrupt, maximum packet size,
	//control registers, FIFO size and start address, transaction protocol, and
	//polling interval.
	musb_restore_context(musb);

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
	mask = MUSB_DEVCTL_BDEVICE | MUSB_DEVCTL_FSDEV | MUSB_DEVCTL_LSDEV;
	if ((devctl & mask) != (musb->context.devctl & mask))
		musb->port1_status = 0;

	musb_enable_interrupts(musb);
	musb_platform_enable(musb);

	/* session might be disabled in suspend */
	if (musb->port_mode == MUSB_HOST &&
	    !(musb->ops->quirks & MUSB_PRESERVE_SESSION)) {
		devctl |= MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
	}

	spin_lock_irqsave(&musb->lock, flags);
	error = musb_run_resume_work(musb);
	if (error)
		dev_err(musb->controller, "resume work failed with %i\n",
			error);
	spin_unlock_irqrestore(&musb->lock, flags);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;
}

//Stores MUSB0/1 registers related to testmode, power, interrupts, maximum
//packet size, control status registers, FIFO addresses, and FIFO sizes, and
//sets flag that MUSB0/1 controller is suspended.
static int musb_runtime_suspend(struct device *dev)
{
//printk("drivers/usb/musb/musb_core.c:musb_runtime_suspend\n");
	struct musb	*musb = dev_to_musb(dev);
	//Stores MUSB0/1 registers related to testmode, power, interrupts, maximum
	//packet size, control status registers, FIFO addresses, and FIFO sizes.
	musb_save_context(musb);
	musb->is_runtime_suspended = 1;

	return 0;
}

//Restores MUSB0/1 registers and checks OTG protocol status.
static int musb_runtime_resume(struct device *dev)
{
//printk("drivers/usb/musb/musb_core.c:musb_runtime_resume\n");
	struct musb *musb = dev_to_musb(dev);
	unsigned long flags;
	int error;

	/*
	 * When pm_runtime_get_sync called for the first time in driver
	 * init,  some of the structure is still not initialized which is
	 * used in restore function. But clock needs to be
	 * enabled before any register access, so
	 * pm_runtime_get_sync has to be called.
	 * Also context restore without save does not make
	 * any sense
	 */
	if (!musb->is_initialized)
		return 0;

	//Restores MUSB0/1 registers related to interrupt, maximum packet size,
	//control registers, FIFO size and start address, transaction protocol, and
	//polling interval.
	musb_restore_context(musb);

	spin_lock_irqsave(&musb->lock, flags);

	error = musb_run_resume_work(musb);	//Checks status of OTG protocol.
	if (error)
		dev_err(musb->controller, "resume work failed with %i\n",
			error);
	musb->is_runtime_suspended = 0;
	spin_unlock_irqrestore(&musb->lock, flags);

	return 0;
}

static const struct dev_pm_ops musb_dev_pm_ops = {
	.suspend	= musb_suspend,
	.resume		= musb_resume,
	.runtime_suspend = musb_runtime_suspend,
	.runtime_resume = musb_runtime_resume,
};

#define MUSB_DEV_PM_OPS (&musb_dev_pm_ops)
#else
#define	MUSB_DEV_PM_OPS	NULL
#endif

static struct platform_driver musb_driver = {
	.driver = {
		.name		= musb_driver_name,
		.bus		= &platform_bus_type,
		.pm		= MUSB_DEV_PM_OPS,
		.dev_groups	= musb_groups,
	},
	.probe		= musb_probe,
	.remove		= musb_remove,
};

module_platform_driver(musb_driver);
