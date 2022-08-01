// SPDX-License-Identifier: GPL-2.0
/*
 * MUSB OTG driver host support
 *
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006-2007 Nokia Corporation
 * Copyright (C) 2008-2009 MontaVista Software, Inc. <source@mvista.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>

#include "musb_core.h"
#include "musb_host.h"
#include "musb_trace.h"

/* MUSB HOST status 22-mar-2006
 *
 * - There's still lots of partial code duplication for fault paths, so
 *   they aren't handled as consistently as they need to be.
 *
 * - PIO mostly behaved when last tested.
 *     + including ep0, with all usbtest cases 9, 10
 *     + usbtest 14 (ep0out) doesn't seem to run at all
 *     + double buffered OUT/TX endpoints saw stalls(!) with certain usbtest
 *       configurations, but otherwise double buffering passes basic tests.
 *     + for 2.6.N, for N > ~10, needs API changes for hcd framework.
 *
 * - DMA (CPPI) ... partially behaves, not currently recommended
 *     + about 1/15 the speed of typical EHCI implementations (PCI)
 *     + RX, all too often reqpkt seems to misbehave after tx
 *     + TX, no known issues (other than evident silicon issue)
 *
 * - DMA (Mentor/OMAP) ...has at least toggle update problems
 *
 * - [23-feb-2009] minimal traffic scheduling to avoid bulk RX packet
 *   starvation ... nothing yet for TX, interrupt, or bulk.
 *
 * - Not tested with HNP, but some SRP paths seem to behave.
 *
 * NOTE 24-August-2006:
 *
 * - Bulk traffic finally uses both sides of hardware ep1, freeing up an
 *   extra endpoint for periodic use enabling hub + keybd + mouse.  That
 *   mostly works, except that with "usbnet" it's easy to trigger cases
 *   with "ping" where RX loses.  (a) ping to davinci, even "ping -f",
 *   fine; but (b) ping _from_ davinci, even "ping -c 1", ICMP RX loses
 *   although ARP RX wins.  (That test was done with a full speed link.)
 */


/*
 * NOTE on endpoint usage:
 *
 * CONTROL transfers all go through ep0.  BULK ones go through dedicated IN
 * and OUT endpoints ... hardware is dedicated for those "async" queue(s).
 * (Yes, bulk _could_ use more of the endpoints than that, and would even
 * benefit from it.)
 *
 * INTERUPPT and ISOCHRONOUS transfers are scheduled to the other endpoints.
 * So far that scheduling is both dumb and optimistic:  the endpoint will be
 * "claimed" until its software queue is no longer refilled.  No multiplexing
 * of transfers between endpoints, or anything clever.
 */
//Gets the MUSB data structure associated with the USB host controller driver.
struct musb *hcd_to_musb(struct usb_hcd *hcd)
{
//printk("drivers/usb/musb/musb_host.c:hcd_to_musb\n");
	return *(struct musb **) hcd->hcd_priv;
}


static void musb_ep_program(struct musb *musb, u8 epnum,
			struct urb *urb, int is_out,
			u8 *buf, u32 offset, u32 len);

/*
 * Clear TX fifo. Needed to avoid BABBLE errors.
 */
//Transmits all packets in TX FIFO.
static void musb_h_tx_flush_fifo(struct musb_hw_ep *ep)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_tx_flush_fifo\n");
	struct musb	*musb = ep->musb;
	void __iomem	*epio = ep->regs;
	u16		csr;
	int		retries = 1000;

	csr = musb_readw(epio, MUSB_TXCSR);								//TX control status register.
	while (csr & MUSB_TXCSR_FIFONOTEMPTY) {							//At least one packet in TX FIFO.
		csr |= MUSB_TXCSR_FLUSHFIFO | MUSB_TXCSR_TXPKTRDY;			//Flush latest packet from TX FIFO and signal to MUSB of packet in FIFO.
		musb_writew(epio, MUSB_TXCSR, csr);
		csr = musb_readw(epio, MUSB_TXCSR);

		/*
		 * FIXME: sometimes the tx fifo flush failed, it has been
		 * observed during device disconnect on AM335x.
		 *
		 * To reproduce the issue, ensure tx urb(s) are queued when
		 * unplug the usb device which is connected to AM335x usb
		 * host port.
		 *
		 * I found using a usb-ethernet device and running iperf
		 * (client on AM335x) has very high chance to trigger it.
		 *
		 * Better to turn on musb_dbg() in musb_cleanup_urb() with
		 * CPPI enabled to see the issue when aborting the tx channel.
		 */
		if (dev_WARN_ONCE(musb->controller, retries-- < 1,
				"Could not flush host TX%d fifo: csr: %04x\n",
				ep->epnum, csr))
			return;
		mdelay(1);
	}
}

//Flushes all packets in the FIFO and resets the FIFO pointer and the control
//register of endpoint zero.
static void musb_h_ep0_flush_fifo(struct musb_hw_ep *ep)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_ep0_flush_fifo\n");
	void __iomem	*epio = ep->regs;
	u16		csr;
	int		retries = 5;

	/* scrub any data left in the fifo */
	do {
		csr = musb_readw(epio, MUSB_TXCSR);								//TX control status register.
		if (!(csr & (MUSB_CSR0_TXPKTRDY | MUSB_CSR0_RXPKTRDY)))			//If no packets are ready, stop iteration.
			break;
		//"The CPU writes a 1 to this bit to flush the next packet to be
		// transmitted/read from the Endpoint 0 FIFO. The FIFO pointer is reset
		// and the TxPktRdy/RxPktRdy bit (below) is cleared."
		musb_writew(epio, MUSB_TXCSR, MUSB_CSR0_FLUSHFIFO);
		csr = musb_readw(epio, MUSB_TXCSR);
		udelay(10);
	} while (--retries);

	WARN(!retries, "Could not flush host TX%d fifo: csr: %04x\n",
			ep->epnum, csr);

	/* and reset for the next transfer */
	musb_writew(epio, MUSB_TXCSR, 0);	//Clear status register, which resets the values.
}

/*
 * Start transmit. Caller is responsible for locking shared resources.
 * musb must be locked.
 */
//Configures endpoint to transmit packet.
static inline void musb_h_tx_start(struct musb_hw_ep *ep)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_tx_start\n");
	u16	txcsr;

	/* NOTE: no locks here; caller should lock and select EP */
	if (ep->epnum) {											//Not endpoint 0.
		txcsr = musb_readw(ep->regs, MUSB_TXCSR);				//Control status register.
		txcsr |= MUSB_TXCSR_TXPKTRDY | MUSB_TXCSR_H_WZC_BITS;	//Sets TXPktRdy, NAKTimeout, RXStall, Error and FIFONotEmpty.
		musb_writew(ep->regs, MUSB_TXCSR, txcsr);
	} else {													//Endpoint 0.
		//"The CPU sets this bit, at the same time as the TxPktRdy bit is set,
		// to send a SETUP token instead of an OUT token for the transaction."
		//"The CPU sets this bit after loading a data packet into the FIFO. It
		// is cleared automatically when a data packet has been transmitted"
		txcsr = MUSB_CSR0_H_SETUPPKT | MUSB_CSR0_TXPKTRDY;
		musb_writew(ep->regs, MUSB_CSR0, txcsr);
	}

}
//Configures endpoint for DMA (enables DMA and sets DMA mode).
static inline void musb_h_tx_dma_start(struct musb_hw_ep *ep)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_tx_dma_start\n");
	u16	txcsr;

	/* NOTE: no locks here; caller should lock and select EP */
	txcsr = musb_readw(ep->regs, MUSB_TXCSR);
	txcsr |= MUSB_TXCSR_DMAENAB | MUSB_TXCSR_H_WZC_BITS;
	if (is_cppi_enabled(ep->musb))	//TRUE
		txcsr |= MUSB_TXCSR_DMAMODE;
	musb_writew(ep->regs, MUSB_TXCSR, txcsr);
}

//Endpoint structure gets pointer to the given qh, depending on qh direction.
static void musb_ep_set_qh(struct musb_hw_ep *ep, int is_in, struct musb_qh *qh)
{
//printk("drivers/usb/musb/musb_host.c:musb_ep_set_qh\n");
	if (is_in != 0 || ep->is_shared_fifo)
		ep->in_qh  = qh;
	if (is_in == 0 || ep->is_shared_fifo)
		ep->out_qh = qh;
}

//Gets the qh of the given endpoint, depending on direction.
static struct musb_qh *musb_ep_get_qh(struct musb_hw_ep *ep, int is_in)
{
//printk("drivers/usb/musb/musb_host.c:musb_ep_get_qh\n");
	return is_in ? ep->in_qh : ep->out_qh;
}

/*
 * Start the URB at the front of an endpoint's queue
 * end must be claimed from the caller.
 *
 * Context: controller locked, irqs blocked
 */
//Configures endpoint associated with qh for transfer the URB associated with
//qh.
static void
musb_start_urb(struct musb *musb, int is_in, struct musb_qh *qh)
{
//printk("drivers/usb/musb/musb_host.c:musb_start_urb\n");
	u32			len;
	void __iomem		*mbase =  musb->mregs;			//Undocumented MUSB registers.
	//qh->"host-side endpoint descriptor and queue"->urb_list = "urbs queued to this endpoint; maintained by usbcore"
	//Gets the next URB for the endpoint queue (qh comes from a struct musb_hw_ep, which represents a hardware endpoint).
	struct urb		*urb = next_urb(qh);
	void			*buf = urb->transfer_buffer;		//Pointer to transfer buffer.
	u32			offset = 0;
	struct musb_hw_ep	*hw_ep = qh->hw_ep;				//Endpoint associated with this QH (URB queue head).
	int			epnum = hw_ep->epnum;					//Endpoint number of associated endpoint.

	/* initialize software qh state */
	qh->offset = 0;						//"in urb->transfer_buffer", which is storage memory data buffer pointer?
	qh->segsize = 0;					//"current xfer fragment"

	/* gather right source of data */
	switch (qh->type) {
	case USB_ENDPOINT_XFER_CONTROL:							//Control transfer.
		/* control transfers always start with SETUP */
		is_in = 0;											//Control is outbound?
		musb->ep0_stage = MUSB_EP0_START;					//"expect ack of setup"
		//"Only used for control transfers, this points to eight bytes of setup
		// data. Control transfers always start by sending this data to the
		// device. Then transfer_buffer is read or written, if needed."
		buf = urb->setup_packet;
		len = 8;
		break;
	case USB_ENDPOINT_XFER_ISOC:							//Isochronous transfer: Reserved real-time transfers.
		qh->iso_idx = 0;						//"in urb->iso_frame_desc[]", array of USB packet descriptors for isochronous transfers.
		qh->frame = 0;										//"for periodic schedule", meaning for isochornous transfer?
		offset = urb->iso_frame_desc[0].offset;
		len = urb->iso_frame_desc[0].length;				//"expected length"
		break;
	default:		/* bulk, interrupt */
		/* actual_length may be nonzero on retry paths */
		buf = urb->transfer_buffer + urb->actual_length;	//Start address + number of transferred bytes = next address to write/read.
		//Data buffer length - number of transferred bytes = number of remaining bytes.
		len = urb->transfer_buffer_length - urb->actual_length;
	}

	trace_musb_urb_start(musb, urb);

	/* Configure endpoint */
	musb_ep_set_qh(hw_ep, is_in, qh);						//Sets the pointer to the structure containing URBs to the given qh.
	//Programs endpoint for tx/rx transfer, including flusing FIFOs and configuring DMA.
	musb_ep_program(musb, epnum, urb, !is_in, buf, offset, len);

	/* transmit may have more work: start it when it is time */
	if (is_in)												//If receive, stop here.
		return;
															//If transmit, continue from here.
	/* determine if the time is right for a periodic transfer */
	switch (qh->type) {
	case USB_ENDPOINT_XFER_ISOC:
	case USB_ENDPOINT_XFER_INT:
		musb_dbg(musb, "check whether there's still time for periodic Tx");
		/* FIXME this doesn't implement that scheduling policy ...
		 * or handle framecounter wrapping
		 */
		if (1) {	/* Always assume URB_ISO_ASAP */
			/* REVISIT the SOF irq handler shouldn't duplicate
			 * this code; and we don't init urb->start_frame...
			 */
			qh->frame = 0;									//Not "for periodic schedule", meaning not for isochronous transfer?
			goto start;
		} else {
			qh->frame = urb->start_frame;					//"Returns the initial frame for isochronous transfers."
			/* enable SOF interrupt so we can count down */
			musb_dbg(musb, "SOF for %d", epnum);
#if 1 /* ifndef	CONFIG_ARCH_DAVINCI */
			musb_writeb(mbase, MUSB_INTRUSBE, 0xff);		//Interrupt USB enable?
#endif
		}
		break;
	default:
start:
		musb_dbg(musb, "Start TX%d %s", epnum,
			hw_ep->tx_channel ? "dma" : "pio");

		if (!hw_ep->tx_channel)						//If no TX DMA channel (DMA not used? or transfer already complete?), then
			musb_h_tx_start(hw_ep);					//write TX endpoint registers to say a packet is ready for transmission?
		else if (is_cppi_enabled(musb) || tusb_dma_omap(musb))
			musb_h_tx_dma_start(hw_ep);				//Configures endpoint for DMA (enables DMA and sets DMA mode).
	}
}

/* Context: caller owns controller lock, IRQs are blocked */
//Removes the URB from the endpoint queue via the Linux kernel and given back to
//the driver.
static void musb_giveback(struct musb *musb, struct urb *urb, int status)
__releases(musb->lock)
__acquires(musb->lock)
{
//printk("drivers/usb/musb/musb_host.c:musb_giveback\n");
	trace_musb_urb_gb(musb, urb);

	//"remove an URB from its endpoint queue"
	//"The actions carried out here are required for URB completion."
	usb_hcd_unlink_urb_from_ep(musb->hcd, urb);
	spin_unlock(&musb->lock);
	//"return URB from HCD to device driver"; "This hands the URB from HCD to its USB device driver, using its completion function."
	usb_hcd_giveback_urb(musb->hcd, urb, status);
	spin_lock(&musb->lock);
}

/*
 * Advance this hardware endpoint's queue, completing the specified URB and
 * advancing to either the next URB queued to that qh, or else invalidating
 * that qh and advancing to the next qh scheduled after the current one.
 *
 * Context: caller owns controller lock, IRQs are blocked
 */
//Complete the processing of the given URB(?), and setup transfer for the next
//urb associated with the given hardware endpoint.
static void musb_advance_schedule(struct musb *musb, struct urb *urb,
				  struct musb_hw_ep *hw_ep, int is_in)
{
//printk("drivers/usb/musb/musb_host.c:musb_advance_schedule\n");
	struct musb_qh		*qh = musb_ep_get_qh(hw_ep, is_in);	//Inwards/outwards URB queue.
	struct musb_hw_ep	*ep = qh->hw_ep;					//Get hardware endpoint associated with next URB.
	int			ready = qh->is_ready;						//"safe to modify hw_ep"
	int			status;
	u16			toggle;

	//Operation now in progress then status is updated to 0 (no error).
	//"read in non-iso completion functions to get the status of the particular
	// request. ISO requests only use it to tell whether the URB was unlinked."
	status = (urb->status == -EINPROGRESS) ? 0 : urb->status;

	/* save toggle eagerly, for paranoia */
	switch (qh->type) {										//Transfer type.
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		//musb_default_get_toggle which flips some bit related to data of
		//undocumented register of MUSB0/1.
		toggle = musb->io.get_toggle(qh, !is_in);
		usb_settoggle(urb->dev, qh->epnum, !is_in, toggle ? 1 : 0);	//Internal to host controller driver.
		break;
	case USB_ENDPOINT_XFER_ISOC:
		if (status == 0 && urb->error_count)
			status = -EXDEV;										//Cross-device link
		break;
	}

	qh->is_ready = 0;							//Not safe to modify hardware endpoint.
	musb_giveback(musb, urb, status);			//"return URB from HCD to device driver" and "remove an URB from its endpoint queue".
	qh->is_ready = ready;						//Previous "safe-modification" satus.

	/* reclaim resources (and bandwidth) ASAP; deschedule it, and
	 * invalidate qh as soon as list_empty(&hep->urb_list)
	 */
	if (list_empty(&qh->hep->urb_list)) {						//No "urbs queued to this endpoint; maintained by usbcore"
		struct list_head	*head;
		struct dma_controller	*dma = musb->dma_controller;	//The DMA controller of the MUSB0/1.

		if (is_in) {											//Input USB packet to be received.
			ep->rx_reinit = 1;									//Reinitialize RX hardware endpoint.
			if (ep->rx_channel) {								//Has receive MUSB DMA channel structure.
				dma->channel_release(ep->rx_channel);		//Calls cppi41_dma_channel_release, marking the structure as free for use.
				ep->rx_channel = NULL;							//No transfers to be performed, and channel is removed.
			}
		} else {												//Output USB packet to be transmitted.
			ep->tx_reinit = 1;									//Reinitialize TX hardware endpoint.
			if (ep->tx_channel) {								//Has transmit MUSB DMA channel structure.
				dma->channel_release(ep->tx_channel);		//Calls cppi41_dma_channel_release, marking the structure as free for use.
				ep->tx_channel = NULL;							//No transfers to be performed, and channel is removed.
			}
		}

		/* Clobber old pointers to this qh */
		musb_ep_set_qh(ep, is_in, NULL);						//Sets the pointer to the structure containing URBs to empty?
		qh->hep->hcpriv = NULL;									//"for use by HCD; typically holds hardware dma queue head (QH) with
																// one or more transfer descriptors (TDs) per urb"

		switch (qh->type) {

		case USB_ENDPOINT_XFER_CONTROL:							//Control
		case USB_ENDPOINT_XFER_BULK:							//Bulk transfer.
			/* fifo policy for these lists, except that NAKing
			 * should rotate a qh to the end (for fairness).
			 */
			if (qh->mux == 1) {									//qh multiplexed to hw_ep (among other qhs?, with the same hardware endpoint?)
				head = qh->ring.prev;							//Last becomes head?
				list_del(&qh->ring);							//Remove list?
				kfree(qh);										//Free head.
				qh = first_qh(head);							//Get the first queue head.
				break;
			}
			fallthrough;

		case USB_ENDPOINT_XFER_ISOC:
		case USB_ENDPOINT_XFER_INT:
			/* this is where periodic bandwidth should be
			 * de-allocated if it's tracked and allocated;
			 * and where we'd update the schedule tree...
			 */
			kfree(qh);
			qh = NULL;
			break;
		}
	}

	if (qh != NULL && qh->is_ready) {			//Additional URBs and is ready, then start the urb at the head of the endpoint queue.
		musb_dbg(musb, "... next ep%d %cX urb %p",
		    hw_ep->epnum, is_in ? 'R' : 'T', next_urb(qh));
		//Configures endpoint associated with qh for transfer the URB associated with qh.
		musb_start_urb(musb, is_in, qh);
	}
}

//Flushes reception FIFO. Means that FIFO is cleared after packet has been
//transferred to memory?
//"The FIFO pointer is reset and the TxPktRdy/RxPktRdy bit (below) is cleared."
//For tx: "and an interrupt is generated."
static u16 musb_h_flush_rxfifo(struct musb_hw_ep *hw_ep, u16 csr)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_flush_rxfifo\n");
	/* we don't want fifo to fill itself again;
	 * ignore dma (various models),
	 * leave toggle alone (may not have been saved yet)
	 */
	csr |= MUSB_RXCSR_FLUSHFIFO | MUSB_RXCSR_RXPKTRDY;
	csr &= ~(MUSB_RXCSR_H_REQPKT
		| MUSB_RXCSR_H_AUTOREQ
		| MUSB_RXCSR_AUTOCLEAR);

	/* write 2x to allow double buffering */
	musb_writew(hw_ep->regs, MUSB_RXCSR, csr);
	musb_writew(hw_ep->regs, MUSB_RXCSR, csr);

	/* flush writebuffer */
	return musb_readw(hw_ep->regs, MUSB_RXCSR);
}

/*
 * PIO RX for a packet (or part of it).
 */
//iso_err means CRC or bit-stuff error.
//Updates URB and endpoint buffer parameters after receiving a packet, and
//request next packet if not complete.
static bool
musb_host_packet_rx(struct musb *musb, struct urb *urb, u8 epnum, u8 iso_err)
{
//printk("drivers/usb/musb/musb_host.c:musb_host_packet_rx\n");
	u16			rx_count;
	u8			*buf;
	u16			csr;
	bool			done = false;							//Not completed reception by default.
	u32			length;
	int			do_flush = 0;
	struct musb_hw_ep	*hw_ep = musb->endpoints + epnum;	//Hardware endpoint data structure.
	void __iomem		*epio = hw_ep->regs;				//Hardware endpoint undocumented registers.
	struct musb_qh		*qh = hw_ep->in_qh;					//URBs associated with endpoint.
	int			pipe = urb->pipe;							//Type of USB transfer (bulk/isochronous/control/interrupt, in/out, etc.)
	void			*buffer = urb->transfer_buffer;			//"pointer to the mapped area" buffer

	/* musb_ep_select(mbase, epnum); */
	rx_count = musb_readw(epio, MUSB_RXCOUNT);				//Number of received bytes in FIFO.
	musb_dbg(musb, "RX%d count %d, buffer %p len %d/%d", epnum, rx_count,
			urb->transfer_buffer, qh->offset,
			urb->transfer_buffer_length);

	/* unload FIFO */
	if (usb_pipeisoc(pipe)) {								//Isochronous (periodic real-time) transfer.
		int					status = 0;						//No error by default.
		struct usb_iso_packet_descriptor	*d;

		if (iso_err) {										//CRC or bit-stuff error.
			status = -EILSEQ;								//"Illegal byte sequence"
			urb->error_count++;
		}

		d = urb->iso_frame_desc + qh->iso_idx;				//Current isochronous frame descriptor.
		buf = buffer + d->offset;							//Get offset from start buffer address?
		length = d->length;									//Get expected length (for periodic isochronous real-time) of buffer?
		if (rx_count > length) {							//Received bytes is larger than memory buffer.
			if (status == 0) {
				status = -EOVERFLOW;						//Value too large for defined data type.
				urb->error_count++;
			}
			musb_dbg(musb, "OVERFLOW %d into %d", rx_count, length);
			do_flush = 1;									//Empty reception FIFO.
		} else
			length = rx_count;								//Length is number of received bytes.
		urb->actual_length += length;			//Total number of transferred bytes of URB is incremented by number of received bytes.
		d->actual_length = length;							//Frame length is received length.

		d->status = status;									//Error code associated with isochronous frame.

		/* see if we are done */
		//Number of isochronous frames is at least equal to the number of packets means done.
		done = (++qh->iso_idx >= urb->number_of_packets);
	} else {												//Not isochronous transfer.
		/* non-isoch */
		buf = buffer + qh->offset;							//Start of buffer plus current offset is next start address.
		length = urb->transfer_buffer_length - qh->offset;	//Memory buffer size minus current offset is remaining buffer size.
		if (rx_count > length) {							//Received number of bytes greater than remaining size.
			if (urb->status == -EINPROGRESS)
				urb->status = -EOVERFLOW;					//Error is overflow.
			musb_dbg(musb, "OVERFLOW %d into %d", rx_count, length);
			do_flush = 1;									//Empty reception FIFO.
		} else
			length = rx_count;								//Received data length is number of received bytes.
		urb->actual_length += length;		//Total received number of recevied bytes of URB incremented with number of received bytes.
		qh->offset += length;								//Next offset from start buffer is incremented by received length.

		/* see if we are done */
		done = (urb->actual_length == urb->transfer_buffer_length)	//Total number of received length equals size of data buffer, or
			|| (rx_count < qh->maxpacket)			//received number of bytes less than maximum packet size (implying no more data), or
			|| (urb->status != -EINPROGRESS);		//URB not in progress.
		if (done											//If done, and
				&& (urb->status == -EINPROGRESS)			//in progress, and
				&& (urb->transfer_flags & URB_SHORT_NOT_OK)	//short packet not allowed, and
				&& (urb->actual_length						//total number of received  less than memory buffer size,
					< urb->transfer_buffer_length))			//then
			urb->status = -EREMOTEIO;						//"Remote I/O error".
	}

	musb_read_fifo(hw_ep, length, buf);						//Calls musb_default_read_fifo: PIO read function from FIFO to memory.

	csr = musb_readw(epio, MUSB_RXCSR);						//Read control status register of endpoint.
	csr |= MUSB_RXCSR_H_WZC_BITS;							//Sets RxStall, Error, DataError, RxPktRdy.
	if (unlikely(do_flush))									//If received bytes is greater than remaining buffer size,
		musb_h_flush_rxfifo(hw_ep, csr);					//empty FIFO, to enable more bytes?
	else {													//Received bytes not exceeding remaining buffer size, then
		/* REVISIT this assumes AUTOCLEAR is never set */
		csr &= ~(MUSB_RXCSR_RXPKTRDY | MUSB_RXCSR_H_REQPKT);	//Clear packet ready and request packet.
		if (!done)
			csr |= MUSB_RXCSR_H_REQPKT;							//If not done, then request packet IN transaction.
		musb_writew(epio, MUSB_RXCSR, csr);						//Update control status register.
	}

	return done;
}

/* we don't always need to reinit a given side of an endpoint...
 * when we do, use tx/rx reinit routine and then construct a new CSR
 * to address data toggle, NYET, and DMA or PIO.
 *
 * it's possible that driver bugs (especially for DMA) or aborting a
 * transfer might have left the endpoint busier than it should be.
 * the busy/not-empty tests are basically paranoia.
 */
//Initialize endpoint for reception. Stop transmission and initialize endpoint
//registers for reception.
static void
musb_rx_reinit(struct musb *musb, struct musb_qh *qh, u8 epnum)
{
//printk("drivers/usb/musb/musb_host.c:musb_rx_reinit\n");
	struct musb_hw_ep *ep = musb->endpoints + epnum;
	u16	csr;

	/* NOTE:  we know the "rx" fifo reinit never triggers for ep0.
	 * That always uses tx_reinit since ep0 repurposes TX register
	 * offsets; the initial SETUP packet is also a kind of OUT.
	 */

	/* if programmed for Tx, put it in RX mode */
	if (ep->is_shared_fifo) {										//Endpoint TX and RX uses share FIFO memory.
		csr = musb_readw(ep->regs, MUSB_TXCSR);
		if (csr & MUSB_TXCSR_MODE) {								//Endpoint is in TX mode?		
			musb_h_tx_flush_fifo(ep);								//Transmits all packets in TX FIFO.
			csr = musb_readw(ep->regs, MUSB_TXCSR);
			musb_writew(ep->regs, MUSB_TXCSR,						//Do data toggling related to integrity?
				    csr | MUSB_TXCSR_FRCDATATOG);
		}

		/*
		 * Clear the MODE bit (and everything else) to enable Rx.
		 * NOTE: we mustn't clear the DMAMODE bit before DMAENAB.
		 */
		if (csr & MUSB_TXCSR_DMAMODE)
			musb_writew(ep->regs, MUSB_TXCSR, MUSB_TXCSR_DMAMODE);
		musb_writew(ep->regs, MUSB_TXCSR, 0);

	/* scrub all previous state, clearing toggle */
	}																//Now TX is stopped.
	csr = musb_readw(ep->regs, MUSB_RXCSR);
	if (csr & MUSB_RXCSR_RXPKTRDY)									//RX packet pending?
		WARNING("rx%d, packet/%d ready?\n", ep->epnum,
			musb_readw(ep->regs, MUSB_RXCOUNT));

	musb_h_flush_rxfifo(ep, MUSB_RXCSR_CLRDATATOG);					//Flush/empty RX FIFO.

	/* target addr and (for multipoint) hub addr/port */
	if (musb->is_multipoint) {										//FALSE
		musb_write_rxfunaddr(musb, epnum, qh->addr_reg);
		musb_write_rxhubaddr(musb, epnum, qh->h_addr_reg);
		musb_write_rxhubport(musb, epnum, qh->h_port_reg);
	} else															//TRUE
		musb_writeb(musb->mregs, MUSB_FADDR, qh->addr_reg);

	/* protocol/endpoint, interval/NAKlimit, i/o size */
	musb_writeb(ep->regs, MUSB_RXTYPE, qh->type_reg);				//Endpoint configured for RX.
	//Writes "NAK timeout interval is 8 (128 uframe or 16ms) for HS and 4 (8 frame or 8ms) for FS device".
	musb_writeb(ep->regs, MUSB_RXINTERVAL, qh->intv_reg);
	/* NOTE: bulk combining rewrites high bits of maxpacket */
	/* Set RXMAXP with the FIFO size of the endpoint
	 * to disable double buffer mode.
	 */
	musb_writew(ep->regs, MUSB_RXMAXP,
			qh->maxpacket | ((qh->hb_mult - 1) << 11));				//Max FIFO DMA packet buffer length.

	ep->rx_reinit = 0;												//The endpoint has been initialized for reception.
}

static void musb_tx_dma_set_mode_mentor(struct musb_hw_ep *hw_ep, 
					struct musb_qh *qh,
					u32 *length, u8 *mode)
{
//printk("drivers/usb/musb/musb_host.c:musb_tx_dma_set_mode_mentor\n");
	struct dma_channel	*channel = hw_ep->tx_channel;
	void __iomem		*epio = hw_ep->regs;
	u16			pkt_size = qh->maxpacket;
	u16			csr;

	if (*length > channel->max_len)
		*length = channel->max_len;

	csr = musb_readw(epio, MUSB_TXCSR);
	if (*length > pkt_size) {
		*mode = 1;
		csr |= MUSB_TXCSR_DMAMODE | MUSB_TXCSR_DMAENAB;
		/* autoset shouldn't be set in high bandwidth */
		/*
		 * Enable Autoset according to table
		 * below
		 * bulk_split hb_mult	Autoset_Enable
		 *	0	1	Yes(Normal)
		 *	0	>1	No(High BW ISO)
		 *	1	1	Yes(HS bulk)
		 *	1	>1	Yes(FS bulk)
		 */
		if (qh->hb_mult == 1 || (qh->hb_mult > 1 &&
					can_bulk_split(hw_ep->musb, qh->type)))
			csr |= MUSB_TXCSR_AUTOSET;
	} else {
		*mode = 0;
		csr &= ~(MUSB_TXCSR_AUTOSET | MUSB_TXCSR_DMAMODE);
		csr |= MUSB_TXCSR_DMAENAB; /* against programmer's guide */
	}
	channel->desired_mode = *mode;
	musb_writew(epio, MUSB_TXCSR, csr);
}
//Sets transferred length of DMA channel to zero and mode to a boolean
//indicating whether packet is zero length.
static void musb_tx_dma_set_mode_cppi_tusb(struct musb_hw_ep *hw_ep,
					   struct urb *urb,
					   u8 *mode)
{
//printk("drivers/usb/musb/musb_host.c:musb_tx_dma_set_mode_cppi_tusb\n");
	struct dma_channel *channel = hw_ep->tx_channel;

	channel->actual_len = 0;			//No bytes transferred.

	/*
	 * TX uses "RNDIS" mode automatically but needs help
	 * to identify the zero-length-final-packet case.
	 */
	*mode = (urb->transfer_flags & URB_ZERO_PACKET) ? 1 : 0;	//Zero length packet?
}

//Attempts to configure DMA transfer.
static bool musb_tx_dma_program(struct dma_controller *dma,
		struct musb_hw_ep *hw_ep, struct musb_qh *qh,
		struct urb *urb, u32 offset, u32 length)
{
//printk("drivers/usb/musb/musb_host.c:musb_tx_dma_program\n");
	struct dma_channel	*channel = hw_ep->tx_channel;						//Endpoint DMA channel structure.
	u16			pkt_size = qh->maxpacket;									//Packet size to send from memory to endpoint.
	u8			mode;

	if (musb_dma_inventra(hw_ep->musb) || musb_dma_ux500(hw_ep->musb))
		musb_tx_dma_set_mode_mentor(hw_ep, qh,
					    &length, &mode);
	else if (is_cppi_enabled(hw_ep->musb) || tusb_dma_omap(hw_ep->musb))	//TRUE
		//Sets transferred length of DMA channel to zero and mode to a boolean
		//indicating whether packet is zero length.
		musb_tx_dma_set_mode_cppi_tusb(hw_ep, urb, &mode);
	else
		return false;

	qh->segsize = length;													//Bytes to transfer.

	/*
	 * Ensure the data reaches to main memory before starting
	 * DMA transfer
	 */
	wmb();																	//Write data to memory before initiating DMA.

	//Calls cppi41_dma_channel_program, which sets memory transfers parameters
	//for DMA channel and callback function, and returns indication of
	//successful or not.
	if (!dma->channel_program(channel, pkt_size, mode,
			urb->transfer_dma + offset, length)) {							//Not successful DMA configuration.
		void __iomem *epio = hw_ep->regs;
		u16 csr;

		dma->channel_release(channel);					//cppi41_dma_channel_release which marks the dma channel structure free for use.
		hw_ep->tx_channel = NULL;						//Endpoint has no DMA channel structure.

		csr = musb_readw(epio, MUSB_TXCSR);
		csr &= ~(MUSB_TXCSR_AUTOSET | MUSB_TXCSR_DMAENAB);	//Clear flags of endpoint.
		musb_writew(epio, MUSB_TXCSR, csr | MUSB_TXCSR_H_WZC_BITS);
		return false;										//Unsuccessful DMA configuration.
	}
	return true;	//Successful DMA configuration.
}

/*
 * Program an HDRC endpoint as per the given URB
 * Context: irqs blocked, controller lock held
 */
//(musb, ep_number, urb, direction, start address, start offset?, bytes to transfer)
//Programs endpoint for tx/rx transfer, including flusing FIFOs and configuring DMA.
static void musb_ep_program(struct musb *musb, u8 epnum,
			struct urb *urb, int is_out,
			u8 *buf, u32 offset, u32 len)
{
//printk("drivers/usb/musb/musb_host.c:musb_ep_program\n");
	struct dma_controller	*dma_controller;
	struct dma_channel	*dma_channel;
	u8			dma_ok;
	void __iomem		*mbase = musb->mregs;					//Undocumented MUSB0/1 registers.		
	struct musb_hw_ep	*hw_ep = musb->endpoints + epnum;		//Endpoint data structure.
	void __iomem		*epio = hw_ep->regs;					//Undocumented MUSB0/1 registers for this endpoint.
	struct musb_qh		*qh = musb_ep_get_qh(hw_ep, !is_out);	//Get URB in our outbound head.
	u16			packet_sz = qh->maxpacket;						//Max FIFO DMA packet buffer length.
	u8			use_dma = 1;									//Default is using DMA.
	u16			csr;

	musb_dbg(musb, "%s hw%d urb %p spd%d dev%d ep%d%s "
				"h_addr%02x h_port%02x bytes %d",
			is_out ? "-->" : "<--",
			epnum, urb, urb->dev->speed,
			qh->addr_reg, qh->epnum, is_out ? "out" : "in",
			qh->h_addr_reg, qh->h_port_reg,
			len);
	//Calls musb_indexed_ep_select with second argument identifying the endpoint.
	//Writes index register that controls the register bank for following
	//register accesses.
	musb_ep_select(mbase, epnum);

	if (is_out && !len) {										//Outbound packet with zero length.
		use_dma = 0;											//Do not use DMA.
		csr = musb_readw(epio, MUSB_TXCSR);						//TX control status register?
		csr &= ~MUSB_TXCSR_DMAENAB;								//Clear DMA enable bit?
		musb_writew(epio, MUSB_TXCSR, csr);						//Update control status register.
		hw_ep->tx_channel = NULL;								//No DMA channel is needed for this outbound transmission.
	}

	/* candidate for DMA? */
	dma_controller = musb->dma_controller;
	//Packet to transmit, linux kernel with DMA support, not endpoint 0, and has
	//DMA controller data structure. Second and fourth conditions are true for
	//our configuration.
	if (use_dma && is_dma_capable() && epnum && dma_controller) {
		dma_channel = is_out ? hw_ep->tx_channel : hw_ep->rx_channel;	//Gets the MUSB DMA channel structure for this transfer.
		if (!dma_channel) {												//No DMA channel data structure.
			dma_channel = dma_controller->channel_alloc(				//Allocate DMA channel data structure.
					dma_controller, hw_ep, is_out);
			if (is_out)													//If outbound transfer, assign tx_channel DMA channel structure.
				hw_ep->tx_channel = dma_channel;
			else														//If inbound transfer, assign rx_channel DMA channel structure.
				hw_ep->rx_channel = dma_channel;
		}
	} else
		dma_channel = NULL;												//Do not use DMA.

	/* make sure we clear DMAEnab, autoSet bits from previous run */

	/* OUT/transmit/EP0 or IN/receive? */
	if (is_out) {														//Transmit.
		u16	csr;
		u16	int_txe;
		u16	load_count;

		csr = musb_readw(epio, MUSB_TXCSR);					//Read undocumented endpoint MUSB register: TX control status register?

		/* disable interrupt in case we flush */
		int_txe = musb->intrtxe;							//Read interrupt register.
		musb_writew(mbase, MUSB_INTRTXE, int_txe & ~(1 << epnum));	//Disable interrupts.

		/* general endpoint setup */
		if (epnum) {										//Not endpoint 0.
			/* flush all old state, set default */
			/*
			 * We could be flushing valid
			 * packets in double buffering
			 * case
			 */
			if (!hw_ep->tx_double_buffered)	//If not double-buffering is used, the FIFO must be flushed before adding a new buffer?
				musb_h_tx_flush_fifo(hw_ep);

			/*
			 * We must not clear the DMAMODE bit before or in
			 * the same cycle with the DMAENAB bit, so we clear
			 * the latter first...
			 */
			csr &= ~(MUSB_TXCSR_H_NAKTIMEOUT
					| MUSB_TXCSR_AUTOSET
					| MUSB_TXCSR_DMAENAB
					| MUSB_TXCSR_FRCDATATOG
					| MUSB_TXCSR_H_RXSTALL
					| MUSB_TXCSR_H_ERROR
					| MUSB_TXCSR_TXPKTRDY
					);
			csr |= MUSB_TXCSR_MODE;

			if (!hw_ep->tx_double_buffered)
				//"You may want to read some introduction of USB site to
				// understand the use of Data Toggle. Basically it is for data
				// integrity over the device side."
				csr |= musb->io.set_toggle(qh, is_out, urb);	//Calls musb_default_set_toggle. Something with data integrity.

			musb_writew(epio, MUSB_TXCSR, csr);			//Writes the right endpoint TXCSR register since register bank was set above.
			/* REVISIT may need to clear FLUSHFIFO ... */
			csr &= ~MUSB_TXCSR_DMAMODE;
			musb_writew(epio, MUSB_TXCSR, csr);					//Clear DMA mode.
			csr = musb_readw(epio, MUSB_TXCSR);
		} else {
			/* endpoint 0: just flush */
			musb_h_ep0_flush_fifo(hw_ep);						//Flush FIFO of endpoint 0, which is for transmission.
		}

		/* target addr and (for multipoint) hub addr/port */
		if (musb->is_multipoint) {								//FALSE.
			musb_write_txfunaddr(musb, epnum, qh->addr_reg);
			musb_write_txhubaddr(musb, epnum, qh->h_addr_reg);
			musb_write_txhubport(musb, epnum, qh->h_port_reg);
/* FIXME if !epnum, do the same for RX ... */
		} else													//TRUE.
			musb_writeb(mbase, MUSB_FADDR, qh->addr_reg);

		/* protocol/endpoint/interval/NAKlimit */
		if (epnum) {											//Not endpoint 0.
			musb_writeb(epio, MUSB_TXTYPE, qh->type_reg);		//Write tx or rx type of transfer.
			if (can_bulk_split(musb, qh->type)) {				//TRUE. Split transfer into multiple transactions?
				qh->hb_mult = hw_ep->max_packet_sz_tx			//FIFO size divided by "get endpoint's max packet size" specifies number
						/ packet_sz;							//of packets that might be needed for transmission to an endpoint?
				musb_writew(epio, MUSB_TXMAXP, packet_sz		//Writes max packet size to endpoint register?
					| ((qh->hb_mult) - 1) << 11);
			} else {
				musb_writew(epio, MUSB_TXMAXP,					//Same as 3 lines above since qh->maxpacket = packet_sz.
						qh->maxpacket |
						((qh->hb_mult - 1) << 11));
			}
			//Writes "NAK timeout interval is 8 (128 uframe or 16ms) for HS and 4 (8 frame or 8ms) for FS device".
			musb_writeb(epio, MUSB_TXINTERVAL, qh->intv_reg);
		} else {
			//Writes "NAK timeout interval is 8 (128 uframe or 16ms) for HS and 4 (8 frame or 8ms) for FS device".
			musb_writeb(epio, MUSB_NAKLIMIT0, qh->intv_reg);
			if (musb->is_multipoint)							//FALSE
				musb_writeb(epio, MUSB_TYPE0,
						qh->type_reg);
		}

		if (can_bulk_split(musb, qh->type))						//TRUE.
			load_count = min((u32) hw_ep->max_packet_sz_tx,		//FIFO size or data length is the number of bytes to load.
						len);
		else
			load_count = min((u32) packet_sz, len);				//Packet size or data length is the number of bytes to load.

		if (dma_channel && musb_tx_dma_program(dma_controller,	//Attempts to configure DMA transfer. Returns true for success.
					hw_ep, qh, urb, offset, len))
			load_count = 0;										//Nothing remains.

		if (load_count) {										//DMA was unsuccessful, so load manually.

			/* PIO to load FIFO */
			qh->segsize = load_count;							//current transfer fragment.
			if (!buf) {											//buf is null pointer. Some scatter-gather list is used instead?
				//qh->sg_miter: "for highmem in PIO mode"
				//
				//urb->sg:
				//"scatter gather buffer list, the buffer size of each element
				// in the list (except the last) must be divisible by the
				// endpoint's max packet size if no_sg_constraint isn't set in
				// 'struct usb_bus'"
				sg_miter_start(&qh->sg_miter, urb->sg, 1,		//start mapping iteration over a sg list with 1 entry.
						SG_MITER_ATOMIC							//use kmap_atomic
						| SG_MITER_FROM_SG);					//nop
				if (!sg_miter_next(&qh->sg_miter)) {			//proceed mapping iterator to the next mapping. Returns true if
					dev_err(musb->controller,					//&qh->sg_miter contains a next mappeing, and false otherwise.
							"error: sg"
							"list empty\n");
					sg_miter_stop(&qh->sg_miter);				//stop mapping iteration.
					goto finish;
				}
				buf = qh->sg_miter.addr + urb->sg->offset +		//Pointer to mapped area + offset + actual transferred length.
					urb->actual_length;
				//Actual transfer size is the minimum of current transfer
				// segment and scatter-gather length.
				load_count = min_t(u32, load_count,
						qh->sg_miter.length);
				musb_write_fifo(hw_ep, load_count, buf);		//Calls musb_default_write_fifo: PIO write function from memory to FIFO.
				qh->sg_miter.consumed = load_count;				//number of consumed bytes
				sg_miter_stop(&qh->sg_miter);					//stop mapping iteration.
			} else												//buf is not null pointer.
				musb_write_fifo(hw_ep, load_count, buf);		//Calls musb_default_write_fifo: PIO write function from memory to FIFO.
		}	//DMA successful.
finish:
		/* re-enable interrupt */
		musb_writew(mbase, MUSB_INTRTXE, int_txe);

	/* IN/receive */
	} else {													//Program en endpoint for input/reception.
		u16 csr = 0;

		if (hw_ep->rx_reinit) {									//Reinitialize receive endpoint.
			//Initialize endpoint for reception. Stop transmission and initialize endpoint registers for reception.
			musb_rx_reinit(musb, qh, epnum);

			csr |= musb->io.set_toggle(qh, is_out, urb);		//Calls musb_default_set_toggle. Something with data integrity.

			if (qh->type == USB_ENDPOINT_XFER_INT)				//Interrupt transfer.
				csr |= MUSB_RXCSR_DISNYET;

		} else {
			csr = musb_readw(hw_ep->regs, MUSB_RXCSR);			//Read rx control status register?

			if (csr & (MUSB_RXCSR_RXPKTRDY						//Packet ready, DMA enabled, or requesting packet?
					| MUSB_RXCSR_DMAENAB
					| MUSB_RXCSR_H_REQPKT))
				ERR("broken !rx_reinit, ep%d csr %04x\n",
						hw_ep->epnum, csr);

			/* scrub any stale state, leaving toggle alone */
			csr &= MUSB_RXCSR_DISNYET;							//Clean state?
		}

		/* kick things off */

		if ((is_cppi_enabled(musb) || tusb_dma_omap(musb)) && dma_channel) {	//TRUE since CPPI41 is enabled.
			/* Candidate for DMA */
			dma_channel->actual_len = 0L;										//No bytes have been transferred.
			qh->segsize = len;													//Current transfer fragment is the bytes to transfer.

			/* AUTOREQ is in a DMA register */
			musb_writew(hw_ep->regs, MUSB_RXCSR, csr);	//Write RX control status register? (cleans state, see 15 lines above.)
			csr = musb_readw(hw_ep->regs, MUSB_RXCSR);

			/*
			 * Unless caller treats short RX transfers as
			 * errors, we dare not queue multiple transfers.
			 */
			//Calls cppi41_dma_channel_program, which sets memory transfers
			//parameters for DMA channel and callback function, and returns
			//indication of successful or not.
			dma_ok = dma_controller->channel_program(dma_channel,
					packet_sz, !(urb->transfer_flags &
						     URB_SHORT_NOT_OK),
					urb->transfer_dma + offset,
					qh->segsize);
			if (!dma_ok) {										//Failed DMA configuration.
				dma_controller->channel_release(dma_channel);	//Calls cppi41_dma_channel_release: marks the structure as free for use.
				hw_ep->rx_channel = dma_channel = NULL;			//Endpoint has no DMA channel data structure.
			} else
				csr |= MUSB_RXCSR_DMAENAB;						//Successful DMA configuration enables DMA.
		}

		csr |= MUSB_RXCSR_H_REQPKT;								//Host requests packet?
		musb_dbg(musb, "RXCSR%d := %04x", epnum, csr);
		musb_writew(hw_ep->regs, MUSB_RXCSR, csr);				//Update RX Control status register?
		csr = musb_readw(hw_ep->regs, MUSB_RXCSR);				//Side effect of reading register?
	}
}

/* Schedule next QH from musb->in_bulk/out_bulk and move the current qh to
 * the end; avoids starvation for other endpoints.
 */
//Stops current transaction, updates DMA channel data structures and schedules
//the next transaction URB.
static void musb_bulk_nak_timeout(struct musb *musb, struct musb_hw_ep *ep,
	int is_in)
{
//printk("drivers/usb/musb/musb_host.c:musb_bulk_nak_timeout\n");
	struct dma_channel	*dma;
	struct urb		*urb;
	void __iomem		*mbase = musb->mregs;									//Undocumented MUSB0/1 registers.
	void __iomem		*epio = ep->regs;										//Undocumented endpoint registers.
	struct musb_qh		*cur_qh, *next_qh;
	u16			rx_csr, tx_csr;
	u16			toggle;

	musb_ep_select(mbase, ep->epnum);					//Configures register bank for current endpoint for following register accesses.
	if (is_in) {										//Input (RX) transfer.
		dma = is_dma_capable() ? ep->rx_channel : NULL;	//Get RX DMA channel data structure.

		/*
		 * Need to stop the transaction by clearing REQPKT first
		 * then the NAK Timeout bit ref MUSBMHDRC USB 2.0 HIGH-SPEED
		 * DUAL-ROLE CONTROLLER Programmer's Guide, section 9.2.2
		 */
		rx_csr = musb_readw(epio, MUSB_RXCSR);
		rx_csr |= MUSB_RXCSR_H_WZC_BITS;				//RxStall, Error, DataError/NAK Timeout, RxPktRdy.
		rx_csr &= ~MUSB_RXCSR_H_REQPKT;					//The CPU writes 1 to this bit to request an IN transaction.
		musb_writew(epio, MUSB_RXCSR, rx_csr);
		//In Bulk mode, set when RX endpoint is halted after receipt of NAK after limit (RXinterval reg).
		rx_csr &= ~MUSB_RXCSR_DATAERROR;
		musb_writew(epio, MUSB_RXCSR, rx_csr);

		cur_qh = first_qh(&musb->in_bulk);				//Get first input queue head, which is the currently processed head.
	} else {											//Output (TX) transfer.
		dma = is_dma_capable() ? ep->tx_channel : NULL;	//Get TX DMA channel data structure.

		/* clear nak timeout bit */
		tx_csr = musb_readw(epio, MUSB_TXCSR);
		tx_csr |= MUSB_TXCSR_H_WZC_BITS;
		tx_csr &= ~MUSB_TXCSR_H_NAKTIMEOUT;				//CPU should clear this bit to allow the endpoint to continue.
		musb_writew(epio, MUSB_TXCSR, tx_csr);

		cur_qh = first_qh(&musb->out_bulk);				//Get first input queue head, which is the currently processed head.
	}
	if (cur_qh) {										//If there is a head with a queue.
		urb = next_urb(cur_qh);							//Get the head of the queue of the head of the queue?
		if (dma_channel_status(dma) == MUSB_DMA_STATUS_BUSY) {	//DMA channel structure records transactions in progress.
			dma->status = MUSB_DMA_STATUS_CORE_ABORT;			//Abort DMA transaction "due to ... dma or memory bus error".
			//Calls cppi41_dma_channel_abort, which aborts transfers of the
			//given DMA channel and marks it as free for use for another
			//transfer and endpoint.
			musb->dma_controller->channel_abort(dma);
			//Number of transferred bytes incremented by the number of
			//transferred bytes of this DMA channel.
			urb->actual_length += dma->actual_len;
			dma->actual_len = 0L;			//Clear number of transferred bytes of DMA channel for initialization of next transfer.
		}
		toggle = musb->io.get_toggle(cur_qh, !is_in);						//Reads toogle of endpoint associated with queue head.
		usb_settoggle(urb->dev, cur_qh->epnum, !is_in, toggle ? 1 : 0);		//Internal to host controller driver.

		if (is_in) {														//Input transfer.
			/* move cur_qh to end of queue */
			list_move_tail(&cur_qh->ring, &musb->in_bulk);					//Moves &cur_qh->ring to end of &musb->in_bulk.

			/* get the next qh from musb->in_bulk */
			next_qh = first_qh(&musb->in_bulk);								//Get next queue head in input direction.

			/* set rx_reinit and schedule the next qh */
			ep->rx_reinit = 1;												//Reinitialize reception for next input URB?
		} else {															//Output transfer.
			/* move cur_qh to end of queue */
			list_move_tail(&cur_qh->ring, &musb->out_bulk);					//Moves &cur_qh->ring to end of &musb->out_bulk.

			/* get the next qh from musb->out_bulk */
			next_qh = first_qh(&musb->out_bulk);							//Get next queue head in output direction.

			/* set tx_reinit and schedule the next qh */
			ep->tx_reinit = 1;												//Reinitialize reception for next output URB?
		}

		if (next_qh)											//If there exists a next URB, configure the endpoint associated
			musb_start_urb(musb, is_in, next_qh);				//with that next qh to transfer the URB associated with that qh.
	}
}

/*
 * Service the default endpoint (ep0) as host.
 * Return true until it's time to start the status stage.
 */
//Transfers data between memory and endpoint 0 FIFO or changes its OTG state.
static bool musb_h_ep0_continue(struct musb *musb, u16 len, struct urb *urb)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_ep0_continue\n");
	bool			 more = false;										//Initialized to last transfer, might change below.
	u8			*fifo_dest = NULL;
	u16			fifo_count = 0;
	struct musb_hw_ep	*hw_ep = musb->control_ep;
	struct musb_qh		*qh = hw_ep->in_qh;
	struct usb_ctrlrequest	*request;

	switch (musb->ep0_stage) {
	case MUSB_EP0_IN:													//Expects in/RX data. len argument is length of received data.
		fifo_dest = urb->transfer_buffer + urb->actual_length;			//Start address + read data length is memory address to write.
		fifo_count = min_t(size_t, len, urb->transfer_buffer_length -	//buffer length-read data length is bytes left to read/transfer.
				   urb->actual_length);
		if (fifo_count < len)										//Bytes left to read in FIFO is less than received data packet size.
			urb->status = -EOVERFLOW;

		musb_read_fifo(hw_ep, fifo_count, fifo_dest);			//Calls musb_default_read_fifo: PIO read function from FIFO to memory.

		urb->actual_length += fifo_count;					//An additional fifo_count bytes have been transferred from FIFO to memory.
		if (len < qh->maxpacket) {							//Received packet size is less than maximum packet size.
			/* always terminate on short read; it's
			 * rarely reported as an error.
			 */
		} else if (urb->actual_length <						//read data length less than data buffer length, hence more data left.
				urb->transfer_buffer_length)
			more = true;
		break;
	case MUSB_EP0_START:										//Expects acknowledgement.
		request = (struct usb_ctrlrequest *) urb->setup_packet;	//Setup packet used for control.

		if (!request->wLength) {		//No data. The USB wLength field of the Setup packet (https://www.usbmadesimple.co.uk/ums_4.htm)
			musb_dbg(musb, "start no-DATA");
			break;
		} else if (request->bRequestType & USB_DIR_IN) {	//Data to host.
			musb_dbg(musb, "start IN-DATA");
			musb->ep0_stage = MUSB_EP0_IN;					//Expect data in.
			more = true;									//More input.
			break;
		} else {
			musb_dbg(musb, "start OUT-DATA");
			musb->ep0_stage = MUSB_EP0_OUT;					//"Expect ack of OUT DATA"
			more = true;									//More output.
		}
		fallthrough;
	case MUSB_EP0_OUT:										//"expect ack of OUT DATA"
		fifo_count = min_t(size_t, qh->maxpacket,			//Max FIFO DMA packet buffer length.
				   urb->transfer_buffer_length -			//Packet buffer length-transferred length is bytes left to transfer.
				   urb->actual_length);
		if (fifo_count) {									//Bytes to transfer from memory to FIFO.
			fifo_dest = (u8 *) (urb->transfer_buffer		//Start address + read data length is memory address to read.
					+ urb->actual_length);
			musb_dbg(musb, "Sending %d byte%s to ep0 fifo %p",
					fifo_count,
					(fifo_count == 1) ? "" : "s",
					fifo_dest);
			musb_write_fifo(hw_ep, fifo_count, fifo_dest);	//Calls musb_default_write_fifo: PIO write function from memory to FIFO.
			urb->actual_length += fifo_count;				//fifo_count bytes actually transferred.
			more = true;									//More bytes to transfer.
		}
		break;
	default:
		ERR("bogus ep0 stage %d\n", musb->ep0_stage);
		break;
	}

	return more;
}

/*
 * Handle default endpoint interrupt as host. Only called in IRQ time
 * from musb_interrupt().
 *
 * called with controller irqlocked
 */
//Handles interrupt related to endpoint 0 which is the control endpoint 0 of the
//USB controller as a whole. Completes the current URB of the endpoint and
//setups transfer of its next URB?
irqreturn_t musb_h_ep0_irq(struct musb *musb)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_ep0_irq\n");
	//USB reqeust block (a message) describes all relevant information to
	//execute any USB transaction and deliver the data and status back. Probably
	//coming from the kernel and is to be given to an endpoint.
	//Each endpoint of a device has a software queue of URBs.
	struct urb		*urb;
	u16			csr, len;
	int			status = 0;
	void __iomem		*mbase = musb->mregs;		//Undocumented Mentor Graphics registers.
	struct musb_hw_ep	*hw_ep = musb->control_ep;	//control_ep is a macro expanding to endpoints, thus obtains endpoint 0.
	void __iomem		*epio = hw_ep->regs;		//Undocumented Mentor Graphics registers for specific endpoint.
	struct musb_qh		*qh = hw_ep->in_qh;			//Endpoint scheduled inwards.
	bool			complete = false;
	irqreturn_t		retval = IRQ_NONE;

	/* ep0 only has one queue, "in" */
	urb = next_urb(qh);								//Get next URB for endpoint 0 in.

	//Calls musb_indexed_ep_select with second argument identifying endpoint 0.
	//Writes index register that controls the register bank for following
	//register accesses.
	musb_ep_select(mbase, 0);
	csr = musb_readw(epio, MUSB_CSR0);				//Reads control status register of endpoint 0?
	len = (csr & MUSB_CSR0_RXPKTRDY)				//If a received packet is ready, read the number
			? musb_readb(epio, MUSB_COUNT0)			//of received bytes.
			: 0;

	musb_dbg(musb, "<== csr0 %04x, qh %p, count %d, urb %p, stage %d",
		csr, qh, len, urb, musb->ep0_stage);

	/* if we just did status stage, we are done */	//"expect ack of STATUS"
	if (MUSB_EP0_STATUS == musb->ep0_stage) {		
		retval = IRQ_HANDLED;
		complete = true;							//Call completion handler at end of this ISR.
	}

	/* prepare status */
	if (csr & MUSB_CSR0_H_RXSTALL) {
		musb_dbg(musb, "STALLING ENDPOINT");
		status = -EPIPE;

	} else if (csr & MUSB_CSR0_H_ERROR) {
		musb_dbg(musb, "no response, csr0 %04x", csr);
		status = -EPROTO;

	} else if (csr & MUSB_CSR0_H_NAKTIMEOUT) {
		musb_dbg(musb, "control NAK timeout");

		/* NOTE:  this code path would be a good place to PAUSE a
		 * control transfer, if another one is queued, so that
		 * ep0 is more likely to stay busy.  That's already done
		 * for bulk RX transfers.
		 *
		 * if (qh->ring.next != &musb->control), then
		 * we have a candidate... NAKing is *NOT* an error
		 */
		musb_writew(epio, MUSB_CSR0, 0);
		retval = IRQ_HANDLED;
	}

	if (status) {									//True if some error is detected above.
		musb_dbg(musb, "aborting");
		retval = IRQ_HANDLED;
		if (urb)									//There is a USB reqeust block.
			urb->status = status;					//Update status information for URB.
		complete = true;							//Call completion handler at end of this routine.

		/* use the proper sequence to abort the transfer */
		if (csr & MUSB_CSR0_H_REQPKT) {
			csr &= ~MUSB_CSR0_H_REQPKT;				//Clear host request packet?
			musb_writew(epio, MUSB_CSR0, csr);
			csr &= ~MUSB_CSR0_H_NAKTIMEOUT;			//Negative acknowledgement for timeout?
			musb_writew(epio, MUSB_CSR0, csr);
		} else {
			musb_h_ep0_flush_fifo(hw_ep);			//Flushes endpoint 0 FIFO containing DMA packets and resets 
		}

		musb_writeb(epio, MUSB_NAKLIMIT0, 0);

		/* clear it */
		musb_writew(epio, MUSB_CSR0, 0);
	}

	if (unlikely(!urb)) {
		/* stop endpoint since we have no place for its data, this
		 * SHOULD NEVER HAPPEN! */
		ERR("no URB for end 0\n");

		musb_h_ep0_flush_fifo(hw_ep);				//Flushes endpoint 0 FIFO containing DMA packets and resets 
		goto done;
	}
	//Remaining operations of the interrupt handler must be performed before
	//completion handler is invoked at the end of this routine.
	if (!complete) {
		/* call common logic and prepare response */
		if (musb_h_ep0_continue(musb, len, urb)) {	//Transfers data between memory and endpoint 0 FIFO, or changes its
			/* more packets required */				//OTG state, and indicates whether more data packets to transfer.
			csr = (MUSB_EP0_IN == musb->ep0_stage)	//If input or output is expected.
				?  MUSB_CSR0_H_REQPKT : MUSB_CSR0_TXPKTRDY;
		} else {									//All data is stransferred.
			/* data transfer complete; perform status phase */
			if (usb_pipeout(urb->pipe)					//Outdata. Stores type of transfer: in/out, bulk/isochronous/control/interrupt.
					|| !urb->transfer_buffer_length)	//packet buffer length i zero.
				csr = MUSB_CSR0_H_STATUSPKT
					| MUSB_CSR0_H_REQPKT;
			else
				csr = MUSB_CSR0_H_STATUSPKT
					| MUSB_CSR0_TXPKTRDY;

			/* disable ping token in status phase */
			csr |= MUSB_CSR0_H_DIS_PING;

			/* flag status stage */
			musb->ep0_stage = MUSB_EP0_STATUS;			//"expect ack of STATUS"

			musb_dbg(musb, "ep0 STATUS, csr %04x", csr);

		}
		musb_writew(epio, MUSB_CSR0, csr);				//Write control status register of endpoint 0.
		retval = IRQ_HANDLED;
	} else
		musb->ep0_stage = MUSB_EP0_IDLE;

	/* call completion handler if done */
	if (complete)
		//Call completion handler with input indicator '1' since endpoint 0 only transmits?
		//Completes processing of the given URB and setups transfer for the next URB of this hardware endpoint 0.
		musb_advance_schedule(musb, urb, hw_ep, 1);
done:
	return retval;
}


#ifdef CONFIG_USB_INVENTRA_DMA

/* Host side TX (OUT) using Mentor DMA works as follows:
	submit_urb ->
		- if queue was empty, Program Endpoint
		- ... which starts DMA to fifo in mode 1 or 0

	DMA Isr (transfer complete) -> TxAvail()
		- Stop DMA (~DmaEnab)	(<--- Alert ... currently happens
					only in musb_cleanup_urb)
		- TxPktRdy has to be set in mode 0 or for
			short packets in mode 1.
*/

#endif

/* Service a Tx-Available or dma completion irq for the endpoint */
//Services interrupts for non-zero transmit endpoints. Includes stopping
//transactions on NAK timeouts, configures next transaction and updates endpoint
//and DMA data structures.
void musb_host_tx(struct musb *musb, u8 epnum)
{
//printk("drivers/usb/musb/musb_host.c:musb_host_tx\n");
	int			pipe;
	bool			done = false;
	u16			tx_csr;
	size_t			length = 0;
	size_t			offset = 0;
	struct musb_hw_ep	*hw_ep = musb->endpoints + epnum;			//Hardware endpoint data structure.
	void __iomem		*epio = hw_ep->regs;						//Hardware endpoint undocumented registers.
	struct musb_qh		*qh = hw_ep->out_qh;						//URBs associated with endpoint.
	struct urb		*urb = next_urb(qh);							//Next URB of endpoint.
	u32			status = 0;
	void __iomem		*mbase = musb->mregs;						//Undocumented registers of USB0/1.
	struct dma_channel	*dma;
	bool			transfer_pending = false;

	musb_ep_select(mbase, epnum);						//Configures register bank for current endpoint for following register accesses.
	tx_csr = musb_readw(epio, MUSB_TXCSR);				//Reads control status register of endpoint.

	/* with CPPI, DMA sometimes triggers "extra" irqs */
	if (!urb) {											
		musb_dbg(musb, "extra TX%d ready, csr %04x", epnum, tx_csr);
		return;
	}

	pipe = urb->pipe;									//Type of USB transfer (bulk/isochronous/control/interrupt, in/out, etc.)
	dma = is_dma_capable() ? hw_ep->tx_channel : NULL;	//Get transmit DMA channel data structure.
	trace_musb_urb_tx(musb, urb);
	musb_dbg(musb, "OUT/TX%d end, csr %04x%s", epnum, tx_csr,
			dma ? ", dma" : "");

	/* check for errors */
	if (tx_csr & MUSB_TXCSR_H_RXSTALL) {
		/* dma was disabled, fifo flushed */
		musb_dbg(musb, "TX end %d stall", epnum);

		/* stall; record URB status */
		status = -EPIPE;

	} else if (tx_csr & MUSB_TXCSR_H_ERROR) {
		/* (NON-ISO) dma was disabled, fifo flushed */
		musb_dbg(musb, "TX 3strikes on ep=%d", epnum);

		status = -ETIMEDOUT;

	} else if (tx_csr & MUSB_TXCSR_H_NAKTIMEOUT) {
		//Timeout: "TX endpoint is halted following the receipt of NAK responses for longer than the time set"
		if (USB_ENDPOINT_XFER_BULK == qh->type && qh->mux == 1
				&& !list_is_singular(&musb->out_bulk)) {
			musb_dbg(musb, "NAK timeout on TX%d ep", epnum);
			//Stops current transaction, updates DMA channel data structures and
			//schedules the next transaction URB. Clears the NAK bit.
			musb_bulk_nak_timeout(musb, hw_ep, 0);						//0 means output direction, since this is transmit.
		} else {
			musb_dbg(musb, "TX ep%d device not responding", epnum);
			/* NOTE:  this code path would be a good place to PAUSE a
			 * transfer, if there's some other (nonperiodic) tx urb
			 * that could use this fifo.  (dma complicates it...)
			 * That's already done for bulk RX transfers.
			 *
			 * if (bulk && qh->ring.next != &musb->out_bulk), then
			 * we have a candidate... NAKing is *NOT* an error
			 */
			musb_ep_select(mbase, epnum);						//Sets register bank for current endpoint.
			musb_writew(epio, MUSB_TXCSR,			
					MUSB_TXCSR_H_WZC_BITS						//Sets NAK, RXStall, Error and FIFONotEmpty.
					| MUSB_TXCSR_TXPKTRDY);						//"The CPU sets this bit after loading a data packet into the FIFO. "
		}
		return;
	}
done:
	if (status) {												//No error.
		if (dma_channel_status(dma) == MUSB_DMA_STATUS_BUSY) {	//DMA channel is performing transfers.
			dma->status = MUSB_DMA_STATUS_CORE_ABORT;			//Abort transaction due to "core error or USB fault".
			//Calls cppi41_dma_channel_abort, which aborts transfers of the
			//given DMA channel and marks it as free for use for another
			//transfer and endpoint.
			musb->dma_controller->channel_abort(dma);
		}

		/* do the proper sequence to abort the transfer in the
		 * usb core; the dma engine should already be stopped.
		 */
		musb_h_tx_flush_fifo(hw_ep);		//Transmits all packets in TX FIFO.
				//If the CPU sets this bit, TxPktRdy will be automatically set
				//when data of the maximum packet size (value in TxMaxP) is
				//loaded into the TX FIFO.
		tx_csr &= ~(MUSB_TXCSR_AUTOSET
				//Disables DMA request for the TX endpoint.
				| MUSB_TXCSR_DMAENAB
				//The USB sets this bit when 3 attempts have been made to send a
				//packet and no handshake packet has been received. CPU should
				//clear this bit.
				| MUSB_TXCSR_H_ERROR
				//This bit is set when a STALL handshake is received. When this
				//bit is set, any DMA request that is in progress is stopped,
				//the FIFO is completely flushed and the TxPktRdy bit is cleared
				//(see below). The CPU should clear this bit.
				| MUSB_TXCSR_H_RXSTALL
				//Bulk endpoints only: This bit will be set when the TX endpoint
				//is halted following the receipt of NAK responses for longer
				//than the time set as the NAK Limit by the TxInterval register.
				//The CPU should clear this bit to allow the endpoint to continue.
				| MUSB_TXCSR_H_NAKTIMEOUT
				);

		musb_ep_select(mbase, epnum);					//Sets register bank for current endpoint.
		musb_writew(epio, MUSB_TXCSR, tx_csr);			//Updates control status register for current endpoint.
		/* REVISIT may need to clear FLUSHFIFO ... */
		musb_writew(epio, MUSB_TXCSR, tx_csr);
		//TxInterval is an 8-bit register that, for Interrupt and Isochronous
		//transfers, defines the polling interval for the currently-selected TX
		//endpoint. For Bulk endpoints, this register sets the number of
		//frames/microframes after which the endpoint should timeout on
		//receiving a stream of NAK responses.
		//
		//Sets polling interval to 0, and dsiables NAK timeout function.
		musb_writeb(epio, MUSB_TXINTERVAL, 0);

		done = true;
	}

	/* second cppi case */
	if (dma_channel_status(dma) == MUSB_DMA_STATUS_BUSY) {
		musb_dbg(musb, "extra TX%d ready, csr %04x", epnum, tx_csr);
		return;
	}

	if (is_dma_capable() && dma && !status) {						//DMA and no error.
		/*
		 * DMA has completed.  But if we're using DMA mode 1 (multi
		 * packet DMA), we need a terminal TXPKTRDY interrupt before
		 * we can consider this transfer completed, lest we trash
		 * its last packet when writing the next URB's data.  So we
		 * switch back to mode 0 to get that interrupt; we'll come
		 * back here once it happens.
		 */
		if (tx_csr & MUSB_TXCSR_DMAMODE) {
			/*
			 * We shouldn't clear DMAMODE with DMAENAB set; so
			 * clear them in a safe order.  That should be OK
			 * once TXPKTRDY has been set (and I've never seen
			 * it being 0 at this moment -- DMA interrupt latency
			 * is significant) but if it hasn't been then we have
			 * no choice but to stop being polite and ignore the
			 * programmer's guide... :-)
			 *
			 * Note that we must write TXCSR with TXPKTRDY cleared
			 * in order not to re-trigger the packet send (this bit
			 * can't be cleared by CPU), and there's another caveat:
			 * TXPKTRDY may be set shortly and then cleared in the
			 * double-buffered FIFO mode, so we do an extra TXCSR
			 * read for debouncing...
			 */
			tx_csr &= musb_readw(epio, MUSB_TXCSR);
			if (tx_csr & MUSB_TXCSR_TXPKTRDY) {
				tx_csr &= ~(MUSB_TXCSR_DMAENAB |	//The CPU sets this bit to enable the DMA request for the TX endpoint.
					    MUSB_TXCSR_TXPKTRDY);
				musb_writew(epio, MUSB_TXCSR,
					    tx_csr | MUSB_TXCSR_H_WZC_BITS);
			}
			//The CPU sets this bit to select DMA Request Mode 1 and clears it
			//to select DMA Request Mode 0. Note: This bit must not be cleared
			//either before or in the same cycle as the above DMAReqEnab bit is
			//cleared.
			//
			//DMA Request Mode 0 can be used equally well for Bulk, Interrupt or
			//Isochronous transfers. Indeed, if the endpoint is configured for
			//Isochronous transfers, DMA Request Mode 0 should always be
			//selected where DMA is used.
			//
			//DMA Request Mode 1 is chiefly valuable where large blocks of data
			//are transferred to a Bulk endpoint. The USB protocol requires such
			//packets to be split into a series of packets of the maximum packet
			//size for the endpoint (512 bytes for high speed, 64 bytes for full
			//speed).
			tx_csr &= ~(MUSB_TXCSR_DMAMODE |				//Sets DMA mode 0.
					//The CPU sets this bit after loading a data packet into the
					//FIFO. It is cleared automatically when a data packet has
					//been transmitted.
				    MUSB_TXCSR_TXPKTRDY);
			musb_writew(epio, MUSB_TXCSR,
				    tx_csr | MUSB_TXCSR_H_WZC_BITS);

			/*
			 * There is no guarantee that we'll get an interrupt
			 * after clearing DMAMODE as we might have done this
			 * too late (after TXPKTRDY was cleared by controller).
			 * Re-read TXCSR as we have spoiled its previous value.
			 */
			tx_csr = musb_readw(epio, MUSB_TXCSR);
		}

		/*
		 * We may get here from a DMA completion or TXPKTRDY interrupt.
		 * In any case, we must check the FIFO status here and bail out
		 * only if the FIFO still has data -- that should prevent the
		 * "missed" TXPKTRDY interrupts and deal with double-buffered
		 * FIFO mode too...
		 */
		if (tx_csr & (MUSB_TXCSR_FIFONOTEMPTY | MUSB_TXCSR_TXPKTRDY)) {
			//FIFO not empty and TX packet in FIFO is ready to be transmitted.
			musb_dbg(musb,
				"DMA complete but FIFO not empty, CSR %04x",
				tx_csr);
			return;
		}
	}

	if (!status || dma || usb_pipeisoc(pipe)) {	//Isochronous transfer with DMA.
		if (dma)								//If DMA, then bytes to transfer is the number of bytes transferred via DMA.
			length = dma->actual_len;
		else
			length = qh->segsize;			//If not DMA, then bytes to transfer is the number of bytes of current transfer fragment.
		qh->offset += length;					//qh->offset is pointer to memory from where to start read data?

		if (usb_pipeisoc(pipe)) {					//Isochronous transfer (real-time).
			struct usb_iso_packet_descriptor	*d;

			d = urb->iso_frame_desc + qh->iso_idx;	//Gets the next isochronous packet descriptor.
			d->actual_length = length;				//Number of bytes to transfer?
			d->status = status;						//Update status.
			if (++qh->iso_idx >= urb->number_of_packets) {	//All isochronous frames have been transferred.
				done = true;
			} else {										//Next isochronous frame.
				d++;
				offset = d->offset;							//Get offset from start buffer address?
				length = d->length;							//Get expected length of buffer?
			}
		} else if (dma && urb->transfer_buffer_length == qh->offset) {	//DMA and data buffer length is equal to next address: Done.
			done = true;
		} else {
			/* see if we need to send more data, or ZLP */
			if (qh->segsize < qh->maxpacket)							//Current transfer segment is less than (maximum?) packet size?
				done = true;
			else if (qh->offset == urb->transfer_buffer_length			//DMA and data buffer length is equal to next address: Done.
					&& !(urb->transfer_flags							//Not zero length packet.
						& URB_ZERO_PACKET))
				done = true;
			if (!done) {		//If not done, update offset to memory offset from start of buffer containing data.
				offset = qh->offset;
				length = urb->transfer_buffer_length - offset;			//Remaining data to transfer.
				transfer_pending = true;								//Transfer not complete?
			}
		}
	}

	/* urb->status != -EINPROGRESS means request has been faulted,
	 * so we must abort this transfer after cleanup
	 */
	if (urb->status != -EINPROGRESS) {	//URB not in progress.
		done = true;
		if (status == 0)
			status = urb->status;
	}

	if (done) {
		/* set status */
		urb->status = status;
		urb->actual_length = qh->offset;								//Transferred length is equal to offset from start.
		//Call completion handler with output indicator '1'-
		//Completes processing of the given URB and setups transfer for the next URB of this hardware endpoint.
		musb_advance_schedule(musb, urb, hw_ep, USB_DIR_OUT);
		return;
	} else if ((usb_pipeisoc(pipe) || transfer_pending) && dma) {		//DMA, transfer pending or isochronous transfer.
		if (musb_tx_dma_program(musb->dma_controller, hw_ep, qh, urb,	//Attempts to configure DMA transfer.
				offset, length)) {
			if (is_cppi_enabled(musb) || tusb_dma_omap(musb))
				musb_h_tx_dma_start(hw_ep);								//Configures endpoint for DMA (enables DMA and sets DMA mode).
			return;
		}
	} else	if (tx_csr & MUSB_TXCSR_DMAENAB) {					//The CPU sets this bit to enable the DMA request for the TX endpoint.
		musb_dbg(musb, "not complete, but DMA enabled?");
		return;
	}

	/*
	 * PIO: start next packet in this URB.
	 *
	 * REVISIT: some docs say that when hw_ep->tx_double_buffered,
	 * (and presumably, FIFO is not half-full) we should write *two*
	 * packets before updating TXCSR; other docs disagree...
	 */
	if (length > qh->maxpacket)									//Remaining data is more than current packet size?
		length = qh->maxpacket;									//Then length to transfer is max length of packet size?
	/* Unmap the buffer so that CPU can use it */
	usb_hcd_unmap_urb_for_dma(musb->hcd, urb);					//Host controller driver unmaps memory buffer of URB?

	/*
	 * We need to map sg if the transfer_buffer is
	 * NULL.
	 */
	if (!urb->transfer_buffer) {								//If memory data buffer is unmapped?
		/* sg_miter_start is already done in musb_ep_program */
		//proceed mapping iterator to the next mapping. Returns true if &qh->sg_miter contains a next mapping, and false otherwise.
		if (!sg_miter_next(&qh->sg_miter)) {					//No next mapping.
			dev_err(musb->controller, "error: sg list empty\n");
			sg_miter_stop(&qh->sg_miter);						//stop mapping iteration.
			status = -EINVAL;
			goto done;
		}														//Has next mapping.
		length = min_t(u32, length, qh->sg_miter.length);		//Gets the length from minimum of data to transfer and mapped length.
		musb_write_fifo(hw_ep, length, qh->sg_miter.addr);		//Calls musb_default_write_fifo: PIO write function from memory to FIFO.
		qh->sg_miter.consumed = length;							//Number of transferred bytes from memory and FIFO.
		sg_miter_stop(&qh->sg_miter);							//stop mapping iteration.
	} else {													//Memory buffer is mapped?
		musb_write_fifo(hw_ep, length, urb->transfer_buffer + offset);	//Calls musb_default_write_fifo, moves data from memory to FIFO.
	}

	qh->segsize = length;										//Current transfer fragment size.

	musb_ep_select(mbase, epnum);								//Select control status register of current endpoint.
	musb_writew(epio, MUSB_TXCSR,								//Transmit packet is ready in FIFO.
			MUSB_TXCSR_H_WZC_BITS | MUSB_TXCSR_TXPKTRDY);		//Sets NAK Timeout but should be cleared. Sets error bit, but should be
																//cleared. Sets RxStall, but should be cleared. Sets FIFONotEmpty, but
																//should be cleared.
}

#ifdef CONFIG_USB_TI_CPPI41_DMA
/* Seems to set up ISO for cppi41 and not advance len. See commit c57c41d */
//Configures DMA channel for URB and associated hardware endpoint for
//isochronous transfer, and returns true if successful and false otherwise.
static int musb_rx_dma_iso_cppi41(struct dma_controller *dma,
				  struct musb_hw_ep *hw_ep,
				  struct musb_qh *qh,
				  struct urb *urb,
				  size_t len)									//Transferred bytes.
{
//printk("drivers/usb/musb/musb_host.c:musb_rx_dma_iso_cppi41\n");
	struct dma_channel *channel = hw_ep->rx_channel;			//Gets the MUSB DMA channel
	void __iomem *epio = hw_ep->regs;							//Undocumented MUSB0/1 registers for this endpoint.
	dma_addr_t *buf;
	u32 length;
	u16 val;

	//Offset from start buffer address + start address = start address for this transaction.
	buf = (void *)urb->iso_frame_desc[qh->iso_idx].offset +
		(u32)urb->transfer_dma;

	length = urb->iso_frame_desc[qh->iso_idx].length;			//Expected length (for periodic isochronous transfers).

	val = musb_readw(epio, MUSB_RXCSR);
	val |= MUSB_RXCSR_DMAENAB;									//"The CPU sets this bit to enable the DMA request for the Rx endpoint."
	musb_writew(hw_ep->regs, MUSB_RXCSR, val);

	//Calls cppi41_dma_channel_program, which sets memory transfers parameters
	//for DMA channel and callback function, and returns indication of
	//successful or not.
	return dma->channel_program(channel, qh->maxpacket, 0,
				   (u32)buf, length);
}
#else
static inline int musb_rx_dma_iso_cppi41(struct dma_controller *dma,
					 struct musb_hw_ep *hw_ep,
					 struct musb_qh *qh,
					 struct urb *urb,
					 size_t len)
{
//printk("drivers/usb/musb/musb_host.c:musb_rx_dma_iso_cppi41\n");
	return false;
}
#endif

#if defined(CONFIG_USB_INVENTRA_DMA) || defined(CONFIG_USB_UX500_DMA) || \
	defined(CONFIG_USB_TI_CPPI41_DMA)
/* Host side RX (IN) using Mentor DMA works as follows:
	submit_urb ->
		- if queue was empty, ProgramEndpoint
		- first IN token is sent out (by setting ReqPkt)
	LinuxIsr -> RxReady()
	/\	=> first packet is received
	|	- Set in mode 0 (DmaEnab, ~ReqPkt)
	|		-> DMA Isr (transfer complete) -> RxReady()
	|		    - Ack receive (~RxPktRdy), turn off DMA (~DmaEnab)
	|		    - if urb not complete, send next IN token (ReqPkt)
	|			   |		else complete urb.
	|			   |
	---------------------------
 *
 * Nuances of mode 1:
 *	For short packets, no ack (+RxPktRdy) is sent automatically
 *	(even if AutoClear is ON)
 *	For full packets, ack (~RxPktRdy) and next IN token (+ReqPkt) is sent
 *	automatically => major problem, as collecting the next packet becomes
 *	difficult. Hence mode 1 is not used.
 *
 * REVISIT
 *	All we care about at this driver level is that
 *       (a) all URBs terminate with REQPKT cleared and fifo(s) empty;
 *       (b) termination conditions are: short RX, or buffer full;
 *       (c) fault modes include
 *           - iff URB_SHORT_NOT_OK, short RX status is -EREMOTEIO.
 *             (and that endpoint's dma queue stops immediately)
 *           - overflow (full, PLUS more bytes in the terminal packet)
 *
 *	So for example, usb-storage sets URB_SHORT_NOT_OK, and would
 *	thus be a great candidate for using mode 1 ... for all but the
 *	last packet of one URB's transfer.
 */
//Configures next transfer if not done.
static int musb_rx_dma_inventra_cppi41(struct dma_controller *dma,
				       struct musb_hw_ep *hw_ep,
				       struct musb_qh *qh,
				       struct urb *urb,
				       size_t len)										//Transferred bytes.
{
//printk("drivers/usb/musb/musb_host.c:musb_rx_dma_inventra_cppi41\n");
	struct dma_channel *channel = hw_ep->rx_channel;					//MUSB DMA channel data structure of hardware enpoint transfer.
	void __iomem *epio = hw_ep->regs;									//Undocumented MUSB0/1 registers for this endpoint.
	u16 val;
	int pipe;
	bool done;

	pipe = urb->pipe;													//Type of transfer.

	if (usb_pipeisoc(pipe)) {											//Isochronous transfer.
		struct usb_iso_packet_descriptor *d;

		d = urb->iso_frame_desc + qh->iso_idx;							//Get current isochronous transfer descriptor.
		d->actual_length = len;											//Transferred length stored in isochronous transfer descriptor?

		/* even if there was an error, we did the dma
		 * for iso_frame_desc->length
		 */
		if (d->status != -EILSEQ && d->status != -EOVERFLOW)
			//Not "illegal byte sequence" and not "Value too large for defined data type"
			d->status = 0;

		if (++qh->iso_idx >= urb->number_of_packets) {					//All isochronous frames have been transferred.
			done = true;
		} else {														//Isochornous frames left.
			/* REVISIT: Why ignore return value here? */
			if (musb_dma_cppi41(hw_ep->musb))							//TRUE.
				//Configures DMA channel for URB and associated hardware
				//endpoint for isochronous transfer, and returns true if
				//successful and false otherwise.
				done = musb_rx_dma_iso_cppi41(dma, hw_ep, qh,
							      urb, len);
			done = false;
		}

	} else  {															//Not isochronous transfer.
		/* done if urb buffer is full or short packet is recd */
		done = (urb->actual_length + len >=				//Transferred length in total + current transfer length at least total length,
			urb->transfer_buffer_length					//which means URB is completed.
			|| channel->actual_len < qh->maxpacket		//Transferred bytes less than packet size? Short packet meaning done?
			|| channel->rx_packet_done);
	}

	/* send IN token for next packet, without AUTOREQ */
	if (!done) {														//Isochronous transfer not complete.
		val = musb_readw(epio, MUSB_RXCSR);
		val |= MUSB_RXCSR_H_REQPKT;	//The CPU writes a 1 to this bit to request an IN transaction. It is cleared when RxPktRdy is set.
		musb_writew(epio, MUSB_RXCSR, MUSB_RXCSR_H_WZC_BITS | val);	//Sets RxStall, Error, DataError, RxPktRdy, but should be cleared.
	}

	return done;
}

/* Disadvantage of using mode 1:
 *	It's basically usable only for mass storage class; essentially all
 *	other protocols also terminate transfers on short packets.
 *
 * Details:
 *	An extra IN token is sent at the end of the transfer (due to AUTOREQ)
 *	If you try to use mode 1 for (transfer_buffer_length - 512), and try
 *	to use the extra IN token to grab the last packet using mode 0, then
 *	the problem is that you cannot be sure when the device will send the
 *	last packet and RxPktRdy set. Sometimes the packet is recd too soon
 *	such that it gets lost when RxCSR is re-set at the end of the mode 1
 *	transfer, while sometimes it is recd just a little late so that if you
 *	try to configure for mode 0 soon after the mode 1 transfer is
 *	completed, you will find rxcount 0. Okay, so you might think why not
 *	wait for an interrupt when the pkt is recd. Well, you won't get any!
 */
//Configures DMA and endpoint for a new receive transfer.
static int musb_rx_dma_in_inventra_cppi41(struct dma_controller *dma,
					  struct musb_hw_ep *hw_ep,
					  struct musb_qh *qh,
					  struct urb *urb,
					  size_t len,
					  u8 iso_err)
{
//printk("drivers/usb/musb/musb_host.c:musb_rx_dma_in_inventra_cppi41\n");
	struct musb *musb = hw_ep->musb;										//MUSB0/1 data structure of endpoint.
	void __iomem *epio = hw_ep->regs;										//Undocumented MUSB0/1 registers for this endpoint.
	struct dma_channel *channel = hw_ep->rx_channel;						//MUSB DMA channel data structure of endpoint.
	u16 rx_count, val;
	int length, pipe, done;
	dma_addr_t buf;

	//"RxCount is a 13-bit read-only register that holds the number of data
	// bytes in the packet currently in line to be read from the Rx FIFO."
	rx_count = musb_readw(epio, MUSB_RXCOUNT);
	pipe = urb->pipe;														//Transfer type.

	if (usb_pipeisoc(pipe)) {												//Isochronous transfer.
		int d_status = 0;
		struct usb_iso_packet_descriptor *d;

		d = urb->iso_frame_desc + qh->iso_idx;								//Isochronous packet descriptor.

		if (iso_err) {														//Error: CRC or bit-stuff error.
			d_status = -EILSEQ;												//Illegal byte sequence.
			urb->error_count++;
		}
		if (rx_count > d->length) {										//Number of received bytes greater than expected length/size?
			if (d_status == 0) {
				d_status = -EOVERFLOW;									//"Value too large for defined data type"
				urb->error_count++;
			}
			musb_dbg(musb, "** OVERFLOW %d into %d",
				rx_count, d->length);

			length = d->length;											//Received length is buffer size.
		} else
			length = rx_count;											//Received length.
		d->status = d_status;											//Update status with potential error code.
		//DMA buffer start address + offset from start buffer address is next
		//transfer start address?
		buf = urb->transfer_dma + d->offset;
	} else {
		length = rx_count;												//Received length.
		buf = urb->transfer_dma + urb->actual_length;	//DMA buffer start address + transferred bytes is next transfer start address?
	}

	//"When operating in DMA Mode 0, the DMA controller can be only programmed
	// to load/unload one packet"
	//"When operating in DMA Mode 1, the DMA controller can be programmed to
	// load/unload a complete bulk transfer (which can be many packets)"
	channel->desired_mode = 0;									//DMA mode 0 preferred.
#ifdef USE_MODE1
	/* because of the issue below, mode 1 will
	 * only rarely behave with correct semantics.
	 */
	if ((urb->transfer_flags & URB_SHORT_NOT_OK)				//"report short reads as errors"
	    && (urb->transfer_buffer_length - urb->actual_length)	//Bytes remaining to receive is more than packet size?
	    > qh->maxpacket)
		channel->desired_mode = 1;								//DMA mode 1 since multiple packets are needed.
	if (rx_count < hw_ep->max_packet_sz_rx) {					//Number of read bytes less than endpoint FIFO size?
		length = rx_count;										//Received length.
		channel->desired_mode = 0;								//DMA mode 0 since transfer can be made with one packet?
	} else {
		length = urb->transfer_buffer_length;					//Received length is eqal to buffer length.
	}
#endif

	/* See comments above on disadvantages of using mode 1 */
	val = musb_readw(epio, MUSB_RXCSR);
	val &= ~MUSB_RXCSR_H_REQPKT;							//CPU not requsting in transaction.

	if (channel->desired_mode == 0)							//DMA mode 0 .
		val &= ~MUSB_RXCSR_H_AUTOREQ;						//ReqPkt bit will not be automatically set when the RxPktRdy bit is cleared.
	else													//DMA mode 1 (many packets).
		val |= MUSB_RXCSR_H_AUTOREQ;						//ReqPkt bit will be automatically set when the RxPktRdy bit is cleared.
	val |= MUSB_RXCSR_DMAENAB;								//"The CPU sets this bit to enable the DMA request for the Rx endpoint."

	/* autoclear shouldn't be set in high bandwidth */
	if (qh->hb_mult == 1)
		//"If the CPU sets this bit then the RxPktRdy bit will be automatically
		// cleared when a packet of RxMaxP bytes has been unloaded from the Rx
		// FIFO."
		val |= MUSB_RXCSR_AUTOCLEAR;	//Is set but should not be set?

	musb_writew(epio, MUSB_RXCSR, MUSB_RXCSR_H_WZC_BITS | val);	//Sets RxStall, Error, DataError, RxPktRdy, but should be cleared?

	/* REVISIT if when actual_length != 0,
	 * transfer_buffer_length needs to be
	 * adjusted first...
	 */
	//Calls cppi41_dma_channel_program, which sets memory transfers parameters
	//for DMA channel and callback function, and returns indication of
	//successful or not.
	done = dma->channel_program(channel, qh->maxpacket,
				   channel->desired_mode,
				   buf, length);

	if (!done) {						//Not successful.
		dma->channel_release(channel);	//Calls cppi41_dma_channel_release: marks the structure as free for use.
		hw_ep->rx_channel = NULL;		//Removes MUSB RX DMA channel data structure.
		channel = NULL;
		val = musb_readw(epio, MUSB_RXCSR);
		val &= ~(MUSB_RXCSR_DMAENAB		//Clear DMA request enable: Endpoint cannot request DMA.
			 | MUSB_RXCSR_H_AUTOREQ		//Clear Auto request: An IN transaction not requested automatically when RxPktRdy is cleared.
			 | MUSB_RXCSR_AUTOCLEAR);	//Clear Auto clear: RxPktRdy not cleared automatically when a maximum sized packet is unloaded.
		musb_writew(epio, MUSB_RXCSR, val);
	}

	return done;						//True if transfer of endpoint is complete. False otherwise.
}
#else
static inline int musb_rx_dma_inventra_cppi41(struct dma_controller *dma,
					      struct musb_hw_ep *hw_ep,
					      struct musb_qh *qh,
					      struct urb *urb,
					      size_t len)
{
//printk("drivers/usb/musb/musb_host.c:musb_rx_dma_inventra_cppi41\n");
	return false;
}

static inline int musb_rx_dma_in_inventra_cppi41(struct dma_controller *dma,
						 struct musb_hw_ep *hw_ep,
						 struct musb_qh *qh,
						 struct urb *urb,
						 size_t len,
						 u8 iso_err)
{
//printk("drivers/usb/musb/musb_host.c:musb_rx_dma_in_inventra_cppi41\n");
	return false;
}
#endif

/*
 * Service an RX interrupt for the given IN endpoint; docs cover bulk, iso,
 * and high-bandwidth IN transfer cases.
 */
//If last transfer, stops current transaction, updates DMA channel data
//structures and schedules the next transaction URB. If fault, abort DMA.
void musb_host_rx(struct musb *musb, u8 epnum)
{
//printk("drivers/usb/musb/musb_host.c:musb_host_rx\n");
	struct urb		*urb;
	struct musb_hw_ep	*hw_ep = musb->endpoints + epnum;			//Hardware endpoint data structure.
	struct dma_controller	*c = musb->dma_controller;				//DMA controller data structure of MUSB0/1.
	void __iomem		*epio = hw_ep->regs;						//Hardware endpoint undocumented registers.
	struct musb_qh		*qh = hw_ep->in_qh;							//URBs associated with endpoint.
	size_t			xfer_len;
	void __iomem		*mbase = musb->mregs;						//Undocumented registers of USB0/1.
	u16			rx_csr, val;
	bool			iso_err = false;
	bool			done = false;
	u32			status;
	struct dma_channel	*dma;
	unsigned int sg_flags = SG_MITER_ATOMIC | SG_MITER_TO_SG;		//"se kmap_atomic" and "flush back to phys on unmap"

	musb_ep_select(mbase, epnum);									//Select register bank for endpoint for following register accesses.

	urb = next_urb(qh);												//Next URB of this endpoint.
	dma = is_dma_capable() ? hw_ep->rx_channel : NULL;				//Get RX DMA channel data structure.
	status = 0;														//No error by default.
	xfer_len = 0;

	rx_csr = musb_readw(epio, MUSB_RXCSR);							//Read control status register of current endpoint.
	val = rx_csr;

	if (unlikely(!urb)) {											//No URB.
		/* REVISIT -- THIS SHOULD NEVER HAPPEN ... but, at least
		 * usbtest #11 (unlinks) triggers it regularly, sometimes
		 * with fifo full.  (Only with DMA??)
		 */
		musb_dbg(musb, "BOGUS RX%d ready, csr %04x, count %d",
			epnum, val, musb_readw(epio, MUSB_RXCOUNT));
		musb_h_flush_rxfifo(hw_ep, MUSB_RXCSR_CLRDATATOG);			//Flush/Empty FIFO to memory.
		return;
	}

	trace_musb_urb_rx(musb, urb);

	/* check for errors, concurrent stall & unlink is not really
	 * handled yet! */
	if (rx_csr & MUSB_RXCSR_H_RXSTALL) {							//Wait for peripheral since it says stall?
		musb_dbg(musb, "RX end %d STALL", epnum);

		/* stall; record URB status */
		status = -EPIPE;

	} else if (rx_csr & MUSB_RXCSR_H_ERROR) {						//No packet received for 3 attempts.
		dev_err(musb->controller, "ep%d RX three-strikes error", epnum);

		/*
		 * The three-strikes error could only happen when the USB
		 * device is not accessible, for example detached or powered
		 * off. So return the fatal error -ESHUTDOWN so hopefully the
		 * USB device drivers won't immediately resubmit the same URB.
		 */
		status = -ESHUTDOWN;
		musb_writeb(epio, MUSB_RXINTERVAL, 0);						//Disable NAK (for bulk) and polling (isochronous/interrupt).

		rx_csr &= ~MUSB_RXCSR_H_ERROR;			//Clear error which USB controller sets after three attempts of receiving a USB packet.
		musb_writew(epio, MUSB_RXCSR, rx_csr);

	} else if (rx_csr & MUSB_RXCSR_DATAERROR) {
		//"When operating in ISO mode, this bit is set when RxPktRdy is set if
		// the data packet has a CRC or bit-stuff error."
		//"In Bulk mode, this bit will be set when the Rx endpoint is halted
		// following the receipt of NAK responses for longer than the time set
		// as the NAK Limit by the RxInterval register."

		if (USB_ENDPOINT_XFER_ISOC != qh->type) {					//Not isochronous transfer, meaning NAK caused data error.
			musb_dbg(musb, "RX end %d NAK timeout", epnum);

			/* NOTE: NAKing is *NOT* an error, so we want to
			 * continue.  Except ... if there's a request for
			 * another QH, use that instead of starving it.
			 *
			 * Devices like Ethernet and serial adapters keep
			 * reads posted at all times, which will starve
			 * other devices without this logic.
			 */
			if (usb_pipebulk(urb->pipe)								//Bulk transfer
					&& qh->mux == 1									//URB multiplexed on multiple endpoints?
					&& !list_is_singular(&musb->in_bulk)) {			//Not one entry in input URB list of MUSB0/1.
				//Stops current transaction, updates DMA channel data structures
				//and schedules the next transaction URB.
				musb_bulk_nak_timeout(musb, hw_ep, 1);
				return;
			}
			musb_ep_select(mbase, epnum);							//Select endpoint control status register bank.
			rx_csr |= MUSB_RXCSR_H_WZC_BITS;			//Sets RxStall, Error, DataError, RxPktReady, but all of them should be cleared.
			rx_csr &= ~MUSB_RXCSR_DATAERROR;						//"The CPU should clear this bit to allow the endpoint to continue."
			musb_writew(epio, MUSB_RXCSR, rx_csr);

			goto finish;
		} else {
			musb_dbg(musb, "RX end %d ISO data error", epnum);
			/* packet error reported later */
			iso_err = true;											//Isochronous transfer, meaning CRC or bit-stuff error.
		}
	} else if (rx_csr & MUSB_RXCSR_INCOMPRX) {
		//"This bit will be set in a high-bandwidth Isochronous or Interrupt
		// transfer if the packet received is incomplete."
		musb_dbg(musb, "end %d high bandwidth incomplete ISO packet RX",
				epnum);
		status = -EPROTO;
	}

	/* faults abort the transfer */
	if (status) {													//If error...
		/* clean up dma and collect transfer count */
		if (dma_channel_status(dma) == MUSB_DMA_STATUS_BUSY) {		//DMA channel structure records transactions in progress.
			dma->status = MUSB_DMA_STATUS_CORE_ABORT;				//Abort DMA transaction "due to ... dma or memory bus error".
			//Calls cppi41_dma_channel_abort, which aborts transfers of the
			//given DMA channel and marks it as free for use for another
			//transfer and endpoint.
			musb->dma_controller->channel_abort(dma);
			xfer_len = dma->actual_len;								//Transferred bytes of this DMA channel.
		}
		//Flush/empty RX FIFO, with CSR initialized to MUSB_RXCSR_CLRDATATOG which resets endpoint data toggle to 0.
		musb_h_flush_rxfifo(hw_ep, MUSB_RXCSR_CLRDATATOG);
		musb_writeb(epio, MUSB_RXINTERVAL, 0);						//Disable NAK for BULK, and interval polling for isochronous.
		done = true;												//Done.
		goto finish;
	}

	if (unlikely(dma_channel_status(dma) == MUSB_DMA_STATUS_BUSY)) {	//Transactions of this DMA channel is in progress.
		/* SHOULD NEVER HAPPEN ... but at least DaVinci has done it */
		ERR("RX%d dma busy, csr %04x\n", epnum, rx_csr);
		goto finish;
	}

	/* thorough shutdown for now ... given more precise fault handling
	 * and better queueing support, we might keep a DMA pipeline going
	 * while processing this irq for earlier completions.
	 */

	/* FIXME this is _way_ too much in-line logic for Mentor DMA */
	if (!musb_dma_inventra(musb) && !musb_dma_ux500(musb) &&			//True.
	    (rx_csr & MUSB_RXCSR_H_REQPKT)) {								//The CPU writes a 1 to this bit to request an IN transaction.
		/* REVISIT this happened for a while on some short reads...
		 * the cleanup still needs investigation... looks bad...
		 * and also duplicates dma cleanup code above ... plus,
		 * shouldn't this be the "half full" double buffer case?
		 */
		if (dma_channel_status(dma) == MUSB_DMA_STATUS_BUSY) {			//DMA channel structure records transactions in progress.
			dma->status = MUSB_DMA_STATUS_CORE_ABORT;					//Abort DMA transaction "due to ... dma or memory bus error".
			//Calls cppi41_dma_channel_abort, which aborts transfers of the
			//given DMA channel and marks it as free for use for another
			//transfer and endpoint.
			musb->dma_controller->channel_abort(dma);
			xfer_len = dma->actual_len;									//Transferred bytes of this DMA channel.
			done = true;
		}

		musb_dbg(musb, "RXCSR%d %04x, reqpkt, len %zu%s", epnum, rx_csr,
				xfer_len, dma ? ", dma" : "");
		rx_csr &= ~MUSB_RXCSR_H_REQPKT;						//Not requesting in transaction.

		musb_ep_select(mbase, epnum);						//Select current endpoint register bank.
		musb_writew(epio, MUSB_RXCSR,
				MUSB_RXCSR_H_WZC_BITS | rx_csr);		//Sets RxStall, Error, DataError, RxPktReady, but all of them should be cleared.
	}

	if (dma && (rx_csr & MUSB_RXCSR_DMAENAB)) {				//The CPU sets this bit to enable the DMA request for the Rx endpoint.
		xfer_len = dma->actual_len;							//Transferred bytes of this DMA channel.

		val &= ~(MUSB_RXCSR_DMAENAB							//Disabling DMA request for RX endpoint.
			| MUSB_RXCSR_H_AUTOREQ							//ReqPkt bit will not be automatically set when the RxPktRdy bit is cleared.
			| MUSB_RXCSR_AUTOCLEAR	//RxPktRdy bit will not be cleared when a packet of RxMaxP bytes has been unloaded from the Rx FIFO.
			| MUSB_RXCSR_RXPKTRDY);	//No packet received.
		musb_writew(hw_ep->regs, MUSB_RXCSR, val);			//Update control status register.

		if (musb_dma_inventra(musb) || musb_dma_ux500(musb) ||
		    musb_dma_cppi41(musb)) {						//TRUE.
			    done = musb_rx_dma_inventra_cppi41(c, hw_ep, qh, urb, xfer_len);	//Configures next transfer if not done.
			    musb_dbg(hw_ep->musb,
				    "ep %d dma %s, rxcsr %04x, rxcount %d",
				    epnum, done ? "off" : "reset",
				    musb_readw(epio, MUSB_RXCSR),
				    musb_readw(epio, MUSB_RXCOUNT));
		} else {											//FALSE.
			done = true;
		}

	} else if (urb->status == -EINPROGRESS) {				//URB transfer in progress.
		/* if no errors, be sure a packet is ready for unloading */
		if (unlikely(!(rx_csr & MUSB_RXCSR_RXPKTRDY))) {							//No received packet for reading.
			status = -EPROTO;
			ERR("Rx interrupt with no errors or packet!\n");

			/* FIXME this is another "SHOULD NEVER HAPPEN" */

/* SCRUB (RX) */
			/* do the proper sequence to abort the transfer */
			musb_ep_select(mbase, epnum);							//Set register bank to control status register for current endpoint.
			val &= ~MUSB_RXCSR_H_REQPKT;							//Does not request an in transfer.
			musb_writew(epio, MUSB_RXCSR, val);						//Updates control status register.
			goto finish;
		}

		/* we are expecting IN packets */
		if ((musb_dma_inventra(musb) || musb_dma_ux500(musb) ||
		    musb_dma_cppi41(musb)) && dma) {						//If DMA.
			musb_dbg(hw_ep->musb,
				"RX%d count %d, buffer 0x%llx len %d/%d",
				epnum, musb_readw(epio, MUSB_RXCOUNT),
				(unsigned long long) urb->transfer_dma
				+ urb->actual_length,
				qh->offset,
				urb->transfer_buffer_length);
			//Configures DMA and endpoint for a new receive transfer. Returns true for success and false otherwise.
			if (musb_rx_dma_in_inventra_cppi41(c, hw_ep, qh, urb,
							   xfer_len, iso_err))
				goto finish;
			else
				dev_err(musb->controller, "error: rx_dma failed\n");
		}

		if (!dma) {											//Has no DMA
			unsigned int received_len;

			/* Unmap the buffer so that CPU can use it */
			usb_hcd_unmap_urb_for_dma(musb->hcd, urb);		//Host controller driver unmaps memory buffer of URB?

			/*
			 * We need to map sg if the transfer_buffer is
			 * NULL.
			 */
			if (!urb->transfer_buffer) {					//If memory data buffer is unmapped?
				qh->use_sg = true;							//Uses Linux kernel scatter gather feature.
				sg_miter_start(&qh->sg_miter, urb->sg, 1,	//Start mapping iteration over a sg list with 1 entry.
						sg_flags);
			}

			if (qh->use_sg) {								//Uses scatter gather from Linux?
				if (!sg_miter_next(&qh->sg_miter)) {		//Mapping iterator goes to next mapping. Returns false if no next mapping.
					dev_err(musb->controller, "error: sg list empty\n");
					sg_miter_stop(&qh->sg_miter);			//stop mapping iteration.
					status = -EINVAL;						//"Invalid argument"
					done = true;							//Transfer is done since no more mappings.
					goto finish;
				}
				urb->transfer_buffer = qh->sg_miter.addr;	//"pointer to the mapped area"
				received_len = urb->actual_length;			//Number of transferred/received bytes.
				qh->offset = 0x0;							//qh->offset is pointer to memory from where to start read data?
				done = musb_host_packet_rx(musb, urb, epnum,	//Updates URB and endpoint buffer parameters after receiving a packet, 
						iso_err);								//and request next packet if not complete.
				/* Calculate the number of bytes received */
				received_len = urb->actual_length -		//Number of transferred bytes - previously transferred bytes is received bytes.
					received_len;
				qh->sg_miter.consumed = received_len;	//Number of transferred/consumed bytes of SG buffer.
				sg_miter_stop(&qh->sg_miter);			//stop mapping iteration.
			} else {									//Does not use scatter-gather from Linux? Does everything internally?
				done = musb_host_packet_rx(musb, urb,	//Updates URB and endpoint buffer parameters after receiving a packet, 
						epnum, iso_err);				//and request next packet if not complete.
			}
			musb_dbg(musb, "read %spacket", done ? "last " : "");
		}
	}

finish:
	urb->actual_length += xfer_len;			//Total number of transferred bytes incremented by transferred bytes of this DMA channel.
	qh->offset += xfer_len;					//Incremented offset from start of buffer by number of received bytes.
	if (done) {								//If done, then
		if (qh->use_sg) {					//If using scatter-gatter
			qh->use_sg = false;				//stop using it, and
			urb->transfer_buffer = NULL;	//start buffer address set to null since no buffer is used anymore.
		}

		if (urb->status == -EINPROGRESS)	//If error status is in progress, then update it.
			urb->status = status;
		//Complete the processing of the given URB(?), and setup transfer for
		//the next urb associated with the given hardware endpoint.
		musb_advance_schedule(musb, urb, hw_ep, USB_DIR_IN);
	}
}

/* schedule nodes correspond to peripheral endpoints, like an OHCI QH.
 * the software schedule associates multiple such nodes with a given
 * host side hardware endpoint + direction; scheduling may activate
 * that hardware endpoint.
 */
//Selects hardware endpoint for qh, and configures the endpoint to perform the
//transfer.
static int musb_schedule(
	struct musb		*musb,
	struct musb_qh		*qh,
	int			is_in)
{
//printk("drivers/usb/musb/musb_host.c:musb_schedule\n");
	int			idle = 0;							//qh not idle by default.
	int			best_diff;
	int			best_end, epnum;
	struct musb_hw_ep	*hw_ep = NULL;
	struct list_head	*head = NULL;
	u8			toggle;
	u8			txtype;
	struct urb		*urb = next_urb(qh);			//Next URB of qh. Each URB gets an allocated qh (see musb_urb_enqueue()).

	/* use fixed hardware for control and bulk */
	if (qh->type == USB_ENDPOINT_XFER_CONTROL) {	//Control transfer.
		head = &musb->control;						//MUSB qh (allocated in musb_urb_enqueue()).
		hw_ep = musb->control_ep;					//Gets hardware endpoint structure of endpoint 0.
		goto success;
	}

	/* else, periodic transfers get muxed to other endpoints */

	/*
	 * We know this qh hasn't been scheduled, so all we need to do
	 * is choose which hardware endpoint to put it on ...
	 *
	 * REVISIT what we really want here is a regular schedule tree
	 * like e.g. OHCI uses.
	 */
	best_diff = 4096;										//Unused bytes of max packet size of endpoint and URB?
	best_end = -1;											//No best endpoint found yet.

	for (epnum = 1, hw_ep = musb->endpoints + 1;
			epnum < musb->nr_endpoints;
			epnum++, hw_ep++) {
		int	diff;

		if (musb_ep_get_qh(hw_ep, is_in) != NULL)			//Endpoint has URB, consider next endpoint.
			continue;

		if (hw_ep == musb->bulk_ep)							//Endpoint is for bulk transfers, consider next endpoint.
			continue;

		if (is_in)											//Input is reception.
			diff = hw_ep->max_packet_sz_rx;
		else
			diff = hw_ep->max_packet_sz_tx;					//Output is transmission.
		diff -= (qh->maxpacket * qh->hb_mult);				//Endpoint maximum packet size times "high bandwidth pkts per uf"; remaining size?

		if (diff >= 0 && best_diff > diff) {				//No size limit exceed and remaining size less than 4096?

			/*
			 * Mentor controller has a bug in that if we schedule
			 * a BULK Tx transfer on an endpoint that had earlier
			 * handled ISOC then the BULK transfer has to start on
			 * a zero toggle.  If the BULK transfer starts on a 1
			 * toggle then this transfer will fail as the mentor
			 * controller starts the Bulk transfer on a 0 toggle
			 * irrespective of the programming of the toggle bits
			 * in the TXCSR register.  Check for this condition
			 * while allocating the EP for a Tx Bulk transfer.  If
			 * so skip this EP.
			 */
			hw_ep = musb->endpoints + epnum;						//Hardware endpoint structure of current endpoint.
			//Toggles of data packets flip bits for synchronization?
			//"An alternating DATA0/DATA1 is used as a part of the error control
			// protocol to (or from) a particular endpoint."
			toggle = usb_gettoggle(urb->dev, qh->epnum, !is_in);
			//"TxType is an 8-bit register that should be written with the
			// endpoint number to be targeted by the endpoint, the transaction
			// protocol to use for the currently-selected TX endpoint, and its
			// operating speed."
			txtype = (musb_readb(hw_ep->regs, MUSB_TXTYPE)	//Protocol currently in use of endpoint: Control, isochronous, bulk or interrupt.
					>> 4) & 0x3;
			//Queue head wants output bulk transfer and shall toggle, and
			//endpoint is in isochronous transfer mode.
			if (!is_in && (qh->type == USB_ENDPOINT_XFER_BULK) &&
				toggle && (txtype == USB_ENDPOINT_XFER_ISOC))
				continue;

			best_diff = diff;	//Update minimum redundant size.
			best_end = epnum;	//Record best endpoint number.
		}
	}
	/* use bulk reserved ep1 if no other ep is free */
	if (best_end < 0 && qh->type == USB_ENDPOINT_XFER_BULK) {				//No endpoint found and URB transfer is bulk.
		hw_ep = musb->bulk_ep;												//Bulk endpoint.
		if (is_in)															//Input transfer to host.
			head = &musb->in_bulk;											//Get first input bulk.
		else																//Output transfer from host.
			head = &musb->out_bulk;											//Get first output bulk.

		/* Enable bulk RX/TX NAK timeout scheme when bulk requests are
		 * multiplexed. This scheme does not work in high speed to full
		 * speed scenario as NAK interrupts are not coming from a
		 * full speed device connected to a high speed device.
		 * NAK timeout interval is 8 (128 uframe or 16ms) for HS and
		 * 4 (8 frame or 8ms) for FS device.
		 */
		if (qh->dev)
			qh->intv_reg =
				(USB_SPEED_HIGH == qh->dev->speed) ? 8 : 4;	//"polling interval for interrupt and isochronous endpoints", and timeout for bulk
		goto success;
	} else if (best_end < 0) {
		dev_err(musb->controller,
				"%s hwep alloc failed for %dx%d\n",
				musb_ep_xfertype_string(qh->type),
				qh->hb_mult, qh->maxpacket);
		return -ENOSPC;
	}

	idle = 1;												//Assume qh is idle.
	qh->mux = 0;											//qh not multiplexed to hw_ep; associated URB not split among multiple endpoints?
	hw_ep = musb->endpoints + best_end;						//Selected hardware endpoint.
	musb_dbg(musb, "qh %p periodic slot %d", qh, best_end);
success:
	if (head) {												//MUSB qh selected.
		idle = list_empty(head);							//Is qh list empty?
		list_add_tail(&qh->ring, head);						//Add qh ring to tail of MUSB qh.
		qh->mux = 1;										//qh multiplexed with other qh (to the same endpoint)?
	}
	qh->hw_ep = hw_ep;										//Set hardware endpoint of qh.
	qh->hep->hcpriv = qh;									//Host controller private date of usb_host_endpoint set to qh.
	if (idle)							//If qh/ep is idle(?), configures endpoint associated with qh to transfer the URB associated with qh.
		musb_start_urb(musb, is_in, qh);
	return 0;
}

//Enqueue I/O request.
//
//Adds an URB to its endpoint queue via the Linux kernel, allocates a qh (queue
//head) for the URB, finds a suitable endpoint for the qh and configures the
//endpoint to perform the transfer.
static int musb_urb_enqueue(
	struct usb_hcd			*hcd,
	struct urb			*urb,
	gfp_t				mem_flags)
{
//printk("drivers/usb/musb/musb_host.c:musb_urb_enqueue\n");
	unsigned long			flags;
	struct musb			*musb = hcd_to_musb(hcd);
	struct usb_host_endpoint	*hep = urb->ep;			//"host-side endpoint descriptor and queue"
	struct musb_qh			*qh;
	struct usb_endpoint_descriptor	*epd = &hep->desc;	//"descriptor for this endpoint, wMaxPacketSize in native byteorder"
	int				ret;
	unsigned			type_reg;
	unsigned			interval;

	/* host role must be active */
	if (!is_host_active(musb) || !musb->is_active)
		return -ENODEV;

	trace_musb_urb_enq(musb, urb);

	spin_lock_irqsave(&musb->lock, flags);
	ret = usb_hcd_link_urb_to_ep(hcd, urb);				//"add an URB to its endpoint queue"; returns 0 for non-error.
	//"for use by HCD; typically holds hardware dma queue head (QH) with one or more transfer descriptors (TDs) per urb"
	//Means a queue (DMA queue "heads") of queues (BDs)?
	qh = ret ? NULL : hep->hcpriv;
	if (qh)
		urb->hcpriv = qh;
	spin_unlock_irqrestore(&musb->lock, flags);

	/* DMA mapping was already done, if needed, and this urb is on
	 * hep->urb_list now ... so we're done, unless hep wasn't yet
	 * scheduled onto a live qh.
	 *
	 * REVISIT best to keep hep->hcpriv valid until the endpoint gets
	 * disabled, testing for empty qh->ring and avoiding qh setup costs
	 * except for the first urb queued after a config change.
	 */
	if (qh || ret)	//Queue head or failing adding URB to endpoint queue.
		return ret;

	/* Allocate and initialize qh, minimizing the work done each time
	 * hw_ep gets reprogrammed, or with irqs blocked.  Then schedule it.
	 *
	 * REVISIT consider a dedicated qh kmem_cache, so it's harder
	 * for bugs in other kernel code to break this driver...
	 */
	qh = kzalloc(sizeof *qh, mem_flags);				//Allocates a queue head.
	if (!qh) {											//Failed allocation.
		spin_lock_irqsave(&musb->lock, flags);
		usb_hcd_unlink_urb_from_ep(hcd, urb);			//"remove an URB from its endpoint queue"
		spin_unlock_irqrestore(&musb->lock, flags);
		return -ENOMEM;
	}

	qh->hep = hep;										//Initializes host endpoint for this URB.
	qh->dev = urb->dev;									//"(in) pointer to associated [USB] device"
	INIT_LIST_HEAD(&qh->ring);							//Initializes the ring field to an empty list.
	qh->is_ready = 1;									//"safe to modify hw_ep"

	qh->maxpacket = usb_endpoint_maxp(epd);				//"get endpoint's max packet size"
	qh->type = usb_endpoint_type(epd);					//"get the endpoint's transfer type"; Control, isochronous, bulk, interrupt.

	/* Bits 11 & 12 of wMaxPacketSize encode high bandwidth multiplier.
	 * Some musb cores don't support high bandwidth ISO transfers; and
	 * we don't (yet!) support high bandwidth interrupt transfers.
	 */
	qh->hb_mult = usb_endpoint_maxp_mult(epd);			//"get endpoint's transactional opportunities"
	if (qh->hb_mult > 1) {								//qh->hb_mult > 1 ==> high-speed isochronous?
		int ok = (qh->type == USB_ENDPOINT_XFER_ISOC);

		if (ok)													//Isochronous (periodic real-time) transfer.
			ok = (usb_pipein(urb->pipe) && musb->hb_iso_rx)		//"high bandwidth iso rx"
				|| (usb_pipeout(urb->pipe) && musb->hb_iso_tx);	//"high bandwidth iso tx""
		if (!ok) {												//Not isochronous transfer.
			dev_err(musb->controller,
				"high bandwidth %s (%dx%d) not supported\n",
				musb_ep_xfertype_string(qh->type),
				qh->hb_mult, qh->maxpacket & 0x7ff);
			ret = -EMSGSIZE;
			goto done;
		}
		qh->maxpacket &= 0x7ff;									//Maximum 11 bits = 2047 bytes.
	}

	qh->epnum = usb_endpoint_num(epd);							//"get the endpoint's number

	/* NOTE: urb->dev->devnum is wrong during SET_ADDRESS */
	qh->addr_reg = (u8) usb_pipedevice(urb->pipe);				//"device address register"; urb->pipe = "(in) pipe information"

	/* precompute rxtype/txtype/type0 register */
	type_reg = (qh->type << 4) | qh->epnum;						//transfer type (isochronous, bulk, interrupt, control), and endpoint number.
	switch (urb->dev->speed) {									//"device speed: high/full/low (or error)"
	case USB_SPEED_LOW:
		type_reg |= 0xc0;
		break;
	case USB_SPEED_FULL:
		type_reg |= 0x80;
		break;
	default:
		type_reg |= 0x40;
	}
	qh->type_reg = type_reg;									//"{rx,tx} type register"

	/* Precompute RXINTERVAL/TXINTERVAL register */
	switch (qh->type) {											//Transfer type (interrupt, isochronous, bulk, control).
	case USB_ENDPOINT_XFER_INT:
		/*
		 * Full/low speeds use the  linear encoding,
		 * high speed uses the logarithmic encoding.
		 */
		if (urb->dev->speed <= USB_SPEED_FULL) {
			interval = max_t(u8, epd->bInterval, 1);			//"bInterval is the polling interval for interrupt and isochronous endpoints."
			break;
		}
		fallthrough;
	case USB_ENDPOINT_XFER_ISOC:
		/* ISO always uses logarithmic encoding */
		interval = min_t(u8, epd->bInterval, 16);				//"bInterval is the polling interval for interrupt and isochronous endpoints."
		break;
	default:
		/* REVISIT we actually want to use NAK limits, hinting to the
		 * transfer scheduling logic to try some other qh, e.g. try
		 * for 2 msec first:
		 *
		 * interval = (USB_SPEED_HIGH == urb->dev->speed) ? 16 : 2;
		 *
		 * The downside of disabling this is that transfer scheduling
		 * gets VERY unfair for nonperiodic transfers; a misbehaving
		 * peripheral could make that hurt.  That's perfectly normal
		 * for reads from network or serial adapters ... so we have
		 * partial NAKlimit support for bulk RX.
		 *
		 * The upside of disabling it is simpler transfer scheduling.
		 */
		interval = 0;
	}
	//"The polling interval depends on the type of host controller and the speed
	// of the device which then determine the frequency that the driver should
	// initiate an isochronous transfer or interrupt. bInterval is not
	// represented as a fixed amount of time. The value of bInterval is an
	// estimate and depends on whether the USB host controller/device is
	// operating in low, full, or high speed."
	qh->intv_reg = interval;

	/* precompute addressing for external hub/tt ports */
	if (musb->is_multipoint) {									//FALSE
		struct usb_device	*parent = urb->dev->parent;

		if (parent != hcd->self.root_hub) {
			qh->h_addr_reg = (u8) parent->devnum;

			/* set up tt info if needed */
			if (urb->dev->tt) {
				qh->h_port_reg = (u8) urb->dev->ttport;
				if (urb->dev->tt->hub)
					qh->h_addr_reg =
						(u8) urb->dev->tt->hub->devnum;
				if (urb->dev->tt->multi)
					qh->h_addr_reg |= 0x80;
			}
		}
	}

	/* invariant: hep->hcpriv is null OR the qh that's already scheduled.
	 * until we get real dma queues (with an entry for each urb/buffer),
	 * we only have work to do in the former case.
	 */
	spin_lock_irqsave(&musb->lock, flags);
	if (hep->hcpriv || !next_urb(qh)) {	//There is a MUSB queue head or qh has no additional URB?
		/* some concurrent activity submitted another urb to hep...
		 * odd, rare, error prone, but legal.
		 */
		kfree(qh);						//Free the queue head.
		qh = NULL;
		ret = 0;
	} else								//No MUSB queue head and qh has additional URB?
		//Selects hardware endpoint for qh, and configures the endpoint to
		//perform the transfer.
		ret = musb_schedule(musb, qh,
				epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK);

	if (ret == 0) {
		urb->hcpriv = qh;
		/* FIXME set urb->start_frame for iso/intr, it's tested in
		 * musb_start_urb(), but otherwise only konicawc cares ...
		 */
	}
	spin_unlock_irqrestore(&musb->lock, flags);

done:
	if (ret != 0) {										//If failure.
		spin_lock_irqsave(&musb->lock, flags);
		usb_hcd_unlink_urb_from_ep(hcd, urb);			//"remove an URB from its endpoint queue"
		spin_unlock_irqrestore(&musb->lock, flags);
		kfree(qh);										//Frees qh.
	}
	return ret;
}


/*
 * abort a transfer that's at the head of a hardware queue.
 * called with controller locked, irqs blocked
 * that hardware queue advances to the next transfer, unless prevented
 */
//Aborts DMA transfer of endpoint and marks the DMA channel data structure as
//free for use again, updates number of transferred bytes of URB, clears
//interrupt status of endpoint, flushes the MUSB endpoint FIFO, and completes
//the processing of the given URB(?), and setups transfer for the next urb
//associated with the given hardware endpoint.
static int musb_cleanup_urb(struct urb *urb, struct musb_qh *qh)
{
//printk("drivers/usb/musb/musb_host.c:musb_cleanup_urb\n");
	struct musb_hw_ep	*ep = qh->hw_ep;
	struct musb		*musb = ep->musb;
	void __iomem		*epio = ep->regs;
	unsigned		hw_end = ep->epnum;
	void __iomem		*regs = ep->musb->mregs;
	int			is_in = usb_pipein(urb->pipe);						//Transfer direction of URB.
	int			status = 0;
	u16			csr;
	struct dma_channel	*dma = NULL;

	musb_ep_select(regs, hw_end);									//Configures indexed control status register of MUSB for endpoint of qh.

	if (is_dma_capable()) {											//TRUE
		dma = is_in ? ep->rx_channel : ep->tx_channel;				//MUSB DMA channel structure for reception/transmission.
		if (dma) {													//If there is a channel structure, calls cppi41_dma_channel_abort, which
			status = ep->musb->dma_controller->channel_abort(dma);	//aborts transfers of the DMA channel and marks it as free for use.
			musb_dbg(musb, "abort %cX%d DMA for urb %p --> %d",
				is_in ? 'R' : 'T', ep->epnum,
				urb, status);
			urb->actual_length += dma->actual_len;	//Total number of transferred bytes incremented by number of bytes transferred by channel.
		}
	}

	/* turn off DMA requests, discard state, stop polling ... */
	if (ep->epnum && is_in) {										//Not endpoint 0 and input.
		/* giveback saves bulk toggle */
		csr = musb_h_flush_rxfifo(ep, 0);							//Flushes reception FIFO.

		/* clear the endpoint's irq status here to avoid bogus irqs */
		if (is_dma_capable() && dma)								//Has DMA channel structure.
			musb_platform_clear_ep_rxintr(musb, ep->epnum);			//Clears TI TRM USB0/1IRQSTAT0 status for given RX endpoint.
	} else if (ep->epnum) {											//Not endpoint 0 and output.
		musb_h_tx_flush_fifo(ep);									//Transmits all packets in TX FIFO.
		csr = musb_readw(epio, MUSB_TXCSR);
			//TxPktRdy will not be automatically set when data of the maximum
			//packet size (value in TxMaxP) is loaded into the TX FIFO.
		csr &= ~(MUSB_TXCSR_AUTOSET
			//Disables DMA request for the TX endpoint.
			| MUSB_TXCSR_DMAENAB
			//This bit is set when a STALL handshake is received. When this
			//bit is set, any DMA request that is in progress is stopped,
			//the FIFO is completely flushed and the TxPktRdy bit is cleared
			//(see below). The CPU should clear this bit.
			| MUSB_TXCSR_H_RXSTALL
			//Bulk endpoints only: This bit will be set when the TX endpoint
			//is halted following the receipt of NAK responses for longer
			//than the time set as the NAK Limit by the TxInterval register.
			//The CPU should clear this bit to allow the endpoint to continue.
			| MUSB_TXCSR_H_NAKTIMEOUT
			//The USB sets this bit when 3 attempts have been made to send a
			//packet and no handshake packet has been received. CPU should
			//clear this bit.
			| MUSB_TXCSR_H_ERROR
			//No packet ready.
			| MUSB_TXCSR_TXPKTRDY);

		musb_writew(epio, MUSB_TXCSR, csr);
		/* REVISIT may need to clear FLUSHFIFO ... */
		musb_writew(epio, MUSB_TXCSR, csr);
		/* flush cpu writebuffer */
		csr = musb_readw(epio, MUSB_TXCSR);
	} else  {
		//Flushes all packets in the FIFO and resets the FIFO pointer and the
		//control register of endpoint zero.
		musb_h_ep0_flush_fifo(ep);
	}
	if (status == 0)
		//Complete the processing of the given URB(?), and setup transfer for
		//the next urb associated with the given hardware endpoint.
		musb_advance_schedule(ep->musb, urb, ep, is_in);
	return status;
}

static int musb_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
//printk("drivers/usb/musb/musb_host.c:musb_urb_dequeue\n");
	struct musb		*musb = hcd_to_musb(hcd);
	struct musb_qh		*qh;
	unsigned long		flags;
	int			is_in  = usb_pipein(urb->pipe);
	int			ret;

	trace_musb_urb_deq(musb, urb);

	spin_lock_irqsave(&musb->lock, flags);
	ret = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (ret)
		goto done;

	qh = urb->hcpriv;
	if (!qh)
		goto done;

	/*
	 * Any URB not actively programmed into endpoint hardware can be
	 * immediately given back; that's any URB not at the head of an
	 * endpoint queue, unless someday we get real DMA queues.  And even
	 * if it's at the head, it might not be known to the hardware...
	 *
	 * Otherwise abort current transfer, pending DMA, etc.; urb->status
	 * has already been updated.  This is a synchronous abort; it'd be
	 * OK to hold off until after some IRQ, though.
	 *
	 * NOTE: qh is invalid unless !list_empty(&hep->urb_list)
	 */
	if (!qh->is_ready
			|| urb->urb_list.prev != &qh->hep->urb_list
			|| musb_ep_get_qh(qh->hw_ep, is_in) != qh) {
		int	ready = qh->is_ready;

		qh->is_ready = 0;
		musb_giveback(musb, urb, 0);
		qh->is_ready = ready;

		/* If nothing else (usually musb_giveback) is using it
		 * and its URB list has emptied, recycle this qh.
		 */
		if (ready && list_empty(&qh->hep->urb_list)) {
			qh->hep->hcpriv = NULL;
			list_del(&qh->ring);
			kfree(qh);
		}
	} else
		ret = musb_cleanup_urb(urb, qh);
done:
	spin_unlock_irqrestore(&musb->lock, flags);
	return ret;
}

/* disable an endpoint */
//Aborts DMA transfer, completes transfer of URBs of endpoint, setups transfer
//for the next urb or removes it depending on whether the first qh of the
//endpoint is the qh of the given host endpoint.
static void
musb_h_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_disable\n");
	u8			is_in = hep->desc.bEndpointAddress & USB_DIR_IN;	//"Specifies the USB-defined endpoint address"
	unsigned long		flags;
	struct musb		*musb = hcd_to_musb(hcd);
	struct musb_qh		*qh;
	struct urb		*urb;

	spin_lock_irqsave(&musb->lock, flags);

	qh = hep->hcpriv;
	if (qh == NULL)
		goto exit;

	/* NOTE: qh is invalid unless !list_empty(&hep->urb_list) */

	/* Kick the first URB off the hardware, if needed */
	qh->is_ready = 0;
	if (musb_ep_get_qh(qh->hw_ep, is_in) == qh) {
		//The first qh of the associated endpoint(?) is the qh of the given Linux endpoint structure?
		urb = next_urb(qh);

		/* make software (then hardware) stop ASAP */
		if (!urb->unlinked)
			urb->status = -ESHUTDOWN;

		/* cleanup */
		//Aborts DMA transfer of endpoint and marks the DMA channel data
		//structure as free for use again, updates number of transferred bytes
		//of URB, clears interrupt status of endpoint, flushes the MUSB endpoint
		//FIFO, and completes the processing of the given URB(?), and setups
		//transfer for the next urb associated with the given hardware endpoint.
		musb_cleanup_urb(urb, qh);

		/* Then nuke all the others ... and advance the
		 * queue on hw_ep (e.g. bulk ring) when we're done.
		 */
		while (!list_empty(&hep->urb_list)) {	//"urbs queued to this endpoint; maintained by usbcore"
			urb = next_urb(qh);
			urb->status = -ESHUTDOWN;
			//Complete the processing of the given URB(?), and setup transfer
			//for the next urb associated with the given hardware endpoint.
			musb_advance_schedule(musb, urb, qh->hw_ep, is_in);
		}
	} else {		//The first qh of the associated endpoint(?) is NOT the qh of the given Linux endpoint structure?
		/* Just empty the queue; the hardware is busy with
		 * other transfers, and since !qh->is_ready nothing
		 * will activate any of these as it advances.
		 */
		while (!list_empty(&hep->urb_list))
			//Removes the next (?) URB from the endpoint queue via the Linux
			//kernel and given back to the driver.
			musb_giveback(musb, next_urb(qh), -ESHUTDOWN);

		hep->hcpriv = NULL;
		list_del(&qh->ring);	//Deletes qh from its associated rint.
		kfree(qh);				//Free memory of queue head.
	}
exit:
	spin_unlock_irqrestore(&musb->lock, flags);
}

static int musb_h_get_frame_number(struct usb_hcd *hcd)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_get_frame_number\n");
	struct musb	*musb = hcd_to_musb(hcd);

	return musb_readw(musb->mregs, MUSB_FRAME);
}

//Used to initialize the host controller driver. Sets the HCD state to running.
static int musb_h_start(struct usb_hcd *hcd)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_start\n");
	struct musb	*musb = hcd_to_musb(hcd);

	/* NOTE: musb_start() is called when the hub driver turns
	 * on port power, or when (OTG) peripheral starts.
	 */
	hcd->state = HC_STATE_RUNNING;
	musb->port1_status = 0;
	return 0;
}

static void musb_h_stop(struct usb_hcd *hcd)
{
//printk("drivers/usb/musb/musb_host.c:musb_h_stop\n");
	musb_stop(hcd_to_musb(hcd));
	hcd->state = HC_STATE_HALT;
}

//Suspends device and updates PHY OTG state.
static int musb_bus_suspend(struct usb_hcd *hcd)
{
//printk("drivers/usb/musb/musb_host.c:musb_bus_suspend\n");
	struct musb	*musb = hcd_to_musb(hcd);
	u8		devctl;
	int		ret;

	ret = musb_port_suspend(musb, true);	//Suspends USB.
	if (ret)
		return ret;

	if (!is_host_active(musb))				//Not host mode.
		return 0;

	switch (musb->xceiv->otg->state) {
	case OTG_STATE_A_SUSPEND:
		return 0;
	case OTG_STATE_A_WAIT_VRISE:
		/* ID could be grounded even if there's no device
		 * on the other end of the cable.  NOTE that the
		 * A_WAIT_VRISE timers are messy with MUSB...
		 */
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if ((devctl & MUSB_DEVCTL_VBUS) == MUSB_DEVCTL_VBUS)	//VBus valid.
			musb->xceiv->otg->state = OTG_STATE_A_WAIT_BCON;
		break;
	default:
		break;
	}

	if (musb->is_active) {
		WARNING("trying to suspend as %s while active\n",
				usb_otg_state_string(musb->xceiv->otg->state));
		return -EBUSY;
	} else
		return 0;
}

//Stops reset and notifies kernel of resumption.
static int musb_bus_resume(struct usb_hcd *hcd)
{
//printk("drivers/usb/musb/musb_host.c:musb_bus_resume\n");
	struct musb *musb = hcd_to_musb(hcd);

	if (musb->config &&
	    musb->config->host_port_deassert_reset_at_resume)
		//Stops reset and notifies kernel of resumption.
		musb_port_reset(musb, false);

	return 0;
}

#ifndef CONFIG_MUSB_PIO_ONLY

#define MUSB_USB_DMA_ALIGN 4

struct musb_temp_buffer {
	void *kmalloc_ptr;
	void *old_xfer_buffer;
	u8 data[];
};

//Transfers data from temporary buffer to URB buffer, and frees the temporary buffer.
static void musb_free_temp_buffer(struct urb *urb)
{
//printk("drivers/usb/musb/musb_host.c:musb_free_temp_buffer\n");
	enum dma_data_direction dir;
	struct musb_temp_buffer *temp;
	size_t length;

	if (!(urb->transfer_flags & URB_ALIGNED_TEMP_BUFFER))				//"Temp buffer was alloc'd"
		return;

	dir = usb_urb_dir_in(urb) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;		//DMA transfer direction.

	temp = container_of(urb->transfer_buffer, struct musb_temp_buffer,	//MUSB temporary transfer buffer?
			    data);

	if (dir == DMA_FROM_DEVICE) {										//Read data from device to memory.
		if (usb_pipeisoc(urb->pipe))									//Isochronous (periodic real-time) transfer.
			length = urb->transfer_buffer_length;						//"data buffer length"
		else															//Control, bulk or interrupt transfer.
			length = urb->actual_length;								//"actual transfer length"

		memcpy(temp->old_xfer_buffer, temp->data, length);				//Copies temp->data to temp->old_xfer_buffer.
	}
	urb->transfer_buffer = temp->old_xfer_buffer;						//"(in) associated data buffer"
	kfree(temp->kmalloc_ptr);											//Frees temporary buffer (temp->data?).

	urb->transfer_flags &= ~URB_ALIGNED_TEMP_BUFFER;					//Not "Temp buffer was alloc'd "
}

//Allocates an "intermediate" temporary buffer for the URB, unless:
//Not zero "(in) number of entries in the sg list", or
//Not null "(in) scatter gather buffer list", or
//Zero length "(in) data buffer length", or
//32-bit aligned "(in) associated data buffer"
//holds, which is the case.
static int musb_alloc_temp_buffer(struct urb *urb, gfp_t mem_flags)
{
//printk("drivers/usb/musb/musb_host.c:musb_alloc_temp_buffer\n");
	enum dma_data_direction dir;
	struct musb_temp_buffer *temp;
	void *kmalloc_ptr;
	size_t kmalloc_size;

	//Not zero "(in) number of entries in the sg list", or
	//Not null "(in) scatter gather buffer list", or
	//Zero length "(in) data buffer length", or
	//32-bit aligned "(in) associated data buffer", then return success.
	if (urb->num_sgs || urb->sg ||										//TRUE: NO TEMPORARY BUFFER IS CREATED.
	    urb->transfer_buffer_length == 0 ||
	    !((uintptr_t)urb->transfer_buffer & (MUSB_USB_DMA_ALIGN - 1)))
		return 0;

	dir = usb_urb_dir_in(urb) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;	//DMA direction.

	/* Allocate a buffer with enough padding for alignment */
	kmalloc_size = urb->transfer_buffer_length +					//Buffer size for data buffer, musb_temp_buffer struct and alignment.
		sizeof(struct musb_temp_buffer) + MUSB_USB_DMA_ALIGN - 1;

	kmalloc_ptr = kmalloc(kmalloc_size, mem_flags);					//Allocates buffer for data buffer, musb_temp_buffer struct and alignment.
	if (!kmalloc_ptr)
		return -ENOMEM;

	/* Position our struct temp_buffer such that data is aligned */
	temp = PTR_ALIGN(kmalloc_ptr, MUSB_USB_DMA_ALIGN);				//Set temp to a multiple of 4.


	temp->kmalloc_ptr = kmalloc_ptr;								//Sets kmalloc_ptr to the start of the buffer as allocated by the kernel.
	temp->old_xfer_buffer = urb->transfer_buffer;					//Sets old_xfer_buffer to the "(in) dma addr for transfer_buffer".
	if (dir == DMA_TO_DEVICE)										//If data is to be transferred from memory to the device,
		memcpy(temp->data, urb->transfer_buffer,					//copy the date from the URB buffer to the temporary buffer.
		       urb->transfer_buffer_length);
	urb->transfer_buffer = temp->data;								//Update urb buffer address to the new temporary buffer.

	urb->transfer_flags |= URB_ALIGNED_TEMP_BUFFER;					//"Temp buffer was alloc'd"

	return 0;
}

//Linux kernel: "allow an HCD to override the default DMA mapping and unmapping routines"
//Maps the URB data buffer for DMA.
static int musb_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb,
				      gfp_t mem_flags)
{
//printk("drivers/usb/musb/musb_host.c:musb_map_urb_for_dma\n");
	struct musb	*musb = hcd_to_musb(hcd);
	int ret;

	/*
	 * The DMA engine in RTL1.8 and above cannot handle
	 * DMA addresses that are not aligned to a 4 byte boundary.
	 * For such engine implemented (un)map_urb_for_dma hooks.
	 * Do not use these hooks for RTL<1.8
	 */
	if (musb->hwvers < MUSB_HWVERS_1800)
		return usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);	//"Map the URB's buffers for DMA access"

	ret = musb_alloc_temp_buffer(urb, mem_flags);				//Returns 0.
	if (ret)
		return ret;

	ret = usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);			//"Map the URB's buffers for DMA access."
	if (ret)													//If mapping the URB's buffers for DMA access failed, 
		musb_free_temp_buffer(urb);								//frees the temporary buffer.

	return ret;
}

//Linux kernel description:
//"allow an HCD to override the default DMA mapping and unmapping routines."
//Unmaps URB from DMA.
static void musb_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
//printk("drivers/usb/musb/musb_host.c:musb_unmap_urb_for_dma\n");
	struct musb	*musb = hcd_to_musb(hcd);

	usb_hcd_unmap_urb_for_dma(hcd, urb);

	/* Do not use this hook for RTL<1.8 (see description above) */
	if (musb->hwvers < MUSB_HWVERS_1800)
		return;

	musb_free_temp_buffer(urb);
}
#endif /* !CONFIG_MUSB_PIO_ONLY */

static const struct hc_driver musb_hc_driver = {
	.description		= "musb-hcd",
	.product_desc		= "MUSB HDRC host driver",
	.hcd_priv_size		= sizeof(struct musb *),
	.flags			= HCD_USB2 | HCD_DMA | HCD_MEMORY,

	/* not using irq handler or reset hooks from usbcore, since
	 * those must be shared with peripheral code for OTG configs
	 */

	.start			= musb_h_start,
	.stop			= musb_h_stop,

	.get_frame_number	= musb_h_get_frame_number,

	.urb_enqueue		= musb_urb_enqueue,
	.urb_dequeue		= musb_urb_dequeue,
	.endpoint_disable	= musb_h_disable,

#ifndef CONFIG_MUSB_PIO_ONLY
	.map_urb_for_dma	= musb_map_urb_for_dma,
	.unmap_urb_for_dma	= musb_unmap_urb_for_dma,
#endif

	.hub_status_data	= musb_hub_status_data,
	.hub_control		= musb_hub_control,
	.bus_suspend		= musb_bus_suspend,
	.bus_resume		= musb_bus_resume,
	/* .start_port_reset	= NULL, */
	/* .hub_irq_enable	= NULL, */
};
//Called by musb_core.c:allocate_instance with a driver internal data structure
//for a USB controller.
int musb_host_alloc(struct musb *musb)
{
//printk("drivers/usb/musb/musb_host.c:musb_host_alloc\n");
	struct device	*dev = musb->controller;	//Linux device structure for the USB controller.

	/* usbcore sets dev->driver_data to hcd, and sometimes uses that... */
	//HCD stands for Host Controller Driver, which is the interface between the
	//Linux USB stack and a host controller driver (in this case a USB
	//controller: musb-hdrc0 and musb-hdrc1 for the two USB controllers of
	//mentor).
	//Creates and initializes an HCD structure for this USB host controller
	//driver, the Linux device structure for this USB controller and its name
	//(musb-hdrc0 and musb-hdrc1).
	musb->hcd = usb_create_hcd(&musb_hc_driver, dev, dev_name(dev));
	if (!musb->hcd)
		return -EINVAL;

	*musb->hcd->hcd_priv = (unsigned long) musb;	//Private data of the Host Controller Driver.
	musb->hcd->self.uses_pio_for_control = 1;		//"Does the host controller use PIO for control transfers?"
	musb->hcd->uses_new_polling = 1;
	musb->hcd->has_tt = 1;							//"Integrated TT in root hub"

	return 0;
}

void musb_host_cleanup(struct musb *musb)
{
//printk("drivers/usb/musb/musb_host.c:musb_host_cleanup\n");
	if (musb->port_mode == MUSB_PERIPHERAL)
		return;
	usb_remove_hcd(musb->hcd);
}

//Frees host controller driver.
void musb_host_free(struct musb *musb)
{
//printk("drivers/usb/musb/musb_host.c:musb_host_free\n");
	usb_put_hcd(musb->hcd);
}

//Finishes initialization of the host controller driver and power wake up source.
int musb_host_setup(struct musb *musb, int power_budget)
{
//printk("drivers/usb/musb/musb_host.c:musb_host_setup START %p\n", musb->xceiv->otg->set_host);
	int ret;
	struct usb_hcd *hcd = musb->hcd;					//USB Host controller driver.

	if (musb->port_mode == MUSB_HOST) {					//USB0/1 role is host.
		MUSB_HST_MODE(musb);							//Set (M)USB0/1 controller in host mode.
		musb->xceiv->otg->state = OTG_STATE_A_IDLE;		//Dual role PHY state is idle?
	}

	//For Host Controller Drivers. Calls drivers/usb/phy/phy-generic.c:nop_set_host.
	//Sets musb->xceiv->otg->host := &hcd->self, where hcd->self is a USB bus with a tree of devices.
	otg_set_host(musb->xceiv->otg, &hcd->self);
	/* don't support otg protocols */
	hcd->self.otg_port = 0;								//"0, or number of OTG/HNP port"
	musb->xceiv->otg->host = &hcd->self;				//Duplicate of otg_set_host.
	hcd->power_budget = 2 * (power_budget ? : 250);		//If power_budget is not 0 then set it to 250.
	//"do not manage the PHY state in the HCD core, instead let the driver
	// handle this (for example if the PHY can only be turned on after a
	// specific event)"
	hcd->skip_phy_initialization = 1;

	ret = usb_add_hcd(hcd, 0, 0);	//"finish generic HCD structure initialization and register", interrupt number and flags. 0=No use.
	if (ret < 0)
		return ret;

	device_wakeup_enable(hcd->self.controller);			//"Enable given device to be a wakeup source."
//printk("drivers/usb/musb/musb_host.c:musb_host_setup END\n");
	return 0;
}

//"submits a workqueue request to resume the root hub"
void musb_host_resume_root_hub(struct musb *musb)
{
//printk("drivers/usb/musb/musb_host.c:musb_host_resume_root_hub\n");
	usb_hcd_resume_root_hub(musb->hcd);
}

//Calls the root hub due to an event if pending URB, otherwise resumes root hub.
void musb_host_poke_root_hub(struct musb *musb)
{
//printk("drivers/usb/musb/musb_host.c:musb_host_poke_root_hub\n");
	MUSB_HST_MODE(musb);								//Set musb in host mode.
	if (musb->hcd->status_urb)							//Pending URB?
		//"Root Hub interrupt transfers are polled using a timer if the driver
		// requests it; otherwise the driver is responsible for calling
		// usb_hcd_poll_rh_status() when an event occurs."
		usb_hcd_poll_rh_status(musb->hcd);
	else
		//"called by HCD to resume its root hub"
		usb_hcd_resume_root_hub(musb->hcd);
}
