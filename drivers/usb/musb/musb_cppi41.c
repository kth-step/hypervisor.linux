// SPDX-License-Identifier: GPL-2.0
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/sizes.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include "cppi_dma.h"
#include "musb_core.h"
#include "musb_trace.h"

#define RNDIS_REG(x) (0x80 + ((x - 1) * 4))

#define EP_MODE_AUTOREQ_NONE		0
#define EP_MODE_AUTOREQ_ALL_NEOP	1
#define EP_MODE_AUTOREQ_ALWAYS		3

#define EP_MODE_DMA_TRANSPARENT		0
#define EP_MODE_DMA_RNDIS		1
#define EP_MODE_DMA_GEN_RNDIS		3

#define USB_CTRL_TX_MODE	0x70
#define USB_CTRL_RX_MODE	0x74
#define USB_CTRL_AUTOREQ	0xd0
#define USB_TDOWN		0xd8

#define MUSB_DMA_NUM_CHANNELS 15

#define DA8XX_USB_MODE		0x10
#define DA8XX_USB_AUTOREQ	0x14
#define DA8XX_USB_TEARDOWN	0x1c

#define DA8XX_DMA_NUM_CHANNELS 4

struct cppi41_dma_controller {
	struct dma_controller controller;
	struct cppi41_dma_channel *rx_channel;
	struct cppi41_dma_channel *tx_channel;
	struct hrtimer early_tx;
	struct list_head early_tx_list;
	u32 rx_mode;
	u32 tx_mode;
	u32 auto_req;

	u32 tdown_reg;
	u32 autoreq_reg;

	void (*set_dma_mode)(struct cppi41_dma_channel *cppi41_channel,
			     unsigned int mode);
	u8 num_channels;
};

//Records in the DMA channel data structure the "current state of the Endpoint 0
//data toggle."
static void save_rx_toggle(struct cppi41_dma_channel *cppi41_channel)
{
//printk("drivers/usb/musb/musb_cppi41.c:save_rx_toggle\n");
	u16 csr;
	u8 toggle;

	if (cppi41_channel->is_tx)
		return;
	if (!is_host_active(cppi41_channel->controller->controller.musb))
		return;

	csr = musb_readw(cppi41_channel->hw_ep->regs, MUSB_RXCSR);
	toggle = csr & MUSB_RXCSR_H_DATATOGGLE ? 1 : 0;

	cppi41_channel->usb_toggle = toggle;
}

//Reads the state of the endpoint 0 data toggle.
static void update_rx_toggle(struct cppi41_dma_channel *cppi41_channel)
{
//printk("drivers/usb/musb/musb_cppi41.c:update_rx_toggle\n");
	struct musb_hw_ep *hw_ep = cppi41_channel->hw_ep;
	struct musb *musb = hw_ep->musb;
	u16 csr;
	u8 toggle;

	if (cppi41_channel->is_tx)
		return;
	if (!is_host_active(musb))
		return;

	musb_ep_select(musb->mregs, hw_ep->epnum);	//Map register bank to control status register of endpoint associated with DMA channel.
	csr = musb_readw(hw_ep->regs, MUSB_RXCSR);	//Read control status register.
	//"When read, this bit indicates the current state of the Endpoint 0 data
	// toggle. If D10 is high, this bit may be written with the required setting
	// of the data toggle. If D10 is low, any value written to this bit is
	// ignored."
	toggle = csr & MUSB_RXCSR_H_DATATOGGLE ? 1 : 0;	//Read the state of the endpoint 0 data toggle.

	/*
	 * AM335x Advisory 1.0.13: Due to internal synchronisation error the
	 * data toggle may reset from DATA1 to DATA0 during receiving data from
	 * more than one endpoint.
	 */
	if (!toggle && toggle == cppi41_channel->usb_toggle) {
		csr |= MUSB_RXCSR_H_DATATOGGLE | MUSB_RXCSR_H_WR_DATATOGGLE;
		musb_writew(cppi41_channel->hw_ep->regs, MUSB_RXCSR, csr);
		musb_dbg(musb, "Restoring DATA1 toggle.");
	}

	cppi41_channel->usb_toggle = toggle;			//Store the state of the endpoint 0 data toggle.
}

//Checks if the transmit FIFO queue is empty (ready to transmit new packets)?
static bool musb_is_tx_fifo_empty(struct musb_hw_ep *hw_ep)
{
//printk("drivers/usb/musb/musb_cppi41.c:musb_is_tx_fifo_empty\n");
	u8		epnum = hw_ep->epnum;		//Endpoint number
	struct musb	*musb = hw_ep->musb;	//MUSB USB0/1 data structure.
	void __iomem	*epio = musb->endpoints[epnum].regs;	//musb_core_init initializes endpoint registers which is USB0/1_CORE_BASE + 0x10.
	u16		csr;
	//Calls musb_indexed_ep_select(USB0/1_CORE_BASE, endpoint_number), which writes MUSB_INDEX to the endpoint corresponding register.
	musb_ep_select(musb->mregs, hw_ep->epnum);
	csr = musb_readw(epio, MUSB_TXCSR);	//musb_readw = musb_default_readw. Reads 16 bits from USB0/1_CORE_BASE + 0x10 + 0x2 (MUSB_TXCSR)
	if (csr & MUSB_TXCSR_TXPKTRDY)		//Control status register signals ready to transmit packet? If not queue is not empty (False)?
		return false;
	return true;
}

static void cppi41_dma_callback(void *private_data,
				const struct dmaengine_result *result);

//Handles interrupt, writes undocumented endpoint registers, and adds new transfer.
static void cppi41_trans_done(struct cppi41_dma_channel *cppi41_channel)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_trans_done\n");
	struct musb_hw_ep *hw_ep = cppi41_channel->hw_ep;	//Get the endpoint data structure of this DMA channel.
	struct musb *musb = hw_ep->musb;					//Get the MUSB USB0/1 data structure of this DMA channel/endpoint.
	void __iomem *epio = hw_ep->regs;					//Undocumented addresses USB0/1 CORE + 0x10.
	u16 csr;

	if (!cppi41_channel->prog_len ||									//No remaining bytes to transfer.
	    (cppi41_channel->channel.status == MUSB_DMA_STATUS_FREE)) {		//DMA channel initialized and idle?

		/* done, complete */
		cppi41_channel->channel.actual_len =							//Number of transferred bytes.
			cppi41_channel->transferred;
		cppi41_channel->channel.status = MUSB_DMA_STATUS_FREE;			//DMA channel has no pending transfers?
		cppi41_channel->channel.rx_packet_done = true;					//Packet processed.	

		/*
		 * transmit ZLP using PIO mode for transfers which size is
		 * multiple of EP packet size.
		 */
		//tx_zlp is 1 if transmit channel and packet length longer than max
		//length, requiring more than one BD (?), and a multiple number of
		//packets have been transferred, then 
		if (cppi41_channel->tx_zlp && (cppi41_channel->transferred %
					cppi41_channel->packet_sz) == 0) {
			musb_ep_select(musb->mregs, hw_ep->epnum);		//Writes MUSB_INDEX to the endpoint corresponding register.
			csr = MUSB_TXCSR_MODE | MUSB_TXCSR_TXPKTRDY;	//Control status register and transmit packet ready for driver?
			musb_writew(epio, MUSB_TXCSR, csr);				//Writes undocumented endpoint register.
		}
		////Defined in drivers/usb/musb/musb_trace.h by DEFINE_EVENT(musb_cppi41, musb_cppi41_done, ...
		trace_musb_cppi41_done(cppi41_channel);
		musb_dma_completion(musb, hw_ep->epnum, cppi41_channel->is_tx);	//Services TX/RX and TX availability interrupts.
	} else {															//Bytes remain to be transferred of DMA channel not idle.
		/* next iteration, reload */
		struct dma_chan *dc = cppi41_channel->dc;						//Linux kernel structure for DMA channels (device and lock).
		struct dma_async_tx_descriptor *dma_desc;	//Linux kernel DMA TX descriptor: Addresses and channel pointers and DB lists.
		enum dma_transfer_direction direction;		//memory2memory, memory2device, device2memory, device2device.
		u32 remain_bytes;

		cppi41_channel->buf_addr += cppi41_channel->packet_sz;	//Increment address by the size of the transmitted packet?

		remain_bytes = cppi41_channel->total_len;				//Bytes to transfer is total length.
		remain_bytes -= cppi41_channel->transferred;			//Remaining bytes to transfer is total length minux transferred bytes.
		//Remaining bytes is minimum of remaining bytes and packet size.
		//SHOULD ALWAYS BE REMAINING BYTES??????????????????????????????????????????????????
		remain_bytes = min(remain_bytes, cppi41_channel->packet_sz);
		cppi41_channel->prog_len = remain_bytes;				//Bytes to transfer is remaining bytes.

		direction = cppi41_channel->is_tx ? DMA_MEM_TO_DEV		//Transmit transfers from memory to USB device.
			: DMA_DEV_TO_MEM;									//Receive transfers from USB device to memory.
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_trans_done CALLS dmaengine_prep_slave_single/cppi41_dma_prep_slave_sg\n");
		dma_desc = dmaengine_prep_slave_single(dc,				//Given DMA channel descriptor, the address of the buffer in memory,
				cppi41_channel->buf_addr,						//bytes to transfer, whether the transfer is from memory to device or
				remain_bytes,									//vice versa, interrupt shall trigger invocation of callback function,
				direction,										//and descriptor can be resued before transfer is acknowledged, returns
				DMA_PREP_INTERRUPT | DMA_CTRL_ACK);				//a linux kernel DMA descriptor data structure.
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_trans_done RETURNS FROM dmaengine_prep_slave_single/cppi41_dma_prep_slave_sg\n");
		if (WARN_ON(!dma_desc))									//drivers/dma/ti/cppi41.c:cppi41_dma_prep_slave_sg.
			return;

		dma_desc->callback_result = cppi41_dma_callback;		//Interrupt service routine when transfer is complete.
		dma_desc->callback_param = &cppi41_channel->channel;	//Linux kernel DMA channel data structure is parameter to callback.
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_trans_done CALLS drivers/dma/ti/cppi41.c:cppi41_tx_submit\n");
		cppi41_channel->cookie = dma_desc->tx_submit(dma_desc);	//Calls drivers/dma/ti/cppi41.c:cppi41_tx_submit.
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_trans_done RETURNS FROM drivers/dma/ti/cppi41.c:cppi41_tx_submit\n");
		trace_musb_cppi41_cont(cppi41_channel);
		dma_async_issue_pending(dc);							//Flush pending BDs to hardware in batches to reduce memory-mapped IO?

		if (!cppi41_channel->is_tx) {							//Receive channel.
			musb_ep_select(musb->mregs, hw_ep->epnum);			//Writes MUSB_INDEX to the endpoint corresponding register.
			csr = musb_readw(epio, MUSB_RXCSR);
			csr |= MUSB_RXCSR_H_REQPKT;
			musb_writew(epio, MUSB_RXCSR, csr);					//Writes undocumented endpoint register.
		}
	}
}
//Checks transmit endpoints, handles interrupts and new transfers if transmit
//queue is empty, and resets the timer.
static enum hrtimer_restart cppi41_recheck_tx_req(struct hrtimer *timer)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_recheck_tx_req\n");
	struct cppi41_dma_controller *controller;
	struct cppi41_dma_channel *cppi41_channel, *n;
	struct musb *musb;
	unsigned long flags;
	enum hrtimer_restart ret = HRTIMER_NORESTART;	//"for a one-shot timer which should not be started again"
	//Gets the cppi41_dma_controller; hrtimer is a at the early_tx field.
	controller = container_of(timer, struct cppi41_dma_controller,	
			early_tx);
	musb = controller->controller.musb;								//Gets the MUSB USB0/1 data structure associated with the timer.

	spin_lock_irqsave(&musb->lock, flags);
	//Iterates over all transmit dma channels/endpoints of the DMA controller.
	list_for_each_entry_safe(cppi41_channel, n, &controller->early_tx_list,	
			tx_check) {														//tx_check is the field of the next entry.
		bool empty;
		struct musb_hw_ep *hw_ep = cppi41_channel->hw_ep;					//Gets the endpoint data structure.
		empty = musb_is_tx_fifo_empty(hw_ep);			//Checks if the transmit FIFO queue is empty (ready to transmit new packets)?
		if (empty) {
			list_del_init(&cppi41_channel->tx_check);	//If the transmit queue is empty, then remove this dma channel from the list.
			cppi41_trans_done(cppi41_channel);		//Handles interrupt, writes undocumented endpoint registers, and adds new transfer.
		}
	}

	if (!list_empty(&controller->early_tx_list) &&						//Not empty list with timers.
	    !hrtimer_is_queued(&controller->early_tx)) {					//DMA controller timer is not queued.
		ret = HRTIMER_RESTART;											//Timer must be restarted.
		hrtimer_forward_now(&controller->early_tx, 20 * NSEC_PER_USEC);	//Postpones the expires time of the timer by 20 000ns = 20µs.
	}

	spin_unlock_irqrestore(&musb->lock, flags);
	return ret;
}

//Reenables interrupt after DMA completion, handles DMA abortions, updates DMA
//channel transfer status, adds new transfers, and restarts timer.
static void cppi41_dma_callback(void *private_data,
				const struct dmaengine_result *result)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_dma_callback\n");
	struct dma_channel *channel = private_data;
	struct cppi41_dma_channel *cppi41_channel = channel->private_data;	//Get the DMA channel associated with this interrupt.
	struct musb_hw_ep *hw_ep = cppi41_channel->hw_ep;					//Get the hardware endpoint associated with this DMA channel.
	struct cppi41_dma_controller *controller;
	struct musb *musb = hw_ep->musb;								//Get the MUSB0/1 data structure associated with this DMA channel.
	unsigned long flags;
	struct dma_tx_state txstate;
	u32 transferred;
	int is_hs = 0;														//USB controller by default not in high-speed mode.
	bool empty;

	controller = cppi41_channel->controller;							//Get the DMA controller associated with this DMA channel.
	if (controller->controller.dma_callback)							//If DMA callback, then
		//"dma_callback: invoked on DMA completion, useful to run platform code
		// such IRQ acknowledgment."
		//Reads the status of the PD_CMP flag after a packet is completed, and
		//sets/enables the corresponding bit/interrupt in the IRQENABLER regiser.
		controller->controller.dma_callback(&controller->controller);

	if (result->result == DMA_TRANS_ABORTED)							//DMA transfer "never submitted / aborted"
		return;

	spin_lock_irqsave(&musb->lock, flags);
	//Invokes cppi41_dma_tx_status, which sets number of remaining bytes to
	//transfer in txstate.
	dmaengine_tx_status(cppi41_channel->dc, cppi41_channel->cookie,
			&txstate);
	transferred = cppi41_channel->prog_len - txstate.residue;		//Transferred bytes = number of bytes to transfer - remaining bytes.
	cppi41_channel->transferred += transferred;						//Increment number of transferred bytes.

	trace_musb_cppi41_gb(cppi41_channel);

	update_rx_toggle(cppi41_channel);								//Reads the data toggle of endpoint 0.

	if (cppi41_channel->transferred == cppi41_channel->total_len ||	//All bytes transferred of this DMA channel transaction?
			transferred < cppi41_channel->packet_sz)				//Not all bytes of the URB transferred?
		cppi41_channel->prog_len = 0;								//Reset programmed length for future transactions?

	if (cppi41_channel->is_tx) {									//Transmit channel.
		u8 type;

		if (is_host_active(musb))									//If USB controller is in host mode,
			type = hw_ep->out_qh->type;								//then transmit output queue determines transfer type.
		else														//If USB controller is in peripheral mode,
			type = hw_ep->ep_in.type;								//then transmit input determines transfer type.

		if (type == USB_ENDPOINT_XFER_ISOC)			//If isochronous (periodic) transfer, ...
			/*
			 * Don't use the early-TX-interrupt workaround below
			 * for Isoch transfter. Since Isoch are periodic
			 * transfer, by the time the next transfer is
			 * scheduled, the current one should be done already.
			 *
			 * This avoids audio playback underrun issue.
			 */
			empty = true;							//then FIFO data is emptied?
		else
			empty = musb_is_tx_fifo_empty(hw_ep);	//Checks if the transmit FIFO queue is empty (ready to transmit new packets)?
	}

	if (!cppi41_channel->is_tx || empty) {			//If not transmission or empty FIFO for transmission, then done.
		cppi41_trans_done(cppi41_channel);			//Handles interrupt, writes undocumented endpoint registers, and adds new transfer.
		goto out;
	}

	/*
	 * On AM335x it has been observed that the TX interrupt fires
	 * too early that means the TXFIFO is not yet empty but the DMA
	 * engine says that it is done with the transfer. We don't
	 * receive a FIFO empty interrupt so the only thing we can do is
	 * to poll for the bit. On HS it usually takes 2us, on FS around
	 * 110us - 150us depending on the transfer size.
	 * We spin on HS (no longer than than 25us and setup a timer on
	 * FS to check for the bit and complete the transfer.
	 */
	if (is_host_active(musb)) {								//USB controller in host mode.
		if (musb->port1_status & USB_PORT_STAT_HIGH_SPEED)	//USB controller in high-speed mode.
			is_hs = 1;
	} else {												//USB controller in peripheral mode.
		if (musb->g.speed == USB_SPEED_HIGH)				//USB controller in high-speed mode.
			is_hs = 1;
	}
	if (is_hs) {											//USB controller is in high-speed mode.
		unsigned wait = 25;

		do {
			empty = musb_is_tx_fifo_empty(hw_ep);	//Checks if the transmit FIFO queue is empty (ready to transmit new packets)?
			if (empty) {
				cppi41_trans_done(cppi41_channel);	//Handles interrupt, writes undocumented endpoint registers, and adds new transfer.
				goto out;
			}
			wait--;
			if (!wait)
				break;
			//"a function which uses the hlt instruction on the x86 architecture
			// to set the processor in a kind of sleeping state where energy
			// consumption is reduced and execution stopped. It wakes up anytime
			// an hardware interrupt occurs."
			cpu_relax();
		} while (1);
	}
	list_add_tail(&cppi41_channel->tx_check,	//Adds &cppi41_channel->tx_check to &controller->early_tx_list, which seems to be a list
			&controller->early_tx_list);		//with associated timers?
	if (!hrtimer_is_queued(&controller->early_tx)) {	//"check, whether the timer is on one of the queues" (Linux kernel queues?).
		unsigned long usecs = cppi41_channel->total_len / 10;

		hrtimer_start_range_ns(&controller->early_tx,	//(re)start a (high-resolution timer) hrtimer if not on Linux kernel list?
				       usecs * NSEC_PER_USEC,
				       20 * NSEC_PER_USEC,
				       HRTIMER_MODE_REL);
	}

out:
	spin_unlock_irqrestore(&musb->lock, flags);
}

//Updates DMA mode for given endpoint via field manipulation.
static u32 update_ep_mode(unsigned ep, unsigned mode, u32 old)
{
//printk("drivers/usb/musb/musb_cppi41.c:update_ep_mode\n");
	unsigned shift;

	shift = (ep - 1) * 2;
	old &= ~(3 << shift);
	old |= mode << shift;
	return old;
}

//Sets TX/RX mode to Transparent mode/RNDIS/CDC/generic RNDIS.
static void cppi41_set_dma_mode(struct cppi41_dma_channel *cppi41_channel,
		unsigned mode)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_set_dma_mode\n");
	struct cppi41_dma_controller *controller = cppi41_channel->controller;
	struct musb *musb = controller->controller.musb;
	u32 port;
	u32 new_mode;
	u32 old_mode;

	if (cppi41_channel->is_tx)
		old_mode = controller->tx_mode;
	else
		old_mode = controller->rx_mode;
	port = cppi41_channel->port_num;								//Endpoint number having this DMA channel structure.
	new_mode = update_ep_mode(port, mode, old_mode);				//Updates DMA mode for given endpoint via field manipulation.

	if (new_mode == old_mode)
		return;
	if (cppi41_channel->is_tx) {
		controller->tx_mode = new_mode;
		musb_writel(musb->ctrl_base, USB_CTRL_TX_MODE, new_mode);
	} else {
		controller->rx_mode = new_mode;
		musb_writel(musb->ctrl_base, USB_CTRL_RX_MODE, new_mode);
	}
}

static void da8xx_set_dma_mode(struct cppi41_dma_channel *cppi41_channel,
		unsigned int mode)
{
//printk("drivers/usb/musb/musb_cppi41.c:da8xx_set_dma_mode\n");
	struct cppi41_dma_controller *controller = cppi41_channel->controller;
	struct musb *musb = controller->controller.musb;
	unsigned int shift;
	u32 port;
	u32 new_mode;
	u32 old_mode;

	old_mode = controller->tx_mode;
	port = cppi41_channel->port_num;

	shift = (port - 1) * 4;
	if (!cppi41_channel->is_tx)
		shift += 16;
	new_mode = old_mode & ~(3 << shift);
	new_mode |= mode << shift;

	if (new_mode == old_mode)
		return;
	controller->tx_mode = new_mode;
	musb_writel(musb->ctrl_base, DA8XX_USB_MODE, new_mode);
}

//Sets the auto request of IN transaction of receive endpoint associated with
//given DMA channel.
static void cppi41_set_autoreq_mode(struct cppi41_dma_channel *cppi41_channel,
		unsigned mode)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_set_autoreq_mode\n");
	struct cppi41_dma_controller *controller = cppi41_channel->controller;
	u32 port;
	u32 new_mode;
	u32 old_mode;

	old_mode = controller->auto_req;
	port = cppi41_channel->port_num;					//Associated endpoint number of DMA channel.
	new_mode = update_ep_mode(port, mode, old_mode);	//Updates DMA mode for given endpoint via field manipulation.

	if (new_mode == old_mode)
		return;
	controller->auto_req = new_mode;
	//Sets endpoint autorequest setting: No auto request, auto request on all but EOP, always auto request.
	//"When the AutoRequest feature is enabled, the REQPKT bit of HOST_RXCSR
	// (bit 5) [MUSB register: CPU sets bit to request an IN transaction] will
	// be automatically set when the RXPKTRDY bit is cleared."
	musb_writel(controller->controller.musb->ctrl_base,
		    controller->autoreq_reg, new_mode);
}

//Sets memory transfer parameters of DMA channel, transfer callback, DMA mode
//(whatever that is?), and assigns a unique per DMA channel cookie to the
//descriptor.
static bool cppi41_configure_channel(struct dma_channel *channel,	//MUSB DMA channel data structure.
				u16 packet_sz, u8 mode,
				dma_addr_t dma_addr, u32 len)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_configure_channel\n");
	struct cppi41_dma_channel *cppi41_channel = channel->private_data;
	struct cppi41_dma_controller *controller = cppi41_channel->controller;
	struct dma_chan *dc = cppi41_channel->dc;	//Linux kernel DMA channel data structure with device and lock.
	struct dma_async_tx_descriptor *dma_desc;	//Linux DMA transmit descriptor.
	enum dma_transfer_direction direction;		//DMA transfer direction between devices and memory.
	struct musb *musb = cppi41_channel->controller->controller.musb;	//MUSB0/1 data structure of given DMA channel.
	unsigned use_gen_rndis = 0;					//Remove Network Driver Interface Specification protocol running on top of USB.

	cppi41_channel->buf_addr = dma_addr;		//Memory address.
	cppi41_channel->total_len = len;			//Number of bytes to transfer.
	cppi41_channel->transferred = 0;			//Number of bytes transferred.
	cppi41_channel->packet_sz = packet_sz;		//Packet size. BDs are generated until packet size fits in one BD?
	//1 <=> "Finish bulk OUT with short packet". zlp = zero length packet?
	cppi41_channel->tx_zlp = (cppi41_channel->is_tx && mode) ? 1 : 0;

	/*
	 * Due to AM335x' Advisory 1.0.13 we are not allowed to transfer more
	 * than max packet size at a time.
	 */
	if (cppi41_channel->is_tx)
		use_gen_rndis = 1;

	if (use_gen_rndis) {
		/* RNDIS mode */
		if (len > packet_sz) {									//At least two BDs?
			musb_writel(musb->ctrl_base,						//Write virtually mapped base registers of USB0_CTRL registers
				RNDIS_REG(cppi41_channel->port_num), len);
			/* gen rndis */
			controller->set_dma_mode(cppi41_channel,			//cppi41_set_dma_mode which writes to undocumented USB0/1 register.
					EP_MODE_DMA_GEN_RNDIS);

			/* auto req */
			cppi41_set_autoreq_mode(cppi41_channel,				//Updates mode by writing to undocumented USB0/1 CORE register.
					EP_MODE_AUTOREQ_ALL_NEOP);
		} else {												//One BD enough?
			musb_writel(musb->ctrl_base,						//Write virtually mapped base registers of USB0_CTRL registers
					RNDIS_REG(cppi41_channel->port_num), 0);
			controller->set_dma_mode(cppi41_channel,			//cppi41_set_dma_mode which writes to undocumented USB0/1 register.
					EP_MODE_DMA_TRANSPARENT);
			cppi41_set_autoreq_mode(cppi41_channel,				//Updates mode by writing to undocumented USB0/1 CORE register.
					EP_MODE_AUTOREQ_NONE);
		}
	} else {
		/* fallback mode */
		controller->set_dma_mode(cppi41_channel,				//cppi41_set_dma_mode which writes to undocumented USB0/1 register.
				EP_MODE_DMA_TRANSPARENT);
		cppi41_set_autoreq_mode(cppi41_channel, EP_MODE_AUTOREQ_NONE);	//Updates mode by writing to undocumented USB0/1 CORE register.
		len = min_t(u32, packet_sz, len);
	}
	cppi41_channel->prog_len = len;										//Number of bytes to transfer.
	direction = cppi41_channel->is_tx ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM;	//Direction depends on TX or RX.
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_trans_done CALLS dmaengine_prep_slave_single/cppi41_dma_prep_slave_sg\n");
	//Given DMA channel descriptor, the address of the buffer in memory,
	//bytes to transfer, whether the transfer is from memory to device or
	//vice versa, interrupt shall trigger invocation of callback function,
	//and descriptor can be resued before transfer is acknowledged, returns
	//a linux kernel DMA descriptor data structure.
	dma_desc = dmaengine_prep_slave_single(dc, dma_addr, len, direction,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_trans_done RETURNS FROM dmaengine_prep_slave_single/cppi41_dma_prep_slave_sg\n");
	if (!dma_desc)
		return false;											//Failure?

	dma_desc->callback_result = cppi41_dma_callback;			//Callback function when transfer completes.
	dma_desc->callback_param = channel;							//Parameter to callback function.
	cppi41_channel->cookie = dma_desc->tx_submit(dma_desc);		//Calls drivers/dma/ti/cppi41.c:cppi41_tx_submit.
	cppi41_channel->channel.rx_packet_done = false;				//Not rx?

	trace_musb_cppi41_config(cppi41_channel);

	save_rx_toggle(cppi41_channel);								//Writes to undocumented CORE0/1 register and updates cppi41_channel.
	dma_async_issue_pending(dc);								//Flush pending BDs to hardware in batches to reduce memory-mapped IO?
	return true;												//Success?
}

//Assigns DMA channel data structure for endpoint?
static struct dma_channel *cppi41_dma_channel_allocate(struct dma_controller *c,
				struct musb_hw_ep *hw_ep, u8 is_tx)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_dma_channel_allocate\n");
	struct cppi41_dma_controller *controller = container_of(c,	//Gets the cppi41_dma_controller containing the musb dma_controller.
			struct cppi41_dma_controller, controller);
	struct cppi41_dma_channel *cppi41_channel = NULL;
	u8 ch_num = hw_ep->epnum - 1;								//hw_ep->epnum cannot be endpoint zero??????????????

	if (ch_num >= controller->num_channels)						//Endpoint ID is greater than highest channel index.
		return NULL;

	if (is_tx)													//Transmit endpoint.
		cppi41_channel = &controller->tx_channel[ch_num];
	else														//Receive endpoint.
		cppi41_channel = &controller->rx_channel[ch_num];

	if (!cppi41_channel->dc)									//No DMA channel data structure created for this endpoint?
		return NULL;

	if (cppi41_channel->is_allocated)							//DMA channel allocated for endpoint?
		return NULL;

	cppi41_channel->hw_ep = hw_ep;								//DMA channel's associated endpoint.
	cppi41_channel->is_allocated = 1;							//DMA channel has allocated endpoint?

	trace_musb_cppi41_alloc(cppi41_channel);
	return &cppi41_channel->channel;
}
//Marks a DMA channel as not allocated, meaning that the associated data
//structure is not used by any endpoint, and can be used for another endpoint?
static void cppi41_dma_channel_release(struct dma_channel *channel)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_dma_channel_release\n");
	struct cppi41_dma_channel *cppi41_channel = channel->private_data;	//DMA channel private data.

	trace_musb_cppi41_free(cppi41_channel);
	if (cppi41_channel->is_allocated) {				//If allocated, then:
		cppi41_channel->is_allocated = 0;			//Not allocated,
		channel->status = MUSB_DMA_STATUS_FREE;		//Allocated in memory but not used.
		channel->actual_len = 0;					//No bytes transferred.
	}
}

//Sets memory transfers parameters for DMA channel and callback function.
static int cppi41_dma_channel_program(struct dma_channel *channel,
				u16 packet_sz, u8 mode,
				dma_addr_t dma_addr, u32 len)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_dma_channel_program\n");
	int ret;
	struct cppi41_dma_channel *cppi41_channel = channel->private_data;
	int hb_mult = 0;

	BUG_ON(channel->status == MUSB_DMA_STATUS_UNKNOWN ||	//Do not program an unallocated channel.
		channel->status == MUSB_DMA_STATUS_BUSY);			//Transfers are being performed.

	if (is_host_active(cppi41_channel->controller->controller.musb)) {	//USB is in host mode.
		if (cppi41_channel->is_tx)										//Transmission.
			hb_mult = cppi41_channel->hw_ep->out_qh->hb_mult;	//Outward (transmit) scheduled endpoint, high bandwidth pkts per uf.
		else															//Reception.
			hb_mult = cppi41_channel->hw_ep->in_qh->hb_mult;	//Inward (receive) scheduled endpoint, high bandwidth pkts per uf.
	}

	channel->status = MUSB_DMA_STATUS_BUSY;					//Channel is busy.
	channel->actual_len = 0;								//Actual length transferred so far is zero.

	if (hb_mult)
		//packet_sz is max_packet initialized for each endpoint in drivers/usb/musb/musb_core.c
		packet_sz = hb_mult * (packet_sz & 0x7FF);

	//Sets memory transfer parameters of DMA channel, transfer callback.
	ret = cppi41_configure_channel(channel, packet_sz, mode, dma_addr, len);
	if (!ret)												//Failure.
		channel->status = MUSB_DMA_STATUS_FREE;				//Allocated but no transfers.

	return ret;												//Failure or success.
}
//Some workaround function that return True or False depending on
//host/peripheral mode and tx/rx bulk transfer?
static int cppi41_is_compatible(struct dma_channel *channel, u16 maxpacket,
		void *buf, u32 length)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_is_compatible\n");
	struct cppi41_dma_channel *cppi41_channel = channel->private_data;
	struct cppi41_dma_controller *controller = cppi41_channel->controller;
	struct musb *musb = controller->controller.musb;

	if (is_host_active(musb)) {	//USB controller (BBB) is a host (which is the case for us, where the USB peripheral is USB pin drive.
		WARN_ON(1);
		return 1;
	}
	//"A USB bulk endpoint can transfer large amounts of data. Bulk transfers
	// are reliable that allow hardware error detection, and involves limited
	// number of retries in the hardware."
	if (cppi41_channel->hw_ep->ep_in.type != USB_ENDPOINT_XFER_BULK)	//Not bulk transfer.
		return 0;
	if (cppi41_channel->is_tx)	//Is bulk transmit transfer .
		return 1;
	/* AM335x Advisory 1.0.13. No workaround for device RX mode */
	return 0;
}

//Aborts transfers of the given DMA channel and marks it as free for use for
//another transfer and endpoint.
static int cppi41_dma_channel_abort(struct dma_channel *channel)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_dma_channel_abort\n");
	struct cppi41_dma_channel *cppi41_channel = channel->private_data;
	//Get DMA controller 0/1 depending on which one the given channel is associated with.
	struct cppi41_dma_controller *controller = cppi41_channel->controller;
	struct musb *musb = controller->controller.musb;	//Get the associated MUSB0/1 data structure.
	void __iomem *epio = cppi41_channel->hw_ep->regs;	//Get the undocumented registers for the associated endpoint of the DMA channel.
	int tdbit;
	int ret;
	unsigned is_tx;
	u16 csr;

	is_tx = cppi41_channel->is_tx;
	trace_musb_cppi41_abort(cppi41_channel);

	//DMA channel is idle so nothing to abort? For reception this means that the queue is empty.
	if (cppi41_channel->channel.status == MUSB_DMA_STATUS_FREE)
		return 0;

	list_del_init(&cppi41_channel->tx_check);	//Remove this dma channel from the list.
	if (is_tx) {
		csr = musb_readw(epio, MUSB_TXCSR);
		csr &= ~MUSB_TXCSR_DMAENAB;
		musb_writew(epio, MUSB_TXCSR, csr);		//Writes undocumented USB0/1 CORE regisers.
	} else {
		cppi41_set_autoreq_mode(cppi41_channel, EP_MODE_AUTOREQ_NONE);	//Updates mode by writing to undocumented USB0/1 CORE register.

		/* delay to drain to cppi dma pipeline for isoch */
		//"Isochronous Transfers are used for transmitting real-time information
		// such as audio and video data, and must be sent at a constant rate."
		udelay(250);													//Waits for 250µs?

		csr = musb_readw(epio, MUSB_RXCSR);
		csr &= ~(MUSB_RXCSR_H_REQPKT | MUSB_RXCSR_DMAENAB);
		musb_writew(epio, MUSB_RXCSR, csr);								//Writes undocumented USB0/1 CORE regisers.

		/* wait to drain cppi dma pipe line */
		udelay(50);														//Waits for 50µs?

		csr = musb_readw(epio, MUSB_RXCSR);
		if (csr & MUSB_RXCSR_RXPKTRDY) {								//USB packet ready to transfer to memory?
			csr |= MUSB_RXCSR_FLUSHFIFO;								//Flush USB receive packet queue?
			musb_writew(epio, MUSB_RXCSR, csr);							//Writes undocumented USB0/1 CORE regisers.
			musb_writew(epio, MUSB_RXCSR, csr);
		}
	}

	/* DA8xx Advisory 2.3.27: wait 250 ms before to start the teardown */
	if (musb->ops->quirks & MUSB_DA8XX)									//dsps_ops does not set MUSB_DA8XX.
		mdelay(250);


	//From TI TRM:
	//"Teardown operation of an endpoint requires three operations. The teardown
	// register in the CPPI DMA must be written, the corresponding endpoint bit
	// in TEARDOWN of the USB module must be set, and the FlushFIFO bit in the
	// Mentor USB controller Tx/RxCSR register must be set."

	tdbit = 1 << cppi41_channel->port_num;	//"endpoint bit"?
	if (is_tx)
		tdbit <<= 16;

	do {
		if (is_tx)
			//Write virtually mapped base registers of USB0_CTRL registers
			musb_writel(musb->ctrl_base, controller->tdown_reg,
				    tdbit);
		//For Linux kernel DMA engine: "Terminate all active DMA transfers".
		//"The teardown register in the CPPI DMA must be written"?
		//Causes the Linux kernel to invoke cppi41_stop_chan?
		ret = dmaengine_terminate_all(cppi41_channel->dc);
	} while (ret == -EAGAIN);

	if (is_tx) {
		//Write virtually mapped base registers of USB0_CTRL registers
		musb_writel(musb->ctrl_base, controller->tdown_reg, tdbit);

		csr = musb_readw(epio, MUSB_TXCSR);
		if (csr & MUSB_TXCSR_TXPKTRDY) {
			csr |= MUSB_TXCSR_FLUSHFIFO;		//FlushFIFO bit
			musb_writew(epio, MUSB_TXCSR, csr);	//is written.
		}
	}
	//Driver has data structures for channel but the channel is idle.
	cppi41_channel->channel.status = MUSB_DMA_STATUS_FREE;
	return 0;
}

//Releases and frees memory of all Linux DMA channel data structures.
static void cppi41_release_all_dma_chans(struct cppi41_dma_controller *ctrl)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_release_all_dma_chans\n");
	struct dma_chan *dc;
	int i;

	for (i = 0; i < ctrl->num_channels; i++) {
		dc = ctrl->tx_channel[i].dc;
		if (dc)
			dma_release_channel(dc);
		dc = ctrl->rx_channel[i].dc;
		if (dc)
			dma_release_channel(dc);
	}
}

static void cppi41_dma_controller_stop(struct cppi41_dma_controller *controller)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_dma_controller_stop\n");
	cppi41_release_all_dma_chans(controller);
}

//Attempts to obtain Linux kernel DMA channel data structures. Fails before
//drivers/dma/ti/cppi41.c:cppi41_dma_probe is invoked, which does the
//corresponding memory allocation.
static int cppi41_dma_controller_start(struct cppi41_dma_controller *controller)
{
printk("drivers/usb/musb/musb_cppi41.c:cppi41_dma_controller_start\n");
	struct musb *musb = controller->controller.musb;
	struct device *dev = musb->controller;
	struct device_node *np = dev->parent->of_node;
	struct cppi41_dma_channel *cppi41_channel;
	int count;
	int i;
	int ret;

	count = of_property_count_strings(np, "dma-names");	//Number of entries in the dma-names properties: 30 endpoints, rx1...tx15
	if (count < 0)
		return count;

	for (i = 0; i < count; i++) {						//For each endpoint.
		struct dma_chan *dc;
		struct dma_channel *musb_dma;
		const char *str;
		unsigned is_tx;
		unsigned int port;

		ret = of_property_read_string_index(np, "dma-names", i, &str);	//Stores rx/tx[i] (rx1...tx15) in str?
		if (ret)
			goto err;
		if (strstarts(str, "tx"))
			is_tx = 1;
		else if (strstarts(str, "rx"))
			is_tx = 0;
		else {
			dev_err(dev, "Wrong dmatype %s\n", str);
			goto err;
		}
		ret = kstrtouint(str + 2, 0, &port);						//Converts rx1...tx15 indexes to an unsigned int in port.
		if (ret)
			goto err;

		ret = -EINVAL;
		if (port > controller->num_channels || !port)				//controller->num_channels is set to 15.
			goto err;
		if (is_tx)
			cppi41_channel = &controller->tx_channel[port - 1];		//Device tree indexes are shifted to start from 0.
		else														//cppi41_channel is allocated for all channel data
			cppi41_channel = &controller->rx_channel[port - 1];		//structures in cppi41_dma_controller_create.
																	//Makes cppi41_channel point to that previously allocated struct.

		cppi41_channel->controller = controller;					//The DMA channel points to its parent controller.
		cppi41_channel->port_num = port;							//The port number field is set to its device tree index.
		cppi41_channel->is_tx = is_tx;								//Marks the DMA channel as transmit or receive.
		//Sets the tx_check field list entry to an empty list (see http://www.makelinux.net/ldd3/chp-11-sect-5.shtml)
		INIT_LIST_HEAD(&cppi41_channel->tx_check);

		musb_dma = &cppi41_channel->channel;						//Get the MUSB DMA channel data structure.
		//The MUSB DMA channel data structure gets a reference to the
		//corresponding CPPI41 DMA channel data structure.
		musb_dma->private_data = cppi41_channel;
		musb_dma->status = MUSB_DMA_STATUS_FREE;					//The DMA channel has now initialized data structures but is idle.
		musb_dma->max_len = SZ_4M;									//4 MB is maximum size.

		dc = dma_request_chan(dev->parent, str);					//Get an initialized Linux kernel DMA channel.

		//Error first time before DMA controller is registered, and
		//drivers/dma/ti/cppi41.c:cppi41_dma_probe is invoked to allocate memory
		//for the CPPI and Linux kernel DMA channel structures.
		if (IS_ERR(dc)) {
			ret = PTR_ERR(dc);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to request %s: %d.\n",
					str, ret);
			goto err;
		} //else printk("drivers/usb/musb/musb_cppi41.c:cppi41_dma_controller_start NO ERROR\n");
		//The CPPI DMA channel data structure of USB0/1 DMA controller gets the
		//allocated DMA channel struct.
		cppi41_channel->dc = dc;
	}
	return 0;
err:
	cppi41_release_all_dma_chans(controller);
	return ret;
}

void cppi41_dma_controller_destroy(struct dma_controller *c)
{
//printk("drivers/usb/musb/musb_cppi41.c:cppi41_dma_controller_destroy\n");
	struct cppi41_dma_controller *controller = container_of(c,
			struct cppi41_dma_controller, controller);

	hrtimer_cancel(&controller->early_tx);
	cppi41_dma_controller_stop(controller);
	kfree(controller->rx_channel);
	kfree(controller->tx_channel);
	kfree(controller);
}
EXPORT_SYMBOL_GPL(cppi41_dma_controller_destroy);
//Called by musb_dsps.c:dsps_dma_controller_create
//Allocates memory for the USB0/1 DMA controller, sets timers and assigns
//functions for the DMA controller driver.
//Called twice, one for each Mentor Graphics USB controller 0 and 1.
struct dma_controller *
cppi41_dma_controller_create(struct musb *musb, void __iomem *base)	//USB0/1 MUSB structure and virtual base address of USB0/1 CORE.
{
printk("drivers/usb/musb/musb_cppi41.c:cppi41_dma_controller_create START\n");
	struct cppi41_dma_controller *controller;
	int channel_size;
	int ret = 0;

	if (!musb->controller->parent->of_node) {
		dev_err(musb->controller, "Need DT for the DMA engine.\n");
		return NULL;
	}

	controller = kzalloc(sizeof(*controller), GFP_KERNEL);		//Zero initialized cppi41_dma_controller structure.
	if (!controller)
		goto kzalloc_fail;

	//initialize a timer to the given clock.
	//early_tx is a struct part of controller.
	//"CLOCK_MONOTONIC represents the absolute elapsed wall-clock time since
	// some arbitrary, fixed point in the past. It isn't affected by changes in
	// the system time-of-day clock."
	//"HRTIMER_MODE_REL		- Time value is relative to now"
	hrtimer_init(&controller->early_tx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	controller->early_tx.function = cppi41_recheck_tx_req;	//"timer expiry callback function". Handles interrupts and new transfers.
	INIT_LIST_HEAD(&controller->early_tx_list);	//Initializes the list of timers of the DMA controller to consist of this timer only.

	//Initializes functions for the DMA controller.
	controller->controller.channel_alloc = cppi41_dma_channel_allocate;	//Allocates DMA channel data structures for endpoint?
	controller->controller.channel_release = cppi41_dma_channel_release; //Marks a DMA channel as unallocated to endpoint (free to use).
	controller->controller.channel_program = cppi41_dma_channel_program; //Sets memory transfers for DMA channel and callback function.
	controller->controller.channel_abort = cppi41_dma_channel_abort;	//Tears down DMA channel.
	//Workaround function computing boolean flag depending on host/peripheral mode and tx/rx bulk transfer?
	controller->controller.is_compatible = cppi41_is_compatible;
	controller->controller.musb = musb;							//The MUSB USB0/1 data structure of the DMA controller.

	if (musb->ops->quirks & MUSB_DA8XX) {						//FALSE. See dsps_ops in musb_dsps.c
		controller->tdown_reg = DA8XX_USB_TEARDOWN;
		controller->autoreq_reg = DA8XX_USB_AUTOREQ;
		controller->set_dma_mode = da8xx_set_dma_mode;
		controller->num_channels = DA8XX_DMA_NUM_CHANNELS;
	} else {													//TRUE
		controller->tdown_reg = USB_TDOWN;						//Offset for TI TRM USB0/1_TDOWN register.
		controller->autoreq_reg = USB_CTRL_AUTOREQ;				//Offset for TI TRM USB0/1AUTOREQ register.
		controller->set_dma_mode = cppi41_set_dma_mode;			//Function.
		controller->num_channels = MUSB_DMA_NUM_CHANNELS;		//Number of channels: 15.
	}

	channel_size = controller->num_channels *
			sizeof(struct cppi41_dma_channel);			//Byte size of all DMA channel data structures of the USB0/1 controller.
	controller->rx_channel = kzalloc(channel_size, GFP_KERNEL);	//Allocates zero initialized kernel memory for all receive channels.
	if (!controller->rx_channel)
		goto rx_channel_alloc_fail;
	controller->tx_channel = kzalloc(channel_size, GFP_KERNEL);	//Allocates zero initialized kernel memory for all transmit channels.
	if (!controller->tx_channel)
		goto tx_channel_alloc_fail;
	//Attempts to get DMA channel structures. Fails before DMA controller is
	//registered and have done the actual memory allocation.
	ret = cppi41_dma_controller_start(controller);
	if (ret)
		goto plat_get_fail;

	return &controller->controller;

plat_get_fail:
	kfree(controller->tx_channel);
tx_channel_alloc_fail:
	kfree(controller->rx_channel);
rx_channel_alloc_fail:
	kfree(controller);
kzalloc_fail:
	if (ret == -EPROBE_DEFER)
		return ERR_PTR(ret);
	return NULL;
}
EXPORT_SYMBOL_GPL(cppi41_dma_controller_create);
