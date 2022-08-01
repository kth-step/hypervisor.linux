// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include "../dmaengine.h"

#define DESC_TYPE	27
#define DESC_TYPE_HOST	0x10
#define DESC_TYPE_TEARD	0x13

#define TD_DESC_IS_RX	(1 << 16)
#define TD_DESC_DMA_NUM	10

#define DESC_LENGTH_BITS_NUM	21

#define DESC_TYPE_USB	(5 << 26)
#define DESC_PD_COMPLETE	(1 << 31)

/* DMA engine */
#define DMA_TDFDQ	4
#define DMA_TXGCR(x)	(0x800 + (x) * 0x20)
#define DMA_RXGCR(x)	(0x808 + (x) * 0x20)
#define RXHPCRA0		4

#define GCR_CHAN_ENABLE		(1 << 31)
#define GCR_TEARDOWN		(1 << 30)
#define GCR_STARV_RETRY		(1 << 24)
#define GCR_DESC_TYPE_HOST	(1 << 14)

/* DMA scheduler */
#define DMA_SCHED_CTRL		0
#define DMA_SCHED_CTRL_EN	(1 << 31)
#define DMA_SCHED_WORD(x)	((x) * 4 + 0x800)

#define SCHED_ENTRY0_CHAN(x)	((x) << 0)
#define SCHED_ENTRY0_IS_RX	(1 << 7)

#define SCHED_ENTRY1_CHAN(x)	((x) << 8)
#define SCHED_ENTRY1_IS_RX	(1 << 15)

#define SCHED_ENTRY2_CHAN(x)	((x) << 16)
#define SCHED_ENTRY2_IS_RX	(1 << 23)

#define SCHED_ENTRY3_CHAN(x)	((x) << 24)
#define SCHED_ENTRY3_IS_RX	(1 << 31)

/* Queue manager */
/* 4 KiB of memory for descriptors, 2 for each endpoint */
#define ALLOC_DECS_NUM		128								//128 descriptors.
#define DESCS_AREAS		1									//1 Linking RAM?
#define TOTAL_DESCS_NUM		(ALLOC_DECS_NUM * DESCS_AREAS)	//128 descriptors.
#define QMGR_SCRATCH_SIZE	(TOTAL_DESCS_NUM * 4)			//512 bytes?

#define QMGR_LRAM0_BASE		0x80
#define QMGR_LRAM_SIZE		0x84
#define QMGR_LRAM1_BASE		0x88
#define QMGR_MEMBASE(x)		(0x1000 + (x) * 0x10)
#define QMGR_MEMCTRL(x)		(0x1004 + (x) * 0x10)
#define QMGR_MEMCTRL_IDX_SH	16
#define QMGR_MEMCTRL_DESC_SH	8

#define QMGR_PEND(x)	(0x90 + (x) * 4)

#define QMGR_PENDING_SLOT_Q(x)	(x / 32)
#define QMGR_PENDING_BIT_Q(x)	(x % 32)

#define QMGR_QUEUE_A(n)	(0x2000 + (n) * 0x10)
#define QMGR_QUEUE_B(n)	(0x2004 + (n) * 0x10)
#define QMGR_QUEUE_C(n)	(0x2008 + (n) * 0x10)
#define QMGR_QUEUE_D(n)	(0x200c + (n) * 0x10)

/* Packet Descriptor */
#define PD2_ZERO_LENGTH		(1 << 19)

struct cppi41_channel {
	struct dma_chan chan;
	struct dma_async_tx_descriptor txd;
	struct cppi41_dd *cdd;
	struct cppi41_desc *desc;
	dma_addr_t desc_phys;
	void __iomem *gcr_reg;
	int is_tx;
	u32 residue;

	unsigned int q_num;
	unsigned int q_comp_num;
	unsigned int port_num;

	unsigned td_retry;
	unsigned td_queued:1;
	unsigned td_seen:1;
	unsigned td_desc_seen:1;

	struct list_head node;		/* Node for pending list */
};

struct cppi41_desc {
	u32 pd0;
	u32 pd1;
	u32 pd2;
	u32 pd3;
	u32 pd4;
	u32 pd5;
	u32 pd6;
	u32 pd7;
} __aligned(32);

struct chan_queues {
	u16 submit;
	u16 complete;
};

struct cppi41_dd {
	struct dma_device ddev;

	void *qmgr_scratch;
	dma_addr_t scratch_phys;

	struct cppi41_desc *cd;
	dma_addr_t descs_phys;
	u32 first_td_desc;
	struct cppi41_channel *chan_busy[ALLOC_DECS_NUM];

	void __iomem *ctrl_mem;
	void __iomem *sched_mem;
	void __iomem *qmgr_mem;
	unsigned int irq;
	const struct chan_queues *queues_rx;
	const struct chan_queues *queues_tx;
	struct chan_queues td_queue;
	u16 first_completion_queue;
	u16 qmgr_num_pend;
	u32 n_chans;
	u8 platform;

	struct list_head pending;	/* Pending queued transfers */
	spinlock_t lock;		/* Lock for pending list */

	/* context for suspend/resume */
	unsigned int dma_tdfdq;

	bool is_suspended;
};

static struct chan_queues am335x_usb_queues_tx[] = {
	/* USB0 ENDP 1 */
	[ 0] = { .submit = 32, .complete =  93},
	[ 1] = { .submit = 34, .complete =  94},
	[ 2] = { .submit = 36, .complete =  95},
	[ 3] = { .submit = 38, .complete =  96},
	[ 4] = { .submit = 40, .complete =  97},
	[ 5] = { .submit = 42, .complete =  98},
	[ 6] = { .submit = 44, .complete =  99},
	[ 7] = { .submit = 46, .complete = 100},
	[ 8] = { .submit = 48, .complete = 101},
	[ 9] = { .submit = 50, .complete = 102},
	[10] = { .submit = 52, .complete = 103},
	[11] = { .submit = 54, .complete = 104},
	[12] = { .submit = 56, .complete = 105},
	[13] = { .submit = 58, .complete = 106},
	[14] = { .submit = 60, .complete = 107},

	/* USB1 ENDP1 */
	[15] = { .submit = 62, .complete = 125},
	[16] = { .submit = 64, .complete = 126},
	[17] = { .submit = 66, .complete = 127},
	[18] = { .submit = 68, .complete = 128},
	[19] = { .submit = 70, .complete = 129},
	[20] = { .submit = 72, .complete = 130},
	[21] = { .submit = 74, .complete = 131},
	[22] = { .submit = 76, .complete = 132},
	[23] = { .submit = 78, .complete = 133},
	[24] = { .submit = 80, .complete = 134},
	[25] = { .submit = 82, .complete = 135},
	[26] = { .submit = 84, .complete = 136},
	[27] = { .submit = 86, .complete = 137},
	[28] = { .submit = 88, .complete = 138},
	[29] = { .submit = 90, .complete = 139},
};

static const struct chan_queues am335x_usb_queues_rx[] = {
	/* USB0 ENDP 1 */
	[ 0] = { .submit =  1, .complete = 109},
	[ 1] = { .submit =  2, .complete = 110},
	[ 2] = { .submit =  3, .complete = 111},
	[ 3] = { .submit =  4, .complete = 112},
	[ 4] = { .submit =  5, .complete = 113},
	[ 5] = { .submit =  6, .complete = 114},
	[ 6] = { .submit =  7, .complete = 115},
	[ 7] = { .submit =  8, .complete = 116},
	[ 8] = { .submit =  9, .complete = 117},
	[ 9] = { .submit = 10, .complete = 118},
	[10] = { .submit = 11, .complete = 119},
	[11] = { .submit = 12, .complete = 120},
	[12] = { .submit = 13, .complete = 121},
	[13] = { .submit = 14, .complete = 122},
	[14] = { .submit = 15, .complete = 123},

	/* USB1 ENDP 1 */
	[15] = { .submit = 16, .complete = 141},
	[16] = { .submit = 17, .complete = 142},
	[17] = { .submit = 18, .complete = 143},
	[18] = { .submit = 19, .complete = 144},
	[19] = { .submit = 20, .complete = 145},
	[20] = { .submit = 21, .complete = 146},
	[21] = { .submit = 22, .complete = 147},
	[22] = { .submit = 23, .complete = 148},
	[23] = { .submit = 24, .complete = 149},
	[24] = { .submit = 25, .complete = 150},
	[25] = { .submit = 26, .complete = 151},
	[26] = { .submit = 27, .complete = 152},
	[27] = { .submit = 28, .complete = 153},
	[28] = { .submit = 29, .complete = 154},
	[29] = { .submit = 30, .complete = 155},
};

static const struct chan_queues da8xx_usb_queues_tx[] = {
	[0] = { .submit =  16, .complete = 24},
	[1] = { .submit =  18, .complete = 24},
	[2] = { .submit =  20, .complete = 24},
	[3] = { .submit =  22, .complete = 24},
};

static const struct chan_queues da8xx_usb_queues_rx[] = {
	[0] = { .submit =  1, .complete = 26},
	[1] = { .submit =  3, .complete = 26},
	[2] = { .submit =  5, .complete = 26},
	[3] = { .submit =  7, .complete = 26},
};

struct cppi_glue_infos {
	const struct chan_queues *queues_rx;
	const struct chan_queues *queues_tx;
	struct chan_queues td_queue;
	u16 first_completion_queue;
	u16 qmgr_num_pend;
};

//Given Linux kernel DMA channel data structures, extracts the CPPI 4.1 DMA
//channel data structure.
static struct cppi41_channel *to_cpp41_chan(struct dma_chan *c)
{
//printk("drivers/dma/ti/cppi41.c:to_cpp41_chan\n");
	return container_of(c, struct cppi41_channel, chan);
}

//Returns CPPI 4.1 DMA channel to which the given BD (address) belongs to.
static struct cppi41_channel *desc_to_chan(struct cppi41_dd *cdd, u32 desc)
{
//printk("drivers/dma/ti/cppi41.c:desc_to_chan\n");
	struct cppi41_channel *c;
	u32 descs_size;
	u32 desc_num;
	descs_size = sizeof(struct cppi41_desc) * ALLOC_DECS_NUM;	//Number of bytes allocated for all descriptors.

	if (!((desc >= cdd->descs_phys) &&							//BD not in region.
			(desc < (cdd->descs_phys + descs_size)))) {
		return NULL;
	}

	desc_num = (desc - cdd->descs_phys) / sizeof(struct cppi41_desc);	//BD index.
	BUG_ON(desc_num >= ALLOC_DECS_NUM);
	c = cdd->chan_busy[desc_num];										//Get CPPI 4.1 DMA channel structure that the BD belongs to.
	cdd->chan_busy[desc_num] = NULL;									//Channel not busy since BD is been consumed.

	/* Usecount for chan_busy[], paired with push_desc_queue() */
	pm_runtime_put(cdd->ddev.dev);									//Drop device usage counter and put on queue for "idle check" if 0.

	return c;
}

//Writes 32 bits at address mem.
static void cppi_writel(u32 val, void *__iomem *mem)
{
//printk("drivers/dma/ti/cppi41.c:cppi_writel\n");
	__raw_writel(val, mem);
}

//Reads 32 bits at address mem.
static u32 cppi_readl(void *__iomem *mem)
{
//printk("drivers/dma/ti/cppi41.c:cppi_readl\n");
	return __raw_readl(mem);
}

//Obtains the packet length field.
static u32 pd_trans_len(u32 val)
{
//printk("drivers/dma/ti/cppi41.c:pd_trans_len\n");
	return val & ((1 << (DESC_LENGTH_BITS_NUM + 1)) - 1);
}

//Extract the descriptor pointer (0 if queue is empty) giving a 32-byte aligned
//physical address of a BD.
static u32 cppi41_pop_desc(struct cppi41_dd *cdd, unsigned queue_num)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_pop_desc\n");
	u32 desc;
	desc = cppi_readl(cdd->qmgr_mem + QMGR_QUEUE_D(queue_num));	//Reads QUEUE_N_D register
	desc &= ~0x1f;			//Extract the descriptor pointer (0 if queue is empty) giving a 32-byte aligned physical address of a BD.
	return desc;
}

//Checks all completion queues (starting from 93 and progresses up to 155), and
//calls the callback function for the completed BD of that channel (if any; seem
//to be no queues and thus cannot be more than one?), which reenables interrupt
//after DMA completion, handles DMA abortions, updates DMA channel transfer
//status, adds new transfers, and restarts timer.
static irqreturn_t cppi41_irq(int irq, void *data)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_irq\n");
	struct cppi41_dd *cdd = data;
	u16 first_completion_queue = cdd->first_completion_queue;	//queue number 93.
	u16 qmgr_num_pend = cdd->qmgr_num_pend;						//5 pending channels.

	struct cppi41_channel *c;
	int i;
	for (i = QMGR_PENDING_SLOT_Q(first_completion_queue); i < qmgr_num_pend;	//93/32 = 2 <= i < 5
			i++) {
		u32 val;
		u32 q_num;

		val = cppi_readl(cdd->qmgr_mem + QMGR_PEND(i));					//Reads the PEND[i], 0<=i<=4, covering 5*32 = 160 queues: 0-159.
		if (i == QMGR_PENDING_SLOT_Q(first_completion_queue) && val) {	//If i corresponds to first completion queue.
			u32 mask;
			/* set corresponding bit for completetion Q 93 */
			mask = 1 << QMGR_PENDING_BIT_Q(first_completion_queue);		//93 has bit 93%32 = 29
			/* not set all bits for queues less than Q 93 */
			mask--;														//Set bits 28-0.
			/* now invert and keep only Q 93+ set */
			val &= ~mask;												//Sets bits 31-29, and clears bits 28-0.
		}

		if (val)														//If 95, 94, 93 have pending BDs?
			__iormb();													//IO read memory barrier: Get pending reads before continuing.

		while (val) {
			u32 desc, len;

			/*
			 * This should never trigger, see the comments in
			 * push_desc_queue()
			 */
			WARN_ON(cdd->is_suspended);

			q_num = __fls(val);							//Find most significant bit of waiting queue.
			val &= ~(1 << q_num);						//Clear pending queue number of highest pending queue.
			q_num += 32 * i;							//Find absolute queue number from 0 instead of relative 32-bit group increments.
			desc = cppi41_pop_desc(cdd, q_num);			//Physical address of the first completed descriptor (0 if queue is empty).
			c = desc_to_chan(cdd, desc);				//Returns CPPI 4.1 DMA channel to which the given BD (address) belongs to.
			if (WARN_ON(!c)) {
				pr_err("%s() q %d desc %08x\n", __func__,
						q_num, desc);
				continue;
			}

			if (c->desc->pd2 & PD2_ZERO_LENGTH)
				//BD word 2 bit 19 is set if this BD (which is the first and
				//last BD, SOP=EOP) has zero length buffer.
				len = 0;
			else
				//
				len = pd_trans_len(c->desc->pd0);				//Obtains the packet length field.

			c->residue = pd_trans_len(c->desc->pd6) - len;		//Original buffer size, minus transferred bytes, equals remaining bytes.
			dma_cookie_complete(&c->txd);						//Signals the DMA kernel that the BD is completed.
			//Calls cppi41_dma_callback: Reenables interrupt after DMA
			//completion, handles DMA abortions, updates DMA channel transfer
			//status, adds new transfers, and restarts timer.
			dmaengine_desc_get_callback_invoke(&c->txd, NULL);
		}
	}
	return IRQ_HANDLED;
}

//"assign a DMA engine cookie to the descriptor"
static dma_cookie_t cppi41_tx_submit(struct dma_async_tx_descriptor *tx)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_tx_submit START\n");
	dma_cookie_t cookie;
	cookie = dma_cookie_assign(tx);
//printk("drivers/dma/ti/cppi41.c:cppi41_tx_submit END\n");
	return cookie;
}

//Linux kernel DMA engine description:
//"allocate resources and return the number of allocated descriptors""
//Initializes a DMA engine cookie to the BD of the given DMA channel.
static int cppi41_dma_alloc_chan_resources(struct dma_chan *chan)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_dma_alloc_chan_resources\n");
	struct cppi41_channel *c = to_cpp41_chan(chan);		//cppi41_channel struct contains the Linux kernel DMA channel structure.
	struct cppi41_dd *cdd = c->cdd;
	int error;

	error = pm_runtime_get_sync(cdd->ddev.dev);			//Resume device operation.
	if (error < 0) {
		dev_err(cdd->ddev.dev, "%s pm runtime get: %i\n",
			__func__, error);
		pm_runtime_put_noidle(cdd->ddev.dev);			//"Drop runtime PM usage counter of a device."

		return error;
	}

	dma_cookie_init(chan);		//"initialize the cookies for a DMA channel", used to track DMA activity via other DMA engine functions.
	dma_async_tx_descriptor_init(&c->txd, chan);		//Sets the Linux kernel DMA channel structure within the cppi41_channel structure.
	c->txd.tx_submit = cppi41_tx_submit;				//"assign a DMA engine cookie to the descriptor"

	if (!c->is_tx)										//Receive channel.
		cppi_writel(c->q_num, c->gcr_reg + RXHPCRA0);	//q_num is in [0, 155], specifying the free BD pool used for the first RX buffer.

	pm_runtime_mark_last_busy(cdd->ddev.dev);			//"Update the last access time of a device."
	pm_runtime_put_autosuspend(cdd->ddev.dev);	//"Drop device usage counter and queue autosuspend if 0." (Add autosuspend queue if 0.)

	return 0;
}

//Linux kernel DMA engine description:
//"release DMA channel's resources"
//Resume device operation of the DMA controller of which the DMA channel is a
//part.
static void cppi41_dma_free_chan_resources(struct dma_chan *chan)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_dma_free_chan_resources\n");
	struct cppi41_channel *c = to_cpp41_chan(chan);
	struct cppi41_dd *cdd = c->cdd;
	int error;
	error = pm_runtime_get_sync(cdd->ddev.dev);	//Resume device operation.
	if (error < 0) {
		pm_runtime_put_noidle(cdd->ddev.dev);	//"Drop runtime PM usage counter of a device."

		return;
	}

	WARN_ON(!list_empty(&cdd->pending));

	pm_runtime_mark_last_busy(cdd->ddev.dev);	//"Update the last access time of a device."
	pm_runtime_put_autosuspend(cdd->ddev.dev);	//"Drop device usage counter and queue autosuspend if 0." (Add autosuspend queue if 0.)
}

//Linux kernel DMA engine description:
//"release DMA channel's resources"
//Given a Linux DMA channel data structure, sets "the remaining number of bytes
//left to transmit on the selected transfer for states DMA_IN_PROGRESS and
//DMA_PAUSED if this is implemented in the driver, else 0"
static enum dma_status cppi41_dma_tx_status(struct dma_chan *chan,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_dma_tx_status\n");
	struct cppi41_channel *c = to_cpp41_chan(chan);
	enum dma_status ret;
	ret = dma_cookie_status(chan, cookie, txstate);

	dma_set_residue(txstate, c->residue);

	return ret;
}

//Enables DMA channel and appends a BD associated with the given DMA channel.
static void push_desc_queue(struct cppi41_channel *c)
{
//printk("drivers/dma/ti/cppi41.c:push_desc_queue\n");
	struct cppi41_dd *cdd = c->cdd;
	u32 desc_num;
	u32 desc_phys;
	u32 reg;
	c->residue = 0;					//No remaining data.

	reg = GCR_CHAN_ENABLE;			//Enables TX/RX channel
	if (!c->is_tx) {				//If receive channel,
		//RX_ERROR_HANDLING: Starvation error (descriptor or buffer starvation),
		//result in retry of BD allocation mode.
		reg |= GCR_STARV_RETRY;
		reg |= GCR_DESC_TYPE_HOST;	//Only option, which means that the default descriptor is for host; host/CPU manages them?
		reg |= c->q_comp_num;		//Default receive queue that this channel should use.
	}

	cppi_writel(reg, c->gcr_reg);	//Writes TX/RXGCR

	/*
	 * We don't use writel() but __raw_writel() so we have to make sure
	 * that the DMA descriptor in coherent memory made to the main memory
	 * before starting the dma engine.
	 */
	__iowmb();

	/*
	 * DMA transfers can take at least 200ms to complete with USB mass
	 * storage connected. To prevent autosuspend timeouts, we must use
	 * pm_runtime_get/put() when chan_busy[] is modified. This will get
	 * cleared in desc_to_chan() or cppi41_stop_chan() depending on the
	 * outcome of the transfer.
	 */
	pm_runtime_get(cdd->ddev.dev);	//"Bump up usage counter and queue up resume of a device." (Add device to queue of re-activation?)

	desc_phys = lower_32_bits(c->desc_phys);								//Physical address of BD of DMA channel.
	desc_num = (desc_phys - cdd->descs_phys) / sizeof(struct cppi41_desc);	//Index of BD in BD memory region.
	WARN_ON(cdd->chan_busy[desc_num]);
	cdd->chan_busy[desc_num] = c;											//Mark DMA channel as active.

	reg = (sizeof(struct cppi41_desc) - 24) / 4;					//Encoding of descriptor size (set to 2): BD size = 24 + DESC_SIZE*4.
	reg |= desc_phys;												//Adds physical address of BD to append: 32-byte aligned.
	cppi_writel(reg, cdd->qmgr_mem + QMGR_QUEUE_D(c->q_num));		//Writes QUEUE_N_D register to append BD of DMA channel.
}

/*
 * Caller must hold cdd->lock to prevent push_desc_queue()
 * getting called out of order. We have both cppi41_dma_issue_pending()
 * and cppi41_runtime_resume() call this function.
 */
//Enables the BD queue of all DMA channels with pending transfers (each channel
//has only one BD?).
static void cppi41_run_queue(struct cppi41_dd *cdd)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_run_queue\n");
	struct cppi41_channel *c, *_c;
	//Processes all DMA channels that have pending transfers (data transfers to be performed but where no BD has been added?).
	list_for_each_entry_safe(c, _c, &cdd->pending, node) {
		push_desc_queue(c);								//Enables DMA channel and appends a BD associated with the given DMA channel.
		list_del(&c->node);
	}
}

//Linux kernel DMA engine description:
//"push pending transactions to hardware"
//Appends the given DMA channel to the list of channels having pending transfers of the DMAC.
//If DMAC is not suspended/paused, enables all DMA channels with pending transfers and
//appends their BDs to their queues.
static void cppi41_dma_issue_pending(struct dma_chan *chan)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_dma_issue_pending\n");
	struct cppi41_channel *c = to_cpp41_chan(chan);
	struct cppi41_dd *cdd = c->cdd;
	unsigned long flags;
	int error;
	//"Bump up usage counter and queue up resume of a device." (Add device to queue of re-activation?)
	error = pm_runtime_get(cdd->ddev.dev);
	if ((error != -EINPROGRESS) && error < 0) {
		pm_runtime_put_noidle(cdd->ddev.dev);	//"Drop runtime PM usage counter of a device."
		dev_err(cdd->ddev.dev, "Failed to pm_runtime_get: %i\n",
			error);

		return;
	}

	spin_lock_irqsave(&cdd->lock, flags);
	list_add_tail(&c->node, &cdd->pending);	//Appends the given DMA channel to the list of channels having pending transfers of the DMAC.
	if (!cdd->is_suspended)					//If DMAC is not suspended/paused, enables all DMA channels with pending transfers and
		cppi41_run_queue(cdd);				//appends their BDs to their queues (each channel has only one BD?).
	spin_unlock_irqrestore(&cdd->lock, flags);

	pm_runtime_mark_last_busy(cdd->ddev.dev);	//"Update the last access time of a device."
	pm_runtime_put_autosuspend(cdd->ddev.dev);	//"Drop device usage counter and queue autosuspend if 0." (Add autosuspend queue if 0.)
}

//Sets the packet length.
static u32 get_host_pd0(u32 length)
{
//printk("drivers/dma/ti/cppi41.c:get_host_pd0\n");
	u32 reg;
	reg = DESC_TYPE_HOST << DESC_TYPE;
	reg |= length;

	return reg;
}

//Stores 0, inconsistent with specification (e.g. descriptor type = 0x10).
static u32 get_host_pd1(struct cppi41_channel *c)
{
//printk("drivers/dma/ti/cppi41.c:get_host_pd1\n");
	u32 reg;
	reg = 0;

	return reg;
}

//Stores 0.
static u32 get_host_pd2(struct cppi41_channel *c)
{
//printk("drivers/dma/ti/cppi41.c:get_host_pd2\n");
	u32 reg;
	reg = DESC_TYPE_USB;
	reg |= c->q_comp_num;

	return reg;
}

//Sets the buffer length of addressed by this BD.
static u32 get_host_pd3(u32 length)
{
//printk("drivers/dma/ti/cppi41.c:get_host_pd3\n");
	u32 reg;
	/* PD3 = packet size */
	reg = length;

	return reg;
}

//Sets original buffer length field of the BD, which is not modified by hardware.
static u32 get_host_pd6(u32 length)
{
//printk("drivers/dma/ti/cppi41.c:get_host_pd6\n");
	u32 reg;
	/* PD6 buffer size */
	reg = DESC_PD_COMPLETE;
	reg |= length;

	return reg;
}

//Buffer pointer (physical address of start of addressed BD buffer), or
//original buffer pointer of the BD, not overwritten by hardware.
static u32 get_host_pd4_or_7(u32 addr)
{
//printk("drivers/dma/ti/cppi41.c:get_host_pd4_or_7\n");
	u32 reg;
	reg = addr;

	return reg;
}

//Stores 0, the next descriptor field, meaning this is the last buffer of the
//packet.
static u32 get_host_pd5(void)
{
//printk("drivers/dma/ti/cppi41.c:get_host_pd5\n");
	u32 reg;
	reg = 0;

	return reg;
}

//Linux kernel DMA engine description:
//"prepares a slave dma operation"
//Initializes a BD for each scatter-gather element on the given DMA channel.
static struct dma_async_tx_descriptor *cppi41_dma_prep_slave_sg(
	struct dma_chan *chan, struct scatterlist *sgl, unsigned sg_len,
	enum dma_transfer_direction dir, unsigned long tx_flags, void *context)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_dma_prep_slave_sg START\n");
	struct cppi41_channel *c = to_cpp41_chan(chan);
	struct dma_async_tx_descriptor *txd = NULL;
	struct cppi41_dd *cdd = c->cdd;
	struct cppi41_desc *d;
	struct scatterlist *sg;
	unsigned int i;
	int error;

	//"Bump up usage counter and queue up resume of a device." (Add device to queue of re-activation?)
	error = pm_runtime_get(cdd->ddev.dev);
	if (error < 0) {
		pm_runtime_put_noidle(cdd->ddev.dev);		//"Drop runtime PM usage counter of a device."
		return NULL;
	}

	if (cdd->is_suspended)							//DMA controller suspended.
		goto err_out_not_ready;

	d = c->desc;									//Channel descriptor.
	//Should be at most one iteration since each channel only has one BD.
	for_each_sg(sgl, sg, sg_len, i) {				//Goes through all scatter gather elements in the scatter-gather list sgl.
		u32 addr;
		u32 len;

		/* We need to use more than one desc once musb supports sg */
		addr = lower_32_bits(sg_dma_address(sg));	//Gets DMA address (physical address of buffer).
		len = sg_dma_len(sg);						//Length of the byte size of the buffer.
		//All BDs are packet descriptors (only one BD in each queue; singelton
		//queues).
		d->pd0 = get_host_pd0(len);					//Sets the packet length.
		d->pd1 = get_host_pd1(c);					//Stores 0, inconsistent with specification (e.g. descriptor type = 0x10).
		d->pd2 = get_host_pd2(c);					//Stores 0.
		d->pd3 = get_host_pd3(len);					//Sets the buffer length of addressed by this BD.
		d->pd4 = get_host_pd4_or_7(addr);			//Buffer pointer (physical address of start of addressed BD buffer).
		d->pd5 = get_host_pd5();					//Stores 0, the next descriptor field, meaning this is the last buffer of the packet.
		d->pd6 = get_host_pd6(len);					//Sets original buffer length field of the BD, which is not modified by hardware.
		d->pd7 = get_host_pd4_or_7(addr);			//Original buffer pointer of the BD, not overwritten by hardware.

		d++;
	}

	txd = &c->txd;

err_out_not_ready:
	pm_runtime_mark_last_busy(cdd->ddev.dev);		//"Update the last access time of a device."
	pm_runtime_put_autosuspend(cdd->ddev.dev);	//"Drop device usage counter and queue autosuspend if 0." (Add autosuspend queue if 0.)
//printk("drivers/dma/ti/cppi41.c:cppi41_dma_prep_slave_sg END\n");
	return txd;
}

//Sets descriptor type to teardown.
static void cppi41_compute_td_desc(struct cppi41_desc *d)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_compute_td_desc\n");
	d->pd0 = DESC_TYPE_TEARD << DESC_TYPE;
}

//Obtains the TD BD.
//
//If the TD BD has been previously queued, initializes it and appends it to the
//TD submit queue and activates teardown for the given DMA channel, and sets the
//TD queued flag for the given DMA channel.
//
//If the TD BD has not been processed or channel has not been torn down, then
//1. Pops TD BD from TD completion queue. If there is no completed TX BD on the
//   TD queue, then pops BD from DMA channel's queue, otherwise keeps the
//   completed TD BD.
//2. If the popped BD belongs to the given DMA channel, sets the td_desc_seen
//   flag. Otherwise, if the popped BD is the TD BD, then sets the td_seen flag.
//
//If teardown is complete, then if the popped BD belongs to the given DMA
//channel, pops the BD from the DMA channel's submit/completion queues.
//
//Clears td_queued, td_desc_seen and td_seen flags of the given DMA channel, and
//disables the DMA channel.
//
//Signals to the Linux DMA engine that the DMA channel is complete/idle.
//
//Reenables interrupt after DMA completion, handles DMA abortions, updates DMA
//channel transfer status, adds new transfers, and restarts timer.
//
//Returns 0 for teardown complete.
static int cppi41_tear_down_chan(struct cppi41_channel *c)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_tear_down_chan\n");
	struct dmaengine_result abort_result;
	struct cppi41_dd *cdd = c->cdd;
	struct cppi41_desc *td;
	u32 reg;
	u32 desc_phys;
	u32 td_desc_phys;
	td = cdd->cd;						//Virtual address of base of memory region containing BDs.
	td += cdd->first_td_desc;			//Gets the virtual address of the teardown BD, which is the last one with index 30?

	td_desc_phys = cdd->descs_phys;		//Physical address of first BD.
	//Physical address of the sole exclusive/first teardown BD, from any DMA channel/endpoint?
	td_desc_phys += cdd->first_td_desc * sizeof(struct cppi41_desc);

	if (!c->td_queued) {								//No teardown operation is queued?
		cppi41_compute_td_desc(td);						//Sets descriptor type to teardown.
		__iowmb();										//Waits here until memory write is performed.

		reg = (sizeof(struct cppi41_desc) - 24) / 4;	//Encoding of descriptor size.
		reg |= td_desc_phys;							//Add 32-byte aligned physical teardown BD address.
		cppi_writel(reg, cdd->qmgr_mem +
				QMGR_QUEUE_D(cdd->td_queue.submit));	//td_queue.submit = 31.	Writes TD BD to the submit queue 31.

		reg = GCR_CHAN_ENABLE;							//Enable bit in TX/RXGCR registers to enable DMA channel.
		if (!c->is_tx) {								//If reception.
			//RX_ERROR_HANDLING: Starvation error (descriptor or buffer
			//starvation), result in retry of BD allocation mode.
			reg |= GCR_STARV_RETRY;
			reg |= GCR_DESC_TYPE_HOST;		//Only option, which means that the default descriptor is for host; host/CPU manages them?
			reg |= cdd->td_queue.complete;	//td_queue.complete = 0.
		}
		reg |= GCR_TEARDOWN;			//Teardown activation flag.
		cppi_writel(reg, c->gcr_reg);	//TX/RXGCR register of DMA channel to tear down is written, requesting channel to be torn down.
		c->td_queued = 1;				//TD BD is queued.
		c->td_retry = 500;				//Number of tear down retries?
	}

	if (!c->td_seen || !c->td_desc_seen) {					//The TD BD has not been processed or channel has not been torn down.
		//Physical address of the first completed BD (0 if queue is empty) from
		//teardown completion queue (with ID 0). Could be from any DMA
		//channel/endpoint?
		desc_phys = cppi41_pop_desc(cdd, cdd->td_queue.complete);
		if (!desc_phys && c->is_tx)
			//Empty teardown queue and transmission (Teardown is only useful for
			//transmission (TRM: 16.2.9.3.5)).
			//Physical address of the first completed descriptor (0 if queue is
			//empty) from completion queue of this DMA channel.
			desc_phys = cppi41_pop_desc(cdd, c->q_comp_num);

		if (desc_phys == c->desc_phys) {					//Completed TD BD is BD of given DMA channel.
			c->td_desc_seen = 1;							//Channel has been torn down.

		} else if (desc_phys == td_desc_phys) {				//Completed TD BD is first teardown BD.
			u32 pd0;

			__iormb();										//Wait until all reads are complete.
			pd0 = td->pd0;									//Reads teardown info of teardown BD.
			WARN_ON((pd0 >> DESC_TYPE) != DESC_TYPE_TEARD);	//Check descriptor type correctness.
			WARN_ON(!c->is_tx && !(pd0 & TD_DESC_IS_RX));	//Check TX/RX BD type consistency.
			WARN_ON((pd0 & 0x1f) != c->port_num);			//Check consistency of DMA channel number (0-29).
			c->td_seen = 1;									//The TD BD has been processed (encountered in TD completion queue).
		} else if (desc_phys) {
			WARN_ON_ONCE(1);
		}
	}
	c->td_retry--;

	/*
	 * If the TX descriptor / channel is in use, the caller needs to poke
	 * his TD bit multiple times. After that he hardware releases the
	 * transfer descriptor followed by TD descriptor. Waiting seems not to
	 * cause any difference.
	 * RX seems to be thrown out right away. However once the TearDown
	 * descriptor gets through we are done. If we have seens the transfer
	 * descriptor before the TD we fetch it from enqueue, it has to be
	 * there waiting for us.
	 */
	if (!c->td_seen && c->td_retry) {
		udelay(1);
		return -EAGAIN;											//Teardown not complete?
	}
	WARN_ON(!c->td_retry);

	if (!c->td_desc_seen) {										//Completed TD BD is not BD of given DMA channel.
		desc_phys = cppi41_pop_desc(cdd, c->q_num);				//Physical address of first BD on submit queue (0 if queue is empty).
		if (!desc_phys)											//A completed BD.
			desc_phys = cppi41_pop_desc(cdd, c->q_comp_num);	//Physical address of first BD on completion queue.
		WARN_ON(!desc_phys);
	}

	c->td_queued = 0;
	c->td_seen = 0;
	c->td_desc_seen = 0;
	cppi_writel(0, c->gcr_reg);									//Disables DMA channel.

	/* Invoke the callback to do the necessary clean-up */
	abort_result.result = DMA_TRANS_ABORTED;
	dma_cookie_complete(&c->txd);								//Signals the DMA kernel that the BD is completed.
	//Calls cppi41_dma_callback: Reenables interrupt after DMA completion,
	//handles DMA abortions, updates DMA channel transfer status, adds new
	//transfers, and restarts timer.
	dmaengine_desc_get_callback_invoke(&c->txd, &abort_result);

	return 0;													//Teardown complete?
}

//Linux DMA engine description: "Aborts all transfers on a channel. Returns 0 or an error code"
//
//If BD of channel is not active, remove it from list with pending transfers and
//return. Otherwise, tears it down and marks it as not busy.
static int cppi41_stop_chan(struct dma_chan *chan)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_stop_chan\n");
	struct cppi41_channel *c = to_cpp41_chan(chan);
	struct cppi41_dd *cdd = c->cdd;
	u32 desc_num;
	u32 desc_phys;
	int ret;

	desc_phys = lower_32_bits(c->desc_phys);								//Physical address of BD of given DMA channel.
	desc_num = (desc_phys - cdd->descs_phys) / sizeof(struct cppi41_desc);	//Index of BD in BD memory region.
	if (!cdd->chan_busy[desc_num]) {			//If BD of channel is not active, remove it from list with pending transfers and return.
		struct cppi41_channel *cc, *_ct;

		/*
		 * channels might still be in the pendling list if
		 * cppi41_dma_issue_pending() is called after
		 * cppi41_runtime_suspend() is called
		 */
		list_for_each_entry_safe(cc, _ct, &cdd->pending, node) {			//Traverse all DMA channels with active BDs
			if (cc != c)
				continue;
			list_del(&cc->node);	//Current iterated DMA channel cc is given DMA channel, remove it from list with pending transfers.
			break;
		}
		return 0;
	}
	//BD of channel is active.
	ret = cppi41_tear_down_chan(c);		//Tears down the given DMA channel.
	if (ret)							//Teardown not complete.
		return ret;
	//Teardown complete.
	WARN_ON(!cdd->chan_busy[desc_num]);
	cdd->chan_busy[desc_num] = NULL;	//Not active BD of channel.

	/* Usecount for chan_busy[], paired with push_desc_queue() */
	pm_runtime_put(cdd->ddev.dev);		//Drop device usage counter and put on queue for "idle check" if 0.

	return 0;							//Return 0 for teardown success.
}

//Allocates memory for all DMA channel structures, including the Linux kernel
//DMA channel structures, as an array with n_chans entries, and initializes the
//DMA channel structures of whether they are for transmission or reception, the
//physical and virtual addresses of their associated descriptors, and the
//virtual addresses of the associated TXGCR and RXGCR registers, which
//enable/disable, requests teardown, default BD queue number to which teardown
//BDs will be given back to the CPU, pause reception channels, controlling
//reception error handling (BD or buffer starvation), number of bytes skipped in
//the buffer from start, the default queue number from which BDs are taken for
//reception, for associated channel for corresponding endpoint.
static int cppi41_add_chans(struct device *dev, struct cppi41_dd *cdd)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_add_chans\n");
	struct cppi41_channel *cchan, *chans;
	int i;
	u32 n_chans = cdd->n_chans;	//30 channels/endpoints.
	/*
	 * The channels can only be used as TX or as RX. So we add twice
	 * that much dma channels because USB can only do RX or TX.
	 */
	n_chans *= 2;	//60 channels.
	//Allocates memory for all DMA channel structures, including the Linux
	//kernel DMA channel structures, as an array with n_chans entries.
	chans = devm_kcalloc(dev, n_chans, sizeof(*chans), GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	for (i = 0; i < n_chans; i++) {	//For each channel, 0...59.
		cchan = &chans[i];	//Copy address of entry to cchan, which is an all-zero initialized data structure (see few lines above).

		cchan->cdd = cdd;
		if (i & 1) {				//Odd channel, 1, 31, 3, 33, ..., etc.
			cchan->gcr_reg = cdd->ctrl_mem + DMA_TXGCR(i >> 1);	//Virtual address of TXGCR[i] register, 30 of which, 0 to 29.
			cchan->is_tx = 1;		//TX endpoint.
		} else {					//Even channel, 2, 32, 4, 34, ..., etc.
			cchan->gcr_reg = cdd->ctrl_mem + DMA_RXGCR(i >> 1);	//Virtual address of RXGCR[i] register, 30 of which, 0 to 29.
			cchan->is_tx = 0;		//RX endpoint.
		}
		//port number is equal to paired channels/endpoints indexes:
		//0 -> 0, 1 -> 0, 2 -> 1, 3 -> 1, 4 -> 2, 5 -> 2, 6 -> 3, 7 -> 3, 8 -> 4, 9 -> 4, etc.
		cchan->port_num = i >> 1;
		cchan->desc = &cdd->cd[i];					//Virtual address of the descriptor of this endpoint/channel with index 0 <= i < 60.
		cchan->desc_phys = cdd->descs_phys;			//Physical address of descriptors in BD memory regions.
		//Physical address of the descriptor of this endpoint/channel with index 0 <= i < 60.
		cchan->desc_phys += i * sizeof(struct cppi41_desc);
		cchan->chan.device = &cdd->ddev;			//The Linux kernel DMA device data structure.
		//Adds the address of the device_node field of this channel's CPPI 4.1 channel data structure.
		list_add_tail(&cchan->chan.device_node, &cdd->ddev.channels);
	}
	cdd->first_td_desc = n_chans;	//ID of queue with teardown descriptors or offset of TD BD?

	return 0;
}

//Frees the memory used to store BDs.
static void purge_descs(struct device *dev, struct cppi41_dd *cdd)
{
//printk("drivers/dma/ti/cppi41.c:purge_descs\n");
	unsigned int mem_decs;
	int i;
	mem_decs = ALLOC_DECS_NUM * sizeof(struct cppi41_desc);	//Number of bytes for descriptors.

	for (i = 0; i < DESCS_AREAS; i++) {						//1.

		//Writes 0 to QMEMRBASE0 register, containing the base address of the BD
		//memory region 0, storing all BDs. Probably unnecessary.
		cppi_writel(0, cdd->qmgr_mem + QMGR_MEMBASE(i));
		//Writes 0 to QMEMCTRL0 register, containing the index of the region in
		//LRAM0 if where linking information is stored, the size of descriptors
		//and the size of the BD memory region. Probably unnecessary.
		cppi_writel(0, cdd->qmgr_mem + QMGR_MEMCTRL(i));
		//Frees the Linux kernel memory allocated for BDs.
		dma_free_coherent(dev, mem_decs, cdd->cd,
				cdd->descs_phys);
	}
}

//Disables the DMA scheduler by writing the DMA_SCHED_CTRL register.
static void disable_sched(struct cppi41_dd *cdd)
{
//printk("drivers/dma/ti/cppi41.c:disable_sched\n");
	cppi_writel(0, cdd->sched_mem + DMA_SCHED_CTRL);
}

//Disables the DMA scheduler and frees the memory regions used to store BDs and
//Linking RAM.
static void deinit_cppi41(struct device *dev, struct cppi41_dd *cdd)
{
//printk("drivers/dma/ti/cppi41.c:deinit_cppi41\n");
	disable_sched(cdd);		//Disables the DMA scheduler by writing the DMA_SCHED_CTRL register.

	purge_descs(dev, cdd);	//Frees the memory used to store BDs.

	//Clears the LRAM0BASE register, storing the base address of LRAM0. Probably
	//unnecessary.
	cppi_writel(0, cdd->qmgr_mem + QMGR_LRAM0_BASE);
	cppi_writel(0, cdd->qmgr_mem + QMGR_LRAM0_BASE);
	dma_free_coherent(dev, QMGR_SCRATCH_SIZE, cdd->qmgr_scratch,	//Frees Linux kernel memory used to store linking RAM information.
			cdd->scratch_phys);
}

//Initializes the location of BD memory region 0, its BD sizes, number of BDs,
//and the location of linking information within Linking RAM 0.
static int init_descs(struct device *dev, struct cppi41_dd *cdd)
{
//printk("drivers/dma/ti/cppi41.c:init_descs\n");
	unsigned int desc_size;
	unsigned int mem_decs;
	int i;
	u32 reg;
	u32 idx;
	BUILD_BUG_ON(sizeof(struct cppi41_desc) &
			(sizeof(struct cppi41_desc) - 1));
	BUILD_BUG_ON(sizeof(struct cppi41_desc) < 32);
	BUILD_BUG_ON(ALLOC_DECS_NUM < 32);

	desc_size = sizeof(struct cppi41_desc);	//Number of bytes of one BD: 8*4 bytes = 32 bytes.
	mem_decs = ALLOC_DECS_NUM * desc_size;	//Number bytes for all descriptors: 128 * 32 = 4 kiB.

	idx = 0;
	for (i = 0; i < DESCS_AREAS; i++) {		//1 Linking RAM.

		reg = idx << QMGR_MEMCTRL_IDX_SH;	//Shift index 16 bits to the left.
		reg |= (ilog2(desc_size) - 5) << QMGR_MEMCTRL_DESC_SH;	//Number of bits to represent BD size - 5 = 0, shifted left 8 bits.
		reg |= ilog2(ALLOC_DECS_NUM) - 5;						//7 - 5 = 2.

		BUILD_BUG_ON(DESCS_AREAS != 1);
		//Allocates 4 KiB bytes in RAM with virtual address stored in
		//cdd->cd, and physical address stored in cdd->descs_phys.
		cdd->cd = dma_alloc_coherent(dev, mem_decs,
				&cdd->descs_phys, GFP_KERNEL);
		if (!cdd->cd)
			return -ENOMEM;

		//Writes physical address to cdd->qmgr_mem + 0x1000 + 0x0 = QMEMRBASE0.
		//This register contains the base address of the memory region 0 (there
		//are 8 for storing BDs).
		cppi_writel(cdd->descs_phys, cdd->qmgr_mem + QMGR_MEMBASE(i));
		//Writes QMEMCTRL0:
		//START_INDEX[29-16]: 0, meaning that descriptor linking information of BD memory region 0 is stored at the start of LRAM0.
		//DESC_SIZE[11-8]: 2^(5 + DESC_SIZE[11-8]) = 2^5 = 32 bytes = the size of each BD in this BD memory region 0.
		//REG_SIZE[2-0]: 2^(5 + REG_SIZE[2-0]) = 2^7 = 128 descriptors are stored in this BD memory region 0.
		cppi_writel(reg, cdd->qmgr_mem + QMGR_MEMCTRL(i));

		idx += ALLOC_DECS_NUM;
	}

	return 0;
}

//Enables scheduling and initializes scheduling for all endpoints, each having
//one transmit and one receive queue: Endpoint 1 gets queue 0, endpoint 2 gets
//queue 1, etc.
static void init_sched(struct cppi41_dd *cdd)
{
//printk("drivers/dma/ti/cppi41.c:init_sched\n");
	unsigned ch;
	unsigned word;
	u32 reg;
	word = 0;
	//Writes 0 to DMA_SCHED_CTRL register: Disabling scheduling and setting
	//number of entries to 1 (one above 0).
	cppi_writel(0, cdd->sched_mem + DMA_SCHED_CTRL);
	//Scheduling: For each endpoint, transmission first, then reception, and
	//then the same for next index.
	//Processes all 30 TX and 30 RX channels/endpoints (2 USBs, each with 15 TX
	//and 15 RX endpoints), two at a time.
	for (ch = 0; ch < cdd->n_chans; ch += 2) {

		reg = SCHED_ENTRY0_CHAN(ch);						//Channel/Endpoint index in bits 4-0, and "sets" TX flag in bit 7.
		reg |= SCHED_ENTRY1_CHAN(ch) | SCHED_ENTRY1_IS_RX;	//Channel/Endpoint index in bits 12-8, and sets RX flag set in bit 15.
//printk("drivers/dma/ti/cppi41.c:init_sched: %d\n", ch);
		reg |= SCHED_ENTRY2_CHAN(ch + 1);					//Channel/Endpoint index + 1 in bits 20-16, and "sets" TX flag in bit 23.
		reg |= SCHED_ENTRY3_CHAN(ch + 1) | SCHED_ENTRY3_IS_RX;	//Channel/Endpoint index + 1 in bits 28-24, and RX flag set in bit 31.
		cppi_writel(reg, cdd->sched_mem + DMA_SCHED_WORD(word));	//Writes WORD[word]
		word++;
	}
	//Number of entries is equal to number of endpoints/channels times 2 for
	//both reception and transmission, giving 60 channels, where 59, means last
	//entry is 60.
	reg = cdd->n_chans * 2 - 1;
	reg |= DMA_SCHED_CTRL_EN;								//Enables scheduling.
	cppi_writel(reg, cdd->sched_mem + DMA_SCHED_CTRL);		//Writes DMA_SCHED_CTRL with last entry and enables scheduling.
}

//Alloctes and maps into the virtual address space memory for linking RAM and
//BDs, initializes LRAM0 registers, assigns the teardown queue, and initializes
//and enables the scheduler.
static int init_cppi41(struct device *dev, struct cppi41_dd *cdd)
{
//printk("drivers/dma/ti/cppi41.c:init_cppi41: %d\n", cdd->td_queue.submit);
	int ret;
	BUILD_BUG_ON(QMGR_SCRATCH_SIZE > ((1 << 14) - 1));
	//Allocates 512 bytes in RAM for linking information with virtual address
	//stored in cdd->qmgr_scratch, and physical address stored in
	//cdd->scratch_phys (linking RAM).
	cdd->qmgr_scratch = dma_alloc_coherent(dev, QMGR_SCRATCH_SIZE,
			&cdd->scratch_phys, GFP_KERNEL);
	if (!cdd->qmgr_scratch)
		return -ENOMEM;

	//Writes the physical address for descriptors into LRAM0BASE register.
	cppi_writel(cdd->scratch_phys, cdd->qmgr_mem + QMGR_LRAM0_BASE);
	//Writes the number of descriptors (128, requiring 128 entries) into the
	//LRAM0SIZE register.
	cppi_writel(TOTAL_DESCS_NUM, cdd->qmgr_mem + QMGR_LRAM_SIZE);
	//Writes 0 to the LRAM1BASE register. Probably unnecessary.
	cppi_writel(0, cdd->qmgr_mem + QMGR_LRAM1_BASE);
	//Initializes the location of BD memory region 0, its BD sizes, number of
	//BDs, and the location of linking information within Linking RAM 0.
	ret = init_descs(dev, cdd);
	if (ret)
		goto err_td;

	//Teardown is only useful for transmission (TRM: 16.2.9.3.5)
	//submit is initialized to 31. See am335x_usb_infos.
	//31 is Free Descriptor Queues/Rx Submit Queues (USB0/1 Rx Endpoints)
	//(TRM: 16.2.9.3.1).
	//CPPI DMA Teardown Free Descriptor Queue Control Register
	//"This field controls which [...] queues [...] should be read in order to
	// allocate channel teardown descriptors."
	cppi_writel(cdd->td_queue.submit, cdd->ctrl_mem + DMA_TDFDQ);
	//Enables scheduling and initializes scheduling for all endpoints, each
	//having one transmit and one receive queue: Endpoint 1 gets queue 0,
	//endpoint 2 gets queue 1, etc.
	init_sched(cdd);

	return 0;
err_td:
	//Disables the DMA scheduler and frees the memory regions used to store BDs
	//and Linking RAM.
	deinit_cppi41(dev, cdd);
	return ret;
}

static struct platform_driver cpp41_dma_driver;
/*
 * The param format is:
 * X Y
 * X: Port
 * Y: 0 = RX else TX
 */
#define INFO_PORT	0
#define INFO_IS_TX	1

//Initializes submit and completion queue numbers of the given DMA channel.
static bool cpp41_dma_filter_fn(struct dma_chan *chan, void *param)
{
//printk("drivers/dma/ti/cppi41.c:cpp41_dma_filter_fn\n");
	struct cppi41_channel *cchan;
	struct cppi41_dd *cdd;
	const struct chan_queues *queues;
	u32 *num = param;
	if (chan->device->dev->driver != &cpp41_dma_driver.driver)
		return false;

	cchan = to_cpp41_chan(chan);

	if (cchan->port_num != num[INFO_PORT])
		return false;
//Correct DMA channel number?
	if (cchan->is_tx && !num[INFO_IS_TX])
		return false;
//Correct DMA channel type?
	cdd = cchan->cdd;
	if (cchan->is_tx)				//Transmit DMA channel.
		queues = cdd->queues_tx;
	else							//Receive DMA channel.
		queues = cdd->queues_rx;

	BUILD_BUG_ON(ARRAY_SIZE(am335x_usb_queues_rx) !=
		     ARRAY_SIZE(am335x_usb_queues_tx));
	if (WARN_ON(cchan->port_num >= ARRAY_SIZE(am335x_usb_queues_rx)))
		return false;

	//Initialized by am335x_usb_queues_rx.
	cchan->q_num = queues[cchan->port_num].submit;			//Initialize the submit queue of the given DMA channel.
	cchan->q_comp_num = queues[cchan->port_num].complete;	//Initialize the completion queue of the given DMA channel.
	return true;
}

static struct of_dma_filter_info cpp41_dma_info = {
	.filter_fn = cpp41_dma_filter_fn,
};

//cppi41_dma_xlate is a translation function converting phandle arguments to a
//Linux kernel dma_chan structure.
static struct dma_chan *cppi41_dma_xlate(struct of_phandle_args *dma_spec,
		struct of_dma *ofdma)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_dma_xlate\n");
	int count = dma_spec->args_count;
	struct of_dma_filter_info *info = ofdma->of_dma_data;
	if (!info || !info->filter_fn)
		return NULL;

	if (count != 2)
		return NULL;
//printk("drivers/dma/ti/cppi41.c:cppi41_dma_xlate\n");
	return dma_request_channel(info->dma_cap, info->filter_fn,	//"try to allocate an exclusive channel"
			&dma_spec->args[0]);
}

static const struct cppi_glue_infos am335x_usb_infos = {
	.queues_rx = am335x_usb_queues_rx,
	.queues_tx = am335x_usb_queues_tx,
	.td_queue = { .submit = 31, .complete = 0 },
	.first_completion_queue = 93,
	.qmgr_num_pend = 5,
};

static const struct cppi_glue_infos da8xx_usb_infos = {
	.queues_rx = da8xx_usb_queues_rx,
	.queues_tx = da8xx_usb_queues_tx,
	.td_queue = { .submit = 31, .complete = 0 },
	.first_completion_queue = 24,
	.qmgr_num_pend = 2,
};

static const struct of_device_id cppi41_dma_ids[] = {
	{ .compatible = "ti,am3359-cppi41", .data = &am335x_usb_infos},
	{ .compatible = "ti,da830-cppi41", .data = &da8xx_usb_infos},
	{},
};
MODULE_DEVICE_TABLE(of, cppi41_dma_ids);

static const struct cppi_glue_infos *get_glue_info(struct device *dev)
{
//printk("drivers/dma/ti/cppi41.c:get_glue_info\n");
	const struct of_device_id *of_id;
	of_id = of_match_node(cppi41_dma_ids, dev->of_node);
	if (!of_id)
		return NULL;
	return of_id->data;
}

#define CPPI41_DMA_BUSWIDTHS	(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
				BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
				BIT(DMA_SLAVE_BUSWIDTH_3_BYTES) | \
				BIT(DMA_SLAVE_BUSWIDTH_4_BYTES))

//Allocate memory for cppi41_dd struct containing dma_device struct.
//
//Maps the virtual addresses to the physical addresses of USB CPPI DMA Controller.
//Maps the virtual addresses to the physical addresses of USB CPPI DMA Scheduler.
//Maps the virtual addresses to the physical addresses of USB Queue Manager.
//
//Initializes power management.
//
//Alloctes and maps into the virtual address space memory for linking RAM and
//BDs, assigns the teardown queue, and initializes and enables the scheduler.
//Allocates and initializes memory for all DMA channel structures (transmission
//or reception, physical and virtual addresses of their associated descriptors,
//virtual addresses of the associated TXGCR and RXGCR registers, default BD
//queue number to which teardown BDs will be given back to the CPU, the default
//queue number from which BDs are taken.
//
//Initializes interrupts.
static int cppi41_dma_probe(struct platform_device *pdev)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_dma_probe: %s\n", pdev->name);
	struct cppi41_dd *cdd;													//Contains the dma_device struct.
	struct device *dev = &pdev->dev;										//0x4740_0000.dma-controller
	//Structure stpring TX and RX channels, teardown queue, completion queue and pending queues in the queue manager.
	const struct cppi_glue_infos *glue_info;
	struct resource *mem;
	int index;
	int irq;
	int ret;
	glue_info = get_glue_info(dev);
	if (!glue_info)
		return -EINVAL;

	cdd = devm_kzalloc(&pdev->dev, sizeof(*cdd), GFP_KERNEL);	//Allocate memory for cppi41_dd struct containing dma_device struct.
	if (!cdd)
		return -ENOMEM;

	dma_cap_set(DMA_SLAVE, cdd->ddev.cap_mask);					//This DMA controller is a slave: Operates on behalf of the CPU.
	//"allocate resources and return the number of allocated descriptors"
	cdd->ddev.device_alloc_chan_resources = cppi41_dma_alloc_chan_resources;
	cdd->ddev.device_free_chan_resources = cppi41_dma_free_chan_resources;	//"release DMA channel's resources"
	cdd->ddev.device_tx_status = cppi41_dma_tx_status;						//"poll for transaction completion"
	cdd->ddev.device_issue_pending = cppi41_dma_issue_pending;				//"push pending transactions to hardware"
	cdd->ddev.device_prep_slave_sg = cppi41_dma_prep_slave_sg;				//"prepares a slave dma operation"
	cdd->ddev.device_terminate_all = cppi41_stop_chan;					//"Aborts all transfers on a channel. Returns 0 or an error code"
	cdd->ddev.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);	//"bit mask of slave directions the device supports."
	cdd->ddev.src_addr_widths = CPPI41_DMA_BUSWIDTHS;				//"bit mask of src addr widths the device supports", 1,2,3 or 4 bytes.
	cdd->ddev.dst_addr_widths = CPPI41_DMA_BUSWIDTHS;				//"bit mask of dst addr widths the device supports", 1,2,3 or 4 bytes.
	cdd->ddev.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;		//"granularity of the transfer residue reported by tx_status"
	cdd->ddev.dev = dev;												//"struct device reference for dma mapping api"
	INIT_LIST_HEAD(&cdd->ddev.channels);
	cpp41_dma_info.dma_cap = cdd->ddev.cap_mask;

	index = of_property_match_string(dev->of_node,
					 "reg-names", "controller");
	if (index < 0)
		return index;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, index);		//mem is memory resource of USB CPPI DMA Controller.
	cdd->ctrl_mem = devm_ioremap_resource(dev, mem);				//Maps the virtual addresses to the physical addresses of the USB.
	if (IS_ERR(cdd->ctrl_mem))
		return PTR_ERR(cdd->ctrl_mem);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, index + 1);	//mem is memory resource of USB CPPI DMA Scheduler.
	cdd->sched_mem = devm_ioremap_resource(dev, mem);				//Maps the virtual addresses to the physical addresses of the USB.
	if (IS_ERR(cdd->sched_mem))
		return PTR_ERR(cdd->sched_mem);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, index + 2);	//mem is memory resource of USB Queue Manager.
	cdd->qmgr_mem = devm_ioremap_resource(dev, mem);				//Maps the virtual addresses to the physical addresses of the USB.
	if (IS_ERR(cdd->qmgr_mem))
		return PTR_ERR(cdd->qmgr_mem);

	spin_lock_init(&cdd->lock);
	INIT_LIST_HEAD(&cdd->pending);							//No pending DMA channels.

	platform_set_drvdata(pdev, cdd);						//Sets the platform data of the device structure to the cppi41_dd struct.

	pm_runtime_enable(dev);									//"Enables runtime power management" of the DMA controller.
	pm_runtime_set_autosuspend_delay(dev, 100);				//"Set a device's autosuspend_delay value."
	pm_runtime_use_autosuspend(dev);						//"Allow autosuspend to be used for a device."
	ret = pm_runtime_get_sync(dev);							//Resume device.
	if (ret < 0)
		goto err_get_sync;

	cdd->queues_rx = glue_info->queues_rx;					//Submit and completion queue numbers of receive DMA channels.
	cdd->queues_tx = glue_info->queues_tx;					//Submit and completion queue numbers of transmit DMA channels.
	cdd->td_queue = glue_info->td_queue;					//Submit and completion queue numbers for teardown.
	cdd->qmgr_num_pend = glue_info->qmgr_num_pend;			//Queue number 5.
	cdd->first_completion_queue = glue_info->first_completion_queue;	//Queue number 93.

	ret = of_property_read_u32(dev->of_node,				//ret = 30 DMA channels (endpoints).
				   "#dma-channels", &cdd->n_chans);
	if (ret)
		goto err_get_n_chans;

	//Alloctes and maps into the virtual address space memory for linking RAM
	//and BDs, initializes LRAM0 registers, assigns the teardown queue, and
	//initializes and enables the scheduler.
	ret = init_cppi41(dev, cdd);
	if (ret)
		goto err_init_cppi;

//Allocates memory for all DMA channel structures, including the Linux kernel
//DMA channel structures, as an array with n_chans entries, and initializes the
//DMA channel structures of whether they are for transmission or reception, the
//physical and virtual addresses of their associated descriptors, and the
//virtual addresses of the associated TXGCR and RXGCR registers, which
//enable/disable, requests teardown, default BD queue number to which teardown
//BDs will be given back to the CPU, pause reception channels, controlling
//reception error handling (BD or buffer starvation), number of bytes skipped in
//the buffer from start, the default queue number from which BDs are taken for
//reception, for associated channel for corresponding endpoint.
	ret = cppi41_add_chans(dev, cdd);
	if (ret)
		goto err_chans;

	//Maps hardware interrupt 17 (see interrupts property of dma-controller in
	//dts) to 52. The Linux kernel must be able to handle multiple interrupt
	//controllers without the interrupt lines overlapping, and therefore the
	//kernel must remap interrupt numbers.
	irq = irq_of_parse_and_map(dev->of_node, 0);

	if (!irq) {
		ret = -EINVAL;
		goto err_chans;
	}
	//"allocate an interrupt line for a managed device", with cppi41_irq being
	//the handler with the IRQ line allowed to be shared among multiple devices.
	//Probably means that interrupt on irq are invoking cppi41_irq,
	//irrespectively of which device triggering the interrupt.
	//
	//cppi41_irq checks all completion queues (starting from 93 and progresses
	//up to 155), and calls the callback function for the completed BD of that
	//channel (if any; seem to be no queues and thus cannot be more than one?),
	//which reenables interrupt after DMA completion, handles DMA abortions,
	//updates DMA channel transfer status, adds new transfers, and restarts
	//timer.
	ret = devm_request_irq(&pdev->dev, irq, cppi41_irq, IRQF_SHARED,
			dev_name(dev), cdd);
	if (ret)
		goto err_chans;
	cdd->irq = irq;

	ret = dma_async_device_register(&cdd->ddev);	//ddev is the DMA device of the "registers DMA devices found"
	if (ret)
		goto err_chans;

	//"Register a DMA controller to DT DMA helpers"
	//cppi41_dma_xlate is a translation function converting phandle arguments to
	//a Linux kernel dma_chan structure.
	ret = of_dma_controller_register(dev->of_node,
			cppi41_dma_xlate, &cpp41_dma_info);
	if (ret)
		goto err_of;

	pm_runtime_mark_last_busy(dev);				//"Update the last access time of a device."
	pm_runtime_put_autosuspend(dev);			//"Drop device usage counter and queue autosuspend if 0." (Add autosuspend queue if 0.)

	return 0;
err_of:
	dma_async_device_unregister(&cdd->ddev);	//"unregister a DMA device"
err_chans:
	deinit_cppi41(dev, cdd);				//Disables the DMA scheduler and frees the memory regions used to store BDs and Linking RAM.
err_init_cppi:
	pm_runtime_dont_use_autosuspend(dev);		//"Prevent autosuspend from being used."
err_get_n_chans:
err_get_sync:
	pm_runtime_put_sync(dev);					//"Drop device usage counter and run "idle check" if 0."
	pm_runtime_disable(dev);					//"Disable runtime PM for a device."
	return ret;
}

//Deregisters DMA controller, frees interrupts and configures power management.
static int cppi41_dma_remove(struct platform_device *pdev)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_dma_remove\n");
	struct cppi41_dd *cdd = platform_get_drvdata(pdev);
	int error;

	error = pm_runtime_get_sync(&pdev->dev);	//Resume device operation.
	if (error < 0)
		dev_err(&pdev->dev, "%s could not pm_runtime_get: %i\n",
			__func__, error);
	of_dma_controller_free(pdev->dev.of_node);	//"Remove a DMA controller from DT DMA helpers list"
	dma_async_device_unregister(&cdd->ddev);	//"unregister a DMA device"

	devm_free_irq(&pdev->dev, cdd->irq, cdd);	//Free mapped IRQ line 52.
	//Disables the DMA scheduler and frees the memory regions used to store BDs
	//and Linking RAM.
	deinit_cppi41(&pdev->dev, cdd);
	pm_runtime_dont_use_autosuspend(&pdev->dev);	//"Prevent autosuspend from being used."
	pm_runtime_put_sync(&pdev->dev);				//"Drop device usage counter and run "idle check" if 0."
	pm_runtime_disable(&pdev->dev);					//"Disable runtime PM for a device."
	return 0;
}

//Disables DMA scheduler.
static int __maybe_unused cppi41_suspend(struct device *dev)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_suspend\n");
	struct cppi41_dd *cdd = dev_get_drvdata(dev);

	//Reads the CPPI DMA Teardown Free Descriptor Queue Control Register
	//"This field controls which [...] queues [...] should be read in order to
	// allocate channel teardown descriptors."
	cdd->dma_tdfdq = cppi_readl(cdd->ctrl_mem + DMA_TDFDQ);
	disable_sched(cdd);	//Disables the DMA scheduler by writing the DMA_SCHED_CTRL register.

	return 0;
}

//Sets the addresses of the BD memory region, address and size of linking RAM,
//and enables scheduling.
static int __maybe_unused cppi41_resume(struct device *dev)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_resume\n");
	struct cppi41_dd *cdd = dev_get_drvdata(dev);
	struct cppi41_channel *c;
	int i;

	//Writes physical address to cdd->qmgr_mem + 0x1000 + 0x0 = QMEMRBASE0. This
	//register contains the base address of the memory region 0 (there are 8 for
	//storing BDs).
	for (i = 0; i < DESCS_AREAS; i++)
		cppi_writel(cdd->descs_phys, cdd->qmgr_mem + QMGR_MEMBASE(i));

	list_for_each_entry(c, &cdd->ddev.channels, chan.device_node)
		if (!c->is_tx)	//Receive channel.
			cppi_writel(c->q_num, c->gcr_reg + RXHPCRA0);	//q_num is in [0, 155], specifying free BD pool used for the first RX buffer.

	//Enables scheduling and initializes scheduling for all endpoints, each
	//having one transmit and one receive queue: Endpoint 1 gets queue 0,
	//endpoint 2 gets queue 1, etc.
	init_sched(cdd);

	//Writes the CPPI DMA Teardown Free Descriptor Queue Control Register
	//"This field controls which [...] queues [...] should be read in order to
	// allocate channel teardown descriptors."
	cppi_writel(cdd->dma_tdfdq, cdd->ctrl_mem + DMA_TDFDQ);
	//Writes the physical address for descriptors into LRAM0BASE register.
	cppi_writel(cdd->scratch_phys, cdd->qmgr_mem + QMGR_LRAM0_BASE);
	//Writes the number of bytes of the descriptors (128, requiring 128 entries)
	//into the LRAM0SIZE register.
	cppi_writel(QMGR_SCRATCH_SIZE, cdd->qmgr_mem + QMGR_LRAM_SIZE);	//!!!!!!!!!!!!!!!BUG!!!!!!!!!!!!!!!SHOULD BE NUMBER OF BDS!!!!!!
	//Writes 0 to the LRAM1BASE register. Probably unnecessary.
	cppi_writel(0, cdd->qmgr_mem + QMGR_LRAM1_BASE);

	return 0;
}

//Sets the suspend flag of the DMA controller data structure.
static int __maybe_unused cppi41_runtime_suspend(struct device *dev)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_runtime_suspend\n");
	struct cppi41_dd *cdd = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&cdd->lock, flags);
	cdd->is_suspended = true;
	WARN_ON(!list_empty(&cdd->pending));
	spin_unlock_irqrestore(&cdd->lock, flags);

	return 0;
}

//Clears suspend flag of DMA controller, and appends 
static int __maybe_unused cppi41_runtime_resume(struct device *dev)
{
//printk("drivers/dma/ti/cppi41.c:cppi41_runtime_resume\n");
	struct cppi41_dd *cdd = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&cdd->lock, flags);
	cdd->is_suspended = false;
	cppi41_run_queue(cdd);	//Appends a BD associated with the given DMA channel, activating that channel.
	spin_unlock_irqrestore(&cdd->lock, flags);

	return 0;
}

static const struct dev_pm_ops cppi41_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(cppi41_suspend, cppi41_resume)
	SET_RUNTIME_PM_OPS(cppi41_runtime_suspend,
			   cppi41_runtime_resume,
			   NULL)
};

static struct platform_driver cpp41_dma_driver = {
	.probe  = cppi41_dma_probe,
	.remove = cppi41_dma_remove,
	.driver = {
		.name = "cppi41-dma-engine",
		.pm = &cppi41_pm_ops,
		.of_match_table = of_match_ptr(cppi41_dma_ids),
	},
};

module_platform_driver(cpp41_dma_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Andrzej Siewior <bigeasy@linutronix.de>");
