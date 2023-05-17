/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __ASM_UACCESS_ASM_H__
#define __ASM_UACCESS_ASM_H__

#include <asm/asm-offsets.h>
#include <asm/domain.h>
#include <asm/memory.h>
#include <asm/thread_info.h>

	.macro	csdb
#ifdef CONFIG_THUMB2_KERNEL
	.inst.w	0xf3af8014
#else
	.inst	0xe320f014
#endif
	.endm

	.macro check_uaccess, addr:req, size:req, limit:req, tmp:req, bad:req
#ifndef CONFIG_CPU_USE_DOMAINS
	adds	\tmp, \addr, #\size - 1
	sbcscc	\tmp, \tmp, \limit
	bcs	\bad
#ifdef CONFIG_CPU_SPECTRE
	movcs	\addr, #0
	csdb
#endif
#endif
	.endm

	.macro uaccess_mask_range_ptr, addr:req, size:req, limit:req, tmp:req
#ifdef CONFIG_CPU_SPECTRE
	sub	\tmp, \limit, #1
	subs	\tmp, \tmp, \addr	@ tmp = limit - 1 - addr
	addhs	\tmp, \tmp, #1		@ if (tmp >= 0) {
	subshs	\tmp, \tmp, \size	@ tmp = limit - (addr + size) }
	movlo	\addr, #0		@ if (tmp < 0) addr = NULL
	csdb
#endif
	.endm

	//Sets DACR such that user addresses are not accessible, and all other
	//addresses accessible as specified by the page tables.
	.macro	uaccess_disable, tmp, isb=1
#ifdef CONFIG_CPU_SW_DOMAIN_PAN	//Defined.
	/*
	 * Whenever we re-enter userspace, the domains should always be
	 * set appropriately.
	 */
	//Sets \tmp to the DACR configuration that specifies user addresses as not
	//accessible, and all other addresses accessible according to page tables.
	mov	\tmp, #DACR_UACCESS_DISABLE
	//Sets DACR to the above DACR configuration
	mcr	p15, 0, \tmp, c3, c0, 0		@ Set domain register
	.if	\isb
	//"Instruction Synchronization Barrier flushes the pipeline in the
	// processor, so that all instructions following the ISB are fetched from
	// cache or memory, after the instruction has been completed."
	instr_sync
	.endif
#endif
	.endm

	//Sets DACR such that all addresses of the kernel, I/O, exception vectors
	//and handlers, and user space, are accessible according to the page tables.
	.macro	uaccess_enable, tmp, isb=1
#ifdef CONFIG_CPU_SW_DOMAIN_PAN
	/*
	 * Whenever we re-enter userspace, the domains should always be
	 * set appropriately.
	 */
	//DACR_UACCESS_ENABLE: All addresses of the kernel, I/O, exception vectors
	//and handlers, and user space, are accessible according to the page tables.
	mov	\tmp, #DACR_UACCESS_ENABLE
	//Set DACR to DACR_UACCESS_ENABLE.
#ifndef CONFIG_TRUSTFULL_HYPERVISOR
	mcr	p15, 0, \tmp, c3, c0, 0
#endif
	.if	\isb
	//"Instruction Synchronization Barrier flushes the pipeline in the
	// processor, so that all instructions following the ISB are fetched from
	// cache or memory, after the instruction has been completed."
	instr_sync
	.endif
#endif
	.endm

//CONFIG_CPU_USE_DOMAINS = FALSE
#if defined(CONFIG_CPU_SW_DOMAIN_PAN) || defined(CONFIG_CPU_USE_DOMAINS)
#define DACR(x...)	x
#else
#define DACR(x...)
#endif

	/*
	 * Save the address limit on entry to a privileged exception.
	 *
	 * If we are using the DACR for kernel access by the user accessors
	 * (CONFIG_CPU_USE_DOMAINS=y), always reset the DACR kernel domain
	 * back to client mode, whether or not \disable is set.
	 *
	 * If we are using SW PAN, set the DACR user domain to no access
	 * if \disable is set.
	 */
	//Stores DACR on the svc_pt_regs stack. If \disable is true, then kernel,
	//I/O and vector (exception table and stubs/handlers) addresses are
	//accessible according to page tables, and user addresses are inaccessible.
	//Otherwise, no change is made.
	.macro	uaccess_entry, tsk, tmp0, tmp1, tmp2, disable
	//Reads DACR and stores it in \tmp0.
 DACR(	mrc	p15, 0, \tmp0, c3, c0, 0)	//EMPTY since !CONFIG_CPU_USE_DOMAINS
	//Stores DACR in svc_pt_regs.dacr.
 DACR(	str	\tmp0, [sp, #SVC_DACR])		//EMPTY since !CONFIG_CPU_USE_DOMAINS
	//CONFIG_CPU_SW_DOMAIN_PAN is defined. uaccess_entry is only used in
	//arch/arm/kernel/kernel/entry-armv.S:svc_entry which uses this macro with
	//disable set to \uaccess, set as follows for the following exception
	//handlers:
	//-\uaccess = 0: __dabt_svc.
	//-\uaccess = 1 (default): __irq_svc, __und_svc, __pabt_svc, __fiq_svc,
	// __fiq_abt.
	.if \disable && IS_ENABLED(CONFIG_CPU_SW_DOMAIN_PAN)
	/* kernel=client, user=no access */
	//arch/arm/include/asm/domain.h:DACR_UACCESS_DISABLE is a constant that
	//includes the following flags:
	//-Kernel domain (all kernel memory) is set to client: Access permission
	// bits determine access rights.
	//-IO domain (all I/O addresses) is set to client: Access permission bits
	// determine access rights.
	//-Vectors domain (all vector addresses; 2 4kB pages containing vector table
	// an stubs) is set to client: Access permission bits determine access
	// rights.
	//-User domain (all user addresses) is set to no access: "Any access to the
	// domain generates a Domain fault."
	//This means that user addresses are not accessible, and all other addresses
	//are accessible according to page tables.
	mov	\tmp2, #DACR_UACCESS_DISABLE
	//Write \tmp2 to DACR.
	mcr	p15, 0, \tmp2, c3, c0, 0
	//"Instruction Synchronization Barrier flushes the pipeline in the
	// processor, so that all instructions following the ISB are fetched from
	// cache or memory, after the instruction has been completed."
	instr_sync
	.elseif IS_ENABLED(CONFIG_CPU_USE_DOMAINS)	//FALSE
	/* kernel=client */
	bic	\tmp2, \tmp0, #domain_mask(DOMAIN_KERNEL)
	orr	\tmp2, \tmp2, #domain_val(DOMAIN_KERNEL, DOMAIN_CLIENT)
	mcr	p15, 0, \tmp2, c3, c0, 0
	instr_sync
	.endif
	.endm

	/* Restore the user access state previously saved by uaccess_entry */
	//Reads DACR from svc_pt_regs, and restores DACR to that value.
	.macro	uaccess_exit, tsk, tmp0, tmp1
 DACR(	ldr	\tmp0, [sp, #SVC_DACR])		//EMPTY since !CONFIG_CPU_USE_DOMAINS
 DACR(	mcr	p15, 0, \tmp0, c3, c0, 0)	//EMPTY since !CONFIG_CPU_USE_DOMAINS
	.endm

#undef DACR

#endif /* __ASM_UACCESS_ASM_H__ */
