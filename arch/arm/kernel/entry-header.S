/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/init.h>
#include <linux/linkage.h>

#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/errno.h>
#include <asm/thread_info.h>
#include <asm/uaccess-asm.h>
#include <asm/v7m.h>

#ifdef CONFIG_TRUSTFULL_HYPERVISOR
#include "asm/trustfull/hypercalls.h"
#endif

@ Bad Abort numbers
@ -----------------
@
#define BAD_PREFETCH	0
#define BAD_DATA	1
#define BAD_ADDREXCPTN	2
#define BAD_IRQ		3
#define BAD_UNDEFINSTR	4

@
@ Most of the stack format comes from struct pt_regs, but with
@ the addition of 8 bytes for storing syscall args 5 and 6.
@ This _must_ remain a multiple of 8 for EABI.
@
#define S_OFF		8

/* 
 * The SWI code relies on the fact that R0 is at the bottom of the stack
 * (due to slow/fast restore user regs).
 */
#if S_R0 != 0
#error "Please fix"
#endif

	.macro	zero_fp
#ifdef CONFIG_FRAME_POINTER
	mov	fp, #0
#endif
	.endm

#ifdef CONFIG_ALIGNMENT_TRAP
#define ATRAP(x...) x
#else
#define ATRAP(x...)
#endif

	//Updates SCRLR to cr_alignment is not equal.
	.macro	alignment_trap, rtmp1, rtmp2, label
#ifdef CONFIG_ALIGNMENT_TRAP
	//Move to ARM core register from coprocessor register.
	mrc	p15, 0, \rtmp2, c1, c0, 0				//\rtmp2 := SCTLR.
	ldr	\rtmp1, \label							//\rtmp1 := memory[__cr_alignment] = &cr_alignment
	ldr	\rtmp1, [\rtmp1]						//\rtmp1 := cr_alignment
	//Test equivalence between cr_alignment and SCTLR. Does exclusive
	//or between cr_alignment and SCTLR, and sets the condition flags
	//accordingly. The Z flag is not set if and only if the result is not zero,
	//equivalent to SCTLR and cr_alignment not being equal.
	teq	\rtmp1, \rtmp2
	//Moves cr_alignment to SCTLR if SCTLR and cr_alignment are not equal, then
	//SCTLR is set to cr_alignment.
	mcrne	p15, 0, \rtmp1, c1, c0, 0
#endif
	.endm

#ifdef CONFIG_CPU_V7M
/*
 * ARMv7-M exception entry/exit macros.
 *
 * xPSR, ReturnAddress(), LR (R14), R12, R3, R2, R1, and R0 are
 * automatically saved on the current stack (32 words) before
 * switching to the exception stack (SP_main).
 *
 * If exception is taken while in user mode, SP_main is
 * empty. Otherwise, SP_main is aligned to 64 bit automatically
 * (CCR.STKALIGN set).
 *
 * Linux assumes that the interrupts are disabled when entering an
 * exception handler and it may BUG if this is not the case. Interrupts
 * are disabled during entry and reenabled in the exit macro.
 *
 * v7m_exception_slow_exit is used when returning from SVC or PendSV.
 * When returning to kernel mode, we don't return from exception.
 */
	.macro	v7m_exception_entry
	@ determine the location of the registers saved by the core during
	@ exception entry. Depending on the mode the cpu was in when the
	@ exception happend that is either on the main or the process stack.
	@ Bit 2 of EXC_RETURN stored in the lr register specifies which stack
	@ was used.
	tst	lr, #EXC_RET_STACK_MASK
	mrsne	r12, psp
	moveq	r12, sp

	@ we cannot rely on r0-r3 and r12 matching the value saved in the
	@ exception frame because of tail-chaining. So these have to be
	@ reloaded.
	ldmia	r12!, {r0-r3}

	@ Linux expects to have irqs off. Do it here before taking stack space
	cpsid	i

	sub	sp, #PT_REGS_SIZE-S_IP
	stmdb	sp!, {r0-r11}

	@ load saved r12, lr, return address and xPSR.
	@ r0-r7 are used for signals and never touched from now on. Clobbering
	@ r8-r12 is OK.
	mov	r9, r12
	ldmia	r9!, {r8, r10-r12}

	@ calculate the original stack pointer value.
	@ r9 currently points to the memory location just above the auto saved
	@ xPSR.
	@ The cpu might automatically 8-byte align the stack. Bit 9
	@ of the saved xPSR specifies if stack aligning took place. In this case
	@ another 32-bit value is included in the stack.

	tst	r12, V7M_xPSR_FRAMEPTRALIGN
	addne	r9, r9, #4

	@ store saved r12 using str to have a register to hold the base for stm
	str	r8, [sp, #S_IP]
	add	r8, sp, #S_SP
	@ store r13-r15, xPSR
	stmia	r8!, {r9-r12}
	@ store old_r0
	str	r0, [r8]
	.endm

        /*
	 * PENDSV and SVCALL are configured to have the same exception
	 * priorities. As a kernel thread runs at SVCALL execution priority it
	 * can never be preempted and so we will never have to return to a
	 * kernel thread here.
         */
	.macro	v7m_exception_slow_exit ret_r0
	cpsid	i
	ldr	lr, =exc_ret
	ldr	lr, [lr]

	@ read original r12, sp, lr, pc and xPSR
	add	r12, sp, #S_IP
	ldmia	r12, {r1-r5}

	@ an exception frame is always 8-byte aligned. To tell the hardware if
	@ the sp to be restored is aligned or not set bit 9 of the saved xPSR
	@ accordingly.
	tst	r2, #4
	subne	r2, r2, #4
	orrne	r5, V7M_xPSR_FRAMEPTRALIGN
	biceq	r5, V7M_xPSR_FRAMEPTRALIGN

	@ ensure bit 0 is cleared in the PC, otherwise behaviour is
	@ unpredictable
	bic	r4, #1

	@ write basic exception frame
	stmdb	r2!, {r1, r3-r5}
	ldmia	sp, {r1, r3-r5}
	.if	\ret_r0
	stmdb	r2!, {r0, r3-r5}
	.else
	stmdb	r2!, {r1, r3-r5}
	.endif

	@ restore process sp
	msr	psp, r2

	@ restore original r4-r11
	ldmia	sp!, {r0-r11}

	@ restore main sp
	add	sp, sp, #PT_REGS_SIZE-S_IP

	cpsie	i
	bx	lr
	.endm
#endif	/* CONFIG_CPU_V7M */

	@
	@ Store/load the USER SP and LR registers by switching to the SYS
	@ mode. Useful in Thumb-2 mode where "stm/ldm rd, {sp, lr}^" is not
	@ available. Should only be called from SVC mode
	@
	.macro	store_user_sp_lr, rd, rtemp, offset = 0
	mrs	\rtemp, cpsr
	eor	\rtemp, \rtemp, #(SVC_MODE ^ SYSTEM_MODE)
	msr	cpsr_c, \rtemp			@ switch to the SYS mode

	str	sp, [\rd, #\offset]		@ save sp_usr
	str	lr, [\rd, #\offset + 4]		@ save lr_usr

	eor	\rtemp, \rtemp, #(SVC_MODE ^ SYSTEM_MODE)
	msr	cpsr_c, \rtemp			@ switch back to the SVC mode
	.endm

	.macro	load_user_sp_lr, rd, rtemp, offset = 0
	mrs	\rtemp, cpsr
	eor	\rtemp, \rtemp, #(SVC_MODE ^ SYSTEM_MODE)
	msr	cpsr_c, \rtemp			@ switch to the SYS mode

	ldr	sp, [\rd, #\offset]		@ load sp_usr
	ldr	lr, [\rd, #\offset + 4]		@ load lr_usr

	eor	\rtemp, \rtemp, #(SVC_MODE ^ SYSTEM_MODE)
	msr	cpsr_c, \rtemp			@ switch back to the SVC mode
	.endm

	//Restores the CPU registers and execution mode to that before the exception
	//occurred with the program counter set to the instruction to be executed
	//next. rpsr is the register containing the CPSR value to be restored.
	.macro	svc_exit, rpsr, irq = 0
	.if	\irq != 0
	@ IRQs already off
#ifdef CONFIG_TRACE_IRQFLAGS	//FALSE
	@ The parent context IRQs must have been enabled to get here in
	@ the first place, so there's no point checking the PSR I bit.
	bl	trace_hardirqs_on
#endif
	.else
	@ IRQs off again before pulling preserved data off the stack
	//Disables IRQs.
	disable_irq_notrace
#ifdef CONFIG_TRACE_IRQFLAGS	//FALSE
	tst	\rpsr, #PSR_I_BIT
	bleq	trace_hardirqs_on
	tst	\rpsr, #PSR_I_BIT
	blne	trace_hardirqs_off
#endif
	.endif
	//Reads DACR from svc_pt_regs, and restores DACR to that value.
	uaccess_exit tsk, r0, r1

#ifdef CONFIG_TRUSTFULL_HYPERVISOR
	//Returns to point before exception now, meaning that the following code is
	//not executed.
	sub	r0, sp, #4			@ uninhabited address
	//r1 := 0 if memory[sp - 4] := r2 is sucessful. Otherwise r1 := 1.
	strex	r1, r2, [r0]			@ clear the exclusive monitor
	mov r0, sp						//Argument to HYPERCALL_RESTORE_REGS is stack pointer.
	swi HYPERCALL_RESTORE_REGS
#endif

#ifndef CONFIG_THUMB2_KERNEL	//TRUE since CONFIG_THUMB2_KERNEL is not set.
	@ ARM mode SVC restore
	//Write \rpsr to SPSR bits c (7:0), x (15:8), s (23:17), f (31:24).
	//\rpsr is set to the CPSR value before the exception was raised.
	msr	spsr_cxsf, \rpsr
#if defined(CONFIG_CPU_V6) || defined(CONFIG_CPU_32v6K)	//TRUE
	@ We must avoid clrex due to Cortex-A15 erratum #830321
	//r0 := sp - 4
	sub	r0, sp, #4			@ uninhabited address
	//r1 := 0 if memory[sp - 4] := r2 is sucessful. Otherwise r1 := 1.
	strex	r1, r2, [r0]			@ clear the exclusive monitor
#endif
	//Loads r0-pc from the values in the svc_pt_regs struct on the stack and
	//CPSR from SPSR (which was set above to the CPSR value before the
	//exception), and pc is set to the preferred exception return address
	//(potentially corrected). This instruction thus results in restoring CPU
	//registers and CPU mode before the exception occurred with the program
	//counter set to the right location, causing the CPU to execute the next
	//suitable instruction.
	ldmia	sp, {r0 - pc}^			@ load r0 - pc, cpsr
#else													//FALSE
	@ Thumb mode SVC restore
	ldr	lr, [sp, #S_SP]			@ top of the stack
	ldrd	r0, r1, [sp, #S_LR]		@ calling lr and pc

	@ We must avoid clrex due to Cortex-A15 erratum #830321
	strex	r2, r1, [sp, #S_LR]		@ clear the exclusive monitor

	stmdb	lr!, {r0, r1, \rpsr}		@ calling lr and rfe context
	ldmia	sp, {r0 - r12}
	mov	sp, lr
	ldr	lr, [sp], #4
	rfeia	sp!
#endif
	.endm

	//Restores registers and CPU mode to what it was before the FIQ occured,
	//with the PC set to the next instruction to execute.
	@
	@ svc_exit_via_fiq - like svc_exit but switches to FIQ mode before exit
	@
	@ This macro acts in a similar manner to svc_exit but switches to FIQ
	@ mode to restore the final part of the register state.
	@
	@ We cannot use the normal svc_exit procedure because that would
	@ clobber spsr_svc (FIQ could be delivered during the first few
	@ instructions of vector_swi meaning its contents have not been
	@ saved anywhere).
	@
	@ Note that, unlike svc_exit, this macro also does not allow a caller
	@ supplied rpsr. This is because the FIQ exceptions are not re-entrant
	@ and the handlers cannot call into the scheduler (meaning the value
	@ on the stack remains correct).
	@
	.macro  svc_exit_via_fiq
	//Reads DACR from svc_pt_regs, and restores DACR to that value.
	uaccess_exit tsk, r0, r1
#ifndef CONFIG_THUMB2_KERNEL
	@ ARM mode restore
	//r0 := pt_regs
	mov	r0, sp
	//Restores r1-r12, sp and lr from pt_regs. (Note ib = increment before,
	//skipping r0).
	ldmib	r0, {r1 - r14}	@ abort is deadly from here onward (it will
				@ clobber state restored below)
	//Move (FIQ_MODE (0b10001 = FIQ) | PSR_I_BIT (0b10000000 = I mask bit =
	//disable IRQ) | PSR_F_BIT (0b01000000 = F mask bit = disable FIQ)) to bits
	//7:0 (c) of special register CPSR. Hence, sets the CPU in FIQ mode with
	//FIQ and IRQ interrupts disabled, where FIQ mode has banked r9-r12, sp, lr
	//and SPSR.
	msr	cpsr_c, #FIQ_MODE | PSR_I_BIT | PSR_F_BIT
	//r8_fiq := address in pt_regs of preferred return address for exception +
	//offset. lr is corrected by fiq_vector (stub) by subtracting 'preferred
	//return address for exception + offset' by 4, where offset is 4 and
	//'preferred return address for exception' is 'Address of next instruction
	//to execute'.
	add	r8, r0, #S_PC
	//r9_fiq := CPSR before exception.
	ldr	r9, [r0, #S_PSR]
	//SPSR_fiq := r9.
	msr	spsr_cxsf, r9
	//r0 := pt_regs.r0. Registers r0-r14 are now restored.
	ldr	r0, [r0, #S_R0]
	//Moves SPSR to CPSR and pc to pt_regs.pc = address of next instruction to
	//execute, restoring the CPU state to what it was before the interrupt
	//occured.
	ldmia	r8, {pc}^
#else
	@ Thumb mode restore
	add	r0, sp, #S_R2
	ldr	lr, [sp, #S_LR]
	ldr	sp, [sp, #S_SP] @ abort is deadly from here onward (it will
			        @ clobber state restored below)
	ldmia	r0, {r2 - r12}
	mov	r1, #FIQ_MODE | PSR_I_BIT | PSR_F_BIT
	msr	cpsr_c, r1
	sub	r0, #S_R2
	add	r8, r0, #S_PC
	ldmia	r0, {r0 - r1}
	rfeia	r8
#endif
	.endm

	//1.	Sets DACR such that all addresses of the kernel, I/O, exception
	//		vectors and handlers, and user space, are accessible according to
	//		the page tables.
	//2.	Restores user mode registers to those stored in pt_regs, and resumes
	//		execution in user mode from the corrected link register address
	//		stored during the exception.
	.macro	restore_user_regs, fast = 0, offset = 0
	//Sets DACR such that all addresses of the kernel, I/O, exception vectors
	//and handlers, and user space, are accessible according to the page tables.
	//r1 is temporary register.
	uaccess_enable r1, isb=0
#ifndef CONFIG_THUMB2_KERNEL	//TRUE
	@ ARM mode restore
	//r2 := sp_svc = &pt_regs
	mov	r2, sp
	//r1 := pt_regs.CPSR = CPSR before exception.
	ldr	r1, [r2, #\offset + S_PSR]	@ get calling cpsr
	//lr := pt_regs.PC = lr_<exception processor mode> - \correction
	//sp_svc := sp_svc + #\offset + S_PC = &pt_regs.pc
	ldr	lr, [r2, #\offset + S_PC]!	@ get pc
	//AND between r1 and 0x0000008F and sets condition flags accordingly.
	tst	r1, #PSR_I_BIT | 0x0f
	//If IRQ interrupts were disabled or the CPU was not in supervisor mode
	//before the exception, then jumps to label 1f to signal an incorrect state.
	bne	1f
	//Move to special register SPSR from ARM core register r1 = CPSR before
	//exception.
	msr	spsr_cxsf, r1			@ save in spsr_svc
#if defined(CONFIG_CPU_V6) || defined(CONFIG_CPU_32v6K)	//TRUE
	@ We must avoid clrex due to Cortex-A15 erratum #830321
	//r1 := 0 if memory[sp - 4] := r2 is sucessful. Otherwise r1 := 1.
	strex	r1, r2, [r2]			@ clear the exclusive monitor
#endif
	.if	\fast
	//r1-r12 := pt_regs.r1-r12
	//sp_usr := sp_usr := pt_regs.sp
	//lr_usr := lr_<exception processor mode> - \correction
	ldmdb	r2, {r1 - lr}^			@ get calling r1 - lr
	.else
	//r0-r12 := pt_regs.r0-r12
	//sp_usr := pt_regs.sp
	//lr_usr := pt_regs.lr
	ldmdb	r2, {r0 - lr}^			@ get calling r0 - lr
	.endif
	//NOP after ldm {}^ as required by ARMv5T and earlier.
	mov	r0, r0				@ ARMv5T and earlier require a nop
						@ after ldm {}^
	//Restores stack pointer.
	add	sp, sp, #\offset + PT_REGS_SIZE
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//SETS THE CPU IN USER MODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//movs pc, lr, jumps to the address in
	//lr = lr_<exception processor mode> - \correction, and sets CPSR to SPSR,
	//which is user mode with IRQs enabled.
	movs	pc, lr				@ return & move spsr_svc into cpsr
1:	bug	"Returning to usermode but unexpected PSR bits set?", \@
#elif defined(CONFIG_CPU_V7M)	//FALSE
	@ V7M restore.
	@ Note that we don't need to do clrex here as clearing the local
	@ monitor is part of the exception entry and exit sequence.
	.if	\offset
	add	sp, #\offset
	.endif
	v7m_exception_slow_exit ret_r0 = \fast
#else							//FALSE
	@ Thumb mode restore
	mov	r2, sp
	load_user_sp_lr r2, r3, \offset + S_SP	@ calling sp, lr
	ldr	r1, [sp, #\offset + S_PSR]	@ get calling cpsr
	ldr	lr, [sp, #\offset + S_PC]	@ get pc
	add	sp, sp, #\offset + S_SP
	tst	r1, #PSR_I_BIT | 0x0f
	bne	1f
	msr	spsr_cxsf, r1			@ save in spsr_svc

	@ We must avoid clrex due to Cortex-A15 erratum #830321
	strex	r1, r2, [sp]			@ clear the exclusive monitor

	.if	\fast
	ldmdb	sp, {r1 - r12}			@ get calling r1 - r12
	.else
	ldmdb	sp, {r0 - r12}			@ get calling r0 - r12
	.endif
	add	sp, sp, #PT_REGS_SIZE - S_SP
	movs	pc, lr				@ return & move spsr_svc into cpsr
1:	bug	"Returning to usermode but unexpected PSR bits set?", \@
#endif	/* !CONFIG_THUMB2_KERNEL */
	.endm

/*
 * Context tracking subsystem.  Used to instrument transitions
 * between user and kernel mode.
 */
	.macro ct_user_exit, save = 1
#ifdef CONFIG_CONTEXT_TRACKING			//FALSE
	.if	\save
	stmdb   sp!, {r0-r3, ip, lr}
	bl	context_tracking_user_exit
	ldmia	sp!, {r0-r3, ip, lr}
	.else
	bl	context_tracking_user_exit
	.endif
#endif
	.endm

	.macro ct_user_enter, save = 1
#ifdef CONFIG_CONTEXT_TRACKING
	.if	\save
	stmdb   sp!, {r0-r3, ip, lr}
	bl	context_tracking_user_enter
	ldmia	sp!, {r0-r3, ip, lr}
	.else
	bl	context_tracking_user_enter
	.endif
#endif
	.endm

	//If the system call number is valid (less than number of system calls),
	//sets the link register to the return address and jumps to the specified
	//system call routine.
	.macro	invoke_syscall, table, nr, tmp, ret, reload=0
#ifdef CONFIG_CPU_SPECTRE
	//\tmp := \nr
	mov	\tmp, \nr
	//Compare system call number to upper limit.
	cmp	\tmp, #NR_syscalls		@ check upper syscall limit
	//If system call number is greater than or equal to number of system calls,
	//\tmp := 0
	movcs	\tmp, #0
	//"Consumption of Speculative Data Barrier is a memory barrier that controls
	// speculative execution and data value prediction. No instruction other
	// than branch instructions and instructions that write to the PC appearing
	// in program order after the CSDB can be speculatively executed using the
	// results of any:
	// • Data value predictions of any instructions.
	// • APSR.{N, Z, C, V} predictions of any instructions other than
	//   conditional branch instructions and conditional instructions that write
	//   to the PC appearing in program order before the CSDB that have not been
	//   architecturally resolved."
	//Used to thwart SPECTRE: See CONFIG_CPU_SPECTRE above.
	csdb
	//lr := address of label \ret.
	badr	lr, \ret			@ return address
	.if	\reload
	//r1 := SP + 0 + 8 = SP + 8 = &pt_regs. local_restart embedding this macro
	//stores r4 and r5 on the stack, so 8 must be added to the stack pointer to
	//get the start address of pt_regs.
	add	r1, sp, #S_R0 + S_OFF		@ pointer to regs
	//If system call number is less than number of system calls:
	//r0-r6 := pt_regs.r0-pt_regs.r6
	ldmiacc	r1, {r0 - r6}			@ reload r0-r6
	//If system call number is less than number of system calls:
	//memory[sp] := r4
	//memory[sp + 4] := r5
	stmiacc	sp, {r4, r5}			@ update stack arguments
	.endif
	//If system call number is less than number of system calls:
	//pc := address of system call table + 4*system call number = address of
	//system call routine, causing execution of that system call.
	ldrcc	pc, [\table, \tmp, lsl #2]	@ call sys_* routine
#else
	cmp	\nr, #NR_syscalls		@ check upper syscall limit
	badr	lr, \ret			@ return address
	.if	\reload
	add	r1, sp, #S_R0 + S_OFF		@ pointer to regs
	ldmiacc	r1, {r0 - r6}			@ reload r0-r6
	stmiacc	sp, {r4, r5}			@ update stack arguments
	.endif
	ldrcc	pc, [\table, \nr, lsl #2]	@ call sys_* routine
#endif
	.endm

/*
 * These are the registers used in the syscall handler, and allow us to
 * have in theory up to 7 arguments to a function - r0 to r6.
 *
 * r7 is reserved for the system call number for thumb mode.
 *
 * Note that tbl == why is intentional.
 *
 * We must set at least "tsk" and "why" when calling ret_with_reschedule.
 */
scno	.req	r7		@ syscall number
tbl	.req	r8		@ syscall table pointer
why	.req	r8		@ Linux syscall (!= 0)
tsk	.req	r9		@ current thread_info
