/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/arch/arm/kernel/entry-common.S
 *
 *  Copyright (C) 2000 Russell King
 */

#include <asm/assembler.h>
#include <asm/unistd.h>
#include <asm/ftrace.h>
#include <asm/unwind.h>
#include <asm/memory.h>
#ifdef CONFIG_AEABI
#include <asm/unistd-oabi.h>
#endif

	.equ	NR_syscalls, __NR_syscalls

#ifdef CONFIG_NEED_RET_TO_USER
#include <mach/entry-macro.S>
#else
	.macro  arch_ret_to_user, tmp1, tmp2
	.endm
#endif

#include "entry-header.S"

#ifdef CONFIG_TRUSTFULL_HYPERVISOR
	.global ret_fast_syscall
#endif

saved_psr	.req	r8
#if defined(CONFIG_TRACE_IRQFLAGS) || defined(CONFIG_CONTEXT_TRACKING)
saved_pc	.req	r9
#define TRACE(x...) x
#else
saved_pc	.req	lr
#define TRACE(x...)
#endif

	.section .entry.text,"ax",%progbits
	.align	5
#if !(IS_ENABLED(CONFIG_TRACE_IRQFLAGS) || IS_ENABLED(CONFIG_CONTEXT_TRACKING) || \
	IS_ENABLED(CONFIG_DEBUG_RSEQ))
/*
 * This is the fast syscall return path.  We do as little as possible here,
 * such as avoiding writing r0 to the stack.  We only use this path if we
 * have tracing, context tracking and rseq debug disabled - the overheads
 * from those features make this path too inefficient.
 */
ret_fast_syscall:
__ret_fast_syscall:
 UNWIND(.fnstart	)
 UNWIND(.cantunwind	)
	disable_irq_notrace			@ disable interrupts
	ldr	r1, [tsk, #TI_FLAGS]		@ re-check for syscall tracing
	movs	r1, r1, lsl #16
	bne	fast_work_pending


	/* perform architecture specific actions before user return */
	arch_ret_to_user r1, lr

	restore_user_regs fast = 1, offset = S_OFF
 UNWIND(.fnend		)
ENDPROC(ret_fast_syscall)

	/* Ok, we need to do extra processing, enter the slow path. */
fast_work_pending:
	str	r0, [sp, #S_R0+S_OFF]!		@ returned r0
	/* fall through to work_pending */
#else
/*
 * The "replacement" ret_fast_syscall for when tracing, context tracking,
 * or rseq debug is enabled.  As we will need to call out to some C functions,
 * we save r0 first to avoid needing to save registers around each C function
 * call.
 */
ret_fast_syscall:
__ret_fast_syscall:
 UNWIND(.fnstart	)
 UNWIND(.cantunwind	)
	str	r0, [sp, #S_R0 + S_OFF]!	@ save returned r0
#if IS_ENABLED(CONFIG_DEBUG_RSEQ)
	/* do_rseq_syscall needs interrupts enabled. */
	mov	r0, sp				@ 'regs'
	bl	do_rseq_syscall
#endif
	disable_irq_notrace			@ disable interrupts
	ldr	r1, [tsk, #TI_FLAGS]		@ re-check for syscall tracing
	movs	r1, r1, lsl #16
	beq	no_work_pending
 UNWIND(.fnend		)
ENDPROC(ret_fast_syscall)

	/* Slower path - fall through to work_pending */
#endif

	tst	r1, #_TIF_SYSCALL_WORK
	bne	__sys_trace_return_nosave
	//slow_work_pending requires tsk to be initialized to &thread_info.
	//1.	Invokes do_work_pending to handle signals and rescheduling,
	//		returning a number indicating whether a system call shall be
	//		restarted.
	//2.	If no system call shall be restarted (after a process has been
	//		stopped by a signal), jumps to no_work_pending, which does not
	//		return, and which:
	//		a)	Sets DACR such that all addresses of the kernel, I/O, exception
	//			vectors and handlers, and user space, are accessible according
	//			to the page tables.
	//		b)	Restores user mode registers to those stored in pt_regs, and
	//			resumes execution in user mode from the corrected link register
	//			address stored during the exception.
	//3.	If a system call shall be restarted
	//		a)	If system call traces, auditing, tracepoint instrumentation, or
	//			seccomp are active, then jumps to __sys_trace, which manages
	//			system call traces, and then jumps to ret_slow_syscall.
	//		b)	Otherwise, invokes the system call, if its number is valid, and
	//			upon completion jumps to __ret_fast_syscall.
	//		c)	If the system call number is invalid, either jumps to
	//			arm_syscall or sys_ni_syscall, depending on the value of the
	//			invalid system call number.
slow_work_pending:
	mov	r0, sp				@ 'regs'
	//r2 := r8. If r8 != 0, then system call caused the exception.
	mov	r2, why				@ 'syscall'
	//Calls do work_pending, which takes pt_regs, thread_flags and a flag
	//indicating whether this exception was raised due to a system call.
	//The considered thread flags are:
	//-TIF_SIGPENDING: Signal is pending, gets the signal and handles it.
	//-TIF_NEED_RESCHED: Rescheduling necessary and the main scheduling function
	// is called.
	//-TIF_NOTIFY_SIGNAL: Signal notifications exist, gets the signal and
	// handles it.
	//-TIF_UPROBE: breakpointed or singlestepping, allowing the task to return
	// from interrupt.
	//-TIF_NOTIFY_RESUME: Callback before returning to user, notifies task to be
	// ready to return to user mode?
	//do_work_pending returns a result stating which system call that shall be
	//restarted after a process has been stopped due to a signal.
	bl	do_work_pending
	cmp	r0, #0
	//If no system call shall be restarted, jumps to no_work_pending, which does
	//not return:
	//1.	Sets DACR such that all addresses of the kernel, I/O, exception
	//		vectors and handlers, and user space, are accessible according to
	//		the page tables.
	//2.	Restores user mode registers to those stored in pt_regs, and resumes
	//		execution in user mode from the corrected link register address
	//		stored during the exception.
	beq	no_work_pending
	//System call to be restarted. scno = r7 = number of system call to be
	//restarted. include/generated/uapi/asm/unistd-eabi.h:__NR_restart_syscall =
	//__NR_SYSCALL_BASE
	//Restart system call: "restart a system call after interruption by a stop
	//signal"
	movlt	scno, #(__NR_restart_syscall - __NR_SYSCALL_BASE)
	//r0-r6 := pt_regs.r0-pt_regs.r6
	ldmia	sp, {r0 - r6}			@ have to reload r0 - r6
	//1.	Stores r4 and r5 on the stack.
	//2.	If system call traces, auditing, tracepoint instrumentation, or
	//		seccomp are active, then jumps to __sys_trace, which manages system
	//		call traces, and then jumps to ret_slow_syscall.
	//3.	Otherwise, invokes the system call, if its number is valid, and upon
	//		completion jumps to __ret_fast_syscall.
	//4.	If the system call number is invalid, either jumps to arm_syscall or
	//		sys_ni_syscall, depending on the value of the invalid system call
	//		number.
	b	local_restart			@ ... and off we go
ENDPROC(ret_fast_syscall)

/*
 * "slow" syscall return path.  "why" tells us if this was a real syscall.
 * IRQs may be enabled here, so always disable them.  Note that we use the
 * "notrace" version to avoid calling into the tracing code unnecessarily.
 * do_work_pending() will update this state if necessary.
 */
	//1.	Checks low-level flags of the thread_info structure, and if work
	//		related flags are set, jumps to slow_work_pending, which calls
	//		do_work_pending with thread_flags being the second argument (r1):
	//		a)	Invokes do_work_pending to handle signals and rescheduling,
	//			returning a number indicating whether a system call shall be
	//			restarted.
	//		b)	If no system call shall be restarted (after a process has been
	//			stopped by a signal), jumps to no_work_pending, which does not
	//			return, and which:
	//			i)	Sets DACR such that all addresses of the kernel, I/O,
	//				exception vectors and handlers, and user space, are
	//				accessible according to the page tables.
	//			ii)	Restores user mode registers to those stored in pt_regs, and
	//				resumes execution in user mode from the corrected link
	//				register address stored during the exception.
	//		b)	If a system call shall be restarted
	//			i)	If system call traces, auditing, tracepoint instrumentation,
	//				or seccomp are active, then jumps to __sys_trace, which
	//				manages system call traces, and then jumps to this code
	//				again (ret_slow_syscall).
	//			ii)	Otherwise, invokes the system call, if its number is valid,
	//				and upon completion jumps to __ret_fast_syscall.
	//			iii)If the system call number is invalid, either jumps to
	//				arm_syscall or sys_ni_syscall, depending on the value of the
	//				invalid system call number.
	//2.	a)	Sets DACR such that all addresses of the kernel, I/O, exception
	//			vectors and handlers, and user space, are accessible according
	//			to the page tables.
	//		b)	Restores user mode registers to those stored in pt_regs, and
	//			resumes execution in user mode from the corrected link register
	//			address stored during the exception.
ENTRY(ret_to_user)
ret_slow_syscall:
#if IS_ENABLED(CONFIG_DEBUG_RSEQ)	//FALSE
	/* do_rseq_syscall needs interrupts enabled. */
	enable_irq_notrace			@ enable interrupts
	mov	r0, sp				@ 'regs'
	bl	do_rseq_syscall
#endif
	//Disables IRQs.
	disable_irq_notrace			@ disable interrupts
ENTRY(ret_to_user_from_irq)
	//DEFINE(TI_FLAGS,		offsetof(struct thread_info, flags));
	//thread_info.flags = low level flags
	//r1 := memory[address of thread_info structure + offset of low level flags]
	ldr	r1, [tsk, #TI_FLAGS]
	//r1 := r1 << 16 and updates condition flags according to the result.
	movs	r1, r1, lsl #16
	//If the flags are not all zero, jumps to slow_work_pending, which calls
	//do_work_pending with thread_flags being the second argument (r1):
	//1.	Invokes do_work_pending to handle signals and rescheduling,
	//		returning a number indicating whether a system call shall be
	//		restarted.
	//2.	If no system call shall be restarted (after a process has been
	//		stopped by a signal), jumps to no_work_pending, which does not
	//		return, and which:
	//		a)	Sets DACR such that all addresses of the kernel, I/O, exception
	//			vectors and handlers, and user space, are accessible according
	//			to the page tables.
	//		b)	Restores user mode registers to those stored in pt_regs, and
	//			resumes execution in user mode from the corrected link register
	//			address stored during the exception.
	//3.	If a system call shall be restarted
	//		a)	If system call traces, auditing, tracepoint instrumentation, or
	//			seccomp are active, then jumps to __sys_trace, which manages
	//			system call traces, and then jumps to ret_slow_syscall.
	//		b)	Otherwise, invokes the system call, if its number is valid, and
	//			upon completion jumps to __ret_fast_syscall.
	//		c)	If the system call number is invalid, either jumps to
	//			arm_syscall or sys_ni_syscall, depending on the value of the
	//			invalid system call number.
	bne	slow_work_pending
no_work_pending:
	asm_trace_hardirqs_on save = 0	//Empty: CONFIG_TRACE_IRQFLAGS is undefined.

	/* perform architecture specific actions before user return */
	arch_ret_to_user r1, lr		//Empty: CONFIG_NEED_RET_TO_USER is undefined.
	ct_user_enter save = 0		//Empty: CONFIG_CONTEXT_TRACKING is undefined.

	//1.	Sets DACR such that all addresses of the kernel, I/O, exception
	//		vectors and handlers, and user space, are accessible according to
	//		the page tables.
	//2.	Restores user mode registers to those stored in pt_regs, and resumes
	//		execution in user mode from the corrected link register address
	//		stored during the exception.
	restore_user_regs fast = 0, offset = 0
ENDPROC(ret_to_user_from_irq)
ENDPROC(ret_to_user)

/*
 * This is how we return from a fork.
 */
ENTRY(ret_from_fork)
	bl	schedule_tail
	cmp	r5, #0
	movne	r0, r4
	badrne	lr, 1f
	retne	r5
1:	get_thread_info tsk
	b	ret_slow_syscall
ENDPROC(ret_from_fork)

/*=============================================================================
 * SWI handler
 *-----------------------------------------------------------------------------
 */

	.align	5
	//System call handler.
ENTRY(vector_swi)
#ifdef CONFIG_CPU_V7M											//FALSE
	v7m_exception_entry
#else
	//Decrements the stack pointer sp_svc with the size of the pt_regs
	//structure, now pointing to the beginning of it.
	sub	sp, sp, #PT_REGS_SIZE
	//Stores r0-r12 in pt_regs.r0 - pt_regs.r12.
	stmia	sp, {r0 - r12}			@ Calling r0 - r12
	//r8 := &pt_regs.pc
 ARM(	add	r8, sp, #S_PC		)
	//pt_regs.lr = memory[sp + offset of pt_regs.pc - 4] := lr_usr
	//pt_regs.sp = memory[sp + offset of pt_regs.pc - 8] := sp_usr
 ARM(	stmdb	r8, {sp, lr}^		)	@ Calling sp, lr
 THUMB(	mov	r8, sp			)
 THUMB(	store_user_sp_lr r8, r10, S_SP	)	@ calling sp, lr
	//Move to register from special register: saved_psr = r8 := spsr_svc := cpsr_usr.
	mrs	saved_psr, spsr			@ called from non-FIQ mode, so ok.
 TRACE(	mov	saved_pc, lr		)						//EMPTY.
	//saved_pc = lr.
	str	saved_pc, [sp, #S_PC]		@ Save calling PC	//pt_regs.pc := lr = pc - 4 = &current instruction + 8 - 4 = &current instruction + 4 = &next instruction.
	str	saved_psr, [sp, #S_PSR]		@ Save CPSR			//pt_regs.psr := cpsr_usr.
	str	r0, [sp, #S_OLD_R0]		@ Save OLD_R0			//pt_regs.ARM_ORIG_r0 := r0.
#endif
	zero_fp												//EMPTY.
	//Updates SCTLR to cr_alignment if not equal.
	alignment_trap r10, ip, __cr_alignment				//ip = r2. __cr_alignment = &cr_alignment.
	asm_trace_hardirqs_on save=0						//Empty: CONFIG_TRACE_IRQFLAGS is undefined.
	enable_irq_notrace									//Sets the interrupt flag of CPSR.
	ct_user_exit save=0									//Empty: CONFIG_CONTEXT_TRACKING is undefined.

	/*
	 * Get the system call number.
	 */

#if defined(CONFIG_OABI_COMPAT)							//UNDEFINED.

	/*
	 * If we have CONFIG_OABI_COMPAT then we need to look at the swi
	 * value to determine if it is an EABI or an old ABI call.
	 */
#ifdef CONFIG_ARM_THUMB
	tst	saved_psr, #PSR_T_BIT
	movne	r10, #0				@ no thumb OABI emulation
 USER(	ldreq	r10, [saved_pc, #-4]	)	@ get SWI instruction
#else
 USER(	ldr	r10, [saved_pc, #-4]	)	@ get SWI instruction
#endif
 ARM_BE8(rev	r10, r10)			@ little endian instruction

#elif defined(CONFIG_AEABI)								//DEFINED: Obviously empty body.

	/*
	 * Pure EABI user space always put syscall number into scno (r7).
	 */
#elif defined(CONFIG_ARM_THUMB)
	/* Legacy ABI only, possibly thumb mode. */
	tst	saved_psr, #PSR_T_BIT		@ this is SPSR from save_user_regs
	addne	scno, r7, #__NR_SYSCALL_BASE	@ put OS number in
 USER(	ldreq	scno, [saved_pc, #-4]	)

#else
	/* Legacy ABI only. */
 USER(	ldr	scno, [saved_pc, #-4]	)	@ get SWI instruction
#endif

	/* saved_psr and saved_pc are now dead */
	//Sets DACR such that user addresses are not accessible, and all other
	//addresses accessible as specified by the page tables. tbl = r12 is
	//used as a temporary register.
	uaccess_disable tbl	
	//tsk = r9 := address of thread_info structure = "The struct thread_info is
	//the architecture-specific context for the task"
	get_thread_info tsk
	//tbl = r12 = address of sys_call_table relative pc. See:
	//-arch/arm/tools/syscall.tbl
	//-arch/arm/tools/Makefile
	//-arch/arm/include/generated/calls-eabi.S (which is included below in this file).
	//-arm/kernel/entry-common.S:__SYSCALL
	adr	tbl, sys_call_table		@ load syscall table pointer

#if defined(CONFIG_OABI_COMPAT)							//FALSE since UNDEFINED.
	/*
	 * If the swi argument is zero, this is an EABI call and we do nothing.
	 *
	 * If this is an old ABI call, get the syscall number into scno and
	 * get the old ABI syscall table address.
	 */
	bics	r10, r10, #0xff000000
	strne	r10, [tsk, #TI_ABI_SYSCALL]
	streq	scno, [tsk, #TI_ABI_SYSCALL]
	eorne	scno, r10, #__NR_OABI_SYSCALL_BASE
	ldrne	tbl, =sys_oabi_call_table
#elif !defined(CONFIG_AEABI)							//FALSE since CONFIG_AEABI is defined.
	bic	scno, scno, #0xff000000		@ mask off SWI op-code
	str	scno, [tsk, #TI_ABI_SYSCALL]
	eor	scno, scno, #__NR_SYSCALL_BASE	@ check OS number
#else													//TRUE since other conditions are false.
	str	scno, [tsk, #TI_ABI_SYSCALL]					//thread_info.abi_syscall := scno = system call number register = r7 = set by user mode svc instruction?
#endif
	/*
	 * Reload the registers that may have been corrupted on entry to
	 * the syscall assembly (by tracing or context tracking.)
	 */
 TRACE(	ldmia	sp, {r0 - r3}		)					//EMPTY.

	//Reqires tsk to be initializeed to &thread_info.
	//1.	Stores r4 and r5 on the stack.
	//2.	If system call traces, auditing, tracepoint instrumentation, or
	//		seccomp are active, then jumps to __sys_trace, which manages system
	//		call traces, and then jumps to ret_slow_syscall.
	//3.	Otherwise, invokes the system call, if its number is valid, and upon
	//		completion jumps to __ret_fast_syscall.
	//4.	If the system call number is invalid, either jumps to arm_syscall or
	//		sys_ni_syscall, depending on the value of the invalid system call
	//		number.
local_restart:
	//r10 := thread_info.flags = low level flags
	ldr	r10, [tsk, #TI_FLAGS]		@ check for syscall tracing
	//Stores r4 and r5 on the stack and updates the stack pointer.
	stmdb	sp!, {r4, r5}			@ push fifth and sixth args

	//_TIF_SYSCALL_WORK:
	//-TIF_SYSCALL_TRACE: "syscall trace active"
	//-TIF_SYSCALL_AUDIT: "syscall auditing active"
	//-TIF_SYSCALL_TRACEPOINT: "syscall tracepoint instrumentation"
	//-TIF_SECCOMP: "seccomp syscall filtering active"
	//ANDs r10 and _TIF_SYSCALL_WORK and sets condition flags accordingly.
	tst	r10, #_TIF_SYSCALL_WORK		@ are we tracing syscalls?
	//If system call traces, auditing, tracepoint instrumentation, or seccomp
	//(Secure Computing; " seccomp allows a process to make a one-way transition
	//into a "secure" state where it cannot make any system calls except exit(),
	//sigreturn(), read() and write() to already-open file descriptors.") are
	//active, then jumps to __sys_trace, which manages system call traces, and
	//then jumps to ret_slow_syscall, causing a loop since this routine is
	//jumped to from slow_work_pending, which is jumped to from
	//ret_slow_syscall. Maybe those routines clear _TIF_SYSCALL_WORK.
	bne	__sys_trace

	//System call traces, auditing, tracepoint instrumentation, and seccomp are
	//all inactive.
	//tbl = r8 = system call table pointer.
	//scno = r7 = system call number.
	//r10 is temporary.
	//__ret_fast_syscall is location to jump to afterwards.
	//If the system call number is valid (less than number of system calls),
	//sets the link register to the return address to __ret_fast_syscall and
	//jumps to the specified system call routine.
	invoke_syscall tbl, scno, r10, __ret_fast_syscall

	//System call is not valid, since invoke_syscall is a macro that gets
	//substituted into this location of the program, and if the control point
	//reaches this point, then the system call number was invalid.
	//r1 := sp + 8 = &pt_regs.
	add	r1, sp, #S_OFF
	//#define __ARM_NR_BASE			(__NR_SYSCALL_BASE+0x0f0000)
	//Compares the specified system call number to 0x000f0000
2:	cmp	scno, #(__ARM_NR_BASE - __NR_SYSCALL_BASE)
	//r0 := specified system call number XOR __NR_SYSCALL_BASE
	//The compiler defines __ARM_EABI__ (see
	//'echo | arm-linux-gnueabihf-gcc -dM -E -', which contains the line
	//'#define __ARM_EABI__ 1'), which means __NR_SYSCALL_BASE = 0.
	//r0 := specified system call number.
	eor	r0, scno, #__NR_SYSCALL_BASE	@ put OS number back
	//If the specified system call number is greater than or equal to 15*2^16,
	//jump to arm_syscall, which "Handle all unrecognised system calls." and
	//which returns to the address in lr.
	bcs	arm_syscall
	//System call is less than 15*2^16.
	//why = r8 := 0
	mov	why, #0				@ no longer a real syscall
	//Returns -ENOSYS in r0 and jumps to the location in lr.
	b	sys_ni_syscall			@ not private func

#if defined(CONFIG_OABI_COMPAT) || !defined(CONFIG_AEABI)	//FALSE
	/*
	 * We failed to handle a fault trying to access the page
	 * containing the swi instruction, but we're not really in a
	 * position to return -EFAULT. Instead, return back to the
	 * instruction and re-enter the user fault handling path trying
	 * to page it in. This will likely result in sending SEGV to the
	 * current task.
	 */
9001:
	sub	lr, saved_pc, #4
	str	lr, [sp, #S_PC]
	get_thread_info tsk
	b	ret_fast_syscall
#endif
ENDPROC(vector_swi)

	/*
	 * This is the really slow path.  We're going to be doing
	 * context switches, and waiting for our parent to respond.
	 */
__sys_trace:
	add	r0, sp, #S_OFF
	bl	syscall_trace_enter
	mov	scno, r0
	invoke_syscall tbl, scno, r10, __sys_trace_return, reload=1
	cmp	scno, #-1			@ skip the syscall?
	bne	2b
	add	sp, sp, #S_OFF			@ restore stack

__sys_trace_return_nosave:
	enable_irq_notrace
	mov	r0, sp
	bl	syscall_trace_exit
	b	ret_slow_syscall

__sys_trace_return:
	str	r0, [sp, #S_R0 + S_OFF]!	@ save returned r0
	mov	r0, sp
	bl	syscall_trace_exit
	b	ret_slow_syscall

	.align	5
#ifdef CONFIG_ALIGNMENT_TRAP
	.type	__cr_alignment, #object
__cr_alignment:
	.word	cr_alignment
#endif
	.ltorg

	.macro	syscall_table_start, sym
	.equ	__sys_nr, 0
	.type	\sym, #object
ENTRY(\sym)
	.endm

	.macro	syscall, nr, func
	.ifgt	__sys_nr - \nr
	.error	"Duplicated/unorded system call entry"
	.endif
	.rept	\nr - __sys_nr
	.long	sys_ni_syscall
	.endr
	.long	\func
	.equ	__sys_nr, \nr + 1
	.endm

	.macro	syscall_table_end, sym
	.ifgt	__sys_nr - __NR_syscalls
	.error	"System call table too big"
	.endif
	.rept	__NR_syscalls - __sys_nr
	.long	sys_ni_syscall
	.endr
	.size	\sym, . - \sym
	.endm

#define __SYSCALL_WITH_COMPAT(nr, native, compat)	__SYSCALL(nr, native)
#define __SYSCALL(nr, func) syscall nr, func

/*
 * This is the syscall table declaration for native ABI syscalls.
 * With EABI a couple syscalls are obsolete and defined as sys_ni_syscall.
 */
	syscall_table_start sys_call_table
#ifdef CONFIG_AEABI
#include <calls-eabi.S>
#else
#include <calls-oabi.S>
#endif
	syscall_table_end sys_call_table

/*============================================================================
 * Special system call wrappers
 */
@ r0 = syscall number
@ r8 = syscall table
sys_syscall:
		bic	scno, r0, #__NR_OABI_SYSCALL_BASE
		cmp	scno, #__NR_syscall - __NR_SYSCALL_BASE
		cmpne	scno, #NR_syscalls	@ check range
#ifdef CONFIG_CPU_SPECTRE
		movhs	scno, #0
		csdb
#endif
		stmialo	sp, {r5, r6}		@ shuffle args
		movlo	r0, r1
		movlo	r1, r2
		movlo	r2, r3
		movlo	r3, r4
		ldrlo	pc, [tbl, scno, lsl #2]
		b	sys_ni_syscall
ENDPROC(sys_syscall)

sys_sigreturn_wrapper:
		add	r0, sp, #S_OFF
		mov	why, #0		@ prevent syscall restart handling
		b	sys_sigreturn
ENDPROC(sys_sigreturn_wrapper)

sys_rt_sigreturn_wrapper:
		add	r0, sp, #S_OFF
		mov	why, #0		@ prevent syscall restart handling
		b	sys_rt_sigreturn
ENDPROC(sys_rt_sigreturn_wrapper)

sys_statfs64_wrapper:
		teq	r1, #88
		moveq	r1, #84
		b	sys_statfs64
ENDPROC(sys_statfs64_wrapper)

sys_fstatfs64_wrapper:
		teq	r1, #88
		moveq	r1, #84
		b	sys_fstatfs64
ENDPROC(sys_fstatfs64_wrapper)

/*
 * Note: off_4k (r5) is always units of 4K.  If we can't do the requested
 * offset, we return EINVAL.
 */
sys_mmap2:
		str	r5, [sp, #4]
		b	sys_mmap_pgoff
ENDPROC(sys_mmap2)

#ifdef CONFIG_OABI_COMPAT

/*
 * These are syscalls with argument register differences
 */

sys_oabi_pread64:
		stmia	sp, {r3, r4}
		b	sys_pread64
ENDPROC(sys_oabi_pread64)

sys_oabi_pwrite64:
		stmia	sp, {r3, r4}
		b	sys_pwrite64
ENDPROC(sys_oabi_pwrite64)

sys_oabi_truncate64:
		mov	r3, r2
		mov	r2, r1
		b	sys_truncate64
ENDPROC(sys_oabi_truncate64)

sys_oabi_ftruncate64:
		mov	r3, r2
		mov	r2, r1
		b	sys_ftruncate64
ENDPROC(sys_oabi_ftruncate64)

sys_oabi_readahead:
		str	r3, [sp]
		mov	r3, r2
		mov	r2, r1
		b	sys_readahead
ENDPROC(sys_oabi_readahead)

/*
 * Let's declare a second syscall table for old ABI binaries
 * using the compatibility syscall entries.
 */
	syscall_table_start sys_oabi_call_table
#undef __SYSCALL_WITH_COMPAT
#define __SYSCALL_WITH_COMPAT(nr, native, compat)	__SYSCALL(nr, compat)
#include <calls-oabi.S>
	syscall_table_end sys_oabi_call_table

#endif

