/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/arch/arm/kernel/entry-armv.S
 *
 *  Copyright (C) 1996,1997,1998 Russell King.
 *  ARM700 fix by Matthew Godbolt (linux-user@willothewisp.demon.co.uk)
 *  nommu support by Hyok S. Choi (hyok.choi@samsung.com)
 *
 *  Low-level vector interface routines
 *
 *  Note:  there is a StrongARM bug in the STMIA rn, {regs}^ instruction
 *  that causes it to save wrong values...  Be aware!
 */

#include <linux/init.h>

#include <asm/assembler.h>
#include <asm/memory.h>
#include <asm/glue-df.h>
#include <asm/glue-pf.h>
#include <asm/vfpmacros.h>
#ifndef CONFIG_GENERIC_IRQ_MULTI_HANDLER
#include <mach/entry-macro.S>
#endif
#include <asm/thread_notify.h>
#include <asm/unwind.h>
#include <asm/unistd.h>
#include <asm/tls.h>
#include <asm/system_info.h>
#include <asm/uaccess-asm.h>

#include "entry-header.S"
#include <asm/entry-macro-multi.S>
#include <asm/probes.h>

#ifdef CONFIG_TRUSTFULL_HYPERVISOR
__und_vector:
	.long __und_svc		//Undefined exception in supervisor mode. svc_pt_regs.
	.long __und_usr		//Undefined exception in user mode. pt_regs.

__pabt_vector:
	.long __pabt_svc	//Prefetch abort in supervisor mode. svc_pt_regs.
	.long __pabt_usr	//Prefetch abort exception in user mode. pt_regs.

__dabt_vector:
	.long __dabt_svc	//Data abort exception in supervisor mode. svc_pt_regs.
	.long __dabt_usr	//Data abort exception in user mode. pt_regs.

__irq_vector:
	.long __irq_svc		//IRQ exception in supervisor mode. svc_pt_regs.
	.long __irq_usr		//IRQ exception in user mode. pt_regs.

//Base address used by hypervisor to find right exception handler.
.global __hypervisor_vector
__hypervisor_vector:
	.long 0					@reset	//Not used since hypervisor catches resets?
	.long __und_vector		@undef
	.long sys_call_table	@swi	//Used for user system calls. Base address of system call table.
	.long __pabt_vector		@pabt
	.long __dabt_vector		@dabt
	.long 0					@fiq	//Address exception: Not used.
	.long __irq_vector		@irq
	.long 0					@7		//This is fiq.
	.long arm_syscall		@8
	.long ret_fast_syscall	@9
	//Not used by hypervisor. Invoked by Linux at end of __dabt_usr and
	//__und_usr.
	.long ret_from_exception@10
#endif

/*
 * Interrupt handling.
 */
	//Calls handle_arch_irq with a pointer to (svc_)pt_regs, and returns to end
	//of macro, while preserving r4-12.
	.macro	irq_handler
#ifdef CONFIG_GENERIC_IRQ_MULTI_HANDLER	//TRUE
	//r1 := address of variable handle_arch_irq declared by kernel/irq/handle.c,
	//as a function pointer.
	//Set by arch/arm/kernel/setup.c:setup_arch by invoking setup_machine_fdt,
	//which invokes drivers/irqchip/irq-omap-intc.c:intc_of_init which assigns
	//drivers/irqchip/irq-omap-intc.c:omap_intc_handle_irq in mdesc.handle_irq
	//which is assigned by arch/arm/kernel/setup.c:setup_arch.
	ldr	r1, =handle_arch_irq
	//sp points to top of stack, which is r0 of pt_regs, a pointer to pt_regs in
	//other words, which is the first argument to handle_arch_irq, which is
	//passed in r0.
	mov	r0, sp
	//lr := address to end of macro. The place the interrupt handler routine
	//shall jump to after it has completed its execution.
	badr	lr, 9997f
	//pc := the address of handle_arch_irq.
	ldr	pc, [r1]
#else									///FALSE
	arch_irq_handler_default
#endif
9997:
	.endm

	//Causes a kernel panic if fault is not handled. Otherwise returns to this point.
	.macro	pabt_helper
	@ PABORT handler takes pt_regs in r2, fault address in r4 and psr in r5
	//FALSE in arch/arm/include/asm/glue-pf.h since only CONFIG_CPU_PABRT_V7 is
	//defined and neither of CONFIG_CPU_PABRT_LEGACY and CONFIG_CPU_PABRT_V6.
#ifdef MULTI_PABORT
	ldr	ip, .LCprocfns
	mov	lr, pc
	ldr	pc, [ip, #PROCESSOR_PABT_FUNC]
#else
	//arch/arm/include/asm/glue-pf.h:define CPU_PABORT_HANDLER v7_pabort
	//r0 := Instruction Fault Address Register = "VA of the faulting access"
	//r1 := Instruction Fault Status Register = "status information about the
	//last instruction fault".
	//
	//arch/arm/mm/pabort-v7.S:v7_pabort calls arch/arm/mm/fault.c:do_PrefetchAbort.
	//
	//arch/arm/mm/fault.c:do_PrefetchAbort uses arch/arm/mm/fsr-2level.c:ifsr_info,
	//which has five different handlers, depending on FSR (fault status register):
	//do_bad,					SIGBUS,  0,												//Causes kernel panic.
	//do_translation_fault,		SIGSEGV, SEGV_MAPERR,	"section translation fault"		//Causes return to this point.
	//do_page_fault,			SIGSEGV, SEGV_MAPERR,	"page translation fault"		//Causes return to this point.
	//do_sect_fault,			SIGSEGV, SEGV_ACCERR,	"section permission fault"		//Causes return to this point.
	//do_page_fault,			SIGSEGV, SEGV_ACCERR,	"page permission fault"			//Causes return to this point.
	//
	//r0 (faulting address), r1 (fault status) and r2 (svc_pte_regs address) are
	//the arguments to do_PrefetchAbort which causes a kernel panic or calls
	//do_exit (which does not return due to the compiler directive __noreturn),
	//terminating Linux.
	bl	CPU_PABORT_HANDLER
#endif
	.endm

	//Reads the Data Fault Status and Data Fault Address Registers, sets DACR
	//such that user addresses are not accessible, and all other addresses
	//accessible as specified by the page tables. If the exception was raised in
	//user mode, then "Force a signal that the process can't ignore". If the
	//exception was raised un supervisor mode, then a kernel panic occurs.
	.macro	dabt_helper

	@
	@ Call the processor-specific abort handler:
	@
	@  r2 - pt_regs
	@  r4 - aborted context pc
	@  r5 - aborted context psr
	@
	@ The abort handler must return the aborted address in r0, and
	@ the fault status register in r1.  r9 must be preserved.
	@
	//Not defined by the sequence of define statements in
	//arch/arm/include/asm/glue-df.h, and which defines CPU_DABORT_HANDLER to
	//v7_early_abort
#ifdef MULTI_DABORT	//FALSE
	ldr	ip, .LCprocfns
	mov	lr, pc
	ldr	pc, [ip, #PROCESSOR_DABT_FUNC]
#else				//TRUE
	//Jumps to v7_early_abort, which reads the Data Fault Status and Data Fault
	//Address Registers, sets DACR such that user addresses are not accessible,
	//and all other addresses accessible as specified by the page tables. If the
	//exception was raised in user mode, then "Force a signal that the process
	//can't ignore". If the exception was raised un supervisor mode, then a
	//kernel panic occurs.
	bl	CPU_DABORT_HANDLER
#endif
	.endm

	.section	.entry.text,"ax",%progbits

/*
 * Invalid mode handlers
 */
	.macro	inv_entry, reason
	sub	sp, sp, #PT_REGS_SIZE
 ARM(	stmib	sp, {r1 - lr}		)
 THUMB(	stmia	sp, {r0 - r12}		)
 THUMB(	str	sp, [sp, #S_SP]		)
 THUMB(	str	lr, [sp, #S_LR]		)
	mov	r1, #\reason
	.endm

__pabt_invalid:	//Causes kernel panic.
	inv_entry BAD_PREFETCH
	b	common_invalid
ENDPROC(__pabt_invalid)

__dabt_invalid:	//Causes kernel panic.
	inv_entry BAD_DATA
	b	common_invalid
ENDPROC(__dabt_invalid)

__irq_invalid:	//Causes kernel panic.
	inv_entry BAD_IRQ
	b	common_invalid
ENDPROC(__irq_invalid)

__und_invalid:	//Causes kernel panic (by fall through).
	inv_entry BAD_UNDEFINSTR

	@
	@ XXX fall through to common_invalid
	@

@
@ common_invalid - generic code for failed exception (re-entrant version of handlers)
@
common_invalid:	//Causes kernel panic.
	zero_fp

	ldmia	r0, {r4 - r6}
	add	r0, sp, #S_PC		@ here for interlock avoidance
	mov	r7, #-1			@  ""   ""    ""        ""
	str	r4, [sp]		@ save preserved r0
	stmia	r0, {r5 - r7}		@ lr_<exception>,
					@ cpsr_<exception>, "old_r0"

	mov	r0, sp
	b	bad_mode
ENDPROC(__und_invalid)

/*
 * SVC mode handlers
 */

#if defined(CONFIG_AEABI) && (__LINUX_ARM_ARCH__ >= 5)
#define SPFIX(code...) code
#else
#define SPFIX(code...)
#endif

	//Invoked by: __dabt_svc, __irq_svc, __und_svc, __pabt_svc, __fiq_svc, and
	//__fiq_abt.
	//The macro vector_stub sets the CPU in supervisor mode before this macro
	//is executed, which has banked SP and LR registers.
	//
	//Performs the following four operations in the given order:
	//1.	Saves r0-r12, sp_svc, lr_svc, and pc on the stack and updates the
	//		stack pointer accordingly.
	//2.	Saves registers r0-r12 to pt_regs[0-12], pt_regs.lr = lr_svc,
	//		svc_pt_regs.pc = lr = preferred return address for exception +
	//		offset,
	//		svc_pt_regs.cpsr = CPSR before exception, and svc_pt_regs.old_r0 =
	//		-1 = 0xFFFF_FFFF (denoting no system call; this macro is not used in
	//		system calls), and updates the stack pointer accordingly.
	//		Where the preferred return address for the exception is as follows
	//		for the different exceptions:
	//		-Undefined instruction: Address of undefined instruction + 4.
	//		-Supervisor call: Address of instruction after svc instruction + 0.
	//		-Prefetch abort: Address of instruction whose fetch was aborted + 4.
	//		-Data abort: Address of instruction whose memory access failed + 8.
	//		-IRQ/FIQ: Address of next instruction to execute + 4.
	//3.	r4 = preferred return address for exception.
	//		r5 = CPSR before exception.
	//		r9 = address of thread_info structure = "The struct thread_info is
	//		the architecture-specific context for the task"
	//4.	If \uaccess is true, then kernel, I/O and vector (exception table
	//		and stubs/handlers) addresses are accessible according to page
	//		tables, and user addresses are inaccessible. Otherwise, no change is
	//		made.
	//
	//sp_svc = &pt_regs.
	.macro	svc_entry, stack_hole=0, trace=1, uaccess=1
 UNWIND(.fnstart		)							//#define UNWIND(i)	i.
	//Synonymous to stmfd sp!, {r0, r1, ..., pc}, which saves r0-pc on the stack
	//addressed by SP_svc, but decrementing SP_svc before each register is
	//stored.
 UNWIND(.save {r0 - pc}		)
#ifndef CONFIG_TRUSTFULL_HYPERVISOR
	//SVC_REGS_SIZE = 18 + 1 registers times 4 bytes = 76. Subtract (76-4=72).
	//svc_pt_regs contains pt_regs and DACR, with DACR following pt_regs. SP is
	//set to point to the base of pt_regs + r, which is the location of r1.
	sub	sp, sp, #(SVC_REGS_SIZE + \stack_hole - 4)								//1.
#ifdef CONFIG_THUMB2_KERNEL
 SPFIX(	str	r0, [sp]	)	@ temporarily saved
 SPFIX(	mov	r0, sp		)
 SPFIX(	tst	r0, #4		)	@ test original stack alignment
 SPFIX(	ldr	r0, [sp]	)	@ restored
#else
	//#define SPFIX(code...) code
	//Bitwise AND of sp with 0x4 and setting condition codes.
 SPFIX(	tst	sp, #4		)														//2.
#endif
	//If AND gives zero (bit 2 is zero), subtract stack pointer by 4, which
	//means that the stack pointer is subtracted by 76. Since sp points to r1,
	//and r0 is 4 bytes below, and which should be 8-byte aligned by the ARM
	//EABI, this subtraction ensures that r0 is located at an address that is
	//8-byte aligned (76 - 4 = 80).
 SPFIX(	subeq	sp, sp, #4	)													//3.
	//Store r1-r12 on the stack, and increment sp after, increments SP by 12*4 =
	//48. Stack is adjusted above to have room for 80-84 bytes, leaving a room
	//of 32-36 bytes = 4-5 words.
	//Stores registers r1-r12 in pt_regs.
	stmia	sp, {r1 - r12}														//4.
	//r0 is set to the stack pointer by the vector_stub macro, which stores
	//r0, lr, and SPSR (CPSR before exception):
	//r3 := r0 before exception.
	//r4 := lr = preferred return address for exception.
	//r5 := SPSR = CPSR before exception.
	ldmia	r0, {r3 - r5}														//5.
	//DEFINE(S_SP, offsetof(struct pt_regs, ARM_sp));
	//S_SP gives offset of SP register. Adds that offset to SP minus 4, making
	//r7 containing the address of r12.
	add	r7, sp, #S_SP - 4	@ here for interlock avoidance						//6.
	//r6 := -1
	mov	r6, #-1			@  ""  ""      ""       ""								//7.
	//r2 := First address beyond svc_pt_regs on the stack (below svc_pt_regs
	//structure which is at the top of the stack).
	add	r2, sp, #(SVC_REGS_SIZE + \stack_hole - 4)								//8.
	//If AND gives zero (bit 2 is zero of original stack pointer), add back 4.
 SPFIX(	addeq	r2, r2, #4	)													//9.
	//Stores r3 = r0 before exception at stack pointer (pointing to r1) minus 4
	//(pointing to r0).
	//This r0 will contain system call result if the exception is svc (and thus,
	//be overwritten), while the original r0 is stored below as r6. Since this
	//macro is not invoked on system calls, 0xFFFF_FFFF is written to orig_r0.
	//Decrements the stack pointer by 4, making it point to r0 and the last
	//element at the top of the stack.
	str	r3, [sp, #-4]!		@ save the "real" r0 copied							//10.
					@ from the exception stack
	//r3 := lr_svc.
	mov	r3, lr																	//11.

	@
	@ We are now ready to fill in the remaining blanks on the stack:
	@
	@  r2 - sp_svc
	@  r3 - lr_svc
	@  r4 - lr_<exception>, already fixed up for correct return/restart
	@  r5 - spsr_<exception>
	@  r6 - orig_r0 (see pt_regs definition in ptrace.h)
	@
	//r7 points to sp register in svc_pt_regs.
	//svc_pt_regs.pt_regs.sp := r2 = First address beyond svc_pt_regs on the
	//stack.
	//svc_pt_regs.pt_regs.lr := r3 = lr_svc
	//svc_pt_regs.pt_regs.pc := r4 = lr = preferred return address for
	//exception.
	//svc_pt_regs.pt_regs.cpsr := r5 = SPSR = CPSR before exception.
	//svc_pt_regs.pt_regs.old_r0 := r6 = -1 = 0xFFFF_FFFF.
	stmia	r7, {r2 - r6}														//12.

	//tsk = r9, according to arch/arm/kernel/entry-header.S.
	//r9 := address of thread_info structure = "The struct thread_info is the
	//architecture-specific context for the task"
	get_thread_info tsk
	//Skip this since hypervisor controls access permissions.
	//Stores DACR on the svc_pt_regs stack. If \disable is true, then kernel,
	//I/O and vector (exception table and stubs/handlers) addresses are
	//accessible according to page tables, and user addresses are inaccessible.
	//Otherwise, no change is made.
	uaccess_entry tsk, r0, r1, r2, \uaccess
#endif

	.if \trace
#ifdef CONFIG_TRACE_IRQFLAGS	//FALSE
	bl	trace_hardirqs_off
#endif
	.endif
	.endm

	.align	5
	//Handler for data abort exception in supervisor mode.
__dabt_svc:
//mov r0, #0x1D
//mov r1, #0x1D
//mov r2, #0x1D
//swi 1045
//1:	b 1b
	//1.	Saves r0-r12, sp_svc, lr_svc, and pc on the stack and updates the
	//		stack pointer accordingly.
	//2.	Saves registers r0-r12 to pt_regs[0-12], pt_regs.lr = lr_svc,
	//		svc_pt_regs.pc = lr = preferred return address for exception +
	//		offset,
	//		svc_pt_regs.cpsr = CPSR before exception, and svc_pt_regs.old_r0 =
	//		-1 = 0xFFFF_FFFF (denoting no system call; this macro is not used in
	//		system calls), and updates the stack pointer accordingly.
	//		Where the preferred return address for the exception is 'address of
	//		instruction whose memory access failed + 8'.
	//3.	r4 = preferred return address for exception.
	//		r5 = CPSR before exception.
	//		r9 = address of thread_info structure = "The struct thread_info is
	//		the architecture-specific context for the task"
	svc_entry uaccess=0

	mov	r2, sp
	//Reads the Data Fault Status and Data Fault Address Registers, sets DACR
	//such that user addresses are not accessible, and all other addresses
	//accessible as specified by the page tables. If the exception was raised in
	//user mode, then "Force a signal that the process can't ignore". If the
	//exception was raised un supervisor mode, then a kernel panic occurs.
	//"A subroutine must preserve the contents of the registers r4-r8, r10, r11
	//and SP" [https://developer.arm.com/documentation/ihi0042/latest].
	//dabt_helper invokes C functions that must preserve r5.
	dabt_helper

 THUMB(	ldr	r5, [sp, #S_PSR]	)	@ potentially updated CPSR
	//Restores the CPU registers and execution mode to that before the exception
	//occurred with the program counter set to the instruction to be executed
	//next.
	//svc_entry above sets r5 to CPSR before exception above, which is used by
	//svc_exit to restore CPSR.
	svc_exit r5				@ return from exception
 UNWIND(.fnend		)
ENDPROC(__dabt_svc)

	.align	5
	//Handler for IRQ exception in supervisor mode.
__irq_svc:
	//Performs the following four operations in the given order:
	//1.	Saves r0-r12, sp_svc, lr_svc, and pc on the stack and updates the
	//		stack pointer accordingly.
	//2.	Saves registers r0-r12 to pt_regs[0-12], pt_regs.lr = lr_svc,
	//		svc_pt_regs.pc = lr = preferred return address for exception +
	//		offset,
	//		svc_pt_regs.cpsr = CPSR before exception, and svc_pt_regs.old_r0 =
	//		-1 = 0xFFFF_FFFF (denoting no system call; this macro is not used in
	//		system calls), and updates the stack pointer accordingly.
	//		Where the preferred return address is 'address of next instruction
	//		to execute + 4'.
	//3.	r4 = preferred return address for exception.
	//		r5 = CPSR before exception.
	//		r9 = address of thread_info structure = "The struct thread_info is
	//		the architecture-specific context for the task"
	//4.	Kernel, I/O and vector (exception table and stubs/handlers)
	//		addresses are accessible according to page tables, and user
	//		addresses are inaccessible. Otherwise, no change is made.
	svc_entry
	//Calls handle_arch_irq with a pointer to (svc_)pt_regs, and returns to end
	//of macro, while preserving r4-12.
	irq_handler
#ifdef CONFIG_PREEMPTION	//FALSE
	ldr	r8, [tsk, #TI_PREEMPT]		@ get preempt count
	ldr	r0, [tsk, #TI_FLAGS]		@ get flags
	teq	r8, #0				@ if preempt count != 0
	movne	r0, #0				@ force flags to 0
	tst	r0, #_TIF_NEED_RESCHED
	blne	svc_preempt
#endif
#ifdef CONFIG_TRUSTFULL_HYPERVISOR
	//The following two lines are from arch/arm/kernel/entry-header.S:svc_exit:
	//r0 := sp - 4
	sub	r0, sp, #4			@ uninhabited address
	//r1 := 0 if memory[sp - 4] := r2 is sucessful. Otherwise r1 := 1.
	strex	r1, r2, [r0]			@ clear the exclusive monitor
    mov r0, sp
    swi HYPERCALL_END_INTERRUPT
#else
	//Restores the CPU registers and execution mode to that before the exception
	//occurred with the program counter set to the instruction to be executed
	//next, which is the instruction that was interrupted.
	//svc_entry above sets r5 to CPSR before exception above, which is used by
	//svc_exit to restore CPSR, and preserved by the irq_handler macro.
	svc_exit r5, irq = 1			@ return from exception
#endif
 UNWIND(.fnend		)
ENDPROC(__irq_svc)

	.ltorg

#ifdef CONFIG_PREEMPTION	//FALSE
svc_preempt:
	mov	r8, lr
1:	bl	preempt_schedule_irq		@ irq en/disable is done inside
	ldr	r0, [tsk, #TI_FLAGS]		@ get new tasks TI_FLAGS
	tst	r0, #_TIF_NEED_RESCHED
	reteq	r8				@ go again
	b	1b
#endif

	//Decrements pt_regs.pc by r1 and causes a kernel panic or handles the
	//undefined instruction exception. Returns to the address in lr.
__und_fault:
	@ Correct the PC such that it is pointing at the instruction
	@ which caused the fault.  If the faulting instruction was ARM
	@ the PC will be pointing at the next instruction, and have to
	@ subtract 4.  Otherwise, it is Thumb, and the PC will be
	@ pointing at the second half of the Thumb instruction.  We
	@ have to subtract 2.
	//r2 := memory[r0 + pt_regs.pc]
	ldr	r2, [r0, #S_PC]
	//r2 := memory[r0 + pt_regs.pc] - r1
	sub	r2, r2, r1
	//memory[r0 + pt_regs.pc] := memory[r0 + pt_regs.pc] - r1
	str	r2, [r0, #S_PC]
	//C-function taking a pointer to pt_regs as first argument (r0). If there is
	//no undefined hook handler (in the list undef_hook defined in
	//arch/arm/kernel/traps.c), a kernel panic occurs, and C-functions returns
	//to the location addressed by lr. If the processor mode before the exception
	//is supervisor mode, then a kernel panic occurs. There is probably a handler
	//if the undefined exception occurs in user mode:
	//arch/arm/kernel/swp_emulate.c:swp_hook.
	b	do_undefinstr
ENDPROC(__und_fault)

	.align	5
	//Handler for undefined instruction exception in supervisor mode.
__und_svc:
//mov r0, #0x1B
//mov r1, #0x1B
//mov r2, #0x1B
//swi 1045
//1: b 1b
#ifdef CONFIG_KPROBES	//FALSE
	@ If a kprobe is about to simulate a "stmdb sp..." instruction,
	@ it obviously needs free stack space which then will belong to
	@ the saved context.
	svc_entry MAX_STACK_SIZE
#else					//TRUE
	//Performs the following four operations in the given order:
	//1.	Saves r0-r12, sp_svc, lr_svc, and pc on the stack and updates the
	//		stack pointer accordingly.
	//2.	Saves registers r0-r12 to pt_regs[0-12], pt_regs.lr = lr_svc,
	//		svc_pt_regs.pc = lr = preferred return address for exception +
	//		offset,
	//		svc_pt_regs.cpsr = CPSR before exception, and svc_pt_regs.old_r0 =
	//		-1 = 0xFFFF_FFFF (denoting no system call; this macro is not used in
	//		system calls), and updates the stack pointer accordingly.
	//		Where the preferred return address for the exception is as 'address
	//		of undefined instruction + 4'.
	//3.	r4 = preferred return address for exception.
	//		r5 = CPSR before exception.
	//		r9 = address of thread_info structure = "The struct thread_info is
	//		the architecture-specific context for the task"
	//4.	Kernel, I/O and vector (exception table and stubs/handlers)
	//		addresses are accessible according to page tables, and user
	//		addresses are inaccessible. Otherwise, no change is made.
	svc_entry
#endif
	//r1 := 4
	mov	r1, #4				@ PC correction to apply
 THUMB(	tst	r5, #PSR_T_BIT		)	@ exception taken in Thumb mode?
 THUMB(	movne	r1, #2			)	@ if so, fix up PC correction
	//r0 := &svc_pt_regs.
	mov	r0, sp				@ struct pt_regs *regs
	//Decrements pt_regs.pc by 4 (r1), making pt_regs.pc point to the
	//instruction causing the exception, and causes a kernel panic, or handles
	//the undefined instruction exception an returns to the __und_svc_finish.
	bl	__und_fault

__und_svc_finish:
	//tsk = r9, according to arch/arm/kernel/entry-header.S.
	//r9 := address of thread_info structure = "The struct thread_info is the
	//architecture-specific context for the task"
	get_thread_info tsk
	//DEFINE(S_PSR,			offsetof(struct pt_regs, ARM_cpsr));
	//r5 := CPSR before exception from svc_pt_regs.pt_regs.CPSR.
	ldr	r5, [sp, #S_PSR]		@ Get SVC cpsr
	//Restores the CPU registers and execution mode to that before the exception
	//occurred with the program counter set to the instruction causing the
	//exception. r5 is to the CPSR value before the exception occurred, which is
	//used by svc_exit to restore CPSR.
	svc_exit r5				@ return from exception
 UNWIND(.fnend		)
ENDPROC(__und_svc)

	.align	5
	//Handler for prefetch abort exception in supervisor mode. Handles exception
	//or performs a kernel panic.
__pabt_svc:
mov r0, #0x2D
mov r1, #0x2D
mov r2, #0x2D
swi 1045
1:	b 1b
	//Performs the following four operations in the given order:
	//1.	Saves r0-r12, sp_svc, lr_svc, and pc on the stack and updates the
	//		stack pointer accordingly.
	//2.	Saves registers r0-r12 to pt_regs[0-12], pt_regs.lr = lr_svc,
	//		svc_pt_regs.pc = lr = preferred return address for exception +
	//		offset,
	//		svc_pt_regs.cpsr = CPSR before exception, and svc_pt_regs.old_r0 =
	//		-1 = 0xFFFF_FFFF (denoting no system call; this macro is not used in
	//		system calls), and updates the stack pointer accordingly.
	//		Where the preferred return address for the exception is 'address of
	//		instruction whose fetch was aborted + 4'.
	//3.	r4 = preferred return address for exception.
	//		r5 = CPSR before exception.
	//		r9 = address of thread_info structure = "The struct thread_info is
	//		the architecture-specific context for the task"
	//4.	Kernel, I/O and vector (exception table and stubs/handlers)
	//		addresses are accessible according to page tables, and user
	//		addresses are inaccessible. Otherwise, no change is made.
	svc_entry
	//r2 := &svc_pt_regs
	mov	r2, sp				@ regs
	//Causes a kernel panic if fault is not handled. Otherwise returns to this point.
	pabt_helper
	svc_exit r5				@ return from exception
 UNWIND(.fnend		)
ENDPROC(__pabt_svc)

	.align	5
	//Handler for FIQ exception in supervisor mode.
__fiq_svc:
	//1.	Saves r0-r12, sp_svc, lr_svc, and pc on the stack and updates the
	//		stack pointer accordingly.
	//2.	Saves registers r0-r12 to pt_regs[0-12], pt_regs.lr = lr_svc,
	//		svc_pt_regs.pc = lr = preferred return address for exception +
	//		offset,
	//		svc_pt_regs.cpsr = CPSR before exception, and svc_pt_regs.old_r0 =
	//		-1 = 0xFFFF_FFFF (denoting no system call; this macro is not used in
	//		system calls), and updates the stack pointer accordingly.
	//		Where the preferred return address for the exception is 'address of
	//		next instruction to execute + 4'.
	//3.	r4 = preferred return address for exception.
	//		r5 = CPSR before exception.
	//		r9 = address of thread_info structure = "The struct thread_info is
	//		the architecture-specific context for the task"
	//4.	Kernel, I/O and vector (exception table and stubs/handlers)
	//		addresses are accessible according to page tables, and user
	//		addresses are inaccessible. Otherwise, no change is made.
	svc_entry trace=0
	//r0 := &svc_pt_regs
	mov	r0, sp				@ struct pt_regs *regs
	//Calls arch/arm/kernel/traps.c:handle_fiq_as_nmi, &pts_regs as argument,
	//and which does not do anything: "nop. FIQ handlers for special arch/arm
	//features can be added here."
	bl	handle_fiq_as_nmi
	//Restores registers and CPU mode to what it was before the FIQ occured,
	//with the PC set to the next instruction to execute.
	svc_exit_via_fiq
 UNWIND(.fnend		)
ENDPROC(__fiq_svc)

//.LC = Local Constant.
	.align	5
.LCcralign:
	.word	cr_alignment	//This word contains the address of cr_alignment.
#ifdef MULTI_DABORT
.LCprocfns:
	.word	processor
#endif
.LCfp:
	.word	fp_enter

/*
 * Abort mode handlers
 */

@
@ Taking a FIQ in abort mode is similar to taking a FIQ in SVC mode
@ and reuses the same macros. However in abort mode we must also
@ save/restore lr_abt and spsr_abt to make nested aborts safe.
@
	.align 5
//Handles FIQ exception in abort mode.
__fiq_abt:
	//1.	Saves r0-r12, sp_svc, lr_svc, and pc on the stack and updates the
	//		stack pointer accordingly.
	//2.	Saves registers r0-r12 to pt_regs[0-12], pt_regs.lr = lr_svc,
	//		svc_pt_regs.pc = lr = preferred return address for exception +
	//		offset,
	//		svc_pt_regs.cpsr = CPSR before exception, and svc_pt_regs.old_r0 =
	//		-1 = 0xFFFF_FFFF (denoting no system call; this macro is not used in
	//		system calls), and updates the stack pointer accordingly.
	//		Where the preferred return address for the exception is the address
	//		of next instruction to execute + 4.
	//3.	r4 = preferred return address for exception.
	//		r5 = CPSR before exception.
	//		r9 = address of thread_info structure = "The struct thread_info is
	//		the architecture-specific context for the task"
	//4.	Kernel, I/O and vector (exception table and stubs/handlers)
	//		addresses are accessible according to page tables, and user
	//		addresses are inaccessible. Otherwise, no change is made.
	svc_entry trace=0

	//Sets the CPU in abort mode with IRQ and FIQ interrupts disabled.
 ARM(	msr	cpsr_c, #ABT_MODE | PSR_I_BIT | PSR_F_BIT )
 THUMB( mov	r0, #ABT_MODE | PSR_I_BIT | PSR_F_BIT )
 THUMB( msr	cpsr_c, r0 )
	//Save link register 
	//r1 := lr_abt
	mov	r1, lr		@ Save lr_abt
	//Move to register r2 from special register SPSR_abt
	mrs	r2, spsr	@ Save spsr_abt, abort is now safe
	//Set the CPU mode to supervisor mode with IRQ and FIQ interrupts disabled.
 ARM(	msr	cpsr_c, #SVC_MODE | PSR_I_BIT | PSR_F_BIT )
 THUMB( mov	r0, #SVC_MODE | PSR_I_BIT | PSR_F_BIT )
 THUMB( msr	cpsr_c, r0 )
	//Stores registers used by handle_fiq_as_nmi. Should be r0-r3 since
	//C-functions can modify r0-r3 according to the arm procedure call
	//conventions? handle_fiq_as_nmi has only one local variable, so this works
	//for the moment?
	//Decrements sp by 4, stores r2 at that location, and then decrements sp by
	//4 and stores r1 at that location.
	//memory[sp - 8] := r1
	//memory[sp - 4] := r2
	//sp := sp - 8
	stmfd	sp!, {r1 - r2}

	//r0 := &svc_pt_regs.
	add	r0, sp, #8			@ struct pt_regs *regs
	//Calls arch/arm/kernel/traps.c:handle_fiq_as_nmi, &pts_regs as argument,
	//and which does not do anything: "nop. FIQ handlers for special arch/arm
	//features can be added here."
	bl	handle_fiq_as_nmi

	//Restores the registers after the call to handle_fiq_as_nmi.
	//r1 := memory[sp]
	//r2 := memory[sp + 4]
	//sp := sp + 8
	ldmfd	sp!, {r1 - r2}
	//Set the CPU to abort mode with IRQ and FIQ interrupts disabled.
 ARM(	msr	cpsr_c, #ABT_MODE | PSR_I_BIT | PSR_F_BIT )
 THUMB( mov	r0, #ABT_MODE | PSR_I_BIT | PSR_F_BIT )
 THUMB( msr	cpsr_c, r0 )
	//Why save and restore lr_abt and spsr_abt when they are banked and
	//handle_fiq_as_nmi is invoked in supervisor mode? Nested abort exceptions?
	//lr_abt := r1
	mov	lr, r1		@ Restore lr_abt, abort is unsafe
	//SPSR_abt := r2.
	msr	spsr_cxsf, r2	@ Restore spsr_abt
	//Set CPU mode to supervisor with IRQ and FIQs disabled.
 ARM(	msr	cpsr_c, #SVC_MODE | PSR_I_BIT | PSR_F_BIT )
 THUMB( mov	r0, #SVC_MODE | PSR_I_BIT | PSR_F_BIT )
 THUMB( msr	cpsr_c, r0 )

	//Restores registers and CPU mode to what it was before the FIQ occured,
	//with the PC set to the next instruction to execute.
	svc_exit_via_fiq
 UNWIND(.fnend		)
ENDPROC(__fiq_abt)

/*
 * User mode handlers
 *
 * EABI note: sp_svc is always 64-bit aligned here, so should PT_REGS_SIZE
 */

#if defined(CONFIG_AEABI) && (__LINUX_ARM_ARCH__ >= 5) && (PT_REGS_SIZE & 7)
#error "sizeof(struct pt_regs) must be a multiple of 8"
#endif

	//Makes room for pt_regs on the stack, and stores in pt_regs r0-r12,
	//lr_usr, sp_usr, memory[r0 + 4] (PC), memory[r0 + 8] (CPSR), and
	//0xFFFF_FFFF (orig_r0). If uaccess = 1, then DACR is set such that user
	//addresses are not accessible, and all other addresses accessible as
	//specified by the page tables. If cr_alignment is not equal to SCTLR, then
	//updates SCTLR to cr_alignment.
	//1.	Decrements the stack to give room for pt_regs.
	//2.	pt_regs.r1 - pt_regs.r12 := r1-r12
	//3.	pt_regs.r0 := memory[r0]
	//4.	pt_regs.pc := memory[r0 + 4]
	//		pt_regs.cpsr := memory[r0 + 8]
	//		pt_regs.orig_r0 := 0xFFFF_FFFF
	//5.	pt_regs.lr := lr_usr
	//6.	pt_regs.sp := sp_usr
	//7.	if uaccess = 1, then sets DACR such that user addresses are not
	//		accessible, and all other addresses accessible as specified by the
	//		page tables.
	//8.	If cr_alignment variable is distinct from SCTLR, then updates SCTLR
	//		to cr_alignment.
	//
	//sp_svc = &pt_regs.
	.macro	usr_entry, trace=1, uaccess=1
 UNWIND(.fnstart	)
	//.cantunwind: "Prevents unwinding through the current function. No
	//personality routine or exception table data is required or permitted."
	//Probably used to prevent the following:
	//"The ABI for the ARM Architecture specifies a standard format for
	// exception unwind information. This information is used when an exception
	// is thrown to determine where control should be transferred. In
	// particular, the unwind information is used to determine which function
	// called the function that threw the exception, and which function called
	// that one, and so forth. This information is also used to restore the
	// values of callee-saved registers in the function catching the exception."
 UNWIND(.cantunwind	)	@ don't unwind the user space
#ifndef CONFIG_TRUSTFULL_HYPERVISOR
	//Decrements the stack pointer with the size of the pt_regs structure.
	sub	sp, sp, #PT_REGS_SIZE
	//Stores r1-r12 in pt_regs.r1 - pt_regs.t12.
 ARM(	stmib	sp, {r1 - r12}	)
 THUMB(	stmia	sp, {r0 - r12}	)
	//arch/arm/kernel/entry-header.S:#define ATRAP(x...) x
	//Move to ARM core register r7 from coprocessor 15 SCTLR register.
	//r7 := SCTLR.
 ATRAP(	mrc	p15, 0, r7, c1, c0, 0)
	//r8 := memory[address of .LCcralign] = memory address of cr_alignment.
 ATRAP(	ldr	r8, .LCcralign)

	//r3 := memory[r0]
	//r4 := memory[r0 + 4]
	//r5 := memory[r0 + 8]
	ldmia	r0, {r3 - r5}
	//r0 := sp + offset of pt_regs.pc
	add	r0, sp, #S_PC		@ here for interlock avoidance
	//r6 := 0xFFFF_FFFF
	mov	r6, #-1			@  ""  ""     ""        ""

	//pt_regs.r0 := r3 = memory[r0]
	str	r3, [sp]		@ save the "real" r0 copied
					@ from the exception stack
	//r8 := cr_aligment.
 ATRAP(	ldr	r8, [r8, #0])

	@
	@ We are now ready to fill in the remaining blanks on the stack:
	@
	@  r4 - lr_<exception>, already fixed up for correct return/restart
	@  r5 - spsr_<exception>
	@  r6 - orig_r0 (see pt_regs definition in ptrace.h)
	@
	@ Also, separately save sp_usr and lr_usr
	@
	//pt_regs.pc = memory[sp + offset of pt_regs.pc] := r4
	//pt_regs.cpsr = memory[sp + offset of pt_regs.pc + 4] := r5
	//pt_regs.orig_r0 = memory[sp + offset of pt_regs.pc + 8] := r6 = 0xFFFFFFFF
	stmia	r0, {r4 - r6}
	//If register to load/store do not include pc, then ^ gives access to user
	//mode registers from privileged mode.
	//Stores user mode registers lr and sp in pt_regs.
	//pt_regs.lr = memory[sp + offset of pt_regs.pc - 4] := lr_usr
	//pt_regs.sp = memory[sp + offset of pt_regs.pc - 8] := sp_usr
 ARM(	stmdb	r0, {sp, lr}^			)
 THUMB(	store_user_sp_lr r0, r1, S_SP - S_PC	)

	.if \uaccess
	//Sets DACR such that user addresses are not accessible, and all other
	//addresses accessible as specified by the page tables. ip register = r12 is
	//used as a temporary register.
	uaccess_disable ip
	.endif

	@ Enable the alignment trap while in kernel mode
	//Test equivalence between r8 = cr_alignment and r7 = SCTLR. Does exclusive
	//or between cr_alignment and SCTLR, and sets the condition flags
	//accordingly. The Z flag is not set if and only if the result is not zero,
	//equivalent to SCTLR and cr_alignment not being equal.
 ATRAP(	teq	r8, r7)
	//If SCTLR and cr_alignment are not equal, move ARM core register r8 =
	//cr_alignment to coprocessor 15 register SCTLR.
	//Thus, updates SCTLR if cr_alignment is distinct from SCTLR.
 ATRAP( mcrne	p15, 0, r8, c1, c0, 0)
#endif	/* !CONFIG_TRUSTFULL_HYPERVISOR */

	@
	@ Clear FP to mark the first stack frame
	@
	zero_fp	//Empty since CONFIG_FRAME_POINTER is undefined.

	.if	\trace
#ifdef CONFIG_TRACE_IRQFLAGS	//FALSE
	bl	trace_hardirqs_off
#endif
	ct_user_exit save = 0	//Empty since CONFIG_CONTEXT_TRACKING is undefined.
	.endif
	.endm

	//EMPTY since CONFIG_CPU_32v6K is defined.
	.macro	kuser_cmpxchg_check
#if !defined(CONFIG_CPU_32v6K) && defined(CONFIG_KUSER_HELPERS)	//FALSE
#ifndef CONFIG_MMU
#warning "NPTL on non MMU needs fixing"
#else
	@ Make sure our user space atomic helper is restarted
	@ if it was interrupted in a critical region.  Here we
	@ perform a quick test inline since it should be false
	@ 99.9999% of the time.  The rest is done out of line.
	ldr	r0, =TASK_SIZE
	cmp	r4, r0
	blhs	kuser_cmpxchg64_fixup
#endif
#endif
	.endm

	.align	5
	//Handles data abort exceptions in user mode.
__dabt_usr:
//mov r0, #0x1E
//mov r1, #0x1E
//mov r2, #0x1E
//swi 1045
//1:	b 1b
	//vector_stub does the following:
	//1.	memory[sp_<exception processor mode>] := r0
	//		memory[sp_<exception processor mode> + 4] :=
	//		lr_<exception processor_mode> - \correction
	//		memory[sp_<exception processor mode> + 8] := CPSR before exception
	//2.	Sets the CPU in supervisor mode.
	//3.	r0 := sp_<exception processor mode>
	//
	//usr_entry does the following:
	//Makes room for pt_regs on the stack, and stores in pt_regs r0-r12, lr_usr,
	//sp_usr, memory[r0 + 4] (PC), memory[r0 + 8] (CPSR), and 0xFFFF_FFFF
	//(orig_r0). If cr_alignment is not equal to SCTLR, then updates SCTLR to
	//cr_alignment.
	//
	//Hence, pt_regs[r0-r12, lr, sp, pc, cpsr, r0_orig] :=
	//[{r0-r12}_<exception processor mode>, lr_usr, sp_usr,
	// lr_<exception processor mode> - \correction,
	// CPSR before exception,
	// 0xFFFF_FFFF (not system call)]
	usr_entry uaccess=0
	kuser_cmpxchg_check	//Empty.
	//r2 := sp_svc
	mov	r2, sp
	//Reads the Data Fault Status and Data Fault Address Registers, sets DACR
	//such that user addresses are not accessible, and all other addresses
	//accessible as specified by the page tables. If the exception was raised in
	//user mode, then "Force a signal that the process can't ignore". If the
	//exception was raised un supervisor mode, then a kernel panic occurs.
	dabt_helper
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
	b	ret_from_exception
 UNWIND(.fnend		)
ENDPROC(__dabt_usr)

	.align	5
__irq_usr:
//mov r0, #0x19
//mov r1, #0x19
//mov r2, #0x19
//swi 1045
//1:	b 1b
	//Makes room for pt_regs on the stack, and stores in pt_regs r0-r12, lr_usr,
	//sp_usr, memory[r0 + 4] (PC), memory[r0 + 8] (CPSR), and 0xFFFF_FFFF
	//(orig_r0). If cr_alignment is not equal to SCTLR, then updates SCTLR to
	//cr_alignment. Sets DACR such that user addresses are not accessible, and
	//all other addresses accessible as specified by the page tables.
	usr_entry
	kuser_cmpxchg_check	//EMPTY.
	//Calls handle_arch_irq with a pointer to (svc_)pt_regs, and returns to end
	//of macro, while preserving r4-12.
	irq_handler
	//tsk = r9, according to arch/arm/kernel/entry-header.S.
	//r9 := address of thread_info structure = "The struct thread_info is the
	//architecture-specific context for the task"
	get_thread_info tsk
	//r8 := 0 = not system call.
	mov	why, #0
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
	b	ret_to_user_from_irq
 UNWIND(.fnend		)
ENDPROC(__irq_usr)

	.ltorg

	.align	5
	//Emulates floating point if undefined instruction is encoded accordingly. Otherwise handles the exception.
__und_usr:
//mov r0, #0x1C
//mov r1, #0x1C
//mov r2, #0x1C
//swi 1045
//1: b 1b
	//Makes room for pt_regs on the stack, and stores in pt_regs r0-r12, lr_usr,
	//sp_usr, memory[r0 + 4] (PC), memory[r0 + 8] (CPSR), and 0xFFFF_FFFF
	//(orig_r0). If cr_alignment is not equal to SCTLR, then updates SCTLR to
	//cr_alignment.
	usr_entry uaccess=0

	mov	r2, r4	//r4 = pt_regs.PC = Preferred exception return address.
	mov	r3, r5	//r5 = pt_regs.CPSR = Status register before exception.

	@ r2 = regs->ARM_pc, which is either 2 or 4 bytes ahead of the
	@      faulting instruction depending on Thumb mode.
	@ r3 = regs->ARM_cpsr
	@
	@ The emulation code returns using r9 if it has emulated the
	@ instruction, or the more conventional lr if we are to treat
	@ this as a real undefined instruction
	@
	badr	r9, ret_from_exception	//r9 := Address of ret_from_exception.

	@ IRQs must be enabled before attempting to read the instruction from
	@ user space since that could cause a page/translation fault if the
	@ page table was modified by another CPU.
	//Sets CPSR.I interrupt enable flag.
	enable_irq

	//AND between r3 = CPSR and 0x00000020 (bit 5 = Thumb mode) and sets
	//condition flags accordingly.
	tst	r3, #PSR_T_BIT			@ Thumb mode?
	// If the flags are not all zero, then jumps to __und_usr_thumb, which does
	//not happen since thumb mode is not used.
	bne	__und_usr_thumb
	//Address of undefined instruction (see page 1206 in B1).
	sub	r4, r2, #4			@ ARM instr at LR - 4
	//Reads the undefined instruction.
1:	ldrt	r0, [r4]
 ARM_BE8(rev	r0, r0)				@ little endian instruction
	//Sets DACR such that user addresses are not accessible, and all other
	//addresses accessible as specified by the page tables. ip register = r12 is
	//used as a temporary register.
#ifndef CONFIG_TRUSTFULL_HYPERVISOR
	uaccess_disable ip
#endif

	@ r0 = 32-bit ARM instruction which caused the exception
	@ r2 = PC value for the following instruction (:= regs->ARM_pc)
	@ r4 = PC value for the faulting instruction
	@ lr = 32-bit undefined instruction function
	badr	lr, __und_usr_fault_32		//lr := Address of __und_usr_fault_32.
	b	call_fpe						//Jumps to call_fpe, which emulates floating point, and which then returns to __und_usr_fault_32.

__und_usr_thumb:
	@ Thumb instruction
	sub	r4, r2, #2			@ First half of thumb instr at LR - 2
#if CONFIG_ARM_THUMB && __LINUX_ARM_ARCH__ >= 6 && CONFIG_CPU_V7
/*
 * Thumb-2 instruction handling.  Note that because pre-v6 and >= v6 platforms
 * can never be supported in a single kernel, this code is not applicable at
 * all when __LINUX_ARM_ARCH__ < 6.  This allows simplifying assumptions to be
 * made about .arch directives.
 */
#if __LINUX_ARM_ARCH__ < 7
/* If the target CPU may not be Thumb-2-capable, a run-time check is needed: */
#define NEED_CPU_ARCHITECTURE
	ldr	r5, .LCcpu_architecture
	ldr	r5, [r5]
	cmp	r5, #CPU_ARCH_ARMv7
	blo	__und_usr_fault_16		@ 16bit undefined instruction
/*
 * The following code won't get run unless the running CPU really is v7, so
 * coding round the lack of ldrht on older arches is pointless.  Temporarily
 * override the assembler target arch with the minimum required instead:
 */
	.arch	armv6t2
#endif
2:	ldrht	r5, [r4]
ARM_BE8(rev16	r5, r5)				@ little endian instruction
	cmp	r5, #0xe800			@ 32bit instruction if xx != 0
	blo	__und_usr_fault_16_pan		@ 16bit undefined instruction
3:	ldrht	r0, [r2]
ARM_BE8(rev16	r0, r0)				@ little endian instruction
	uaccess_disable ip
	add	r2, r2, #2			@ r2 is PC + 2, make it PC + 4
	str	r2, [sp, #S_PC]			@ it's a 2x16bit instr, update
	orr	r0, r0, r5, lsl #16
	badr	lr, __und_usr_fault_32
	@ r0 = the two 16-bit Thumb instructions which caused the exception
	@ r2 = PC value for the following Thumb instruction (:= regs->ARM_pc)
	@ r4 = PC value for the first 16-bit Thumb instruction
	@ lr = 32bit undefined instruction function

#if __LINUX_ARM_ARCH__ < 7
/* If the target arch was overridden, change it back: */
#ifdef CONFIG_CPU_32v6K
	.arch	armv6k
#else
	.arch	armv6
#endif
#endif /* __LINUX_ARM_ARCH__ < 7 */
#else /* !(CONFIG_ARM_THUMB && __LINUX_ARM_ARCH__ >= 6 && CONFIG_CPU_V7) */
	b	__und_usr_fault_16
#endif
 UNWIND(.fnend)
ENDPROC(__und_usr)

/*
 * The out of line fixup for the ldrt instructions above.
 */
	.pushsection .text.fixup, "ax"
	.align	2
4:	str     r4, [sp, #S_PC]			@ retry current instruction
	ret	r9
	.popsection
	.pushsection __ex_table,"a"
	.long	1b, 4b
#if CONFIG_ARM_THUMB && __LINUX_ARM_ARCH__ >= 6 && CONFIG_CPU_V7
	.long	2b, 4b
	.long	3b, 4b
#endif
	.popsection

/*
 * Check whether the instruction is a co-processor instruction.
 * If yes, we need to call the relevant co-processor handler.
 *
 * Note that we don't do a full check here for the co-processor
 * instructions; all instructions with bit 27 set are well
 * defined.  The only instructions that should fault are the
 * co-processor instructions.  However, we have to watch out
 * for the ARM6/ARM7 SWI bug.
 *
 * NEON is a special case that has to be handled here. Not all
 * NEON instructions are co-processor instructions, so we have
 * to make a special case of checking for them. Plus, there's
 * five groups of them, so we have a table of mask/opcode pairs
 * to check against, and if any match then we branch off into the
 * NEON handler code.
 *
 * Emulators may wish to make use of the following registers:
 *  r0  = instruction opcode (32-bit ARM or two 16-bit Thumb)
 *  r2  = PC value to resume execution after successful emulation
 *  r9  = normal "successful" return address
 *  r10 = this threads thread_info structure
 *  lr  = unrecognised instruction return address
 * IRQs enabled, FIQs enabled.
 */
	@
	@ Fall-through from Thumb-2 __und_usr
	@
#ifdef CONFIG_NEON
	get_thread_info r10			@ get current thread
	adr	r6, .LCneon_thumb_opcodes
	b	2f
#endif
call_fpe:
	//r10 := address of thread_info structure = "The struct thread_info is the
	//architecture-specific context for the task"
	get_thread_info r10			@ get current thread
#ifdef CONFIG_NEON													//TRUE
	//.LCneon_arm_opcodes:
	//	.word	0xfe000000			@ mask
	//	.word	0xf2000000			@ opcode
	//	.word	0xff100000			@ mask
	//	.word	0xf4000000			@ opcode
	//	.word	0x00000000			@ mask
	//	.word	0x00000000			@ opcode
	//r6 := address of .LCneon_arm_opcodes relative pc.
	adr	r6, .LCneon_arm_opcodes
2:	ldr	r5, [r6], #4			@ mask value						//r5 := .LCneon_arm_opcodes[0] = 0xfe000000; r6 := r6 + 4 = &.LCneon_arm_opcodes + 4.
	ldr	r7, [r6], #4			@ opcode bits matching in mask		//r7 := .LCneon_arm_opcodes[1] = 0xf2000000; r6 := r6 + 4 = &.LCneon_arm_opcodes + 8.
	cmp	r5, #0				@ end mask?								//r5 = 0xFE00_0000 != 0.
	beq	1f															//Does not branch to 1: during the first iteration since r5 is not 0, but does after later iterations in case the undefined instruction is not a valid neon opcode.
	and	r8, r0, r5													//r8 := undefined instruction encoding AND mask value (0xFE00_0000).
	cmp	r8, r7				@ NEON instruction?						//Compares AND result of undefined instruction encoding and mask value with neon OP code 0xF200_0000.
	bne	2b															//Masked undefined instruction not equal to opcode results in backward jump to 2: to check the next neon opcode.
	mov	r7, #1														//Masked undefined instruction leads to execution of this instruction: r7 := 1.
	strb	r7, [r10, #TI_USED_CP + 10]	@ mark CP#10 as used		//thread_info.used_cp[10] = memory[&thread_info + TI_USED_CP_OFFSET + 10] := r7 = 1. TI_USED_CP_OFFSET = offsetof(struct thread_info, used_cp).
	strb	r7, [r10, #TI_USED_CP + 11]	@ mark CP#11 as used		//thread_info.used_cp[11] = memory[&thread_info + TI_USED_CP_OFFSET + 11] := r7 = 1. TI_USED_CP_OFFSET = offsetof(struct thread_info, used_cp).
	b	do_vfp				@ let VFP handler handle this			//Jumps to arch/arm/vfp/entry.S:do_vfp, which calls arch/arm/vfp/vfphw.S:vfp_support_entry which returns to undefined instruction.
1:
#endif																//No floating point operation to emulate due to no neon opcode match.
	tst	r0, #0x08000000			@ only CDP/CPRT/LDC/STC have bit 27	//AND between r0 (binary code of undefined instruction) and 0x0800_0000 and sets condition flags accordingly.
	tstne	r0, #0x04000000			@ bit 26 set on both ARM and Thumb-2	//If not all flags are zero, executes this instruction of anding and setting condition flags.
	reteq	lr														//If all flags are zero, then returns to __und_usr_fault_32.
	and	r8, r0, #0x00000f00		@ mask out CP number				//Extract bits 11:8 of undefined instruction.
	mov	r7, #1														//r7 := 1.
	add	r6, r10, r8, lsr #8		@ add used_cp[] array offset first	//r6 = &thread_info + (r8 >> 8).
	strb	r7, [r6, #TI_USED_CP]		@ set appropriate used_cp[]	//thread_info.used_cp[r8 >> 8] := r7 = 1.
#ifdef CONFIG_IWMMXT												//FALSE
	@ Test if we need to give access to iWMMXt coprocessors
	ldr	r5, [r10, #TI_FLAGS]
	rsbs	r7, r8, #(1 << 8)		@ CP 0 or 1 only
	movscs	r7, r5, lsr #(TIF_USING_IWMMXT + 1)
	bcs	iwmmxt_task_enable
#endif
 ARM(	add	pc, pc, r8, lsr #6	)									//pc := pc + ((r8 >> 8) << 2). Find index to the table below.
 THUMB(	lsr	r8, r8, #6		)
 THUMB(	add	pc, r8			)
	nop

	ret.w	lr				@ CP#0									//Return to undefined instruction.
	W(b)	do_fpe				@ CP#1 (FPE)						//Does nothing, returns to undefined instruction via no_fp.
	W(b)	do_fpe				@ CP#2 (FPE)						//Does nothing, returns to undefined instruction via no_fp.
	ret.w	lr				@ CP#3									//Return to undefined instruction.
	ret.w	lr				@ CP#4									//Return to undefined instruction.
	ret.w	lr				@ CP#5									//Return to undefined instruction.
	ret.w	lr				@ CP#6									//Return to undefined instruction.
	ret.w	lr				@ CP#7									//Return to undefined instruction.
	ret.w	lr				@ CP#8									//Return to undefined instruction.
	ret.w	lr				@ CP#9									//Return to undefined instruction.
#ifdef CONFIG_VFP
	W(b)	do_vfp				@ CP#10 (VFP)						//Jumps to arch/arm/vfp/entry.S:do_vfp, which calls arch/arm/vfp/vfphw.S:vfp_support_entry which returns from exception.
	W(b)	do_vfp				@ CP#11 (VFP)						//Jumps to arch/arm/vfp/entry.S:do_vfp, which calls arch/arm/vfp/vfphw.S:vfp_support_entry which returns from exception.
#else
	ret.w	lr				@ CP#10 (VFP)							//Return to undefined instruction.
	ret.w	lr				@ CP#11 (VFP)							//Return to undefined instruction.
#endif
	ret.w	lr				@ CP#12									//Return to undefined instruction.
	ret.w	lr				@ CP#13									//Return to undefined instruction.
	ret.w	lr				@ CP#14 (Debug)							//Return to undefined instruction.
	ret.w	lr				@ CP#15 (Control)						//Return to undefined instruction.

#ifdef NEED_CPU_ARCHITECTURE
	.align	2
.LCcpu_architecture:
	.word	__cpu_architecture
#endif

#ifdef CONFIG_NEON
	.align	6

.LCneon_arm_opcodes:
	.word	0xfe000000			@ mask
	.word	0xf2000000			@ opcode

	.word	0xff100000			@ mask
	.word	0xf4000000			@ opcode

	.word	0x00000000			@ mask
	.word	0x00000000			@ opcode

.LCneon_thumb_opcodes:
	.word	0xef000000			@ mask
	.word	0xef000000			@ opcode

	.word	0xff100000			@ mask
	.word	0xf9000000			@ opcode

	.word	0x00000000			@ mask
	.word	0x00000000			@ opcode
#endif

do_fpe:
	ldr	r4, .LCfp					//r4 := memory[&.LCfp] = &fp_enter
	add	r10, r10, #TI_FPSTATE		@ r10 = workspace
	ldr	pc, [r4]			@ Call FP module USR entry point	//pc := memory[&fp_enter] = &no_fp, which points to ret lr.

/*
 * The FP module is called with these registers set:
 *  r0  = instruction
 *  r2  = PC+4
 *  r9  = normal "successful" return address
 *  r10 = FP workspace
 *  lr  = unrecognised FP instruction return address
 */

	.pushsection .data
	.align	2
ENTRY(fp_enter)
	.word	no_fp
	.popsection

ENTRY(no_fp)
	ret	lr
ENDPROC(no_fp)

__und_usr_fault_32:
	mov	r1, #4						//r1 := 4.
	b	1f							//Branch to 1: below.
__und_usr_fault_16_pan:
	uaccess_disable ip
__und_usr_fault_16:
	mov	r1, #2
1:	mov	r0, sp						//r0 := &pt_regs.
	badr	lr, ret_from_exception	//lr := address of ret_from_exception.
	b	__und_fault					//Jumps to __und_fault: Kernel panic or handles the exception, depending on whether the undefined instruction was executed in kernel mode or not.
ENDPROC(__und_usr_fault_32)
ENDPROC(__und_usr_fault_16)

	.align	5
	//Handles prefetch abort exception from user mode and may send a SIGSEGV
	//signal or handle the fault depending on the type of fault.
__pabt_usr:
//mov r0, #0x20
//mov r1, #0x20
//mov r2, #0x20
//swi 1045
//1:	b 1b
	//Makes room for pt_regs on the stack, and stores in pt_regs r0-r12, lr_usr,
	//sp_usr, memory[r0 + 4] (PC), memory[r0 + 8] (CPSR), and 0xFFFF_FFFF
	//(orig_r0). If cr_alignment is not equal to SCTLR, then updates SCTLR to
	//cr_alignment. Sets DACR such that user addresses are not accessible, and
	//all other addresses accessible as specified by the page tables.
	usr_entry
	//r2 = svc_sp = &pt_regs
	mov	r2, sp				@ regs
	//Causes a kernel panic if fault is not handled. Otherwise returns to this point.
	pabt_helper
 UNWIND(.fnend		)
	/* fall through */
/*
 * This is the return code to user mode for abort handlers
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
	//			vectors and handlers, and user space, are accessible according 		//			to the page tables.
	//		b)	Restores user mode registers to those stored in pt_regs, and
	//			resumes execution in user mode from the corrected link register
	//			address stored during the exception.
ENTRY(ret_from_exception)
 UNWIND(.fnstart	)
 UNWIND(.cantunwind	)
	//r9 := address of thread_info structure = "The struct thread_info is the
	//architecture-specific context for the task"
	get_thread_info tsk
	//r8 := 0 = not system call.
	mov	why, #0
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
	b	ret_to_user
 UNWIND(.fnend		)
ENDPROC(__pabt_usr)
ENDPROC(ret_from_exception)

	.align	5
	//Handles FIQs occurring in user mode. Does nothing.
__fiq_usr:
	//Makes room for pt_regs on the stack, and stores in pt_regs r0-r12, lr_usr,
	//sp_usr, memory[r0 + 4] (PC), memory[r0 + 8] (CPSR), and 0xFFFF_FFFF
	//(orig_r0). If cr_alignment is not equal to SCTLR, then updates SCTLR to
	//cr_alignment. Sets DACR such that user addresses are not accessible, and
	//all other addresses accessible as specified by the page tables.
	usr_entry trace=0
	kuser_cmpxchg_check									//EMPTY.
	mov	r0, sp				@ struct pt_regs *regs		//r0 := sp_svc = &pt_regs.
	//Calls arch/arm/kernel/traps.c:handle_fiq_as_nmi, &pts_regs as argument,
	//and which does not do anything: "nop. FIQ handlers for special arch/arm
	//features can be added here."
	bl	handle_fiq_as_nmi
	//tsk = r9, according to arch/arm/kernel/entry-header.S.
	//r9 := address of thread_info structure = "The struct thread_info is the
	//architecture-specific context for the task"
	get_thread_info tsk
	//1.	Sets DACR such that all addresses of the kernel, I/O, exception
	//		vectors and handlers, and user space, are accessible according to
	//		the page tables.
	//2.	Restores user mode registers to those stored in pt_regs, and resumes
	//		execution in user mode from the corrected link register address
	//		stored during the exception.
	restore_user_regs fast = 0, offset = 0
 UNWIND(.fnend		)
ENDPROC(__fiq_usr)

/*
 * Register switch for ARMv3 and ARMv4 processors
 * r0 = previous task_struct, r1 = previous thread_info, r2 = next thread_info
 * previous and next are guaranteed not to be the same.
 */
ENTRY(__switch_to)
 UNWIND(.fnstart	)
 UNWIND(.cantunwind	)
	add	ip, r1, #TI_CPU_SAVE
 ARM(	stmia	ip!, {r4 - sl, fp, sp, lr} )	@ Store most regs on stack
 THUMB(	stmia	ip!, {r4 - sl, fp}	   )	@ Store most regs on stack
 THUMB(	str	sp, [ip], #4		   )
 THUMB(	str	lr, [ip], #4		   )
	ldr	r4, [r2, #TI_TP_VALUE]
	ldr	r5, [r2, #TI_TP_VALUE + 4]
#ifdef CONFIG_CPU_USE_DOMAINS
	mrc	p15, 0, r6, c3, c0, 0		@ Get domain register
	str	r6, [r1, #TI_CPU_DOMAIN]	@ Save old domain register
	ldr	r6, [r2, #TI_CPU_DOMAIN]
#endif
	switch_tls r1, r4, r5, r3, r7
#if defined(CONFIG_STACKPROTECTOR) && !defined(CONFIG_SMP)
	ldr	r7, [r2, #TI_TASK]
	ldr	r8, =__stack_chk_guard
	.if (TSK_STACK_CANARY > IMM12_MASK)
	add	r7, r7, #TSK_STACK_CANARY & ~IMM12_MASK
	.endif
	ldr	r7, [r7, #TSK_STACK_CANARY & IMM12_MASK]
#endif
#ifdef CONFIG_CPU_USE_DOMAINS
	mcr	p15, 0, r6, c3, c0, 0		@ Set domain register
#endif
	mov	r5, r0
	add	r4, r2, #TI_CPU_SAVE
	ldr	r0, =thread_notify_head
	mov	r1, #THREAD_NOTIFY_SWITCH
	bl	atomic_notifier_call_chain
#if defined(CONFIG_STACKPROTECTOR) && !defined(CONFIG_SMP)
	str	r7, [r8]
#endif
 THUMB(	mov	ip, r4			   )
	mov	r0, r5
 ARM(	ldmia	r4, {r4 - sl, fp, sp, pc}  )	@ Load all regs saved previously
 THUMB(	ldmia	ip!, {r4 - sl, fp}	   )	@ Load all regs saved previously
 THUMB(	ldr	sp, [ip], #4		   )
 THUMB(	ldr	pc, [ip]		   )
 UNWIND(.fnend		)
ENDPROC(__switch_to)

	__INIT

/*
 * User helpers.
 *
 * Each segment is 32-byte aligned and will be moved to the top of the high
 * vector page.  New segments (if ever needed) must be added in front of
 * existing ones.  This mechanism should be used only for things that are
 * really small and justified, and not be abused freely.
 *
 * See Documentation/arm/kernel_user_helpers.rst for formal definitions.
 */
 THUMB(	.arm	)

	.macro	usr_ret, reg
#ifdef CONFIG_ARM_THUMB
	bx	\reg
#else
	ret	\reg
#endif
	.endm

	.macro	kuser_pad, sym, size
	.if	(. - \sym) & 3
	.rept	4 - (. - \sym) & 3
	.byte	0
	.endr
	.endif
	.rept	(\size - (. - \sym)) / 4
	.word	0xe7fddef1
	.endr
	.endm

#ifdef CONFIG_KUSER_HELPERS
	.align	5
	.globl	__kuser_helper_start
__kuser_helper_start:

/*
 * Due to the length of some sequences, __kuser_cmpxchg64 spans 2 regular
 * kuser "slots", therefore 0xffff0f80 is not used as a valid entry point.
 */

__kuser_cmpxchg64:				@ 0xffff0f60

#if defined(CONFIG_CPU_32v6K)

	stmfd	sp!, {r4, r5, r6, r7}
	ldrd	r4, r5, [r0]			@ load old val
	ldrd	r6, r7, [r1]			@ load new val
	smp_dmb	arm
1:	ldrexd	r0, r1, [r2]			@ load current val
	eors	r3, r0, r4			@ compare with oldval (1)
	eorseq	r3, r1, r5			@ compare with oldval (2)
	strexdeq r3, r6, r7, [r2]		@ store newval if eq
	teqeq	r3, #1				@ success?
	beq	1b				@ if no then retry
	smp_dmb	arm
	rsbs	r0, r3, #0			@ set returned val and C flag
	ldmfd	sp!, {r4, r5, r6, r7}
	usr_ret	lr

#elif !defined(CONFIG_SMP)

#ifdef CONFIG_MMU

	/*
	 * The only thing that can break atomicity in this cmpxchg64
	 * implementation is either an IRQ or a data abort exception
	 * causing another process/thread to be scheduled in the middle of
	 * the critical sequence.  The same strategy as for cmpxchg is used.
	 */
	stmfd	sp!, {r4, r5, r6, lr}
	ldmia	r0, {r4, r5}			@ load old val
	ldmia	r1, {r6, lr}			@ load new val
1:	ldmia	r2, {r0, r1}			@ load current val
	eors	r3, r0, r4			@ compare with oldval (1)
	eorseq	r3, r1, r5			@ compare with oldval (2)
2:	stmiaeq	r2, {r6, lr}			@ store newval if eq
	rsbs	r0, r3, #0			@ set return val and C flag
	ldmfd	sp!, {r4, r5, r6, pc}

	.text
kuser_cmpxchg64_fixup:
	@ Called from kuser_cmpxchg_fixup.
	@ r4 = address of interrupted insn (must be preserved).
	@ sp = saved regs. r7 and r8 are clobbered.
	@ 1b = first critical insn, 2b = last critical insn.
	@ If r4 >= 1b and r4 <= 2b then saved pc_usr is set to 1b.
	mov	r7, #0xffff0fff
	sub	r7, r7, #(0xffff0fff - (0xffff0f60 + (1b - __kuser_cmpxchg64)))
	subs	r8, r4, r7
	rsbscs	r8, r8, #(2b - 1b)
	strcs	r7, [sp, #S_PC]
#if __LINUX_ARM_ARCH__ < 6
	bcc	kuser_cmpxchg32_fixup
#endif
	ret	lr
	.previous

#else
#warning "NPTL on non MMU needs fixing"
	mov	r0, #-1
	adds	r0, r0, #0
	usr_ret	lr
#endif

#else
#error "incoherent kernel configuration"
#endif

	kuser_pad __kuser_cmpxchg64, 64

__kuser_memory_barrier:				@ 0xffff0fa0
	smp_dmb	arm
	usr_ret	lr

	kuser_pad __kuser_memory_barrier, 32

__kuser_cmpxchg:				@ 0xffff0fc0

#if __LINUX_ARM_ARCH__ < 6

#ifdef CONFIG_MMU

	/*
	 * The only thing that can break atomicity in this cmpxchg
	 * implementation is either an IRQ or a data abort exception
	 * causing another process/thread to be scheduled in the middle
	 * of the critical sequence.  To prevent this, code is added to
	 * the IRQ and data abort exception handlers to set the pc back
	 * to the beginning of the critical section if it is found to be
	 * within that critical section (see kuser_cmpxchg_fixup).
	 */
1:	ldr	r3, [r2]			@ load current val
	subs	r3, r3, r0			@ compare with oldval
2:	streq	r1, [r2]			@ store newval if eq
	rsbs	r0, r3, #0			@ set return val and C flag
	usr_ret	lr

	.text
kuser_cmpxchg32_fixup:
	@ Called from kuser_cmpxchg_check macro.
	@ r4 = address of interrupted insn (must be preserved).
	@ sp = saved regs. r7 and r8 are clobbered.
	@ 1b = first critical insn, 2b = last critical insn.
	@ If r4 >= 1b and r4 <= 2b then saved pc_usr is set to 1b.
	mov	r7, #0xffff0fff
	sub	r7, r7, #(0xffff0fff - (0xffff0fc0 + (1b - __kuser_cmpxchg)))
	subs	r8, r4, r7
	rsbscs	r8, r8, #(2b - 1b)
	strcs	r7, [sp, #S_PC]
	ret	lr
	.previous

#else
#warning "NPTL on non MMU needs fixing"
	mov	r0, #-1
	adds	r0, r0, #0
	usr_ret	lr
#endif

#else

	smp_dmb	arm
1:	ldrex	r3, [r2]
	subs	r3, r3, r0
	strexeq	r3, r1, [r2]
	teqeq	r3, #1
	beq	1b
	rsbs	r0, r3, #0
	/* beware -- each __kuser slot must be 8 instructions max */
	ALT_SMP(b	__kuser_memory_barrier)
	ALT_UP(usr_ret	lr)

#endif

	kuser_pad __kuser_cmpxchg, 32

__kuser_get_tls:				@ 0xffff0fe0
	ldr	r0, [pc, #(16 - 8)]	@ read TLS, set in kuser_get_tls_init
	usr_ret	lr
	mrc	p15, 0, r0, c13, c0, 3	@ 0xffff0fe8 hardware TLS code
	kuser_pad __kuser_get_tls, 16
	.rep	3
	.word	0			@ 0xffff0ff0 software TLS value, then
	.endr				@ pad up to __kuser_helper_version

__kuser_helper_version:				@ 0xffff0ffc
	.word	((__kuser_helper_end - __kuser_helper_start) >> 5)

	.globl	__kuser_helper_end
__kuser_helper_end:

#endif

 THUMB(	.thumb	)

/*
 * Vector stubs.
 *
 * This code is copied to 0xffff1000 so we can use branches in the
 * vectors, rather than ldr's.  Note that this code must not exceed
 * a page size.
 *
 * Common stub entry macro:
 *   Enter in IRQ mode, spsr = SVC/USR CPSR, lr = SVC/USR PC
 *
 * SP points to a minimal amount of processor-private memory, the address
 * of which is copied into r0 for the mode specific abort handler.
 */
	//0.	lr_<exception processor mode> :=
	//		lr_<exception processor mode> - \correction
	//1.	memory[sp_<exception processor mode>] := r0
	//		memory[sp_<exception processor mode> + 4] :=
	//		lr_<exception processor_mode> - \correction
	//		memory[sp_<exception processor mode> + 8] := CPSR before exception
	//2.	Sets the CPU in supervisor mode.
	//3.	r0 := sp_<exception processor mode>
	.macro	vector_stub, name, mode, correction=0
	.align	5						//2^5 = 32-byte aligned.

vector_\name:
	//If correction is not zero, then decrement return address by correction.
	//For instance, for IRQ/FIQ, LR contains the address of next instruction to
	//execute. Since the current instruction that got interrupted must be
	//re-executed, the correction is 4 for IRQ/FIQ exceptions.
	.if \correction
	sub	lr, lr, #\correction
	.endif

	@
	@ Save r0, lr_<exception> (parent PC) and spsr_<exception>
	@ (parent CPSR)
	@
	//The handlers <dabt, irq, und, pabt, fiq>_svc calls svc_entry,
	//and fiq_abt. abt, irq, und and fiq exceptions uses a 12-byte stack, which
	//is initialized by arch/arm/kernel/setup.c:cpu_init with the structure
	//stack and its array stacks (one for each CPU). 12 bytes is sufficient to
	//store r0, lr and SPSR on the stack, with one empty slot.
	//
	//store r0 and lr on stack and increment (not decrement?) sp afterwards. sp
	//is banked to the current execution mode taken to by the exception.
	//According to the comment above, the stack pointer "points to a minimal
	//processor-private memory."
	//r0 is not banked.
	//lr is banked. lr contains the preferred return address for the exception:
	//-Undefined instruction: Address of undefined instruction.
	//-Supervisor call: Address of instruction after svc instruction.
	//-Prefetch abort: Address of instruction whose fetch was aborted.
	//-Data abort: Address of instruction whose memory access failed.
	//-IRQ/FIQ: Address of next instruction to execute.
	//
	//sp is banked.
	//
	//memory[sp_<exception processor mode>] := r0
	//memory[sp_<exception processor mode> + 4] :=
	//lr_<exception processor_mode> - \correction
	stmia	sp, {r0, lr}		@ save r0, lr
	//Moves (saved/copied CPSR before exception) SPSR to lr.
	mrs	lr, spsr
	//Saves SPSR on stack.
	//memory[sp_<exception processor mode> + 8] := SPSR = CPSR before exception.
	str	lr, [sp, #8]		@ save spsr
	//Now we have
	//memory[sp_<exception processor mode>] := r0
	//memory[sp_<exception processor mode> + 4] := lr
	//memory[sp_<exception processor mode> + 8] := SPSR = CPSR before exception.

	@
	@ Prepare for SVC32 mode.  IRQs remain disabled.
	@
	mrs	r0, cpsr								//Moves CPSR to R0.
	//#define SVC_MODE	0x00000013 = supervisor
	//#define PSR_ISETSTATE	0
	//Exclusive ORs r0 with exclusive or of exception mode and supervisor mode.
	//\mode ^ SVC_MODE inverts bits 4, 1 and 0 of \mode, where bit 4 is always
	//set, and thus bit 4 gets cleared and 1 and 0 inverted.
	//r0 =
	//cpsr[0] ^ (\mode[0] ^ 1) =
	//cpsr[0] ^ ~mode[0] =
	//current_mode_according_to_cpsr[0]^~current_mode_according_to_exception[0]=
	//current_mode[0] ^ ~current_mode[0] =
	//1 ^ ~1 = 1 ^ 0 = 1
	//0 ^ ~0 = 0 ^ 1 = 1
	//Same conclusion applies on bit 1.
	//All bits in r0 are preserved except bits 1 and 0 which are set.
	//This means that the mode is changed to be in supervisor mode.
	eor	r0, r0, #(\mode ^ SVC_MODE | PSR_ISETSTATE)
	//Updates bits c (7:0), x (15:8), s (23:17), f (31:24) of SPSR. The mode in
	//SPSR is now supervisor mode, BUT THE CPU IS NOT YET IN supervisor mode.
	msr	spsr_cxsf, r0

	@
	@ the branch table must immediately follow this code
	@
	//Bit 4 of processor mode is always 1 so only lower 4 bits matter.
	//Extracts relevant processor mode bits of processor mode before exception
	//was raised.
	and	lr, lr, #0x0f
 THUMB(	adr	r0, 1f			)
 THUMB(	ldr	lr, [r0, lr, lsl #2]	)
	//r0 := sp_<exception processor mode>
	mov	r0, sp
	//lr is set to the address of the current instruction + 8 +
	//(previous_processor_mode_before_exception_code times 4), which is an
	//address of a handler, depending on the processor mode before the exception
	//occured:
	//-usr (is previous_mode) = 0000: PC + 0 = __<exception>_usr
	//-fiq (is previous_mode) = 0001: PC + 4 = __<exception>_invalid
	//-irq (is previous_mode) = 0010: PC + 8 = __<exception>_invalid
	//-svc (is previous_mode) = 0011: PC + 12 = __<exception>_svc
	//-abt (is previous_mode) = 0111: PC + 28 = __<exception>_invalid
	//-und (is previous_mode) = 1011: PC + 44 = __<exception>_invalid
	//
	//Except for exceptions from fiq, whose handlers are:
	//-previous mode is usr (is fiq): __fiq_usr
	//-previous mode is abt (is fiq): __fiq_abt
	//-other previous modes (is fiq): __fiq_svc
	//
	//And except for exceptions from svc.
	//
	//__<current_exception>_invalid causes a jump to common_invalid which jumps
	//to bad_mode which causes a panic.
 ARM(	ldr	lr, [pc, lr, lsl #2]	)	//lsl = left shift 2 bits = multiply 4.
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//SETS THE CPU IN SUPERVISOR MODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//movs pc, lr, jumps to the address in lr, and set CPSR to SPSR, which is
	//supervisor mode.
	movs	pc, lr			@ branch to handler in SVC mode
ENDPROC(vector_\name)

	.align	2
	@ handler addresses follow this label
1:
	.endm

	.section .stubs, "ax", %progbits
	@ This must be the first word
	.word	vector_swi

vector_rst:
mov r0, #0x1A
mov r1, #0x1A
mov r2, #0x1A
swi 1045
1: b 1b
 ARM(	swi	SYS_ERROR0	)
 THUMB(	svc	#0		)
 THUMB(	nop			)
	b	vector_und

/*
 * Interrupt dispatcher
 */
	vector_stub	irq, IRQ_MODE, 4

	.long	__irq_usr			@  0  (USR_26 / USR_32)
	.long	__irq_invalid			@  1  (FIQ_26 / FIQ_32)
	.long	__irq_invalid			@  2  (IRQ_26 / IRQ_32)
	.long	__irq_svc			@  3  (SVC_26 / SVC_32)
	.long	__irq_invalid			@  4
	.long	__irq_invalid			@  5
	.long	__irq_invalid			@  6
	.long	__irq_invalid			@  7
	.long	__irq_invalid			@  8
	.long	__irq_invalid			@  9
	.long	__irq_invalid			@  a
	.long	__irq_invalid			@  b
	.long	__irq_invalid			@  c
	.long	__irq_invalid			@  d
	.long	__irq_invalid			@  e
	.long	__irq_invalid			@  f

/*
 * Data abort dispatcher
 * Enter in ABT mode, spsr = USR CPSR, lr = USR PC
 */
	vector_stub	dabt, ABT_MODE, 8

	.long	__dabt_usr			@  0  (USR_26 / USR_32)
	.long	__dabt_invalid			@  1  (FIQ_26 / FIQ_32)
	.long	__dabt_invalid			@  2  (IRQ_26 / IRQ_32)
	.long	__dabt_svc			@  3  (SVC_26 / SVC_32)
	.long	__dabt_invalid			@  4
	.long	__dabt_invalid			@  5
	.long	__dabt_invalid			@  6
	.long	__dabt_invalid			@  7
	.long	__dabt_invalid			@  8
	.long	__dabt_invalid			@  9
	.long	__dabt_invalid			@  a
	.long	__dabt_invalid			@  b
	.long	__dabt_invalid			@  c
	.long	__dabt_invalid			@  d
	.long	__dabt_invalid			@  e
	.long	__dabt_invalid			@  f

/*
 * Prefetch abort dispatcher
 * Enter in ABT mode, spsr = USR CPSR, lr = USR PC
 */
	vector_stub	pabt, ABT_MODE, 4

	.long	__pabt_usr			@  0 (USR_26 / USR_32)
	.long	__pabt_invalid			@  1 (FIQ_26 / FIQ_32)
	.long	__pabt_invalid			@  2 (IRQ_26 / IRQ_32)
	.long	__pabt_svc			@  3 (SVC_26 / SVC_32)
	.long	__pabt_invalid			@  4
	.long	__pabt_invalid			@  5
	.long	__pabt_invalid			@  6
	.long	__pabt_invalid			@  7
	.long	__pabt_invalid			@  8
	.long	__pabt_invalid			@  9
	.long	__pabt_invalid			@  a
	.long	__pabt_invalid			@  b
	.long	__pabt_invalid			@  c
	.long	__pabt_invalid			@  d
	.long	__pabt_invalid			@  e
	.long	__pabt_invalid			@  f

/*
 * Undef instr entry dispatcher
 * Enter in UND mode, spsr = SVC/USR CPSR, lr = SVC/USR PC
 */
	vector_stub	und, UND_MODE

	.long	__und_usr			@  0 (USR_26 / USR_32)
	.long	__und_invalid			@  1 (FIQ_26 / FIQ_32)
	.long	__und_invalid			@  2 (IRQ_26 / IRQ_32)
	.long	__und_svc			@  3 (SVC_26 / SVC_32)
	.long	__und_invalid			@  4
	.long	__und_invalid			@  5
	.long	__und_invalid			@  6
	.long	__und_invalid			@  7
	.long	__und_invalid			@  8
	.long	__und_invalid			@  9
	.long	__und_invalid			@  a
	.long	__und_invalid			@  b
	.long	__und_invalid			@  c
	.long	__und_invalid			@  d
	.long	__und_invalid			@  e
	.long	__und_invalid			@  f

	.align	5

/*=============================================================================
 * Address exception handler
 *-----------------------------------------------------------------------------
 * These aren't too critical.
 * (they're not supposed to happen, and won't happen in 32-bit data mode).
 */

vector_addrexcptn:
	b	vector_addrexcptn

/*=============================================================================
 * FIQ "NMI" handler
 *-----------------------------------------------------------------------------
 * Handle a FIQ using the SVC stack allowing FIQ act like NMI on x86
 * systems.
 */
	vector_stub	fiq, FIQ_MODE, 4

	.long	__fiq_usr			@  0  (USR_26 / USR_32)
	.long	__fiq_svc			@  1  (FIQ_26 / FIQ_32)
	.long	__fiq_svc			@  2  (IRQ_26 / IRQ_32)
	.long	__fiq_svc			@  3  (SVC_26 / SVC_32)
	.long	__fiq_svc			@  4
	.long	__fiq_svc			@  5
	.long	__fiq_svc			@  6
	.long	__fiq_abt			@  7
	.long	__fiq_svc			@  8
	.long	__fiq_svc			@  9
	.long	__fiq_svc			@  a
	.long	__fiq_svc			@  b
	.long	__fiq_svc			@  c
	.long	__fiq_svc			@  d
	.long	__fiq_svc			@  e
	.long	__fiq_svc			@  f

	.globl	vector_fiq

	//.vectors section placed in own 4kB page.
	.section .vectors, "ax", %progbits
.L__vectors_start:
	W(b)	vector_rst						//Not used.
	W(b)	vector_und						//Undefined instruction.
	//Supervisor Call. The system call handler is located in the .stubs section
	//(see above), which is located by arch/arm/include/asm/vmlinux.lds.S at the
	//start of the following 4kB page.
	//'arm-linux-gnueabihf-objdump -S --start-address=0xffff0000 --stop-address=0xffff0040 vmlinux'
	//gives
	//'ffff0008:	e59ffff0 	ldr	pc, [pc, #4080]	; ffff1000 <PRRR+0xf48e58>'
	//which does pc := memory[pc + 4080] (where pc is 8 bytes above, so we have
	//pc := memory[pc + 4088], and since W(ldr)	pc, .L__vectors_start + 0x1000
	//is 8 bytes into this page, there are 4096 - 8 = 4088 bytes to the
	//beginning of the next page). This means that the PC is loaded with the
	//first word of the next page, which is the first word of the .stubs
	//section, which begins with '.word vector_swi' (see above). This word is
	//the address of arch/arm/kernel/entry-common.S:vector_swi, which is the
	//system call handler.
	W(ldr)	pc, .L__vectors_start + 0x1000
	W(b)	vector_pabt						//Prefetch Abort.
	W(b)	vector_dabt						//Data Abort.
	W(b)	vector_addrexcptn				//Not used.
	W(b)	vector_irq						//IRQ interrupt.
	W(b)	vector_fiq						//FIQ interrupt.

	.data
	.align	2

	.globl	cr_alignment
cr_alignment:
	.space	4
