/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/arch/arm/kernel/head-common.S
 *
 *  Copyright (C) 1994-2002 Russell King
 *  Copyright (c) 2003 ARM Limited
 *  All Rights Reserved
 */
#include <asm/assembler.h>

#define ATAG_CORE 0x54410001
#define ATAG_CORE_SIZE ((2*4 + 3*4) >> 2)
#define ATAG_CORE_SIZE_EMPTY ((2*4) >> 2)

#ifdef CONFIG_CPU_BIG_ENDIAN
#define OF_DT_MAGIC 0xd00dfeed
#else
#define OF_DT_MAGIC 0xedfe0dd0 /* 0xd00dfeed in big-endian */
#endif

/*
 * Exception handling.  Something went wrong and we can't proceed.  We
 * ought to tell the user, but since we don't have any guarantee that
 * we're even running on the right architecture, we do virtually nothing.
 *
 * If CONFIG_DEBUG_LL is set we try to print out something about the error
 * and hope for the best (useful if bootloader fails to pass a proper
 * machine ID for example).
 */
	__HEAD

/* Determine validity of the r2 atags pointer.  The heuristic requires
 * that the pointer be aligned, in the first 16k of physical RAM and
 * that the ATAG_CORE marker is first and present.  If CONFIG_OF_FLATTREE
 * is selected, then it will also accept a dtb pointer.  Future revisions
 * of this function may be more lenient with the physical address and
 * may also be able to move the ATAGS block if necessary.
 *
 * Returns:
 *  r2 either valid atags pointer, valid dtb pointer, or zero
 *  r5, r6 corrupted
 */
__vet_atags:
	tst	r2, #0x3			@ aligned?	//Bitwise AND of r2 with 0x11 and setting condition codes.
	bne	1f								//Branch if AND is not zero.
	ldr	r5, [r2, #0]
#ifdef CONFIG_OF_FLATTREE	//TRUE
	ldr	r6, =OF_DT_MAGIC		@ is it a DTB?
	cmp	r5, r6
	beq	2f								//If the DTB is indeed a DTB by the magic number, branch to 2.
#endif
	cmp	r5, #ATAG_CORE_SIZE		@ is first tag ATAG_CORE?
	cmpne	r5, #ATAG_CORE_SIZE_EMPTY
	bne	1f
	ldr	r5, [r2, #4]
	ldr	r6, =ATAG_CORE
	cmp	r5, r6
	bne	1f

2:	ret	lr				@ atag/dtb pointer is ok	//Returns to next instruction whose adress is in lr.

1:	mov	r2, #0
	ret	lr
ENDPROC(__vet_atags)

/*
 * The following fragment of code is executed with the MMU on in MMU mode,
 * and uses absolute addresses; this is not position independent.
 *
 *  r0  = cp#15 control register (exc_ret for M-class)
 *  r1  = machine ID
 *  r2  = atags/dtb pointer
 *  r9  = processor ID
 */
	__INIT
//__INIT is .init.text, "ax", causing __mmap_switched to be placed in the
//INIT_TEXT_SECTION (defined in include/asm-generic/vmlinux.lds.h) and
//referenced by arch/arm/kernel/vmlinux.lds.S.
//
//Virtual address of __mmap_switched_data = c0c002e0 (System.map).
//Entered after arch/arm/kernel/head.S:__turn_mmu_on.
__mmap_switched:
	mov	r7, r1			//machine/architecture number.
	mov	r8, r2			//Physical address of first byte of DTB stored in r8.
	mov	r10, r0			//SCTLR contents = 0. See the comment before the call
						//to arch/arm/kernel/head.S:__create_page_tables.

	//Virtual address of __mmap_switched_data = c0c00330 (System.map).
	adr	r4, __mmap_switched_data

	//"the fp stands for the frame pointer which is used as a base pointer to
	// local variables on the stack. Although sp (stack pointer) varies
	// according to function calls from a function, fp holds a fixed value. The
	// relationship between the sp and the fp is a just difference of offsets
	// for the stack. For a wise compiler, the fp might not be needed because it
	// could be realized by adjusting offsets for the sp.  Therefore it would be
	// no problems unless the fp would be used. As for the gcc, you can see the
	// behavior of the fp by using -fno-omit-frame-pointer switch."
	mov	fp, #0							//Frame pointer set to zero.
#if defined(CONFIG_XIP_DEFLATED_DATA)	//FALSE
   ARM(	ldr	sp, [r4], #4 )
 THUMB(	ldr	sp, [r4] )
 THUMB(	add	r4, #4 )
	bl	__inflate_kernel_data		@ decompress .data to RAM
	teq	r0, #0
	bne	__error
#elif defined(CONFIG_XIP_KERNEL)		//FALSE
   ARM(	ldmia	r4!, {r0, r1, r2, sp} )
 THUMB(	ldmia	r4!, {r0, r1, r2, r3} )
 THUMB(	mov	sp, r3 )
	sub	r2, r2, r1
	bl	__memcpy			@ copy .data to RAM
#endif
	//r4 = __mmap_switched_data, causing loads &__bss_start into r0, &__bss_stop
	//into r1, and the virtual address of the stack. The virtual address of the
	//stack is set to init_thread_union + THREAD_START_SP.
	//init_thread_union is defined in include/asm-generic/vmlinux.lds.h of size
	//THREAD_SIZE = 2 pages, and which is page aligned since it is a linker
	//script section, and used by RW_DATA in arch/arm/kernel/vmlinux.lds.S.
	//THREAD_START_SP is defined to THREAD_SIZE - 8, where each thread has
	//kernel stack size THREAD_SIZE. The first word used one this stack is
	//init_thread_union + THREAD_START_SP =
	//init_thread_union + THREAD_SIZE - 8, leraving one word unused.
	//
	//bss is uninitialized kernel data.
	//Increments r4, r4 + 4, r4 + 8 and r4 + 12 to point to the labels
	//processor_id, __machine_arch_type, __atags_pointer, cr_alignment.
   ARM(	ldmia	r4!, {r0, r1, sp} )
 THUMB(	ldmia	r4!, {r0, r1, r3} )		//EMPTY
 THUMB(	mov	sp, r3 )					//EMPTY
	sub	r2, r1, r0						//Size of bss.
	mov	r1, #0
	//r0 = bss start, r1 = 0 (value to write), r2 = bss size. Sets all bytes in
	//bss to 0. This is done because the C code in the kernel assumes zero
	//initialized uninitialized variables.
	bl	__memset			@ clear .bss

	//Loads addresses of the variables processor_id, __machine_arch_type,
	//__atags_pointer, cr_alignment into r0, into r1, r2, and r3, respectively.
	ldmia	r4, {r0, r1, r2, r3}
	str	r9, [r0]			@ Save processor ID
	str	r7, [r1]			@ Save machine type
	//Stores physical address of first byte of DTB in variable __atags_pointer.
	str	r8, [r2]			@ Save atags pointer

	//If pointer cr_alignment is not null store cp#15 control register in that
	//variable.
	cmp	r3, #0
	strne	r10, [r3]			@ Save control register values
#ifdef CONFIG_KASAN				//FALSE
	bl	kasan_early_init
#endif
	mov	lr, #0
	b	start_kernel			//Jumps to init/main.c:start_kernel.
ENDPROC(__mmap_switched)

	.align	2
	.type	__mmap_switched_data, %object
__mmap_switched_data:
#ifdef CONFIG_XIP_KERNEL				//FALSE
#ifndef CONFIG_XIP_DEFLATED_DATA
	.long	_sdata				@ r0
	.long	__data_loc			@ r1
	.long	_edata_loc			@ r2
#endif
	.long	__bss_stop			@ sp (temporary stack in .bss)
#endif

	.long	__bss_start			@ r0
	.long	__bss_stop			@ r1
	.long	init_thread_union + THREAD_START_SP @ sp

	.long	processor_id			@ r0	//Address of variable arch/arm/kernel/setup.c:processor_id?
	.long	__machine_arch_type		@ r1	//Address of variable
											//arch/arm/boot/compressed/misc.c:__machine_arch_type?
	.long	__atags_pointer			@ r2	//Address of variable arch/arm/mm/mmu.c:__atags_pointer.
#ifdef CONFIG_CPU_CP15					//TRUE
	.long	cr_alignment			@ r3	//Address of space
											//arch/arm/kernel/entry-armv.S:cr_alignment
#else
M_CLASS(.long	exc_ret)			@ r3
AR_CLASS(.long	0)				@ r3
#endif
	.size	__mmap_switched_data, . - __mmap_switched_data

	__FINIT
	.text

/*
 * This provides a C-API version of __lookup_processor_type
 */
ENTRY(lookup_processor_type)
	stmfd	sp!, {r4 - r6, r9, lr}
	mov	r9, r0
	bl	__lookup_processor_type
	mov	r0, r5
	ldmfd	sp!, {r4 - r6, r9, pc}
ENDPROC(lookup_processor_type)

/*
 * Read processor ID register (CP#15, CR0), and look up in the linker-built
 * supported processor list.  Note that we can't use the absolute addresses
 * for the __proc_info lists since we aren't running with the MMU on
 * (and therefore, we are not in the correct address space).  We have to
 * calculate the offset.
 *
 *	r9 = cpuid
 * Returns:
 *	r3, r4, r6 corrupted
 *	r5 = proc_info pointer in physical address space
 *	r9 = cpuid (preserved)
 */
__lookup_processor_type:
	/*
	 * Look in <asm/procinfo.h> for information about the __proc_info
	 * structure.
	 */
	adr_l	r5, __proc_info_begin
	adr_l	r6, __proc_info_end
1:	ldmia	r5, {r3, r4}			@ value, mask
	and	r4, r4, r9			@ mask wanted bits
	teq	r3, r4
	beq	2f
	add	r5, r5, #PROC_INFO_SZ		@ sizeof(proc_info_list)
	cmp	r5, r6
	blo	1b
	mov	r5, #0				@ unknown processor
2:	ret	lr
ENDPROC(__lookup_processor_type)

__error_lpae:
#ifdef CONFIG_DEBUG_LL
	adr	r0, str_lpae
	bl 	printascii
	b	__error
str_lpae: .asciz "\nError: Kernel with LPAE support, but CPU does not support LPAE.\n"
#else
	b	__error
#endif
	.align
ENDPROC(__error_lpae)

__error_p:
#ifdef CONFIG_DEBUG_LL
	adr	r0, str_p1
	bl	printascii
	mov	r0, r9
	bl	printhex8
	adr	r0, str_p2
	bl	printascii
	b	__error
str_p1:	.asciz	"\nError: unrecognized/unsupported processor variant (0x"
str_p2:	.asciz	").\n"
	.align
#endif
ENDPROC(__error_p)

__error:
#ifdef CONFIG_ARCH_RPC
/*
 * Turn the screen red on a error - RiscPC only.
 */
	mov	r0, #0x02000000
	mov	r3, #0x11
	orr	r3, r3, r3, lsl #8
	orr	r3, r3, r3, lsl #16
	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
#endif
1:	mov	r0, r0
	b	1b
ENDPROC(__error)
