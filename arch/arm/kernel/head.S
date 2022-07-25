/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/arch/arm/kernel/head.S
 *
 *  Copyright (C) 1994-2002 Russell King
 *  Copyright (c) 2003 ARM Limited
 *  All Rights Reserved
 *
 *  Kernel startup code for all 32-bit CPUs
 */
#include <linux/linkage.h>
#include <linux/init.h>
#include <linux/pgtable.h>

#include <asm/assembler.h>
#include <asm/cp15.h>
#include <asm/domain.h>
#include <asm/ptrace.h>
#include <asm/asm-offsets.h>
#include <asm/memory.h>
#include <asm/thread_info.h>

#ifdef CONFIG_TRUSTFULL_HYPERVISOR
#include <asm/trustfull/hypercalls.h>
#endif

#if defined(CONFIG_DEBUG_LL) && !defined(CONFIG_DEBUG_SEMIHOSTING)
#include CONFIG_DEBUG_LL_INCLUDE
#endif
/*
 * swapper_pg_dir is the virtual address of the initial page table.
 * We place the page tables 16K below KERNEL_RAM_VADDR.  Therefore, we must
 * make sure that KERNEL_RAM_VADDR is correctly set.  Currently, we expect
 * the least significant 16 bits to be 0x8000, but we could probably
 * relax this restriction to KERNEL_RAM_VADDR >= PAGE_OFFSET + 0x4000.
 */
#define KERNEL_RAM_VADDR	(KERNEL_OFFSET + TEXT_OFFSET)
#if (KERNEL_RAM_VADDR & 0xffff) != 0x8000
#error KERNEL_RAM_VADDR must start at 0xXXXX8000
#endif

#ifdef CONFIG_ARM_LPAE
	/* LPAE requires an additional page for the PGD */
#define PG_DIR_SIZE	0x5000
#define PMD_ORDER	3
#else
#define PG_DIR_SIZE	0x4000
#define PMD_ORDER	2
#endif

	.globl	swapper_pg_dir
	.equ	swapper_pg_dir, KERNEL_RAM_VADDR - PG_DIR_SIZE

	/*
	 * This needs to be assigned at runtime when the linker symbols are
	 * resolved. These are unsigned 64bit really, but in this assembly code
	 * We store them as 32bit.
	 */
	.pushsection .data
	.align	2
	.globl	kernel_sec_start
	.globl	kernel_sec_end
kernel_sec_start:
	.long	0
	.long	0
kernel_sec_end:
	.long	0
	.long	0
	.popsection

	.macro	pgtbl, rd, phys
	add	\rd, \phys, #TEXT_OFFSET
	sub	\rd, \rd, #PG_DIR_SIZE
	.endm

/*
 * Kernel startup entry point.
 * ---------------------------
 *
 * This is normally called from the decompressor code.  The requirements
 * are: MMU = off, D-cache = off, I-cache = dont care, r0 = 0,
 * r1 = machine nr, r2 = atags or dtb pointer.
 *
 * This code is mostly position independent, so if you link the kernel at
 * 0xc0008000, you call this at __pa(0xc0008000).
 *
 * See linux/arch/arm/tools/mach-types for the complete list of machine
 * numbers for r1.
 *
 * We're trying to keep crap to a minimum; DO NOT add any machine specific
 * crap here - that's what the boot loader (or in extreme, well justified
 * circumstances, zImage) is for.
 */
	.arm

	__HEAD
//Entry point from arch/arm/boot/compressed/head.S:__enter_kernel
ENTRY(stext)
 ARM_BE8(setend	be )			@ ensure we are in BE8 mode			//Empty

 THUMB(	badr	r9, 1f		)	@ Kernel is always entered in ARM.	//Empty
 THUMB(	bx	r9		)	@ If this is a Thumb-2 kernel,				//Empty
 THUMB(	.thumb			)	@ switch to Thumb now.					//Empty
 THUMB(1:			)												//Empty

#ifdef CONFIG_ARM_VIRT_EXT											//TRUE
	bl	__hyp_stub_install
#endif
#ifdef CONFIG_TRUSTFULL_HYPERVISOR
	swi 1051		@ get processor id								//in r9.
#else
	@ ensure svc mode and all interrupts masked
	safe_svcmode_maskall r9
	mrc	p15, 0, r9, c0, c0		@ get processor id
#endif
	//Obtains address to procinfo struct (arch/arm/include/asm/procinfo.h),
	//which is initialized during compilation by section ".proc.info.init"
	//in arch/arm/mm/proc-v7.S (which defines a number of processors, e.g.
	//A5, A9 and A8, R7, ...) and placed accordingly in the named section
	//.init.proc.info in arch/arm/kernel/vmlinux.lds.S.
	bl	__lookup_processor_type		@ r5=procinfo r9=cpuid
	//r10 = address of procinfo struct.
	movs	r10, r5				@ invalid processor (r5=0)?
 THUMB( it	eq )		@ force fixup-able long branch encoding		//Empty
	beq	__error_p			@ yes, error 'p'

#ifdef CONFIG_ARM_LPAE												//FALSE
	mrc	p15, 0, r3, c0, c1, 4		@ read ID_MMFR0
	and	r3, r3, #0xf			@ extract VMSA support
	cmp	r3, #5				@ long-descriptor translation table format?
 THUMB( it	lo )				@ force fixup-able long branch encoding
	blo	__error_lpae			@ only classic page table format
#endif

#ifndef CONFIG_XIP_KERNEL							//TRUE = NOT DEFINED
	//"_text and _etext are the start and end address of the text section that
	// contains the compiled kernel code." _text is assigned in
	//arch/arm/kernel/vmlinux.lds.S to KERNEL_OFFSET + TEXT_OFFSET = 0xC000_8000.
	//r8 obtains the physical address of the "label" = 0x81008000.
	adr_l	r8, _text			@ __pa(_text)
	//r8 = 0x8100_0000 since TEXT_OFFSET = 0x00008000.
	sub	r8, r8, #TEXT_OFFSET		@ PHYS_OFFSET
#else
	ldr	r8, =PLAT_PHYS_OFFSET		@ always constant in this case
#endif

	/*
	 * r1 = machine no, r2 = atags or dtb,
	 * r8 = phys_offset, r9 = cpuid, r10 = procinfo
	 */
	//Checks alignment of first byte address of DTB, where r2 contains first
	//byte address of DTB, set by __enter_kernel in
	//arch/arm/boot/compressed/head.s
	bl	__vet_atags
#ifdef CONFIG_SMP_ON_UP				//FALSE
	bl	__fixup_smp
#endif
#ifdef CONFIG_ARM_PATCH_PHYS_VIRT	//TRUE
	//r1 = machine/architecture number, r2 = dtb pointer,
	//r8 = 0x8100_0000 = physical (offset) memory start address relative 0,
	//r9 = cpuid, r10 = physical address of procinfo structure.
	//Updates the phys_to_virt and virt_to_phys function implementations (their
	//assembly instructions by updating the physical memory offset address
	//0x81000000, used in those calculations, since that address is unknown at
	//compilation time, and used as a linear shift with the virtual memory
	//offset address 0xC0000000. See arch/arm/kernel/phys2virt.S.
	bl	__fixup_pv_table
#endif
#ifdef CONFIG_TRUSTFULL_HYPERVISOR
	//Unavailable registers: r1, r2, r9.
	mov r10, r1					//Save r1 = machine/architecture number
	mov r11, r2					//Save r2 = DTB address.
	//Now available registers: r0, r1, r2, r3, r4, r5, r6, r7, r8.



	//Map 0xC000_0000 to physical start address of Linux, up to and including
	//uninitialized data.
	ldr r0, =#KERNEL_OFFSET		//0xC000_0000. Virtual start address of kernel.
	adr_l	r1, _text			//Physical start address of code = 0x8100_8000.
	sub	r1, r1, #TEXT_OFFSET	//Physical start address of memory = 0x8100_0000.
	
	adr_l	r4, _text			//Physical start address of code = 0x8100_8000.
	sub	r4, r4, #TEXT_OFFSET	//Physical start address of memory = 0x8100_0000.
	ldr	r3, =(_end - 1)			//Virtual end address of kernel, including data.
	sub r3, r3, #KERNEL_OFFSET	//Virtual end address offset from start.
	add r2, r4, r3				//Last physical address of used memory.
	ldr r4, =#0xFFF00000
	and r2, r2, r4				//Section address containing last kernel byte.
	add r2, r2, #0x00100000		//End section of kernel, exclusive.
	adr_l	r5, kernel_sec_start
	str r1, [r5]				//Saves physical start address of memory to kernel_sec_start.
	adr_l	r5, kernel_sec_end
	str r2, [r5]				//Saves physical end section address of memory to kernel_sec_end.
	swi 1053					//Hypercall 1: Initial boot virtual Linux map.



	//Map 2MB of DTB from FDT_FIXED_BASE to start of DTB.
	//r0 = start physical address
	//r1 = start virtual address
	mov r0, r11					//DTB address was saved to r11.
	ldr r1, =FDT_FIXED_BASE		//Virtual DTB address.
	swi 1054



	//Restore r1 and r2.
	mov r1, r10
	mov r2, r11



 	mov r0, #0			//Argument to HYPERCALL_CACHE_OP to flush all caches.
 	swi HYPERCALL_CACHE_OP
	//r0 does not contain SCTLR, which __mmap_switched stores into cr_alignment,
	//which in turn is only read by arch/arm/mm/init.c:__clear_cr, which in turn
	//is only used by arch/arm/include/asm/cp15.h:set_cr which writes SCTLR,
	//causing an exception since Linux executes in unprivileged mode, which the
	//hypervisor will intercept and notice. Hence, this can be ignored.
	ldr	r13, =__mmap_switched		@ address to jump to after
	mov pc, r13
#else
	bl	__create_page_tables
#endif

	/*
	 * The following calls CPU specific code in a position independent
	 * manner.  See arch/arm/mm/proc-*.S for details.  r10 = base of
	 * xxx_proc_info structure selected by __lookup_processor_type
	 * above.
	 *
	 * The processor init function will be called with:
	 *  r1 - machine type
	 *  r2 - boot data (atags/dt) pointer
	 *  r4 - translation table base (low word)
	 *  r5 - translation table base (high word, if LPAE)
	 *  r8 - translation table base 1 (pfn if LPAE)
	 *  r9 - cpuid
	 *  r13 - virtual address for __enable_mmu -> __turn_mmu_on
	 *
	 * On return, the CPU will be ready for the MMU to be turned on,
	 * r0 will hold the CPU control register value, r1, r2, r4, and
	 * r9 will be preserved.  r5 will also be preserved if LPAE.
	 */
	//At this point, the following registers contain the following data, needed
	//by __enable_mmu.
	//r1 = machine/architecture number
	//r2 = physical start address of DTB.
	//r4 = page table physical start address
	//r9 = cpuid
	//r13 = compile-time address (see arch/arm/boot/compressed/vmlinux.lds.S
	//INIT_TEXT_SECTION) to jump to after MMU has been enabled.
	//arch/arm/kernel/head-common.S:__mmap_switched.
	ldr	r13, =__mmap_switched		@ address to jump to after
						@ mmu has been enabled
	//Sets r14 = lr to physical address of label 1 forward. See
	//arch/arm/include/asm/assembler.h:badr.
	badr	lr, 1f				@ return (PIC) address
#ifdef CONFIG_ARM_LPAE			//FALSE
	mov	r5, #0				@ high TTBR0
	mov	r8, r4, lsr #12			@ TTBR1 is swapper_pg_dir pfn
#else							//TRUE
	//r8 = page table physical start address
	mov	r8, r4				@ set TTBR1 to swapper_pg_dir
#endif
	//__cpu_flush field of procinfo (see arch/arm/kernel/asm-offsets.c and
	//arch/arm/include/asm/procinfo.h) which is initialized by
	//arch/arm/mm/proc-v7.S:__v7_ca9mp_setup to initialize TLB, caches and MMU
	//so that the MMU can be turned on.
	//r12 = (virtual address of __v7_ca9mp_setup) -
	//      (virtual address of __v7_ca9mp_proc_info)
	ldr	r12, [r10, #PROCINFO_INITFUNC]
	//r12 = (virtual address of __v7_ca9mp_setup) -
	//      (virtual address of __v7_ca9mp_proc_info) +
	//		(physical address of __v7_ca9mp_proc_info) =
	//      (offset of __v7_ca9mp_setup from __v7_ca9mp_proc_info) +
	//		(physical address of __v7_ca9mp_proc_info) =
	//physical address of __v7_ca9mp_setup.
	add	r12, r12, r10
	//ret is macro in arch/arm/include/asm/assembler.h. with r12 being used as
	//return address. Executes __v7_ca9mp_setup, which then returns to the next
	//instruction b __enable_mmu, since lr is set to 1f. __enable_mmu then
	//returns to the virtual address (since MMU is turned on) specified in
	//r13 = __mmap_switched:__mmap_switched.
	ret	r12
1:	b	__enable_mmu
ENDPROC(stext)
	.ltorg

/*
 * Setup the initial page tables.  We only setup the barest
 * amount which are required to get the kernel running, which
 * generally means mapping in the kernel code.
 *
 * r8 = phys_offset, r9 = cpuid, r10 = procinfo
 *
 * Returns:
 *  r0, r3, r5-r7 corrupted
 *  r4 = physical page table address
 */
__create_page_tables:
	//r4 = r8 + 0x00008000 =
	//physical start memory address (0x81000000) + 0x00008000 = 0x81008000.
	//r4 = r4 - PG_DIR_SIZE = 0x81008000 - 0x00004000 = 0x81004000 =
	//physical memory start address + 16kB.
	//start of initial page table, that will later be replaced.
	pgtbl	r4, r8				@ page table address

	/*
	 * Clear the swapper page table
	 */
	mov	r0, r4
	mov	r3, #0
	add	r6, r0, #PG_DIR_SIZE		//End physical address of page table.
1:	str	r3, [r0], #4				//Clear the initial page table with zeros.
	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
	teq	r0, r6						//Test current address equal to end address.
	bne	1b							//If not end address reached, go back.







#ifdef CONFIG_ARM_LPAE				//FALSE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	/*
	 * Build the PGD table (first level) to point to the PMD table. A PGD
	 * entry is 64-bit wide.
	 */
	mov	r0, r4
	add	r3, r4, #0x1000			@ first PMD table address	
	orr	r3, r3, #3			@ PGD block type
	mov	r6, #4				@ PTRS_PER_PGD
	mov	r7, #1 << (55 - 32)		@ L_PGD_SWAPPER
1:
#ifdef CONFIG_CPU_ENDIAN_BE8
	str	r7, [r0], #4			@ set top PGD entry bits
	str	r3, [r0], #4			@ set bottom PGD entry bits
#else
	str	r3, [r0], #4			@ set bottom PGD entry bits
	str	r7, [r0], #4			@ set top PGD entry bits
#endif
	add	r3, r3, #0x1000			@ next PMD table
	subs	r6, r6, #1
	bne	1b

	add	r4, r4, #0x1000			@ point to the PMD tables
#ifdef CONFIG_CPU_ENDIAN_BE8
	add	r4, r4, #4			@ we only write the bottom word
#endif
#endif							//END OF FALSE CONFIG_ARM_LPAE!!!!!!!!!!!!!!!!!!









	//Load mmu flags from procinfo struct into r7, which are page table
	//descriptor configuration bits.
	ldr	r7, [r10, #PROCINFO_MM_MMUFLAGS] @ mm_mmuflags

	/*
	 * Create identity mapping to cater for __enable_mmu.
	 * This identity mapping will be removed by paging_init().
	 */
	//Physical start address of code turning on MMU.
	adr_l	r5, __turn_mmu_on		@ _pa(__turn_mmu_on)
	//Physical end address of code turning on MMU.
	adr_l	r6, __turn_mmu_on_end		@ _pa(__turn_mmu_on_end)
	//Initial and final 1MB indexes of __turn_mmu_on and __turn_mmu_on_end, to
	//their "section" table indexes.
	mov	r5, r5, lsr #SECTION_SHIFT
	mov	r6, r6, lsr #SECTION_SHIFT

	//Restore base address of __turn_mmu_on, which clears bottom 20 bits and
	//defines the section base address, and or it with the MMU Flags which
	//define the section properties.
1:	orr	r3, r7, r5, lsl #SECTION_SHIFT	@ flags + kernel base
	//Stores the section descriptor at the beginning of the initial page table.
	//PMD_ORDER = 2 corresponds to multiplication by 4 bytes, since each section
	//descriptor is 4 bytes (for LPAE, PMD_ORDER = 3, corresponding to
	//multiplication by 8 bytes since each section descriptor is 8 bytes for
	//LPAE).
	str	r3, [r4, r5, lsl #PMD_ORDER]	@ identity mapping
	cmp	r5, r6
	addlo	r5, r5, #1			@ next section	//If next section is lower than
	blo	1b										//final section, create another
												//section.

	/*
	 * The main matter: map in the kernel using section mappings, and
	 * set two variables to indicate the physical start and end of the
	 * kernel.
	 */
	//r4 is page table start address.
	//KERNEL_OFFSET = PAGE_OFFSET = 0xC0000000,
	//SECTION_SHIFT - PMD_ORDER = 20 - 2 = 18:
	//KERNEL_OFFSET >> (SECTION_SHIFT - PMD_ORDER) =
	//Byte index of the section descriptor mapping the first 1MB of kernel
	//virtual memory.
	//r0 = physical address of section descriptor of first MB of virtual kernel
	//memory.
	add	r0, r4, #KERNEL_OFFSET >> (SECTION_SHIFT - PMD_ORDER)
	//Final virtual address of uninitialized data, inclusive.
	ldr	r6, =(_end - 1)
	//r5 = address of 2 longs = 8 bytes.
	adr_l	r5, kernel_sec_start		@ _pa(kernel_sec_start)
#if defined CONFIG_CPU_ENDIAN_BE8 || defined CONFIG_CPU_ENDIAN_BE32	//FALSE
	str	r8, [r5, #4]			@ Save physical start of kernel (BE)
#else																//TRUE
	//Stores start physical address of the kernel (0x81000000) into the memory
	//of kernel_sec_start. Used by arch/arm/mm/mmu.c:lowmem.
	str	r8, [r5]			@ Save physical start of kernel (LE)
#endif
	//r3 = Physical start address of kernel ORed with MMU flags.
	orr	r3, r8, r7			@ Add the MMU flags
	//r6 = page table start address + section descriptor byte index of last 1MB
	//of kernel = physical byte address of last section descriptor of kernel
	//(bss/uninitialized data).
	add	r6, r4, r6, lsr #(SECTION_SHIFT - PMD_ORDER)
	//Stores section descriptor of first virtual kernel MB at its corresponding
	//index in the page table, mapped to its first physical kernel MB
	//(0x81000000). Then updates the pointer r0 by incrementing it by 4.
1:	str	r3, [r0], #1 << PMD_ORDER
	//Increments r3 by 1MB (section base address field) to map to next physical
	//MB.
	add	r3, r3, #1 << SECTION_SHIFT
	cmp	r0, r6			//Compare next (since update) pointer with last pointer.
	bls	1b				//If <=, write next section.
	//All sections are now mapped linearly from physical start kernel address
	//(0x81000000) up to and including uninitialized data.
	//Exclusive or between r3 and r7, which clears MMU flags.
	eor	r3, r3, r7			@ Remove the MMU flags
	//r5 = physical address of label kernel_sec_end, containing 8 bytes.
	adr_l	r5, kernel_sec_end		@ _pa(kernel_sec_end)





#if defined CONFIG_CPU_ENDIAN_BE8 || defined CONFIG_CPU_ENDIAN_BE32	//FALSE!!!!!
	str	r3, [r5, #4]			@ Save physical end of kernel (BE)
#else																//TRUE
	//Stores physical address of last mapped MB of kernel memory (end of
	//BSS/uninitialized kernel data). Used by arch/arm/mm/mmu.c:lowmem.
	str	r3, [r5]			@ Save physical end of kernel (LE)
#endif






#ifdef CONFIG_XIP_KERNEL										//FALSE!!!!!!!!!
	/*
	 * Map the kernel image separately as it is not located in RAM.
	 */
#define XIP_START XIP_VIRT_ADDR(CONFIG_XIP_PHYS_ADDR)
	mov	r3, pc
	mov	r3, r3, lsr #SECTION_SHIFT
	orr	r3, r7, r3, lsl #SECTION_SHIFT
	add	r0, r4,  #(XIP_START & 0xff000000) >> (SECTION_SHIFT - PMD_ORDER)
	str	r3, [r0, #((XIP_START & 0x00f00000) >> SECTION_SHIFT) << PMD_ORDER]!
	ldr	r6, =(_edata_loc - 1)
	add	r0, r0, #1 << PMD_ORDER
	add	r6, r4, r6, lsr #(SECTION_SHIFT - PMD_ORDER)
1:	cmp	r0, r6
	add	r3, r3, #1 << SECTION_SHIFT
	strls	r3, [r0], #1 << PMD_ORDER
	bls	1b
#endif															//End of FALSE!!







	/*
	 * Then map boot params address in r2 if specified.
	 * We map 2 sections in case the ATAGs/DTB crosses a section boundary.
	 */
	//r2 = first byte address of DTB.
	mov	r0, r2, lsr #SECTION_SHIFT	//r0 = 1MB index of DTB by logical shift
									//right DTB address 20 bits.
	cmp	r2, #0						//Is DTB pointer zero? If so, no DTB.
	//If DTB (pointer not equal to zero):
	//FDT_FIXED_BASE = 0xFF80_0000 shifted right 18 bits to get its virtual
	//1MB byte offset.
	//The virtual address of the DTB is computed by ORing the virtual address of
	//the first MB containing the DTB (0xFF800000 = FDT_FIXED_BASE) with the
	//lower 20 bits of the physical start address of the DTB.
	ldrne	r3, =FDT_FIXED_BASE >> (SECTION_SHIFT - PMD_ORDER)
	//Adds page table physical start address to FDT page table index of
	//FDT_FIXED_BASE = Physical address of page table entry mapping DTB.
	addne	r3, r3, r4
	//r6 = section descriptor of first DTB MB, by ORing its 1MB aligned physical
	//address (since r2 above and r0 here are shifted 20 bits) with the MMU
	//flags.
	orrne	r6, r7, r0, lsl #SECTION_SHIFT
	//Stores section descriptor mapping DTB into the page table and increments
	//the page table index pointer to the next section descriptor (r3).
	strne	r6, [r3], #1 << PMD_ORDER
	//Increments section descriptor base address to next 1MB.
	addne	r6, r6, #1 << SECTION_SHIFT
	//Stores the section descriptor mapping a second MB of the DTB.
	strne	r6, [r3]
	//After this instruction only ret lr remains since all other configurations
	//are undefined.
#if defined(CONFIG_ARM_LPAE) && defined(CONFIG_CPU_ENDIAN_BE8)		//FALSE
	sub	r4, r4, #4			@ Fixup page table pointer
						@ for 64-bit descriptors
#endif








#ifdef CONFIG_DEBUG_LL								//FALSE!!!!!!!!!!!!!!!!!!!!!
#if !defined(CONFIG_DEBUG_ICEDCC) && !defined(CONFIG_DEBUG_SEMIHOSTING)
	/*
	 * Map in IO space for serial debugging.
	 * This allows debug messages to be output
	 * via a serial console before paging_init.
	 */
	addruart r7, r3, r0

	mov	r3, r3, lsr #SECTION_SHIFT
	mov	r3, r3, lsl #PMD_ORDER

	add	r0, r4, r3
	mov	r3, r7, lsr #SECTION_SHIFT
	ldr	r7, [r10, #PROCINFO_IO_MMUFLAGS] @ io_mmuflags
	orr	r3, r7, r3, lsl #SECTION_SHIFT
#ifdef CONFIG_ARM_LPAE
	mov	r7, #1 << (54 - 32)		@ XN
#ifdef CONFIG_CPU_ENDIAN_BE8
	str	r7, [r0], #4
	str	r3, [r0], #4
#else
	str	r3, [r0], #4
	str	r7, [r0], #4
#endif
#else
	orr	r3, r3, #PMD_SECT_XN
	str	r3, [r0], #4
#endif

#else /* CONFIG_DEBUG_ICEDCC || CONFIG_DEBUG_SEMIHOSTING */
	/* we don't need any serial debugging mappings */
	ldr	r7, [r10, #PROCINFO_IO_MMUFLAGS] @ io_mmuflags
#endif

#if defined(CONFIG_ARCH_NETWINDER) || defined(CONFIG_ARCH_CATS)
	/*
	 * If we're using the NetWinder or CATS, we also need to map
	 * in the 16550-type serial port for the debug messages
	 */
	add	r0, r4, #0xff000000 >> (SECTION_SHIFT - PMD_ORDER)
	orr	r3, r7, #0x7c000000
	str	r3, [r0]
#endif
#ifdef CONFIG_ARCH_RPC
	/*
	 * Map in screen at 0x02000000 & SCREEN2_BASE
	 * Similar reasons here - for debug.  This is
	 * only for Acorn RiscPC architectures.
	 */
	add	r0, r4, #0x02000000 >> (SECTION_SHIFT - PMD_ORDER)
	orr	r3, r7, #0x02000000
	str	r3, [r0]
	add	r0, r4, #0xd8000000 >> (SECTION_SHIFT - PMD_ORDER)
	str	r3, [r0]
#endif
#endif												//END FALSE CONFIG_DEBUG_LL!







#ifdef CONFIG_ARM_LPAE				//FALSE
	sub	r4, r4, #0x1000		@ point to the PGD table
#endif
	ret	lr
ENDPROC(__create_page_tables)
	.ltorg

#if defined(CONFIG_SMP)
	.text
	.arm
ENTRY(secondary_startup_arm)
 THUMB(	badr	r9, 1f		)	@ Kernel is entered in ARM.
 THUMB(	bx	r9		)	@ If this is a Thumb-2 kernel,
 THUMB(	.thumb			)	@ switch to Thumb now.
 THUMB(1:			)
ENTRY(secondary_startup)
	/*
	 * Common entry point for secondary CPUs.
	 *
	 * Ensure that we're in SVC mode, and IRQs are disabled.  Lookup
	 * the processor type - there is no need to check the machine type
	 * as it has already been validated by the primary processor.
	 */

 ARM_BE8(setend	be)				@ ensure we are in BE8 mode

#ifdef CONFIG_ARM_VIRT_EXT
	bl	__hyp_stub_install_secondary
#endif
	safe_svcmode_maskall r9

	mrc	p15, 0, r9, c0, c0		@ get processor id
	bl	__lookup_processor_type
	movs	r10, r5				@ invalid processor?
	moveq	r0, #'p'			@ yes, error 'p'
 THUMB( it	eq )		@ force fixup-able long branch encoding
	beq	__error_p

	/*
	 * Use the page tables supplied from  __cpu_up.
	 */
	adr_l	r3, secondary_data
	mov_l	r12, __secondary_switched
	ldrd	r4, r5, [r3, #0]		@ get secondary_data.pgdir
ARM_BE8(eor	r4, r4, r5)			@ Swap r5 and r4 in BE:
ARM_BE8(eor	r5, r4, r5)			@ it can be done in 3 steps
ARM_BE8(eor	r4, r4, r5)			@ without using a temp reg.
	ldr	r8, [r3, #8]			@ get secondary_data.swapper_pg_dir
	badr	lr, __enable_mmu		@ return address
	mov	r13, r12			@ __secondary_switched address
	ldr	r12, [r10, #PROCINFO_INITFUNC]
	add	r12, r12, r10			@ initialise processor
						@ (return control reg)
	ret	r12
ENDPROC(secondary_startup)
ENDPROC(secondary_startup_arm)

ENTRY(__secondary_switched)
	ldr_l	r7, secondary_data + 12		@ get secondary_data.stack
	mov	sp, r7
	mov	fp, #0
	b	secondary_start_kernel
ENDPROC(__secondary_switched)

#endif /* defined(CONFIG_SMP) */



/*
 * Setup common bits before finally enabling the MMU.  Essentially
 * this is just loading the page table pointer and domain access
 * registers.  All these registers need to be preserved by the
 * processor setup function (or set in the case of r0)
 *
 *  r0  = cp#15 control register
 *  r1  = machine ID
 *  r2  = atags or dtb pointer
 *  r4  = TTBR pointer (low word)
 *  r5  = TTBR pointer (high word if LPAE)
 *  r9  = processor ID
 *  r13 = *virtual* address to jump to upon completion
 */
__enable_mmu:
//arch/arm/mm/proc-v7.S:__v7_ca9mp_setup updates r0 with control coprocessor 15
//C1 contents, with the previous code initialize r1, r2, r4 and r9.
#if defined(CONFIG_ALIGNMENT_TRAP) && __LINUX_ARM_ARCH__ < 6	//TRUE
	orr	r0, r0, #CR_A
#else															//FALSE
	//arch/arm/include/asm/cp15.h:CR_A = 2 = alignement abort enable.
	bic	r0, r0, #CR_A
#endif
#ifdef CONFIG_CPU_DCACHE_DISABLE								//FALSE
	bic	r0, r0, #CR_C
#endif
#ifdef CONFIG_CPU_BPREDICT_DISABLE								//FALSE
	bic	r0, r0, #CR_Z
#endif
#ifdef CONFIG_CPU_ICACHE_DISABLE								//FALSE
	bic	r0, r0, #CR_I
#endif
#ifdef CONFIG_ARM_LPAE											//FALSE
	mcrr	p15, 0, r4, r5, c2		@ load TTBR0
#else															//TRUE
	//CONFIG_CPU_SW_DOMAIN_PAN is defined means DACR_INIT is defined (see
	//arch/arm/include/asm/domain.h). It specifies (since CONFIG_IO_36 and
	//CONFIG_CPU_USE_DOMAINS are undefined) 4 domains:
	//3 = domain vectors set to DOMAIN_CLIENT = 1 <=> access permissions are
	//checked in translation tables.
	//2 = kernel set to DOMAIN_MANAGER = 1 <=> access permissions are checked in
	//translation tables.
	//1 = user set to DOMAIN_NOACCESS = 0 <=> access generates domain fault.
	//0 = IO set to DOMAIN_CLIENT = 1 <=> access permissions are checked in
	//translation tables.
	mov	r5, #DACR_INIT	//Domain Access Control Register initialization value.
	//"MCR p15, 0, <Rt>, c3, c0, 0    ; Write Rt to DACR"
	mcr	p15, 0, r5, c3, c0, 0		@ load domain access register
	//Writes the physical address of the page table into TTBR0.
	//"MCR p15, 0, <Rd>, c2, c0, 0; Write Translation Table Base Register 0TTBR0"
	mcr	p15, 0, r4, c2, c0, 0		@ load page table pointer
#endif
	b	__turn_mmu_on
ENDPROC(__enable_mmu)

/*
 * Enable the MMU.  This completely changes the structure of the visible
 * memory space.  You will not be able to trace execution through this.
 * If you have an enquiry about this, *please* check the linux-arm-kernel
 * mailing list archives BEFORE sending another post to the list.
 *
 *  r0  = cp#15 control register
 *  r1  = machine ID
 *  r2  = atags or dtb pointer
 *  r9  = processor ID
 *  r13 = *virtual* address to jump to upon completion
 *
 * other registers depend on the function called upon completion
 */
	.align	5
	.pushsection	.idmap.text, "ax"
//section .idmap.text is compiled into the start of a separate 1MB section,
//meaning that it is possible to map this code to have the same virtual and
//physical addresses.
//
//"If the PA of the software that enables or disables an MMU differs from its
// VA, speculative instruction fetching can cause complications. ARM strongly
// recommends that the PA and VA of any software that enables or disables an MMU
// are identical if that MMU controls address translations that apply to the
// software currently being executed."
ENTRY(__turn_mmu_on)
	//NOP due to the following instruction flushing the pipeline? Or the special
	//purpose of this function and its mapping?
	//Answer:
	//"wait for coprocessor operation to finish this is the canonical way to
	// wait for cp updates"
	mov	r0, r0
	//"Instruction Synchronization Barrier flushes the pipeline in the
	// processor, so that all instructions following the ISB are fetched from
	// cache or memory, after the instruction has been completed."
	//This is done to prevent other code above with physical addresses to be
	//incorrectly mapped by the MMU?
	instr_sync
	//"MCR p15, 0, <Rt>, c1, c0, 0 ; Write Rt to SCTLR"
	//Writes the contents of the CP15 control register into SCTLR to enable the
	//MMU (bit 0 set to 1; see end of arch/arm/mm/proc-v7.S:__v7_setup
	//"orr	r0, r0, #1").
	//
	//ENABLES MMU.
	mcr	p15, 0, r0, c1, c0, 0		@ write control reg
	//"The MIDR provides identification information for the processor, including
	// an implementer code for the device and a device ID number."
	//"MRC p15, 0, <Rt>, c0, c0, 0 ; Read MIDR into Rt"
	//
	//Useless since r3 is overwritten few lines below.
	//
	//" If we insert the following instruction after the mcr, then this should
	// solve your issue.
	//
	// mrc p15, 0, r0, c1, c0 [not same as c0, c0 below.]
	//
	// Since the read-back of the same register is _guaranteed_ by the ARM
	// architecture manual to return the value that was written there (if it
	// doesn't, the CPU isn't an ARM compliant implementation), this means we
	// can guarantee that the write to the register has taken effect."
	mrc	p15, 0, r3, c0, c0, 0		@ read id reg
	//Executes all instructions in the pipeline before executing the following
	//instructions. This is done to make sure the MMU is enabled before the code
	//with virtual addresses is executed?
	instr_sync
	//NOP
	//"wait for coprocessor operation to finish this is the canonical way to
	// wait for cp updates"
	mov	r3, r3
	mov	r3, r13	//Overwrites r3. Moves virtual address __mmap_switched into r3.
	ret	r3		//Returns to the virtual address __mmap_switched.
__turn_mmu_on_end:
ENDPROC(__turn_mmu_on)
	.popsection


#ifdef CONFIG_SMP_ON_UP
	__HEAD
__fixup_smp:
	and	r3, r9, #0x000f0000	@ architecture version
	teq	r3, #0x000f0000		@ CPU ID supported?
	bne	__fixup_smp_on_up	@ no, assume UP

	bic	r3, r9, #0x00ff0000
	bic	r3, r3, #0x0000000f	@ mask 0xff00fff0
	mov	r4, #0x41000000
	orr	r4, r4, #0x0000b000
	orr	r4, r4, #0x00000020	@ val 0x4100b020
	teq	r3, r4			@ ARM 11MPCore?
	reteq	lr			@ yes, assume SMP

	mrc	p15, 0, r0, c0, c0, 5	@ read MPIDR
	and	r0, r0, #0xc0000000	@ multiprocessing extensions and
	teq	r0, #0x80000000		@ not part of a uniprocessor system?
	bne    __fixup_smp_on_up	@ no, assume UP

	@ Core indicates it is SMP. Check for Aegis SOC where a single
	@ Cortex-A9 CPU is present but SMP operations fault.
	mov	r4, #0x41000000
	orr	r4, r4, #0x0000c000
	orr	r4, r4, #0x00000090
	teq	r3, r4			@ Check for ARM Cortex-A9
	retne	lr			@ Not ARM Cortex-A9,

	@ If a future SoC *does* use 0x0 as the PERIPH_BASE, then the
	@ below address check will need to be #ifdef'd or equivalent
	@ for the Aegis platform.
	mrc	p15, 4, r0, c15, c0	@ get SCU base address
	teq	r0, #0x0		@ '0' on actual UP A9 hardware
	beq	__fixup_smp_on_up	@ So its an A9 UP
	ldr	r0, [r0, #4]		@ read SCU Config
ARM_BE8(rev	r0, r0)			@ byteswap if big endian
	and	r0, r0, #0x3		@ number of CPUs
	teq	r0, #0x0		@ is 1?
	retne	lr

__fixup_smp_on_up:
	adr_l	r4, __smpalt_begin
	adr_l	r5, __smpalt_end
	b	__do_fixup_smp_on_up
ENDPROC(__fixup_smp)

	.pushsection .data
	.align	2
	.globl	smp_on_up
smp_on_up:
	ALT_SMP(.long	1)
	ALT_UP(.long	0)
	.popsection
#endif

	.text
__do_fixup_smp_on_up:
	cmp	r4, r5
	reths	lr
	ldmia	r4, {r0, r6}
 ARM(	str	r6, [r0, r4]	)
 THUMB(	add	r0, r0, r4	)
	add	r4, r4, #8
#ifdef __ARMEB__
 THUMB(	mov	r6, r6, ror #16	)	@ Convert word order for big-endian.
#endif
 THUMB(	strh	r6, [r0], #2	)	@ For Thumb-2, store as two halfwords
 THUMB(	mov	r6, r6, lsr #16	)	@ to be robust against misaligned r0.
 THUMB(	strh	r6, [r0]	)
	b	__do_fixup_smp_on_up
ENDPROC(__do_fixup_smp_on_up)

ENTRY(fixup_smp)
	stmfd	sp!, {r4 - r6, lr}
	mov	r4, r0
	add	r5, r0, r1
	bl	__do_fixup_smp_on_up
	ldmfd	sp!, {r4 - r6, pc}
ENDPROC(fixup_smp)

#include "head-common.S"
