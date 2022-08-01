/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/arch/arm/vfp/entry.S
 *
 *  Copyright (C) 2004 ARM Limited.
 *  Written by Deep Blue Solutions Limited.
 */
#include <linux/init.h>
#include <linux/linkage.h>
#include <asm/thread_info.h>
#include <asm/vfpmacros.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>

@ VFP entry point.
@
@  r0  = instruction opcode (32-bit ARM or two 16-bit Thumb)
@  r2  = PC value to resume execution after successful emulation
@  r9  = normal "successful" return address
@  r10 = this threads thread_info structure
@  lr  = unrecognised instruction return address
@  IRQs enabled.
@
ENTRY(do_vfp)
	inc_preempt_count r10, r4						//EMPTY in arch/arm/include/asm/assembler.h.
 	ldr	r4, .LCvfp									//r4 := address of .word at label .LCvpf, which contains an address of the function arch/arm/vfp/vfpmodule.c:vfp_vector().
	ldr	r11, [r10, #TI_CPU]	@ CPU number			//r11 := thread_info.cpu. arch/arm/kernel/asm-offsets.c:TI_CPU = DEFINE(TI_CPU, offsetof(struct thread_info, cpu));
	add	r10, r10, #TI_VFPSTATE	@ r10 = workspace	//r10 := &thread_info + TI_VFPSTATE_OFFSET. TI_VFPSTATE_OFFSET = DEFINE(TI_VFPSTATE, offsetof(struct thread_info, vfpstate));
	ldr	pc, [r4]		@ call VFP entry point		//Jumps to the address of the label vfp_vector, which is initialized to
													//arch/arm/vfp/entry.S:vfp_null_entry below, but set to
													//arch/arm/vfp/vfphw.S:vfp_support_entry by core_initcall in
													//arch/arm/vfp/vfpmodule.c:vfp_init.
ENDPROC(do_vfp)

ENTRY(vfp_null_entry)
	dec_preempt_count_ti r10, r4				//EMPTY in arch/arm/include/asm/assembler.h.
	//ret is macro in arch/arm/include/asm/assembler.h. with lr being used as
	//return address. Executes __v7_ca9mp_setup, which then returns to the next
	//instruction
	ret	lr
ENDPROC(vfp_null_entry)

	.align	2
.LCvfp:
	.word	vfp_vector
