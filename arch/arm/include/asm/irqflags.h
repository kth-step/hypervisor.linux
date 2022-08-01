/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_ARM_IRQFLAGS_H
#define __ASM_ARM_IRQFLAGS_H

#ifdef __KERNEL__

#include <asm/ptrace.h>

#ifdef CONFIG_TRUSTFULL_HYPERVISOR
#include <asm/trustfull/hypercalls.h>

#define ARM_IRQ_MASK 0x80
#define ARM_FIQ_MASK 0x40
#endif

/*
 * CPU interrupt mask handling.
 */
#ifdef CONFIG_CPU_V7M
#define IRQMASK_REG_NAME_R "primask"
#define IRQMASK_REG_NAME_W "primask"
#define IRQMASK_I_BIT	1
#else
#define IRQMASK_REG_NAME_R "cpsr"
#define IRQMASK_REG_NAME_W "cpsr_c"
#define IRQMASK_I_BIT	PSR_I_BIT
#endif

#if __LINUX_ARM_ARCH__ >= 6

#define arch_local_irq_save arch_local_irq_save
static inline unsigned long arch_local_irq_save(void)
{
	unsigned long flags;

#ifdef CONFIG_TRUSTFULL_HYPERVISOR
	HYPERCALL_2(HYPERCALL_INTERRUPT_SET, ARM_IRQ_MASK, 3);
	asm volatile (
		"	mov %0, r0 	@arch_local_irq_save\n "
		: "=r" (flags) : : "memory", "cc");
#else
	asm volatile(
		"	mrs	%0, " IRQMASK_REG_NAME_R "	@ arch_local_irq_save\n"
		"	cpsid	i"
		: "=r" (flags) : : "memory", "cc");
#endif
	return flags;
}

#define arch_local_irq_enable arch_local_irq_enable
static inline void arch_local_irq_enable(void)
{
#ifdef CONFIG_TRUSTFULL_HYPERVISOR
	HYPERCALL_2(HYPERCALL_INTERRUPT_SET, ARM_IRQ_MASK, 1);
#else
	asm volatile(
		"	cpsie i			@ arch_local_irq_enable"
		:
		:
		: "memory", "cc");
#endif
}

#define arch_local_irq_disable arch_local_irq_disable
static inline void arch_local_irq_disable(void)
{
#ifdef CONFIG_TRUSTFULL_HYPERVISOR
	HYPERCALL_2(HYPERCALL_INTERRUPT_SET, ARM_IRQ_MASK, 0);
#else
	asm volatile(
		"	cpsid i			@ arch_local_irq_disable"
		:
		:
		: "memory", "cc");
#endif
}

#ifdef CONFIG_TRUSTFULL_HYPERVISOR
#define local_fiq_enable()  HYPERCALL_2(HYPERCALL_INTERRUPT_SET, ARM_FIQ_MASK, 1);
#define local_fiq_disable() HYPERCALL_2(HYPERCALL_INTERRUPT_SET, ARM_FIQ_MASK, 0);
#else
#define local_fiq_enable()  __asm__("cpsie f	@ __stf" : : : "memory", "cc")
#define local_fiq_disable() __asm__("cpsid f	@ __clf" : : : "memory", "cc")
#endif

#ifndef CONFIG_CPU_V7M
#define local_abt_enable()  __asm__("cpsie a	@ __sta" : : : "memory", "cc")
#define local_abt_disable() __asm__("cpsid a	@ __cla" : : : "memory", "cc")
#else
#define local_abt_enable()	do { } while (0)
#define local_abt_disable()	do { } while (0)
#endif
#else	//__LINUX_ARM_ARCH__ < 6

/*
 * Save the current interrupt enable state & disable IRQs
 */
#define arch_local_irq_save arch_local_irq_save
static inline unsigned long arch_local_irq_save(void)
{
	unsigned long flags, temp;

	asm volatile(
		"	mrs	%0, cpsr	@ arch_local_irq_save\n"
		"	orr	%1, %0, #128\n"
		"	msr	cpsr_c, %1"
		: "=r" (flags), "=r" (temp)
		:
		: "memory", "cc");
	return flags;
}

/*
 * Enable IRQs
 */
#define arch_local_irq_enable arch_local_irq_enable
static inline void arch_local_irq_enable(void)
{
	unsigned long temp;
	asm volatile(
		"	mrs	%0, cpsr	@ arch_local_irq_enable\n"
		"	bic	%0, %0, #128\n"
		"	msr	cpsr_c, %0"
		: "=r" (temp)
		:
		: "memory", "cc");
}

/*
 * Disable IRQs
 */
#define arch_local_irq_disable arch_local_irq_disable
static inline void arch_local_irq_disable(void)
{
	unsigned long temp;
	asm volatile(
		"	mrs	%0, cpsr	@ arch_local_irq_disable\n"
		"	orr	%0, %0, #128\n"
		"	msr	cpsr_c, %0"
		: "=r" (temp)
		:
		: "memory", "cc");
}

/*
 * Enable FIQs
 */
#define local_fiq_enable()					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ stf\n"		\
"	bic	%0, %0, #64\n"					\
"	msr	cpsr_c, %0"					\
	: "=r" (temp)						\
	:							\
	: "memory", "cc");					\
	})

/*
 * Disable FIQs
 */
#define local_fiq_disable()					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs	%0, cpsr		@ clf\n"		\
"	orr	%0, %0, #64\n"					\
"	msr	cpsr_c, %0"					\
	: "=r" (temp)						\
	:							\
	: "memory", "cc");					\
	})

#define local_abt_enable()	do { } while (0)
#define local_abt_disable()	do { } while (0)
#endif	//__LINUX_ARM_ARCH__

/*
 * Save the current interrupt enable state.
 */
#define arch_local_save_flags arch_local_save_flags
static inline unsigned long arch_local_save_flags(void)
{
	unsigned long flags;
#ifdef CONFIG_TRUSTFULL_HYPERVISOR
	HYPERCALL_2(HYPERCALL_INTERRUPT_SET, 0, 3);
	asm volatile(
		"	mov	%0, r0	@ local_save_flags"
		: "=r" (flags) : : "memory", "cc");
#else
	asm volatile(
		"	mrs	%0, " IRQMASK_REG_NAME_R "	@ local_save_flags"
		: "=r" (flags) : : "memory", "cc");
#endif
	return flags;
}

/*
 * restore saved IRQ & FIQ state
 */
#define arch_local_irq_restore arch_local_irq_restore
static inline void arch_local_irq_restore(unsigned long flags)
{
#ifdef CONFIG_TRUSTFULL_HYPERVISOR
	HYPERCALL_2(HYPERCALL_INTERRUPT_SET, flags, 2);
#else
	asm volatile(
		"	msr	" IRQMASK_REG_NAME_W ", %0	@ local_irq_restore"
		:
		: "r" (flags)
		: "memory", "cc");
#endif
}

#define arch_irqs_disabled_flags arch_irqs_disabled_flags
static inline int arch_irqs_disabled_flags(unsigned long flags)
{
	return flags & IRQMASK_I_BIT;
}

#include <asm-generic/irqflags.h>

#endif /* ifdef __KERNEL__ */
#endif /* ifndef __ASM_ARM_IRQFLAGS_H */
