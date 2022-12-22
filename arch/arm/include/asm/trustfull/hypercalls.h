#ifndef HYPERCALLS_H_
#define HYPERCALLS_H_

#ifndef __ASSEMBLER__

#define STR(x) #x
#define HYPERCALL_NUM(n) "#"#n

#define HYPERCALL_3(num, reg0, reg1, reg2) \
		asm volatile ("mov R0, %0			\n\t"  \
					  "mov R1, %1			\n\t"	\
					  "mov R2, %2			\n\t"	\
					  "SWI " HYPERCALL_NUM((num)) "\n\t" \
					  ::"r" (reg0), "r" (reg1), "r" (reg2) : "memory", "r0", "r1", "r2" \
);

#define HYPERCALL_2(num, reg0, reg1) \
		asm volatile ("mov R0, %0 			\n\t"  	\
					  "mov R1, %1			\n\t"   \
					  "SWI " HYPERCALL_NUM((num)) "\n\t" \
					  ::"r" (reg0), "r" (reg1) : "memory", "r0", "r1" \
);

#define HYPERCALL_1(num, reg0) \
		asm volatile ("mov R0, %0 			\n\t"  	\
					  "SWI " HYPERCALL_NUM((num)) "\n\t" \
					  ::"r" (reg0) : "memory", "r0" \
);


#define HYPERCALL_0(num) \
	asm volatile ( \
		"SWI " HYPERCALL_NUM((num)) "         \n\t" \
		: : : "memory"			\
	);

/*Enums used to specify hypercall operations
 *
 * */

/*HYPERCALL_CACHE_OP
 * */
enum hyp_cache_op {
	FLUSH_ALL 			=0,
	FLUSH_D_CACHE_AREA	=1,
	CLEAN_D_CACHE_AREA	=2,
	INVAL_D_CACHE_MVA	=3,
	FLUSH_I_CACHE_ALL	=4,
	FLUSH_I_CACHE_MVA	=5,
	INVAL_ALL_BRANCH	=6,
	INVAL_TLB_ALL		=7,
	INVAL_TLB_MVA		=8,
	INVAL_TLB_ASID		=9,
	FLUSH_D_CACHE		=10,
	COHERENT_RANGE		=13,
};

/*HYPERCALL_VFP_OP
 * */

enum hyp_vfp_op {
	VFP_ENABLE 			=0,
	VFP_ID,
	VFP_MVFR,
	VFP_THREAD_FLUSH,
	VFP_EXC_READ,
	VFP_EXC_ENABLE,
	VFP_EXC_RESTORE,
	VFP_SAVE_STATE

};

enum hyp_interrupt_op {
	UNMASK =0,
	MASK   =1,
	ACK    =2
};

#endif

/*
 * Hypercalls
 */
/*Register number of system calls and exception vector. */
#define HYPERCALL_GUEST_INIT			1000

/*INTERRUPT */
#define HYPERCALL_INTERRUPT_SET     	1001
#define HYPERCALL_END_INTERRUPT     	1002
#define HYPERCALL_INTERRUPT_CTRL		1003
//#define HYPERCALL_REGISTER_HANDLER	1003

/*CACHE */
#define HYPERCALL_CACHE_OP				1004
#define HYPERCALL_TLB_OP				1005
////////
#define FLUSH_D_CACHE_AREA 1
#define INV_D_CACHE_REGION 11	//Defined as macro if it is used in assembly code.
#define CLEAN_D_CACHE_REGION 12	//Defined as macro if it is used in assembly code.
////////

/*PAGE TABLE*/
#define HYPERCALL_NEW_PGD				1006
#define HYPERCALL_FREE_PGD				1007
#define HYPERCALL_SWITCH_MM				1008
#define HYPERCALL_CREATE_SECTION    	1009
#define HYPERCALL_SET_PMD				1010
#define HYPERCALL_SET_PTE				1011

/*CONTEXT*/
#define HYPERCALL_RESTORE_LINUX_REGS 	1012
#define HYPERCALL_RESTORE_REGS			1013
#define HYPERCALL_SET_CTX_ID			1014
#define HYPERCALL_SET_TLS_ID			1015

/*RPC*/
#define HYPERCALL_RPC					1020
#define HYPERCALL_END_RPC				1021

/*VFP*/
#define HYPERCALL_VFP					1022

/*Debug*/
#define HYPERCALL_DBG                   1030

/*UPDATE MONITOR*/
#define HYPERCALL_UPDATE_MONITOR		1040
#define HYPERCALL_LINUX_INIT_END		1042

#define HYPERCALL_MAP_L2				1062
#define HYPERCALL_UPDATE_PMD_SINGLE		1073

#endif /* HYPERCALLS_H_ */
