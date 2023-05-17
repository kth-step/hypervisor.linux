#include <linux/syscalls.h>	//For SYSCALL_DEFINE1.
#include <asm/memory.h>		//For virt_to_phys.
#include <linux/slab.h>		//For kmalloc, kfree and GFP_KERNEL.

/*
 *	Error code table:
 *	0 = No error.
 *	3 = Could not allocate kernel memory for the image.
 *	4 =	Could not copy image from user space to kernel space.
 *	5 = @number_of_signatures < 2 or @number_of_signatures > 1024.
 *	6 = Image not completely located in Linux kernel image when the update is handed to the hypervisor.
 *	7 = Certificate of update is invalid.
 *	8 = Signature of an executable block is not in the new golden image.
 */

SYSCALL_DEFINE2(cctv_photo, uint32_t *, buf, uint32_t, byte_size)
{
	long result = 0;

	//Allocates kernel memory for the update.
	uint32_t *kernel_buf = (uint32_t *) kmalloc(byte_size, GFP_KERNEL);
	if (kernel_buf == NULL)
		return 3;

	//Invokes the hypercall to take a CCTV photo and receives it in the kernel buffer.
	HYPERCALL_1(HYPERCALL_CCTV_PHOTO, virt_to_phys(kernel_buf))
	asm volatile("mov %0, r0" : "=r" (result) ::);

	//Copies the update from user space to kernel space.
	if (copy_to_user((void *) buf, (void *) kernel_buf, byte_size) != 0) {
		kfree((void *) kernel_buf);
		return 4;
	}

	//Frees kernel memory.
	kfree((void *) kernel_buf);

	return result;
}
