/*
 * include/asm-arm/arch-iop32x/memory.h
 */

#ifndef __MEMORY_H
#define __MEMORY_H

#include <asm/hardware.h>

#ifdef CONFIG_ARCH_ACQ200


#define TASK_SIZE    (0x70000000)

#define TASK_UNMAPPED_BASE (0x60000000)

/*
 * we need to cover 1GB contiguous phys memory.
 * PHYS_OFFSET needs to be on a 1GB boundary, can't mask direct write
 * or pci IO areas. We lose the top 64K :-(
 * PAGE_OFFSET needs to be lower than normal to allow addtional
 * mappings at higher vaddrs
 */ 
#define PAGE_OFFSET 0x80000000 
#define PHYS_OFFSET 0xc0000000

#else
/*
 * Physical DRAM offset.
 */
#define PHYS_OFFSET	UL(0xa0000000)

#endif  /* CONFIG_ARCH_ACQ200 */

/*
 * Virtual view <-> PCI DMA view memory address translations
 * virt_to_bus: Used to translate the virtual address to an
 *		address suitable to be passed to set_dma_addr
 * bus_to_virt: Used to convert an address for DMA operations
 *		to an address that the kernel can use.
 */
#define __virt_to_bus(x)	(__virt_to_phys(x))
#define __bus_to_virt(x)	(__phys_to_virt(x))


#endif
