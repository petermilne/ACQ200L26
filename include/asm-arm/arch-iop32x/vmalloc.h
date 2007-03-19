/*
 * include/asm-arm/arch-iop32x/vmalloc.h
 */
#ifdef CONFIG_ARCH_ACQ200
/* our memory is 1GB from 0x80000000 .. 0xc0000000 - 
 * so we can afford a lower VMALLOC_END 
 */
#define VMALLOC_END       (0xe0000000) 
#else
#define VMALLOC_END	0xfe000000
#endif
