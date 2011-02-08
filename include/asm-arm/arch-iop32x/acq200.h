/*
 * linux/include/asm/arch-iop3xx/acq200.h
 *
 * D-TACQ acq200 memory map defs
 */

#ifndef _ACQ200_H_
#define _ACQ200_H_

/* Memory Map 

vaddr      paddr      baddr        descr
0x80000000 0xc0000000              physical memory
0xbfff0000 0xffff0000              
0x70000000 0x70000000 0x70000000   direct mapped pci memory
0xfe000000 0x90000000              pci IO space
0xf0000000 0xa0000000              PBI's
0xfff00000 0xffffe000              pmmrs
---------- 0xff000000 0xb0000000   MU (iop access thru PMMR )
0xffff0000 0xffff0000              vectors, mini ic etc

*/


/*
 * But on acq100, we cannot use direct mapped, must be translated
 */
#define ACQ100_PCIMEM_START  0xe0000000
#define ACQ100_PCIMEM_END    0xe8000000
#define ACQ100_PCIMEM_P      0x80000000  /* translate window 1 */

/*
 * Physical address map defs
 */
#define ACQ200_PCIMEM       0x70000000UL
#define ACQ200_RAMBASE      0xc0000000UL    /* PHYS_OFFSET  */
#define ACQ200_PARAMS       (ACQ200_RAMBASE+0x100)

#define ACQ200_MU           0xb0000000UL  /* PCI slave addr of Message Unit*/

#define ACQ200_PBI_START_P  0xa0000000
#define ACQ200_FLASH_P      0xa0000000    /* CS0 16 Flash                  */
#define ACQ200_CPLD_P       0xa8000000    /* CS1  8 CPLD Core Logic        */
#define ACQ200_FPGA_P       0xa9000000    /* CS2  8 FPGA load and deb regs */
#define ACQ200_UART_P       0xaa000000    /* CS3  8 UART                   */
#define ACQ200_LOCALIO_P    0xab000000    /* CS4 16 Internal PBI periphs   */
#define ACQ200_EXTERNIO_P   0xac000000    /* CS5 16 External PBI periphs   */
#define ACQ200_PBI_END_P    0xad000000


#define ACQ200_FLASH_LEN    (16*1024*1024)
#define ACQ200_CPLD_LEN     (256)
#define ACQ200_UART_LEN     (256)
#define ACQ200_FPGA_LEN     (0x1000000)
#define ACQ200_LOCALIO_LEN  (1024)
#define ACQ200_EXTERNIO_LEN (0x01000000)

#define ACQ100_FPGA_LEN     (0x1000000)
#define ACQ100_ETHERNET_LEN (0x100000)

/*
 * Virtual address map defs
 */

#define ACQ200_PCI_VSTAT    0xe0000000UL  /* PCI static address mapping */
#define ACQ200_PBI_START    0xf0000000UL
#define ACQ200_FLASH        0xf0000000UL
#define ACQ200_FLASH_V      (void*)0xf0000000UL
#define ACQ200_CPLD         0xf8000000UL
#define ACQ200_FPGA         0xf9000000UL
#define ACQ200_UART         0xfa000000UL
#define ACQ200_LOCALIO      0xfb000000UL
#define ACQ200_EXTERNIO     0xfc000000UL
#define ACQ200_PBI_END      0xfd000000UL

#define ACQ100_ETHERNET     0xfb000000UL
/*
 * PBI defs. NB: PBI's are set up by u-boot, exceptions ACQ100
 */
#define PBAR_FLASH      0x200
#define PBAR_RCWAIT_1   0x000
#define PBAR_RCWAIT_4   0x040
#define PBAR_RCWAIT_8   0x080
#define PBAR_RCWAIT_12  0x0C0
#define PBAR_RCWAIT_16  0x100
#define PBAR_RCWAIT_20  0x1C0
#define PBAR_ADWAIT_4   0x000
#define PBAR_ADWAIT_8   0x004
#define PBAR_ADWAIT_12  0x008
#define PBAR_ADWAIT_16  0x00C
#define PBAR_ADWAIT_20  0x01C
#define PBAR_BUS_8      0x000
#define PBAR_BUS_16     0x001
#define PBAR_BUS_32     0x002

#define PBAR_MASK       0xfff


#define ACQ200_PBI0_BA      ACQ200_FLASH_P
#define ACQ200_PBI0_WIDTH   PBAR_BUS_16
#define ACQ200_PBI0_LEN     ACQ200_FLASH_LEN

#define ACQ200_PBI1_BA      ACQ200_CPLD_P
#define ACQ200_PBI1_WIDTH   PBAR_BUS_8
#define ACQ200_PBI1_LEN     ACQ200_CPLD_LEN


#define ACQ200_PBI2_BA      ACQ200_FPGA_P
#define ACQ200_PBI2_WIDTH   PBAR_BUS_8
#define ACQ200_PBI2_LEN     ACQ200_FPGA_LEN

#define ACQ200_PBI3_BA      ACQ200_UART_P
#define ACQ200_PBI3_WIDTH   PBAR_BUS_8
#define ACQ200_PBI3_LEN     ACQ200_UART_LEN

#define ACQ200_PBI4_BA      ACQ200_LOCALIO_P
#define ACQ200_PBI4_WIDTH   PBAR_BUS_16
#define ACQ200_PBI4_LEN     ACQ200_LOCALIO_LEN

#define ACQ200_PBI5_BA      ACQ200_EXTERNIO_P
#define ACQ200_PBI5_WIDTH   PBAR_BUS_16
#define ACQ200_PBI5_LEN     ACQ200_EXTERNIO_LEN



#undef PCIBIOS_MIN_MEM
#define PCIBIOS_MIN_MEM     ACQ200_PCIMEM

#define PCIMEM_BASE         0x70000000UL       /* va of IO SWAG ??? */
#define ACQ200_PCIMEM_SIZE  0x10000000UL

/*
 * PCI interrupts
 */
#define	IRQ_ACQ200_HB	IRQ_IOP32X_XINT0
#define	IRQ_ACQ200_FP	IRQ_IOP32X_XINT1
#define	IRQ_ACQ200_E0	IRQ_IOP32X_XINT2
#define	IRQ_ACQ200_E1	IRQ_IOP32X_XINT3

/* PGMWASHERE: not used, but keeps fiq.c happy */
#define FIQ_START 64

#define ACQ200_LEDS 0xf
#define ACQ200_LED1 0x1
#define ACQ200_LED2 0x2
#define ACQ200_LED3 0x4
#define ACQ200_LED4 0x8


/*
 * MU DEFS
 */

#define ACQ200_MU_QORDER  12                    /* 4096 entries */
#define ACQ200_MU_QLEN    (1<<ACQ200_MU_QORDER) /* Entries */
#define ACQ200_MU_QSZ     (ACQ200_MU_QLEN*4)    /* Bytes   */
#define ACQ200_MU_QTOTSZ  (ACQ200_MU_QSZ*4)     /* Total size in bytes */
#define ACQ200_MU_QINC    4                     /* Q increment value, bytes */

#define ACQ200_MU_QALIGN  0x100000              /* Align on 1MB boundary */


/*
 * Tblock
 */

#define ACQ200_TBLOCK_SIZE  0x600000
#define ACQ200_TBLOCK_ALIGN 0x100000

#define AT_TBLOCK_START(addr) (((addr)&0x1fffff) == 0 && (((addr)>>20)%6) == 0)

#ifndef __ASSEMBLY__
extern void acq200_map_io(void);

struct machine_desc;
struct tag;
struct meminfo;

extern void fixup_acq200(
	struct machine_desc *desc, 
	struct tag *params,
	char **cmdline, 
	struct meminfo *mi );

extern void acq200_init_irq(void);


extern void acq100_map_io(void);
extern void acq100_init_irq(void);
extern void acq132_init_irq(void);

extern void acqX00_init(void);

/*
 * resource hook
 */
extern struct resource* acq200_pbi_resource;


#endif /* !__ASSEMBLY__ */

#endif	/* _ACQ200_H_ */



