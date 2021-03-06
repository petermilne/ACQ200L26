/*
 * wav232.h - acq216 REGDEFS
 */


#ifndef __WAV232_H__
#define __WAV232_H__

#define ACQ200_FPGA_REG_BAR  0
#define ACQ200_FPGA_FIFO_BAR 1

#ifdef __ASSEMBLER__
#define FPGA_REG(offset)   offset
#else
#define FPGA_REG(offset) ((volatile u32*)((unsigned)ACQ200_FPGA_REGS+offset))
#endif



/*
 * REGISTERS
 */

#define ACQ200_BDR      FPGA_REG( 0x00 )
#define ACQ200_FIFCON   FPGA_REG( 0x08 )
#define WAV232_SYSCON   FPGA_REG( 0x10 )
#define WAV232_CLKCON   FPGA_REG( 0x18 )
#define ACQ200_CLKDAT   FPGA_REG( 0x20 )
#define ACQ200_DIOCON   FPGA_REG( 0x28 )
#define WAV232_DIOSFR   FPGA_REG( 0x30 )

#define ACQ200_ICR_OFFSET 0x80
#define ACQ200_ICR     FPGA_REG( ACQ200_ICR_OFFSET )


/*
 * FIELDS
 */

#define ACQ200_BDR_DEFAULT 0xdeadbeef


#define ACQ200_FIFCON_HOTPOINT  0xf0000000
#define ACQ200_FIFCON_HOTFULL   0x08000000
#define ACQ200_FIFCON_HOTEMPTY  0x04000000
#define ACQ200_FIFCON_HOTOVER   0x02000000
#define ACQ200_FIFCON_HOTUNDER  0x01000000

#define ACQ200_FIFCON_COLDPOINT 0x00f00000
#define ACQ200_FIFCON_COLDFULL  0x00080000
#define ACQ200_FIFCON_COLDEMPTY 0x00040000
#define ACQ200_FIFCON_COLDOVER  0x00020000
#define ACQ200_FIFCON_COLDUNDER 0x00010000

#define ACQ200_FIFCON_HITIDE    0x0000f000
#define ACQ200_FIFCON_LOTIDE    0x00000f00
#define ACQ200_FIFCON_HOTHITIDE 0x00000f00

#define WAV232_FIFCON_ABOVE_HT  0x00000080
#define WAV232_FIFCON_BELOW_LT  0x00000040
/* ACQ216:
#define ACQ200_FIFCON_EVENT_FLG 0x00000080
*/
/* not used                     0x00000070 */
#define ACQ200_FIFCON_HITIE     0x00000008
#define WAV232_FIFCON_LOTIE     0x00000004
/* ACQ216
#define ACQ200_FIFCON_HOTHITIE  0x00000004
*/
#define ACQ200_FIFCON_HC_RESET  0x00000002
#define ACQ200_FIFCON_HC_ENABLE 0x00000001


#define ACQ200_FIFCON_HOTP_SHIFT  28
#define ACQ200_FIFCON_COLDP_SHIFT 20
#define ACQ200_FIFCON_HIT_SHIFT   12
#define ACQ200_FIFCON_LOT_SHIFT    8

#define ACQ200_FIFCON_MASK      0xf

#define ACQ200_COLDFIFO_SZ      0x1000     /* 4K bytes */
#define ACQ200_FIFCON_COLD_HALF 0x00800000
#define ACQ200_FIFCON_COLD_HLFE 0x00700000 /* th ensures BELOW half */
#define ACQ200_FIFCON_COLD_HLFF 0x00900000 /* th ensures ABOVE half 7K fill */
#define ACQ200_FIFCON_HOT_HALF  0x80000000


#define WAV232_SYSCON_TRMD_SOFT 0x00080000
#define WAV232_SYSCON_TRSEL     0x00070000
/* not used                     0x00008000 */
#define WAV232_SYSCON_NTRACKS   0x00007000
/* not used                     0x00000c00 */
#define WAV232_SYSCON_CH_CONFIG 0x00000300
/* not used                     0x00000080 */
#define WAV232_SYSCON_LOOPBACK  0x00000040
#define WAV232_SYSCON_NDACRESET 0x00000020
#define WAV232_SYSCON_SOFTCLKMD 0x00000010
#define WAV232_SYSCON_SOFTCLKS  0x00000008
#define WAV232_SYSCON_SOFTTRG   0x00000004
#define WAV232_SYSCON_EXTCLK    0x00000002
#define WAV232_SYSCON_DAQEN     0x00000001

#define WAV232_SYSCON_TRSEL_SHL   16
#define WAV232_SYSCON_NTRACKS_SHL 12

#define WAV232_SYSCON_CH_CONFIG_32 0x00000000
#define WAV232_SYSCON_CH_CONFIG_24 0x00000100
#define WAV232_SYSCON_CH_CONFIG_16 0x00000200
#define WAV232_SYSCON_CH_CONFIG_08 0x00000300

#define WAV232_SYSCON_CH_CONFIG_XX 0x00000300



#define WAV232_CLKCON_CS_MASK   0x000000f0

#define WAV232_CLKCON_CS_DI0    0x00000080
#define WAV232_CLKCON_CS_DI1    0x00000090
#define WAV232_CLKCON_CS_DI2    0x000000a0
#define WAV232_CLKCON_CS_DI3    0x000000b0
#define WAV232_CLKCON_CS_DI4    0x000000c0
#define WAV232_CLKCON_CS_DI5    0x000000d0
#define WAV232_CLKCON_CS_80M    0x000000e0   /* NOT IMPLEMENTED */
#define WAV232_CLKCON_CS_66M    0x000000f0

#define WAV232_CLKCON_CLKMAS    0x00000008

#define WAV232_CLKCON_OCS_MASK  0x00000007

#define WAV232_CLKCON_OCS_DO0   0x00000000
#define WAV232_CLKCON_OCS_DO1   0x00000001
#define WAV232_CLKCON_OCS_DO2   0x00000002
#define WAV232_CLKCON_OCS_DO3   0x00000003
#define WAV232_CLKCON_OCS_DO4   0x00000004
#define WAV232_CLKCON_OCS_DO5   0x00000005

#define WAV232_CLKCON_OCS_SHL   0


#define ACQ200_DIOCON_SETOUT    0x00ff0000
#define ACQ200_DIOCON_OUTDAT    0x0000ff00
#define ACQ200_DIOCON_INPDAT    0x000000ff

#define ACQ200_DIOCON_SETOUT_SHL 16
#define ACQ200_DIOCON_OUTDAT_SHL 8
#define ACQ200_DIOCON_INPDAT_SHL 0




#define WAV232_DIOSFR_ET_EN     0x00002000
#define WAV232_DIOSFR_ET_STA    0x00001000
#define WAV232_DIOSFR_ET_RISING 0x00000800
#define WAV232_DIOSFR_ET_MASK   0x00000700
/* not used                     0x000000e0 */
#define WAV232_DIOSFR_EC_STA    0x00000010
#define WAV232_DIOSFR_EC_RISING 0x00000008
#define WAV232_DIOSFR_EC_MASK   0x00000007


#define WAV232_DIOSFR_ET_MASK_SHL 8
#define WAV232_DIOSFR_EC_MASK_SHL 0

#define WAV232_DIOSFR_EX_MASK   0x00000007
#define WAV232_DIOSFR_EX_DI0    0x00000000
#define WAV232_DIOSFR_EX_DI1    0x00000001
#define WAV232_DIOSFR_EX_DI2    0x00000002
#define WAV232_DIOSFR_EX_DI3    0x00000003
#define WAV232_DIOSFR_EX_DI4    0x00000004
#define WAV232_DIOSFR_EX_DI5    0x00000005

#define WAV232_DIOSFR_ET_FALLING 0x00000000


#define COLD_FIFO_SZ         8192
#define COLD_FIFO_ENTRIES    16
#define COLD_FIFO_ENTRY_SIZE (COLD_FIFO_SZ/COLD_FIFO_ENTRIES)
#define COLD_FIFO_ENT

#define HOT_FIFO_SZ              4096
#define HOT_FIFO_ENTRIES         16
#define HOT_FIFO_ENTRY_SIZE      (FIFO_SZ/FIFO_ENTRIES)

#define HOT_FIFO_SZK             (HOT_FIFO_SZ/1024)

#define COLD_FIFO_HALF(fifcon) (fifcon&ACQ200_FIFCON_COLD_HALF)
#define HOT_FIFO_HALF(fifcon)  (fifcon&ACQ200_FIFCON_HOT_HALF)
#define HOT_FULL(fifcon)       (HOT_FIFO_FULL_ENTRIES(fifcon)==15)


#endif /* wav232.h */
