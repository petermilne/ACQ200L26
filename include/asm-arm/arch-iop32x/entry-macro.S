/*
 * include/asm-arm/arch-iop3xx/entry-macro.S
 *
 * Low-level IRQ helper macros for IOP3xx-based platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <asm/arch/irqs.h>


                .macro  get_irqnr_preamble, base, tmp
                .endm

                .macro  arch_ret_to_user, tmp1, tmp2
                .endm

			
#if defined(CONFIG_ARCH_ACQ200)
		.macro  disable_fiq
		.endm

#define HIPRIO  (IRQ_IOP32X_DMA0_EOT|IRQ_IOP32X_DMA0_EOC|\
		 IRQ_IOP32X_DMA1_EOT|IRQ_IOP32X_DMA1_EOC   ) 
#define LOPRIO  (IRQ_IOP32X_HPI|IRQ_IOP32X_XINT2)

		.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
	
/* p6 coprocessor locked? 		
		mrc     p6, 0, \irqstat, c8, c0, 0      @ Read IINTSRC
*/
		ldr	\base, =0xfeffe7d8
		ldr	\irqstat, [\base]		@ Read IINTSRC

		ldr	\base, =HIPRIO            @ check hi
		ands	\tmp, \irqstat, \base
		bne	37f

		ldr	\base, =LOPRIO            @ check med by removing lo
		mvn	\base,\base
		ands	\tmp, \irqstat, \base
		bne	37f

		mvn	\base,\base               @ check lo
		ands	\tmp, \irqstat, \base
		bne	37f

		mov	\irqnr, #0                @ none
		b	1001f

	/* clz gives D31 highest priority, D0 lowest */
37:		
		clz     \irqnr, \tmp
		mov     \base, #31
		sub     \irqnr,\base,\irqnr

		add     \irqnr,\irqnr,#IRQ_IOP32X_DMA0_EOT
1001:
		.endm
#elif defined(CONFIG_ARCH_IOP321)
		.macro	disable_fiq
		.endm

		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
		ldr	\base, =IOP3XX_REG_ADDR(0x07D8)
		ldr	\irqstat, [\base]		@ Read IINTSRC
		cmp	\irqstat, #0
		clzne	\irqnr, \irqstat
		rsbne	\irqnr, \irqnr, #31
		.endm
#endif
