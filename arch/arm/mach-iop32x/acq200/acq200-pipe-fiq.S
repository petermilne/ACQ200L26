/* ------------------------------------------------------------------------- */
/* acq200-pipe-fiq.S                                                         */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2003 Peter Milne, D-TACQ Solutions Ltd
 *                      <Peter dot Milne at D hyphen TACQ dot com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

#ifndef DTACQ_MACH
#error DTACQ_MACH not defined:	 please do not compile this module directly
#endif
	
@ 102 - rb_get sequence change to avoid stall / fault?
@ 103 - take care to clean up on end of shot
@ 104 - drained data - not an error
@ 105 - infact, last int after drain can avoid starve
@ 107 - change front end branch to allow easy final bp setting
@ 108 - try stmneea - just for the hell of it, also no stall/fail on reg rec.
@ 109 - split acq216, wav232 versions. acq216 now double blocks on hot=0xf
@ 110 - add nop to put (matches get)
@ 111 - fix ipc not initialised in time bug
@ 112 - drop 108. end shot does full push/pop
@ 113 add cafedebug again. Stubbed 2b (ie not 2b :-)
@ 114 - experimental modified rb_put
@ 115 - fixed bug in rb_put
@ 116 - ISR_ADDS_ENDSTOP
@ 117 - better validation of head_pa on restart
@ 118 - and shift it left valid pa 0xc0 ...0xff
@ 119 - signal completion by setting head to 0 - goes the distance, 6MHz
@ 120 - now reinstate double blocking
@ 121 - attempt to avert DMAC still busy error - read FIFCON first
@ 122 - that made no diffs, squeeze another way
@ 123 - put hot histo back in, reschedule to remove nops
@ 124 - add acq196 be VERY AFRAID
@ 125 - 124 was good - fix special case of 2b starve
@ 126 - CHECK_2B == 2 add third block
@ 127 - new ACQ196 fpga, HOTPOINT is 4 bit, trim one instr
@ 128 - oops, add that instr back
@ 129 - resume_dmac_chain - retard as far as possible to allow chain def "soak"
@ 150 - trim unwanted instructions
@ 160 - return endstop directly
@ 163 - initial CAF test fails on ACQ196. But how can it?.
@ 165 - attempt to fix 1/1000 broken chain with DWB before DMAC arm
@ 170 - re-introduce ISR_ADDS_ENDSTOP for ACQ216/16K
@ 171 - remember to Q to active
@ 172 - CAF fail (196) caused stack fail
@ 173 - DMA_DELAY - force tight loop in FIQ to allow DMA priority
@ 174 - HOT_HISTO_LSR for ACQ132 (5 bit hot fifo)
	
#define VERSION 0x00000174

/*
 * WE DONT CARE if CAF still set. 'cos first thing we do is DISABLE IT
 * and the DMAC's job is DONE or there would be no interrupt
 */
#define TEST_CAF_AT_START 0

		
/* REGS: R8_FIQ to R14_FIQ are banked.  Use them well:
 * r14_fiq .. r8_fiq - all accounted for (see below)
 *
 * shared regs - available after extreme low latency past
 *
 * r7-r4 :	 specific, see below
 * r3-r0 :	 general purpose
 */
	
/* lr_fiq :	 normal */
/* sp_fiq:	 must hold pipe fiq addr on exit */
#define dg	r12	/* r12_fiq */
#define dmac	r11	/* r11_fiq */
#define fifcon  r10	/* r10_fiq */
#define fpga	r9      /* r9_fiq  */

	
#define head_pa r8      /* r8_fiq  */
#define dmacsta head_pa	/* sp_fiq => overload */

/* shared regs - top 4 predefined, bottom 4 working regs */	
#define next	r7
#define stats	next
#define ipc	r6

#define head_va r5   
#define putbuf	r4
#define getbuf  r3   /* also used as gash reg */
#define histo   r3   /* overload histogram base pointer */

#define iblock  r2

#define STARV_DONE        0          /* DONE: not error */
#define STARV_BAD_HEAD_PA 0xd0000007
#define STARV_DMAC_CAF    0xd0000001
#define STARV_NO_ENDSTOP  0xd0000003

	
#define ACQ200_DD_FIFSTAT DMA_DESCR_PUAD
	
		.macro	DWB
		mov	r0, #0
		mcr	p15, 0, r0, c7, c10, 4	@ drain WB
		.endm
	
		.macro CPWAIT                   @ from XSCALE PRM
		mrc	p15, 0, r0, c0, c0, 0
		mov	r0, r0
@		sub	pc, pc, #4              @ how's that then?
		.endm
	
		.macro rb_incr ii
		add	\ii, \ii, #1
		bic	\ii, \ii, #RBLEN
		.endm

		.macro rb_decr ii
		sub	\ii, \ii, #1
		bic	\ii, \ii, #RBLEN
		.endm

		.macro nop rx
		orr	\rx, \rx, \rx
		.endm
		
		.macro rb_get rb, buf, t1, t2
# inputs:	\rb
# outputs:	\buf
# destroys:	\t1, \t2
# status:	N if OK
	
#define IGET	\t1
#define IPUT	\buf

	/* get next buffer, output in buf, N if found */
		ldrh	IGET, [\rb, #RB_IGET]
		ldrh	IPUT, [\rb, #RB_IPUT]
		cmp	IGET, IPUT
		beq	99f

		ldr	\buf, [\rb, #RB_BUFFERS]
#ifdef WAV232	
		nop	\t2
#endif	
		ldr	\t2,  [\rb, #RB_NGET]
#ifdef WAV232
		nop	\t1
#endif	
		add	\t2, \t2, #1
		ldr	\buf, [\buf, IGET, lsl #2]

		rb_incr	IGET
		strh	IGET, [\rb, #RB_IGET]
		str	\t2, [\rb, #RB_NGET]
		cmp	\buf, #0	
99:	
#undef IGET
#undef IPUT		
		.endm
	
		.macro RB_PUT rb, buf, t1, t2

# inputs:	\rb, \buf
# destroys:	\t1, \t2
# status:	N if OK	

#define IGET	\t1
#define IPUT 	\t2
		ldrh	IPUT, [\rb, #RB_IPUT]
		ldrh	IGET, [\rb, #RB_IGET]
		rb_decr IGET
		cmp	IPUT, IGET
		ldr	\t1, [\rb, #RB_BUFFERS]	
		beq	99f                       @ result of CMP
		nop	IPUT			  @ stall for \t1 valid
		str	\buf, [\t1, IPUT, lsl #2]
		rb_incr	IPUT
		strh	IPUT, [\rb, #RB_IPUT]
		ldr	\t1, [\rb, #RB_NPUT]
		add	\t1, \t1, #1
		str	\t1, [\rb, #RB_NPUT]
99:	
#undef IPUT
#undef IGET				
		.endm

		.macro	STARV cond code
		mov\cond r0, \code
		b\cond starve
		.endm

		.text
		.global acq200_pipe_fiq_end

		.macro HISTO tmp hsel
		add	histo, stats, \hsel
#if !defined(ACQ196)
		and	r0, r0, #ACQ200_FIFCON_MASK
#endif
		ldr	r1, [histo, r0, lsl #2]
		add	r1, r1, #1
		str	r1, [histo, r0, lsl #2]
		.endm
		
	
ENTRY(acq200_pipe_fiq)			
	/* record FIFCON, dmacsta */
	/* Start DMA */
	/* stash regs */
	/* act on bad DMAC stat */
	/* act on bad fifcon */
	/* build chain */
		/* Q head to active */
		/* optionally add block 2 */
		/* add endstop */
	/* reload DMAC */
	/* toggle DMAC */
	/* get next head */
	/* prime DMAC */
	/* cleanup and exit */

/* CRITICAL:
 * check DMAC, clear int, teeup & kick off DMA, snap FIFCON
 * finish condition:	 head_pa == 0, in this case DONT start DMAC.
 */
		ldr	fifcon, [fpga, #ACQ200_FIFCON]
		ldr	sp, [dmac, #DMA_CSR]	
		mov	sp, sp, lsl #8			@ does not interfere CCR
		str	sp, [dmac, #DMA_CCR]		@ 0 => disable CCR
		str	sp, [fpga, #ACQ200_ICR]         @ 0 => disable irq
		tst	head_pa, #0xc0000000	        @ all PA have hi bits
		strne	head_pa, [dmac, #DMA_NDAR]
		orrne	sp, sp, #IOP321_CCR_CE
		strne	sp, [dmac, #DMA_CCR]		@ 1 => enable CCR

/*
 * CRITICAL ends - now set up our "stack", gain use of other regs
 */
		movne	dmacsta, sp, lsr #8	
		add	sp, dg, #DG_ISTACK  
		stmea	sp!, {r0-r7,lr}            /* whew - now we have regs*/
				
		bne	err_chk                    	   /* data to process ... */
		STARV	eq, #STARV_BAD_HEAD_PA     /* report BAD head_pa  */

@ point not REACHED - bne, beq previous covers all cases

/*
 * put private functions here for best disassembly listing :	-)
 * functions are private and share reg defs. (cf Java nested class)
 */

rb_put:
# inputs:	putbuf, head_va
# destroys:	r0, r1
# status:	N if OK
	
		RB_PUT	putbuf, head_va, r0, r1
		mov	pc, lr

rb_get:
# inputs:	getbuf
# outputs:	next
# destroys:	r0
# status:	N if OK
	
		RB_GET	getbuf, next, r0, r1
		mov	pc, lr	

starve:
		ldr	r1, [fpga, #ACQ200_SYSCON]
		mov	head_pa, #0
		str	head_pa, [dg, #DG_HEAD]		@ signal complete
		str	r0, [dg, #DG_FIFERR]
		bic	r1, r1, #1                      @ clr DAQEN
		str	r1, [fpga, #ACQ200_SYSCON]	
		b	fiq_done			
	
/*
 * main line resumes here:
 * ERROR check - dmac still busy (unlikely), FIFERR
 * nb:	 tst produces ne for a match
 */
err_chk:
#if TEST_CAF_AT_START
		tst	dmacsta, #IOP321_CSR_CAF
		STARV   ne, #STARV_DMAC_CAF      /* OOPS:	 still busy! */
#endif
	
#if FIQDEBUG
#warning FIQDEBUG ON
@		ldr	r0,  [dg, #DG_CAFEBABE]
@		ldr	r1,  [dg, #DG_FEEDCODE]
@		ldr	dmacsta,  [dg, #DG_DEADBEEF]
#endif	

		ldr	r0, [dg, #DG_FIFERR_MASK]
		ands	r0, fifcon, r0
		STARV	ne, fifcon
/*
 * now build rest of chain - check if second block (2b) required
 */
		ldr	ipc, [dg, #DG_IPC]         /* locate the Queues */
		mov	iblock, #0			
		ldr	head_va, [dg, #DG_HEAD]
		add	putbuf, ipc, #IPC_ACTIVE
		str	fifcon, [head_va, #ACQ200_DD_FIFSTAT]
	
#if (CHECK_2B > 0)
check_2b:
		DO_CHECK_2B r0, fifcon, done_movin

add_2b:
		add	iblock, iblock, #1
		bl	rb_put			@rb_put	active, head_va, r1, r0
		
		add	getbuf, ipc, #IPC_EMPTIES
		bl	rb_get		        @ rb_get getbuf, next, r0
		beq	done_movin              @ no 2b, but still need endstop

		ldr	r0, [next, #DMA_DESCR_SW_PA]
		str	r0, [head_va, #DMA_DESCR_NDA]
		mov	head_va, next
		cmp	iblock, #CHECK_2B
		blt	add_2b
#endif /* CHECK_2B */
done_movin:
#if (ISR_ADDS_ENDSTOP == 1)
add_endstop:
		add	getbuf, ipc, #IPC_ENDSTOPS
		bl	rb_get		@ rb_get	getbuf, next, r0
		STARV   eq, #STARV_NO_ENDSTOP
	
		ldr	r0, [next, #DMA_DESCR_SW_PA]
		str	r0, [head_va, #DMA_DESCR_NDA]

dmac_chain_resume:	
		bl	rb_put			@rb_put	active, head_va, r1, r0
		ldr	r3, [dg, #DG_WO]

/*
 * replace endstop direct back to endstop Q
 */
		add	putbuf, ipc, #IPC_ENDSTOPS
		mov	head_va, next
		bl	rb_put			@rb_put active, next, r1, r0
#else
dmac_chain_resume:	
		bl	rb_put			@rb_put	active, head_va, r1, r0
		ldr	r3, [dg, #DG_WO]	
#endif		
		
do_stats_wo_next_load:	
		ldr	r0, [r3, #WO_NEXT_LOAD]
		cmp	iblock, #0
		add	stats, dg, #DG_STATS          @ !next
		add	r0, r0, #DMA_BLOCK_LEN	
		addgt	r0, r0, #DMA_BLOCK_LEN
		str	r0, [r3, #WO_NEXT_LOAD]
	
do_stats_dma_blocks:
		add	r3, stats, #STATS_DMA_BLOCKS
		ldr	r1, [r3, iblock, lsl #2]
		add	r1, r1, #1
		str	r1, [r3, iblock, lsl #2]

do_stats_histo:
#if UPDATE_COLD_HISTO
		mov	r0, fifcon, lsr #ACQ200_FIFCON_COLDP_SHIFT
		HISTO   r0, #STATS_COLD_FIFO_HISTO
#elif UPDATE_HOT_HISTO
		GET_HOT r0, fifcon
		HISTO   r0, #STATS_HOT_FIFO_HISTO
#endif
do_stats_nfints:		
		ldr	r0, [stats, #STATS_NFINTS]
		add	r0, r0, #1
		str	r0, [stats, #STATS_NFINTS]

/*	
 * THEORY:	 better to rearm DMAC LATE to allow descriptor updates to soak
 * SWAG:	 drain write buffer is a good way to do this even tho all bufs
 *               should be marked unbufferable
 */

resume_dmac_chain:
		DWB
		CPWAIT
		mov	r0, #IOP321_CCR_CR|IOP321_CCR_CE
		str	r0, [dmac, #DMA_CCR]		
						
teeup_next_call:	
		add	getbuf, ipc, #IPC_EMPTIES
		bl	rb_get			@ rb_get getbuf, next, r0
		strne	next, [dg, #DG_HEAD]
		STARV	eq, #STARV_DONE                  @ NOT an error 
		ldr	head_pa, [next, #DMA_DESCR_SW_PA]

fiq_done:
#ifdef ACQ216	
		mov	r0,#DMA_DELAY
_dma_delay:	
		subs	r0,r0,#1
		bne	_dma_delay
#endif		
		ldmea	sp, {r0-r7,lr}
		subs	pc, lr, #4

/*
 * Above must fit 0x200 bytes.
 * Below, no worries, this gets executed at original location!
 */	
		
ENTRY(acq200_pipe_fiq_end)
		.word	0xdeadbeef         @ should never get loaded


version:	
		.word	VERSION	|(0xd1<<24)|(DTACQ_MACH<<20)|(MACH_VER<<12)

ENTRY(acq200_pipe_fiq_isr_adds_endstop)
		mov	r0, #ISR_ADDS_ENDSTOP
		mov	pc, lr

ENTRY(acq200_pipe_fiq_isr_version)
		ldr	r0, version
		mov	pc, lr
		
