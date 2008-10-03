/*
 * acq200hml.h - hardware model layer
 */


#define HOT_FIFO_FULL_ENTRIES(fifcon) \
        (((fifcon)>>ACQ200_FIFCON_HOTP_SHIFT)&ACQ200_FIFCON_MASK)

#define COLD_FIFO_FULL_ENTRIES(fifcon) \
        (((fifcon)>>ACQ200_FIFCON_COLDP_SHIFT)&ACQ200_FIFCON_MASK)

#define HOT_FIFO_FREE_ENTRIES(fifcon) \
        (HOT_FIFO_ENTRIES-HOT_FIFO_FULL_BLOCKS(fifcon)-1)

#define COLD_FIFO_FREE_ENTRIES(fifcon) \
        (COLD_FIFO_ENTRIES-COLD_FIFO_FULL_BLOCKS(fifcon)-1)


static inline void disable_acq(void)
{
#ifdef ACQ216
	*ACQ200_SYSCON &= ~(ACQ200_SYSCON_DAQEN|ACQ200_SYSCON_LOOPBACK);
#endif
}

static inline void enable_acq(void)
{
#ifdef ACQ216
	*ACQ200_SYSCON |= ACQ200_SYSCON_DAQEN;
#endif
}

static inline void disable_fifo(void)
{
#ifdef ACQ216
	*ACQ200_FIFCON &= ~(ACQ200_FIFCON_LOTIE|
			    ACQ200_FIFCON_HITIE|
			    ACQ200_FIFCON_HC_ENABLE);
#endif
}

static inline void reset_fifo(void)
{
#ifdef ACQ216
/** @todo more ?? */
	*ACQ200_FIFCON |= ACQ200_FIFCON_HC_RESET;
	*ACQ200_FIFCON &=~ ACQ200_FIFCON_HC_RESET;
#endif
}


static inline void enable_fifo(void)
{
#ifdef ACQ216
	*ACQ200_FIFCON |= ACQ200_FIFCON_HC_ENABLE;
#endif
}

static inline void blip_fifo_reset(void)
{
#ifdef ACQ216
	u32 fifcon = *ACQ200_FIFCON;
	*ACQ200_FIFCON = fifcon &= ~ACQ200_FIFCON_HC_ENABLE;
	*ACQ200_FIFCON = fifcon |= ACQ200_FIFCON_HC_RESET;
	*ACQ200_FIFCON = fifcon &=~ ACQ200_FIFCON_HC_RESET;
	*ACQ200_FIFCON = fifcon |= ACQ200_FIFCON_HC_ENABLE;
#endif
}

static inline int hot_fifo_full_entries(void)
{
	return HOT_FIFO_FULL_ENTRIES(*ACQ200_FIFCON);
}
static inline int hot_fifo_free_entries(void)
{
	return HOT_FIFO_ENTRIES - hot_fifo_full_entries();
}

static inline int cold_fifo_full_entries(void)
{
	return COLD_FIFO_FULL_ENTRIES(*ACQ200_FIFCON);
}
static inline int cold_fifo_free_entries(void)
{
	return COLD_FIFO_ENTRIES - cold_fifo_full_entries();
}

#ifdef ACQ216
extern void acq216_stop_capture(void);

static inline void stop_capture(void) 
{
	acq216_stop_capture();
}
#endif


/*
 * HOTFIFO : 4K = 1 x 4096b DBLK
 * COLDFIFO: 8K = 1 x 4096b DBLK, 0x8>>1 = 1 DBLK
 */
#define HOT_FIFO_DBLK 1
#define COLD_FIFO_DBLK(coldpoint) ((coldpoint)>>3)

static inline int cold_fifo_dblkfree(void)
{
	return COLD_FIFO_DBLK(cold_fifo_free_entries());
}

static inline int cold_fifo_dblkfull(void)
{
	return COLD_FIFO_DBLK(cold_fifo_full_entries());
}



