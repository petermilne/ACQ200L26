

struct BOXCAR32 {
	int maxelems;		/* MUST BE POWER OF 2 */
	u32 *history;
	int cursor;
	u32 mask;
};

#define BOX32_INCR(b32, ii) (((ii)+1) & b32->mask)

static inline void boxcar32_init(struct BOXCAR32 *b32, int maxelems)
{
	int ibit;

	for (ibit = 0; (1<<ibit) < maxelems; ++ibit){
		;
	}
	maxelems = 1 << ibit;
	b32->history = kzalloc(sizeof(u32)*maxelems, GFP_KERNEL);
	b32->maxelems = maxelems;
	b32->cursor = 0;
	b32->mask = maxelems - 1;
}

static inline void boxcar32_free(struct BOXCAR32 *b32)
{
	kfree(b32->history);
}

static inline u32 boxcar32_process(struct BOXCAR32 *b32, u32 xx)
{
	int cursor = b32->cursor;
	int ic = b32->cursor = BOX32_INCR(b32, cursor);
	u32 sum = xx;

	b32->history[cursor] = sum = xx;

	for ( ;  ic != cursor; ic = BOX32_INCR(b32, ic)){
		sum += b32->history[ic];
	}

	return sum;				/* USER can scale */
}
