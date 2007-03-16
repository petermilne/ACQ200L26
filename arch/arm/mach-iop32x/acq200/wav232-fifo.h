/*
 * shared data defs specific to WAV232
 */


#define WAV232_MAXCHAN   32
#define WAV232_MAXTRACKS 8

struct WAV232_PRIVATE {
	int buffer_maxlen;
	int ntracks;
	int nchan;
	int fill_cursor;
	unsigned track_ends[WAV232_MAXTRACKS][WAV232_MAXCHAN];
	unsigned fill_state[WAV232_MAXTRACKS][WAV232_MAXCHAN];
};

#define FILL_STATE_BLOCK 0x00ff0000
#define FILL_STATE_FILL  0x80000000
#define FILL_STATE_COPY  0x40000000
#define FILL_STATE_WRITE 0x20000000
#define FILL_STATE_VALUE 0x0000ffff  /* last value */


#define WAVDEF ((struct WAV232_PRIVATE*)CAPDEF->private)

#define SET_WAVDEF(p) (CAPDEF->private = p)

