/*
 * class TBLOCK - a block of data in bb
 * methods:
 */

typedef int (*tblock_copy_action)(
	struct TBLOCK *this,
	/* const */ short* u_buf, int maxbuf, 
	int channel, int offset, int stride);
int tblock_raw_extractor(
	struct TBLOCK *this,
	short* u_buf, int maxbuf, 
	int channel, int offset, int stride);
int tblock_raw_extractor32(
	struct TBLOCK *this,
	short* u_buf, int maxbuf, 
	int channel, int offset, int stride);
int tblock_cooked_extractor(
	struct TBLOCK *this,
	short* u_buf, int maxbuf, 
	int channel, int offset, int stride);
int tblock_cooked_extractor32(
	struct TBLOCK *this,
	short* u_buf, int maxbuf, 
	int channel, int offset, int stride);
int tblock_raw_filler(
	struct TBLOCK *this,
	/* const */ short* u_buf, int maxbuf, 
	int channel, int offset, int stride);
int tblock_cooked_filler(
	struct TBLOCK *this,
	/* const */ short* u_buf, int maxbuf, 
	int channel, int offset, int stride);
int tblock_lock(struct TBLOCK *this);
int tblock_unlock(struct TBLOCK *this);



void acq200_tblock_init_top(void);
void acq200_tblock_remove(void);
void acq200_init_tblock_list(void);

void acq200_phase_release_tblocks(struct Phase* phase);
void acq200_empties_release_tblocks(void);
void acq200_phase_release_tblock_entry(struct TblockListElement* tle);
int acq200_phase_tblocks_diag(struct Phase* phase, char *buf);
void acq200_phase_gather_tblocks(struct Phase* phase);
void acq200_phase_rollup_excess(struct Phase* phase);
