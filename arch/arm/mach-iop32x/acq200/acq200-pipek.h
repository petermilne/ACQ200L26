/*
 * acq200-pipek.h semi autom generated file : check it syncs with kernel
 *
 * cat /sys/bus/pci/drivers/acq200_fpga/asm_consts
 * 
 */


/* acq200-fifo.h $Revision: 1.23 $ 20080417 */
#define CONSTS_VERSION       5
#define FIQDEBUG             0
#define RBLEN                16384
#define DG_CAFEBABE            40 /* 0x28 */
#define DG_FEEDCODE           136 /* 0x88 */
#define DG_DEADBEEF            68 /* 0x44 */
#define RB_IPUT                 0 /* 0x00 */
#define RB_IGET                 2 /* 0x02 */
#define RB_NPUT                 4 /* 0x04 */
#define RB_NGET                 8 /* 0x08 */
#define RB_BUFFERS             12 /* 0x0c */
#define IPC_EMPTIES             0 /* 0x00 */
#define IPC_ACTIVE             24 /* 0x18 */
#define IPC_ENDSTOPS           48 /* 0x30 */
#define STATS_NFINTS            0 /* 0x00 */
#define STATS_DMA_BLOCKS        4 /* 0x04 */
#define STATS_COLD_FIFO_HISTO   68 /* 0x44 */
#define STATS_HOT_FIFO_HISTO  132 /* 0x84 */
#define STATS_STARVE_FIFCON   388 /* 0x184 */
#define DG_STATS              192 /* 0xc0 */
#define DG_IPC                 44 /* 0x2c */
#define DG_WO                  48 /* 0x30 */
#define DG_FPGA_VA            164 /* 0xa4 */
#define DG_ISTACK              72 /* 0x48 */
#define DG_FIFERR              56 /* 0x38 */
#define DG_FIFERR_MASK         52 /* 0x34 */
#define DG_HEAD                60 /* 0x3c */
#define WO_NEXT_LOAD           28 /* 0x1c */
