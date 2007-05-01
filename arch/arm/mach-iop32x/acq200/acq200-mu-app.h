/*
 * application layer for acq200-mu
 */

/*
 * read and write calls on /dev/mu_rma have the following definition
 */



struct mu_rma {
	u32 magic;
	u32 buffer_offset;
	u32 length;
	u32 status;
	u32 bb_remote_pci_offset;  /** MU_MAGIC_BB ONLY */
};

#define MU_MAGIC     0xd1acbabe
#define MU_MAGIC_BB  0xd1acbbb0      /** @@todo offset in bigbuf */
#define MU_HOSTBOUND 0x1             /* tag on to magic */
#define MU_ACQBOUND  0x0

#define MU_RMA_MAGIC(rma)         ((rma)->magic&~MU_HOSTBOUND)
#define MU_RMA_MAGIC_OK(rma)      (((rma)->magic&~MU_HOSTBOUND)==MU_MAGIC)
#define MU_RMA_IS_HOSTBOUND(rma)  (((rma)->magic&MU_HOSTBOUND)!=0)
#define MU_RMA_IS_ACQBOUND(rma)   (!MU_RMA_IS_HOSTBOUND(rma))
#define MU_STATUS_OK 0

#define MU_SAMPLE_SIZE(status, sz)    ((sz = (status&0xffff)>>16)? sz: 2)

#define MU_RMA_SZ (sizeof(struct mu_rma))


#define E_MU_BAD_STRUCT 0xd100
#define E_MU_BAD_MAGIC  0xd101
