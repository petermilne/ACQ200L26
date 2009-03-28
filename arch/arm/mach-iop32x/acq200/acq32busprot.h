/*****************************************************************************
 *
 * File: acq32busprot.h
 *
 * $RCSfile: acq32busprot.h,v $
 * 
 * Copyright (C) 1999 D-TACQ Solutions Ltd
 * not to be used without owner's permission
 *
 * Description: defines ACQ32 bus command protocol
 *
<<<<<<< acq32busprot.h
 * $Id: acq32busprot.h,v 1.85.2.28 2009/03/27 21:24:21 pgm Exp $
=======
 * $Id: acq32busprot.h,v 1.85.2.28 2009/03/27 21:24:21 pgm Exp $
>>>>>>> 1.85.2.1
 * $Log: acq32busprot.h,v $
 * Revision 1.85.2.28  2009/03/27 21:24:21  pgm
 * SYNC2V_AO32
 *
 * Revision 1.85.2.25  2007/04/18 14:54:45  pgm
 * *** empty log message ***
 *
 * Revision 1.85.2.24  2007/02/16 16:31:55  pgm
 * *** empty log message ***
 *
 * Revision 1.85.2.23  2006/02/17 11:59:45  pgm
 * *** empty log message ***
 *
 * Revision 1.85.2.22  2006/01/26 19:05:36  pgm
 * *** empty log message ***
 *
 * Revision 1.85.2.20  2005/12/31 11:14:53  pgm
 * SYNC2V
 *
 * Revision 1.85.2.19  2005/12/06 20:29:39  pgm
 * LLC200_INIT_CHANNEL_MASK_ANTIPHASE
 *
 * Revision 1.85.2.18  2005/12/01 15:39:35  pgm
 * LLC200_INIT_MASK_DDS_QDAC
 *
 * Revision 1.85.2.17  2005/11/30 11:13:55  pgm
 * *** empty log message ***
 *
 * Revision 1.85.2.16  2005/11/30 10:51:02  pgm
 * *** empty log message ***
 *
 * Revision 1.85.2.15  2005/10/20 19:37:28  pgm
 * reinstate ull
 *
 * Revision 1.85.2.14  2005/10/20 19:30:33  pgm
 * LLC200_INIT_TRIG consts
 *
 * Revision 1.4  2005/10/03 12:54:51  pgm
 * LLC200_INIT_TRIG_EXTRIG
 *
 * Revision 1.3  2005/10/01 19:27:36  pgm
 * ACQ200LLC
 *
 * Revision 1.85.2.12  2005/10/01 17:02:44  pgm
 * LLC200_INIT
 *
 * Revision 1.85.2.11  2005/08/11 10:01:53  pgm
 * *** empty log message ***
 *
 * Revision 1.85.2.10  2005/07/30 16:16:59  pgm
 * LLC_V2 additions
 *
 * Revision 1.85.2.9  2005/07/26 13:02:08  pgm
 * *** empty log message ***
 *
 * Revision 1.85.2.8  2004/11/14 09:35:40  pgm
 * handles SPCLID for rsh
 *
 * Revision 1.85.2.7  2004/06/12 16:57:20  pgm
 * MFX defines
 *
 * Revision 1.85.2.6  2003/12/12 14:17:58  pgm
 * *** empty log message ***
 *
 * Revision 1.85.2.5  2003/12/02 15:22:13  pgm
 * HRD_WAVETRACK
 *
 * Revision 1.85.2.4  2003/11/11 09:00:21  pgm
 * align pre- post- doc with reality
 *
 * Revision 1.85.2.3  2003/10/30 14:13:27  pgm
 * *** empty log message ***
 *
 * Revision 1.85.2.2  2003/09/08 21:12:06  pgm
 * *** empty log message ***
 *
<<<<<<< acq32busprot.h
 * Revision 1.84  2003/06/02 11:50:20  pgm
 * HTM V2
 *
 * Revision 1.84  2003/06/02 11:40:53  pgm
 * HTM V2
=======
 * Revision 1.85.2.1  2003/09/07 16:59:45  pgm
 * new host data exchange opts
 *
 * Revision 1.85  2003/07/03 11:28:46  pgm
 * LLC_DAC
>>>>>>> 1.85.2.1
 *
 * Revision 1.83  2002/08/29 18:52:45  pgm
 * boost opt
 *
 * Revision 1.82  2002/08/23 18:29:42  pgm
 * *** empty log message ***
 *
 * Revision 1.81  2002/07/19 18:40:29  pgm
 * adjust hrdr
 *
 * Revision 1.80  2002/06/11 14:52:20  pgm
 * ditto
 *
 * Revision 1.79  2002/06/10 20:04:17  pgm
 * MESSAGE gets more structure, less casts
 *
 * Revision 1.77  2002/04/10 20:20:44  pgm
 * htstream correct
 *
 * Revision 1.76  2002/03/28 15:27:05  pgm
 * add debounce opts
 *
 * Revision1.75  2002/03/25 08:58:12  pgm
 * cycle field in status reg
 *
 * Revision 1.74  2002/03/21 11:55:44  pgm
 * untabify
 *
 * Revision 1.73  2002/03/14 12:03:48  pgm
 * compiled on benbecula
 *
 * Revision 1.72  2002/03/12 15:40:46  pgm
 * emacs format rools OK
 *
 * Revision 1.71  2002/02/10 17:59:42  pgm
 * event waiting, FAST_MULTI opt
 *
 * Revision 1.70  2001/11/23 19:18:34  pgm
 * keep record lengths the same
 *
 * Revision 1.68  2001/11/05 16:04:45  pgm
 * text debug opt added
 *
 * Revision 1.67  2001/10/27 21:38:46  pgm
 * rt timing
 *
 * Revision 1.66  2001/10/20 13:44:12  pgm
 * set ext clock
 *
 * Revision 1.65  2001/10/06 13:07:53  pgm
 * DEBUG_GET_REGS
 *
 * Revision 1.64  2001/10/01 21:27:17  pgm
 * DTACQ coding of header.id to avoid i2o clash
 *
 * Revision 1.63  2001/10/01 09:43:34  pgm
 * ID_DTACQ on incoming messages (no i2o clash)
 *
 * Revision 1.62  2001/09/02 19:02:27  pgm
 * DATA_TRIGGER_MULTI
 *
 * Revision 1.61  2001/09/02 19:02:30  pgm
 * available channels, volt range queries
 *
 * Revision 1.60  2001/08/18 18:06:55  pgm
 * gut reset, ext debug
 *
 * Revision 1.57  2001/06/25 10:28:49  pgm
 * add get status defs
 *
 * Revision 1.56  2001/06/11 09:03:20  pgm
 * setClock, clockNow hooked up
 *
 * Revision 1.55  2001/06/02 08:43:29  pgm
 * BP_FC_RESERVE_AO
 *
 * Revision 1.54  2001/06/01 20:10:59  pgm
 * add GUT
 *
 * Revision 1.53  2001/05/25 14:41:30  pgm
 * improved comments
 *
 * Revision 1.52  2001/05/25 10:19:10  pgm
 * fix LLC_CSR_S_IS_ARMED
 *
 * Revision 1.51  2001/05/20 17:26:19  pgm
 * added LLC_CSR_READY
 *
 * Revision 1.50  2001/05/19 11:41:50  pgm
 * LLC entry prams added
 *
 * Revision 1.49  2001/05/19 10:30:04  pgm
 * low latency compiles
 *
 * Revision 1.48  2001/05/19 07:04:53  pgm
 * LLC defs done
 *
 * Revision 1.47  2001/05/15 18:12:58  pgm
 * BP_FC_SET_XO_FUNCTION
 *
 * Revision 1.46  2001/05/15 17:53:56  pgm
 * *** empty log message ***
 *
 * Revision 1.45  2001/05/13 11:47:05  pgm
 * BP_FC_SET_XO options
 *
 * Revision 1.44  2001/05/13 10:54:39  pgm
 * BP_FC_SET_XO options
 *
 * Revision 1.43  2001/05/07 07:54:14  pgm
 * SET DO, message buffer defined
 *
 * Revision 1.42  2001/05/05 06:33:10  pgm
 * magic number removal - this works
 *
 * Revision 1.41  2001/04/14 20:37:51  pgm
 * CPCI routing coded, needs testing
 *
 * Revision 1.40  2001/04/01 17:18:55  pgm
 * TEST mode data encode
 *
 * Revision 1.39  2001/03/29 05:43:20  pgm
 * field name changes
 *
 * Revision 1.38  2001/03/25 18:58:11  pgm
 * MT_COMMAND stuff now compiles
 *
 * Revision 1.37  2001/03/25 18:24:59  pgm
 * MAKE_MT_COMMAND and friends - i2o packet commands
 *
 * Revision 1.36  2001/03/17 12:50:26  pgm
 * builds in restructured tree
 *
 * Revision 1.35  2001/03/10 10:32:01  pgm
 * nodos
 *
 * Revision 1.34  2001/02/16 19:26:08  pgm
 * LowLatency defs added
 *
 * Revision 1.33  2000/10/22 19:21:07  pgm
 * GET_INTCLOCK
 *
 * Revision 1.32  2000/10/07 15:59:42  pgm
 * TC mode
 *
 * Revision 1.31  2000/10/07 15:48:04  pgm
 * SET_DIST, SET_ROUTE commands
 *
 * Revision 1.30  2000/10/01 14:38:29  pgm
 * bigdump, firmware rev, return messazge types
 *
 * Revision 1.29  2000/10/01 14:30:13  pgm
 * bigdump, firmware rev
 *
 * Revision 1.28  2000/09/26 19:59:34  pgm
 * split SET_MODE_GC
 *
 * Revision 1.27  2000/09/26 19:46:52  pgm
 * GATED_CONTINUOUS pre/post, lseek, spin
 *
 * Revision 1.26  2000/09/18 13:11:06  pgm
 * MTP_STREAM_SLEN in bytes
 *
 * Revision 1.25  2000/08/13 08:22:05  pgm
 * add message def
 *
 * Revision 1.24  2000/06/25 06:17:27  pgm
 * including tag def
 *
 * Revision 1.23  2000/06/24 15:13:02  pgm
 * BP_FC_SET_HOST_DMABUF
 *
 * Revision 1.22  2000/06/08 21:53:20  pgm
 * compiles
 *
 * Revision 1.21  2000/06/05 18:30:11  pgm
 * BP_FC_SC_POT
 *
 * Revision 1.20  2000/06/04 18:41:00  pgm
 * Signal Conditioning protocol implemented
 *
 * Revision 1.19  2000/01/08 21:14:57  pgm
 * BP_INT_STATUS_CHANGE
 *
 * Revision 1.18  2000/01/08 21:07:20  pgm
 * GET MASK query added
 *
 * Revision 1.17  2000/01/04 19:06:19  pgm
 * interrupt enable command
 *
 * Revision 1.16  1999/11/16 10:56:13  pgm
 * pretrigger flag added
 *
 * Revision 1.15  1999/11/08 17:30:36  pgm
 * BS_1K is enough for fetch
 *
 * Revision 1.14  1999/11/06 21:18:08  pgm
 * DACPOT adjusts updated from uist
 *
 * Revision 1.11  1999/11/01 09:45:59  pgm
 * modify Fetch protocol
 *
 * Revision 1.10  1999/11/01 09:15:36  pgm
 * prot updated from ARM
 *
 * Revision 1.2  1999/10/30 21:00:37  pgm
 * pgm backed up
 *
 * Revision 1.9  1999/10/28 21:24:38  pgm
 * define mask/interleave mech
 *
 * Revision 1.8  1999/10/28 21:19:19  pgm
 * data fetch/transfer mech
 *
 * Revision 1.7  1999/10/28 13:40:16  pgm
 * add revision
 *
 * Revision 1.6  1999/10/28 13:38:34  pgm
 * add revision
 *
 * Revision 1.5  1999/10/28 13:36:39  pgm
 * assign bits in priority order
 *
 * Revision 1.4  1999/10/28 08:10:36  pgm
 * BP_CI_COMMAND bit allows slave to cancel command
 *
 * Revision 1.3  1999/10/28 08:05:51  pgm
 * BP_CI_COMMAND bit allows slave to cancel command
 *
F
F
F
F
 * Revision 1.1  1999/10/22 16:26:49  pgm
 * first entry to cvs
 *
 *
\*****************************************************************************/

/** @file acq32busprot.h pci bus command protocol definition.

Applies to all D-TACQ pci/CompactPCI products

- ACQ32 is controlled via commands issued thru the PCI mailboxes

 - MAILBOX_[0,1,2] are used for master slave commands and responses
 - MAILBOX_3       is used for unsolicited ACQ32 status output


- Master slave commands have the following format

The Host PC is the Master, ACQ32 is the Slave

 - COMMAND <31:0> is MAILBOX_0
 - A3      <31:0> is MAILBOX_1     ; optional 32 bit parameter
 - A4      <31:0> is MAILBOX_2     ; optional 32 bit parameter

 - STATUS  <31:0> is MAILBOX_3

 - COMMAND <31:0> : breaks up into
  -    Immediate <31:24>:      
   -    CI_ACK     31           cleared by M, set by S on completion
   -    CI_DONE    30           set by M when a commmand is complete
                                may be cleared by S to indicate fault
   -    CI_A3      28           set by M to indicate A3 valid
   -    CI_A4      27           set by M to indicate A4 valid

    -   CI_COMMAND 26           set by M to indicate COMMAND
    -   CI_QUERY   25           set by M to indicate QUERY

                                can be cleared by S to validate COMMAND
    - FunCode <23:16>:            function code
    - A1      <15:8>:             optional 8 bit parameter
    - A2      <8:0>:              optional 8 bit parameter


 - STATUS <31:0> : breaks up into
   - ST_MODE <31:24> : current mode
   - ST_STATE <23:16> : current state
   - CYCLE <15:12> : cycle (shot number).
   - HEARTBEAT <11:0> : number increments indicating card is alive.

     
*/

#ifndef _ACQ32BUSPROT_H_
#define _ACQ32BUSPROT_H_



#define BP_REV            "$Revision: 1.85.2.28 $"

#define BP_MB_COMMAND     0
#define BP_MB_A3          1
#define BP_MB_A4          2
#define BP_MB_STATUS      3


#define BP_CI_ACK_BIT     31     // 0x80000000  
#define BP_CI_DONE_BIT    30     // 0x40000000
#define BP_CI_COMMAND_BIT 29     // 0x20000000
#define BP_CI_QUERY_BIT   28     // 0x10000000

#define BP_CI_A3_BIT      27     // 0x080000000
#define BP_CI_A4_BIT      26     // 0x04000000

/*
 * list the funcodes as an enum to reduce chance of overload
 */
enum FUNCODES {
    FCA = 'A',
    FCB = 'B',
    FCC = 'C',
    FCD = 'D',
    FCE = 'E',
    FCF = 'F',
    FCG = 'G',
    FCH = 'H',
    FCI = 'I',

    FCK = 'K',
    FCL = 'L',
    FCM = 'M',
    FCN = 'N',
    FCP = 'P',
    FCQ = 'Q',
    FCR = 'R',
    FCS = 'S',
    FCT = 'T',
    FCU = 'U',
    FCV = 'V',
    FCW = 'W',
    FCX = 'X',
    FCY = 'Y',
    FCZ = 'Z',

    fca = 'a',
    fcb = 'b',
    fcc = 'c',
    fcd = 'd',
    fce = 'e',

    fcf = 'f',
    fch = 'h',    
    fci = 'i',
    fck = 'k',
    fcl = 'l',

    fcm = 'm',
    fcn = 'n',
    
    fcp = 'p',

    fcr = 'r',
    fcs = 's',
    fct = 't',
    fcu = 'u',
    fcw = 'w',
    fcy = 'y',  
    fcz = 'z', 
    
    fc1 = '1',
    fc2 = '2',
    fc3 = '3',
    fc4 = '4', 
    fc5 = '5',
    fc6 = '6',
    
    fc80 = '\x80',
    keep_arm_sdt_happy = 0xdead
};
  

enum ACQXX_FLAVOR {
    ACQXX_FLAVOR32 = 32,
    ACQXX_FLAVOR16 = 16,
    
    ACQXX_FLAVOR_NOT_FOUND
};

    
#define MASK( bit )              (1U<<(bit))

#define BP_CI_ACK               MASK(BP_CI_ACK_BIT)
#define BP_CI_DONE              MASK(BP_CI_DONE_BIT)
#define BP_CI_A3                MASK(BP_CI_A3_BIT)
#define BP_CI_A4                MASK(BP_CI_A4_BIT)
#define BP_CI_QUERY             MASK(BP_CI_QUERY_BIT)
#define BP_CI_COMMAND           MASK(BP_CI_COMMAND_BIT)

#define BP_GET_FUNCODE( command )   (((command)>>16)&0x0ff)
#define BP_SET_FUNCODE( funcode )   ((funcode)<<16)

#define BP_GET_A1( command )        (((command)>>8)&0x0ff)
#define BP_SET_A1( a1 )             ((a1)<<8)

#define BP_GET_A2( command )        ((command)&0x0ff)
#define BP_SET_A2( a2 )             (a2)

#define BP_GET_MODE( status )       (((status)>>24)&0x07f)
#define BP_SET_MODE( mode )         ((mode)<<24)

#define BP_GET_ERROR( status )      ((status&0x80000000)!=0)
#define BP_SET_ERROR( status )      ((status)<<31)

#define BP_GET_STATE( status )      (((status)>>16)&0x0ff)
#define BP_SET_STATE( state )       ((state)<<16)

/*
 * CYCLE is bumped on every setArm
 */
#define BP_GET_CYCLE( status )      (((status)>>12)&0xf)
#define BP_SET_CYCLE( status )      ((status)<<12)

#define BP_GET_HEARTBEAT( status )  ((status)&0x0fff)
#define BP_SET_HEARTBEAT( status )  ((status)&0x0fff)

/*
 * valid function codes
 */

#define BP_FC_SET_DEBUG    fcb        // debug level in A1

#define BP_FC_SET_INTERRUPTS FCI    // 0 or 1 in A1

enum ACQ32_DIOSELECT {
    ACQ32_DIO_NONE = 0,
    ACQ32_DI_0 = MASK(0),
    ACQ32_DI_1 = MASK(1),
    ACQ32_DI_2 = MASK(2),
    ACQ32_DI_3 = MASK(3),
    ACQ32_DI_4 = MASK(4),
    ACQ32_DI_5 = MASK(5),
    ACQ32_DO_0 = MASK(0),
    ACQ32_DO_1 = MASK(1),
    ACQ32_DO_2 = MASK(2),
    ACQ32_DO_3 = MASK(3),
    ACQ32_DO_4 = MASK(4),
    ACQ32_DO_5 = MASK(5)
};

#define BP_FC_SET_INTCLOCK   fci    // A3 = freq in Hz 
                                    // [A2{ACQ32_DIOSELECT}==MASTER] output
#define BP_FC_GET_INTCLOCK       fci    // returns A3= freq in Hz, A2 master line

#define BP_FC_SET_EXTCLOCK   fce    // A1 {ACQ32_DIOSELECT} == incoming, 
                                    // [A2{ACQ32_DIOSELECT} == outgoing A3 is div ]
#define BP_FC_GET_EXTCLOCK   fce    // returns A1, A2 state
                                    
#define BP_FC_SET_TAGGING    FCT    // 0 or 1 in A1

#define BP_FC_SET_HOST_DMABUF FCB       // A3 = pci addr, A4 length in bytes

#define BP_FC_SET_SPIN        FCY    // spin to allow flash programming

#define BP_FC_BIG_DUMP  FCU // A3 = host pci addr, A4 length in bytes

#define BP_FC_GET_FWREV FCR         // firmware rev returned via I2O
#define BP_FC_GET_CONFIG FCV        // lca rev, id returned via I2O
#define BP_FC_GET_CALINFO FCW       // cal date returned via I2O
#define BP_FC_GET_CAPTURESTATS FCE  // get capture info A1 = phase
#define BP_FC_DEBUG_GET_REGS FCQ    // return regs page via I2O

#define BP_FC_GET_NUMSAMPLES FCN // returns Numsamples in A3



#define BP_FC_SET_ROUTE fcr
/*
 * additional pram for BP_FC_SET_ROUTE defines mem interleave in A1
 * RAW - physical order
 * ROW - no deinterleave, logical order
 * CHAN - sort by channel
 */
#define BP_FC_SET_MASK_RAW       'r'
#define BP_FC_SET_MASK_ROW       'R'
#define BP_FC_SET_MASK_CHAN      'C'


#define BP_FC_SET_CALDACS FCC    // data for 2 channels in A3, A4, repeat*16

#define BP_FC_SET_DACPOT  FCP    // set dac pot
#define BP_FC_SET_DACPOT_UP     BP_SET_A2( 'u' )
#define BP_FC_SET_DACPOT_DN     BP_SET_A2( 'd' )


#define BP_FC_SET_MODE    FCM    // mode in A1, n-transient A3
#define BP_FC_SET_MODE_GC FCG    // pre in A4, post in A3, AItrig DIX A2
#define BP_FC_SET_MODE_TC FCH    // pre in A4, post in A3 TRIGGER activation

#define BP_FC_SM_FLAGS_DEB 0x80  // debounce trigger, OR into AItrig DIX
#define BP_FC_SM_FLAGS_NEM 0x40  // turn off embedded trigger bit

#define BP_FC_SET_ARM     FCA    
#define BP_FC_SET_ABORT   FCZ
#define BP_FC_SET_MASK    FCK    // 32 bit mask in A3, [boost in A4]
#define BP_FC_GET_MASK    FCK    // 32 bit mask in A3

/* configuration, setup questions */

#define BPFC_GET_MODEL     fck    // model in A1, subtype in A2
#define BPFC_GET_NCHANNELS fcn    // #input channels in A1, outputs in A2
#define BPFC_GET_RANGE     fcm    // input range in A1, output range in A2

enum BPFC_RANGE {
    BPFC_RANGE_UNKNOWN = 0,
    BPFC_RANGE_10_10 = 1,        // +/- 10V
    BPFC_RANGE_05_05,       
    BPFC_RANGE_02_02,
    BPFC_RANGE_01_01,
    BPFC_RANGE_2p5_2p5
};


/* 
 * AO, DO Waveform output
 */
 
#define BP_FC_RESERVE_AO  fca    // samples to reserve in A3
#define BP_FC_QUERY_AO    fca    // returns reserved samples in A3

#define BP_FC_SET_DACS    FCD    // data for 2 channels in A3, flags A2
#define BP_FC_SET_DO      fcf    // data in A3, flags A0

    /*
     * flags are:
     */

    /* 
     * A1 == 'F' means "function" else immediate
     */
#define BP_FC_SET_XO_FUNCTION 'F'
 
#define BP_FC_SET_XO_CLEAR 0x80    // clear list, then add datum
#define BP_FC_SET_XO_END   0x40    // end of list (opt)
#define BP_FC_SET_XO_DATA  0x02    // data in A3
#define BP_FC_SET_XO_CYCLE 0x01    // list is cyclic.


/* 
 * Signal Conditioning SC1
 */
#define BP_FC_SC_GET_DIO     fcd    // 32 bit mask in A3
#define BP_FC_SC_SET_DIO     fcd    // 32 I/O mask in A3 + O values in A4

#define BP_FC_SC_SET_CHANNEL fcc    // 32 bit functional code in A3
/*
 * A3 arg def for BP_FC_SC_SET_CHANNEL
 *
 */
#define BP_FC_SC_SET_CHANNEL_CH_LSB     24
#define BP_FC_SC_SET_CHANNEL_G1_LSB     16
#define BP_FC_SC_SET_CHANNEL_G2_LSB      8
#define BP_FC_SC_SET_CHANNEL_V_EX_LSB    0

#define BYTE_ENCODE_BIT( lsb, val )  ((val)<<(lsb))
#define BYTE_DECODE_BIT( lsb, val )  (((val)>>(lsb))&0xff)




#define BP_FC_SC_POT         fcp    // {board}{volts} in A1
/*
 * A1 arg def for BP_FC_SC_POT
 */

#define BP_FC_SC_POT_BOARD0     0x00
#define BP_FC_SC_POT_BOARD1     0x80

#define BP_FC_SC_POT_1V         0x00
#define BP_FC_SC_POT_2V         0x04
#define BP_FC_SC_POT_5V         0x08
#define BP_FC_SC_POT_10V        0x0c





/*
 * set the data distributor - post processing options
 */
#define BP_FC_SET_DISTRIBUTOR fcs  // A1==0 normal, other magics sel special rt
#define BP_FC_SET_DIST_NORMAL    0
#define BP_FC_SET_DIST_ID        1
#define BP_FC_SET_DIST_ID_ALL    2
#define BP_FC_SET_DATA_TRIGGER               0x0d
#define BP_FC_SET_DATA_TRIGGER_MULTI         0x0e
#define BP_FC_SET_DATA_EDGE_TRIGGER          0x1d
#define BP_FC_SET_DATA_EDGE_TRIGGER_MULTI    0x1e
#define BP_FC_SET_DATA_FAST_TRIGGER_MULTI    0x1f


/*
 * calibration - use of multiple calsets
 */
#define BP_FC_SELECT_CALSET    fcl    // calset {0-7} in A1


/*
 * trigger compensation
 *
 * BP_FC_GET_ATRIGGER ask the board if it declared an ATRIGGER,
 * and if it did, to report the channel and offset
 * The tell all the boards to adjust by offset using BP_FC_ADJUST_TRIGGER
 */
 
#define BP_FC_GET_ATRIGGER      fc80 // returns adj in A1, ch in A2 {1..32}
#define BP_FC_ADJUST_TRIGGER    fc80 // adjust (signed) in A1

/*
 * GUT - it's easier to use mbox than i2o after all, so
 */
enum PHASE_PROPERTY {
    PP_REQUESTED_SAMPLES = 1,
    PP_ACTUAL_SAMPLES,
    PP_STATE
};

enum EVENT_CONDITION {
    EC_NONE,            
    EC_TRUE,
    EC_SOFT,
    EC_TRIGGER_RISING,          // <d>
    EC_TRIGGER_FALLING,         // <d>
    EC_DATA_ABOVE,              // <channel> <value>
    EC_DATA_BELOW              // <channel> <value>
};

#define BP_GUT_FC_FX_AI    0x80
#define BP_GUT_FC_FX_AO    0x40
#define BP_GUT_FC_FX_DO    0x20
#define BP_GUT_FC_EV       0x10   // else phase
#define BP_GUT_FC_EPMASK   0x0f   // 1..15 events/phases possible

#define BP_GUT_SETPHASE    fc1    // FX/PH in A2, NSAMPLES in A1
#define BP_GUT_GETPHASE    fc1    // FX/PH in A2, mode in A1
#define BP_GUT_SETEVENT    fc2    // FX/EV in A2, descr in A1,A3
#define BP_GUT_GETEVENT    fc2    // FX/EV in A2, reply in A1,A3
#define BP_GUT_FIREEVENT   fc3    // FX/EV in A2, 

enum CLOCK_SOURCE {
    CS_DI0, CS_DI1, CS_DI2, CS_DI3, CS_DI4, CS_DI5, 
    CS_SOFT_CLOCK, CS_INT_CLOCK
};
#define BP_GUT_SETCLOCK    fc4    // FX in A2,    source in A1
#define BP_GUT_CLOCKNOW    fc5    // fire soft clock
#define BP_GUT_RESET       fc6    // clear all GUT state

/*
 * Data dependent triggering
 */
 
#define BP_FC_SET_DATA_THRESHOLD        fct    
/*
 * A1 = channel 00 .. 1f, 'X'
 * A2 = FX/EV
 * A3 = trigger_below
 * A4 = trigger_above
 */


#define BP_FC_USER    fcu        // flags in A3
#define BP_FC_USER_LED3_ON    0x1
#define BP_FC_USER_LED3_OFF   0x2
#define BP_FC_USER_LED3_FLIP  0x3
#define BP_FC_USER_LED4_ON    0x4
#define BP_FC_USER_LED4_OFF   0x8
#define BP_FC_USER_LED4_FLIP  0xc


/*
 * Event waiting: mask in in A3
 */
 
#define BP_FC_WAIT_EVENT    fcw

/*
 * Getting Data:
 * Fetch - place in pci window
 * Transfer - dma to pc
 * Args:
 *   A1 - start bs, length bs
 *   A2 - channel {0..31} 0x8x == all
 *      - if (all) the 0x7x is decimation factor
 *   A3 - start samples K, length Samples
 *   A4 - host memory base address (Transfer)
 * Outputs: (fetch)
 *   A3 - target base base address
 *   A4 - number of samples
 *  
 */

#define BP_FC_FETCH_DATA FCF
#define BP_FC_TXDATA     FCX

#define BP_FC_STREAM    FCS     // A1 is stride, A2 = BP_FC_STREAM_

#define BP_FC_STREAM_MEAN    0x01    /* not implemented */
#define BP_FC_STREAM_NPAIRS  0xf0    /* # pairs of channels 0=>all 1=1, 2=2 etc */
#define BP_FC_STREAM_BURST   0x02    /* frame is a burst of full rate data */

#define BP_FC_GET_NPAIRS(f)    (((f)&BP_FC_STREAM_NPAIRS)>>4)
#define BP_FC_SET_NPAIRS(p)    (((p)<<4)&BP_FC_STREAM_NPAIRS)

/* interrupt defs */

#define BP_INT_COMMAND_ACK    0x0001
#define BP_INT_STATUS_CHANGE  0x0002
#define BP_INT_LLC_DMA_DONE   0x0004   /* LLC mode: dma complete when enabled*/
#define BP_INT_LLC_ERROR      0x0008   /* LLC mode: errored */

/*
 * start, length have a "block size" - default 1 samples,
 * may be upped to 16K if these bits are set:
 */
#define BS_16K                   0x4000
#define BS_1K                    0x400

#define BP_A1_START_BS_16K       0x01
#define BP_A1_LENGTH_BS_16K      0x02
#define BP_A1_START_BS_1K        0x04
#define BP_A1_LENGTH_BS_1K       0x08

#define BP_A1_FETCH_PRETRIGGER   0x10

#define MAX_START_CODING         0xffffU
#define MAX_LENGTH_CODING        0xffffU




/*
 * Bits def for BP_FC_SET_DIO
 * Direction 1=>Output
 */


// sample tagging

     /*
      * sample tag structure (ICD rev 6 )
      *
      *       d1111110000000000
      *        5432109876543210
      *        nTssssssdddddddd
      *        XT54321076543210
      *
      *  where nX { X 0..47 } is the SAMPLE NUMBER for subframe 0
      *  T                    Trigger
      *  s{5..0}              is the subframe number
      *  d{7..0}              DIO state (D0..7) on even subframe,
      *                                 (D15..8) on odd subframe,
      *  nX { X 48..63 } are extra bits (not used)
      */

#define SF0 0xfe
#define SF1 0xed

/*
 * generate saample tags ... should be good for non-simul too
 */
#define SUBFRAME_MASK 0x3f
#define NSUBFRAMES    0x40
#define NID_BITS      0x30

#define nX_bit 15
#define T_bit  14
#define s0_bit  8

/*
 * MULTI-FRAME: ICD rev 24
 */
#define MFX_SF0    0             /* Single frame marker */
#define MFX_SF1    1
#define MFX_MF2    2             /* Multi frame marker */
#define MFX_MF3    3
#define MFX_FNa    4             /* Frame number in Multi Frame {15:0} */
#define MFX_FNb    5
#define MFX_Ja     6             /* Jiffies {31:0} */
#define MFX_Jb     7
#define MFX_Jc     8
#define MFX_Jd     9
#define MFX_ESa   10            /* Event Signature {31:0} */
#define MFX_ESb   11
#define MFX_ESc   12
#define MFX_ESd   13
#define MFX_ESOFa 14            /* Event 2 Sample Offset {23:0} */
#define MFX_ESOFb 15
#define MFX_ESOFc 16
#define MFX_DIO   17
#define MFX_EDIOa 18
#define MFX_EDIOb 19
#define MFX_EDIOc 20
#define MFX_EDIOd 21
#define MFX_BLENa 22            /* Burst Length (frames) {15:0} */
#define MFX_BLENb 23
#define MFX_BDELa 24            /* Burst Delay (samples) {15:0} */
#define MFX_BDELb 25
#define MFX_OVER  26
/* 27 spare */
#define MFX_MFNa  28            /* Multi Frame Number */
#define MFX_MFNb  29
#define MFX_MFNc  30
#define MFX_MFNd  31
#define MFX_TVSa  32            /* TimeVal Seconds */
#define MFX_TVSb  33
#define MFX_TVSc  34
#define MFX_TVSd  35
#define MFX_TVUSa 36            /* Time Val Micro Seconds */
#define MFX_TVUSb 37
#define MFX_TVUSc 38
#define MFX_TVUSd 39




#define MF_MF2 0xf0
#define MF_MF3 0x01

#define MF_FILLa 0xf1
#define MF_FILLb 0x11


/*
 * Message - defined at the end to get benefit of all the good things in this
 * file.
 */

/*
 * encoding of type
 */

#define MT_TYPE_SHIFT      24
#define MT_TYPE_MASK       (0xffU<<MT_TYPE_SHIFT)
#define MT_PRAM_MASK       ~(MT_TYPE_MASK)

#define MT_ID( type )      ((type)&MT_TYPE_MASK)
#define MT_PRAM(type)      ((type)&MT_PRAM_MASK)


/*
 * Function #1: Streaming Data
 */

#define MT_STREAM        (BP_FC_STREAM<<MT_TYPE_SHIFT)

#define MTP_STREAM_IS_TAGGED  0x800000   /* tagged if this bit set */
#define MTP_STREAM_END_PACKET 0x400000   /* hint to transmit this packet on */

#define MTP_STREAM_NCHAN_MASK 0x0000ff   /* #channels              */
#define MTP_STREAM_NSAM_MASK  0x00ff00   /* #samples in message    */

#define MTP_STREAM_NCHAN(type) ((type)&MTP_STREAM_NCHAN_MASK)
#define MTP_STREAM_NSAM(type)  (((type)&MTP_STREAM_NSAM_MASK)>>8)

#define MTP_STREAM_SLEN(type)  \
    ((MTP_STREAM_NCHAN(type)+(((type)&MTP_STREAM_IS_TAGGED)!=0?2:0))*2)
/*
 * data is arranged: data[NSAM][(IS_TAGGED?2:0)+NCHAN]
 *
 * and a sample is filler:tag:pair0:pair1....
 */

/* Function #2: get FW Revision */

#define MT_FWREV        (BP_FC_GET_FWREV<<MT_TYPE_SHIFT)

/* Function #3: notify BIGDUMP completion */

#define MT_BIGDUMP_COMPLETE (BP_FC_BIG_DUMP<<MT_TYPE_SHIFT)

/* Function #4: get LCA details */

#define MT_GETCONFIG    BP_FC_GET_CONFIG

#define FLAVOR_KEY    "FLAVOR"  // first line FLAVOR=modelXXX

/* Function #5: get CAL details */

#define MT_GETCALINFO    (BP_FC_GET_CALINFO<<MT_TYPE_SHIFT)

/* Function #6: get CAPTURE STATUS */

#define MT_GETCAPTURESTATS (BP_FC_GET_CAPTURESTATS<<MT_TYPE_SHIFT)


/*
 * LLC LLC LLC LLC LLC LLC LLC
 *
 * Low Latency Control Mode - works like this ...
 * 
 *
 * prams for setup command: BP_FC_SET_MODE_LLC
 * A1 - prams
 * A3 - clock div for extclock mode
 * LLCV2:
 * A2 - host buffer with target address settings when BP_FC_SET_LLCV2_INIT on
 */
 
#define BP_FC_SET_MODE_LLC FCL   // switch into Low Latency Control Mode.
 
#define BP_FC_SET_MODE_LLC_SOFTCLOCK    0x80  // EXCLUSIVE!
#define BP_FC_SET_MODE_LLC_EXTCLOCK     0x40

#define BP_FC_SET_MODE_LLC_CLKPOL_POS   0x20  // set for rising edge active
#define BP_FC_SET_MODE_LLC_TRPOL_POS    0x10  // set for rising edge active
#define BP_FC_SET_LLCV2_INIT            0x08  // address of init block in A4
#define BP_FC_SET_RTCLOCK_TIMING        0x04

#define BP_FC_SET_MODE_LLC_INTSOFT_CLK  0x01  // test mode
#define BP_FC_SET_MODE_LLC_INTDIV_CLK   0x02  // test mode


/* 
 * and after entering LLC, the following defs apply:
 */

/* mailboxes: */

#define BP_MB_LLC_CSR           0    // M - Control S - Status
#define BP_MB_LLC_DATA_ADDR     1    // M puts host data target phys addr here
#define BP_MB_LLC_TADC          2    // S puts latched time here on request
#define BP_MB_LLC_TINST         3    // S puts inst time here on request


/* CSR bits: */
/* WARNING:unfortunately this is a shared reg, so extreme care is needed*/
/*
 * Master sends a command by clearing SACK and SNACK
 * Slave signals command complete by setting SACK or SNACK
 */
#define LLC_CSR_SACK        0x80000000  // S reports command ACK
#define LLC_CSR_SNACK       0x40000000  // S slave reports negative ACK

#define LLC_CSR_READY       0x10000000  // S reports ready for commands
#define LLC_CSR_S_IS_ARMED  0x08000000  // S reports ADC ARMED
#define LLC_CSR_S_CTR_RUN   0x04000000  // S reports counter running
#define LLC_CSR_S_DMA_DONE  0x02000000  // S reports DMA done  
#define LLC_CSR_S_ERROR     0x01000000  // S reports ERROR.

#define LLC_CSR_S_TCYCLE    0x00ff0000  // usecs clock to copy done

#define LLC_MAKE_TCYCLE( tc ) ((tc)<<16)
#define LLC_GET_TCYCLE( csr ) (((csr)&LLC_CSR_S_TCYCLE)>>16)


#define LLC_CSR_M_DECIM     0x00000f00  // M sets decimation value

#define LLC_MAKE_DECIM( decim ) ((decim)<<8)
#define LLC_GET_DECIM( csr )    (((csr)&LLC_CSR_M_DECIM)>>8)


#define LLC_CSR_M_LLC200_INIT 0x00000080 // M A4 holds LLC200_INIT_BUF
/* Possible, not implemented

#define LLC_CSR_M_TIMSTAMP  0x00000040  // M append TADC, CSR to data
*/

#define LLC_CSR_M_AUTOINCR  0x00000020  // M Auto increment target addr

// P: (set by M, reset by S), L:level set by M

#define LLC_CSR_M_SETADDR   0x00000010  // MP load new DATA_ADDR
#define LLC_CSR_M_READCTR   0x00000008  // MP request to read counter
#define LLC_CSR_M_ESC       0x00000004  // MP SET TRUE TO ESCAPE to normal ops
#define LLC_CSR_M_ARM       0x00000002  // ML set true to arm
#define LLC_CSR_M_SOFTCLOCK 0x00000001  // MP ADC soft clock


/** LLCV2 ... host side buffer for host side initialization 
 * treat as u32[index]
 */

#define LLCV2_INIT_MAGIC_MARKER 0xfeedc0de

#define LLCV2_INIT_MARKER  0      /* set to LLCV2_INIT_MAGIC_MARKER to init */
#define LLCV2_INIT_AI_HSBT 1
#define LLCV2_INIT_AO_HSBS 2
#define LLCV2_INIT_DO_HSBS 3
#define LLCV2_INIT_STATUS_HSBT 4

#define LLCV2_INIT_LAST 5

/** LLCV2 ... host side buffer for host side initialization 
 * treat as u32[index]
 * allows up to 8 x AO32 cards
 * each AO32 is represented by a non-zero PA
 * AO32/DO6 values are presented in the AO vector ..
 */
#define LLCV2_INIT_MAGIC_AO32	0xfeedcade
/*
#define LLCV2_INIT_AI_HSBT 1
#define LLCV2_INIT_AO_HSBS 2
#define LLCV2_INIT_DO_HSBS 3
#define LLCV2_INIT_STATUS_HSBT 4
*/
#define LLCV2_INIT_AO32PA0 5
#define LLCV2_INIT_AO32_MAX 8

#define LLCV2_INIT_AO32_LAST 13

#define LLC200_INIT_MAGIC_MARKER 0x200cafe0


/** LLC200_INIT_MASK change the parameter when its mask is set. */

#define LLC200_INIT_MASK_DDS_FTW  0x0001     
#define LLC200_INIT_MASK_RANGE    0x0002
#define LLC200_INIT_MASK_OFFSETS  0x0004
#define LLC200_INIT_MASK_TRIG     0x0008
#define LLC200_INIT_MASK_CHANNEL  0x0010
#define LLC200_INIT_MASK_INTCLK   0x0020
#define LLC200_INIT_MASK_DDS_QDAC 0x0040    /** adjust mark/space on DDS */

#define LLC200_INIT_TRIG_EXTRIG   0x0010    /* external trigger if set */
#define LLC200_INIT_TRIG_EXTLINE  0x000f    /* mask 0..5 => DI0..DI5   */
#define LLC200_INIT_TRIG_RISING   0x0020    /* Rising if set           */


/** LLC200_INIT acq216 host side buffer for host side initialisation.
 *  we use a struct else it's just too tedious, but enforce a u32 alignment
 *  between fields
 */


struct LLC200_INIT  {
	unsigned marker;	      /** LLC200_INIT_MAGIC_MARKER. */
	unsigned mask;                /** indicates the fields to change. */
	unsigned char  dds_ftw[8];    /** 6 byte ftw, 2 byte QDAC */
	union {
		unsigned long long ull;
		unsigned short w[4];      /* r1, r2, r3, pad */
	} vranges;
	unsigned short offsets[16];   /** channels 1..16.  */
	unsigned trig;                /** trigger definition @todo */
	unsigned channel_mask;        /** 0xf,0xff,0xfff,0xffff {4m,8,12,16} */
	unsigned int_clk;             /** internal clock in Hz 
				       * (alternate to dds) 
				       */
}
 __attribute__ ((packed)); 

/* OR to LLC200_INIT.channel_mask if antiphase */
#define LLC200_INIT_CHANNEL_MASK_ANTIPHASE 0x80000000

/* block length definition in channel_mask block length in 1KBytes units */
#define LLC200_INIT_CHANNEL_MASK_BLOCKLEN_MASK 0x0fff0000
#define LLC200_INIT_CHANNEL_MASK_BLOCKLEN_SHIFT 16
#define LLC200_INIT_CHANNEL_BLOCKLEN_1K 10



/* ... possibility to init other params from here .. */

/** format of host side status area u32[index]. */
#define LLCV2_STATUS_MBOX0  0
#define LLCV2_STATUS_MBOX1  1 
#define LLCV2_STATUS_MBOX2  2
#define LLCV2_STATUS_MBOX3  3
#define LLCV2_STATUS_DIO    4     /** {5:0} DI status */
#define LLCV2_STATUS_TINST  5     /** TINST  {11:0} */
#define LLCV2_STATUS_TLATCH 6     /** TLATCH {11:0} */
#define LLCV2_STATUS_BDR    7     /** Ident, default 0xdeadbeef */


/** LLC SYNC2V - two vector case, standard offsets */

/* offset in Output vector u32[index] */
#define LLC_SYNC2V_DO       8

/* offset 9 : spare  */

/* and AO32s tag on at the back : */
#define LLC_SYNC2V_AO32	    10

/* AO32VEC is always 32AO + 64DO ... fitted or not */
#define AO32_VECLEN	(32*sizeof(short)+8*sizeof(char))

/* each AO32 card's AO+DO vector must be positioned here in the Output Vector: */
#define INDEX_OF_LLC_SYNC2V_AO32(icard)	\
	(LLC_SYNC2V_AO32 + (icard)*AO32_VECLEN/sizeof(u32))



/* offset in Input vector u32[index] */

#define LLC_SYNC2V_IN_MBOX0  LLCV2_STATUS_MBOX0    /* [ 0] */
#define LLC_SYNC2V_IN_MBOX1  LLCV2_STATUS_MBOX1    /* [ 1] */
#define LLC_SYNC2V_IN_MBOX2  LLCV2_STATUS_MBOX2    /* [ 2] */
#define LLC_SYNC2V_IN_MBOX3  LLCV2_STATUS_MBOX3    /* [ 3] */
#define LLC_SYNC2V_IN_DIO6   LLCV2_STATUS_DIO      /* [ 4] DI6 values */
#define LLC_SYNC2V_IN_TINST  LLCV2_STATUS_TINST    /* [ 5] Garbage for SYNC2V*/
#define LLC_SYNC2V_IN_TLATCH LLCV2_STATUS_TLATCH   /* [ 6] HW TLATCH 11:0 */
#define LLC_SYNC2V_IN_BDR    LLCV2_STATUS_BDR      /* [ 7] 0xdeadbeef */
#define LLC_SYNC2V_IN_ITER   (LLCV2_STATUS_BDR+1)  /* [ 8] Iteration */
#define LLC_SYNC2V_IN_DI32   (LLCV2_STATUS_BDR+2)  /* [ 9] DI32 values */
#define LLC_SYNC2V_IN_LASTE  (LLCV2_STATUS_BDR+3)  /* [10] ITER last error */
#define LLC_SYNC2V_IN_TLAT32 (LLCV2_STATUS_BDR+4)  /* [11] SW TLATCH 31:0 [OPT] */
#define LLC_SYNC2V_IN_VERID  (LLCV2_STATUS_BDR+5)  /* [12] Version ID */
#define LLC_SYNC2V_IN_SCOUNT (LLCV2_STATUS_BDR+6)  /* [13] Sample Count */
#define LLC_SYNC2V_IN_FIFSTA (LLCV2_STATUS_BDR+7)  /* [14] FIFSTAT */
#define LLC_SYNC2V_IN_LAST   (LLCV2_STATUS_MBOX0+15) /* [15] */
#define LLC_SYNC2V_IDLE_PAT 0x2f2fc0de       /* fills remainder */

/** HTM HTM HTM HTM HTM
 *
 * High thruput streaming modes
 *
 * Works like this:
 *
 * prams for setup command BP_FC_SETMODE_HTM
 * A1 = prams
 */


#define BP_FC_SETMODE_HTM fch 

/*
 * V2 mode supplies full local status, local status buffer is 
 * data_buffer_length / (32*sizeof(short)) * sizeof(unsigned) long
 */
#define BP_FC_SETMODE_HTM_V2    0x80
#define BP_FC_SETMODE_HTM_STATUSBUF 0x40 // Status increments in host buffer
#define BP_FC_SETMODE_HTM_HOFF  0x1f  // Specify holdoff in usecs

/*
 * Holdoff: ACQ32 delays sending data for #microsecs set in HOFF field
 * Most useful to ensure a give board is LAST - this is the one that
 * should interrupt
 */

/*
 * in V2, low bits of next address have significance
 */
#define HTM_V2_STATUS_OVERRUN 0x1
#define HTM_V2_STATUS_COMPLETE 0x2


/*
 * and after entering HTM, the following defs apply
 */

/* mailboxes: */

#define BP_MB_HTM_CSR            0       // M - Control Status reg
#define BP_MB_HTM_DATA_ADDR      1       // M - address of base of host mem buf
#define BP_MB_HTM_STATUS_ADDR    2       // M - address of status word host mem
#define BP_MB_HTM_CURRENT_ADDR   3       // S - current data mem pointer

/* WARNING:unfortunately this is a shared reg, so extreme care is needed*/
/* Control reg bits */

#define HTM_CSR_SACK        0x80000000  // S reports command ACK
#define HTM_CSR_SNACK       0x40000000  // S slave reports negative ACK

#define HTM_CSR_READY       0x10000000  // S reports ready for commands
#define HTM_CSR_S_IS_ARMED  0x08000000  // S reports ADC ARMED
#define HTM_CSR_S_OVERRUN   0x04000000  // S reports FIFO overrun
#define HTM_CSR_S_COMPLETE  0x02000000

#define HTM_CSR_M_SETADDR   0x00000010  // MP load new DATA_ADDR
#define HTM_CSR_M_RECYCLE   0x00000020  // M buffer is cyclic
#define HTM_CSR_M_INTERRUPT 0x00000100  // M interrupt on trnasfer done
#define HTM_HOSTBUF_MB      0x00fff000  // M host buf size in MB

#define HTM_CSR_M_ESC       0x00000004  // MP SET TRUE TO ESCAPE to normal ops
#define HTM_CSR_M_ARM       0x00000002  // ML set true to arm

#define HTM_GET_BUFFLEN_BYTES(csr) ( ((csr)&HTM_HOSTBUF_MB)<<(20-12) )
#define HTM_SET_BUFFLEN_BYTES(len) ( ((len)>>20)<<12 )

// prams - none

/*
 * sending commands over i2o
 */
 
#define MT_COMMAND_PACKET    (fcz<<MT_TYPE_SHIFT)

/*
 * subtype codes into PRAM field
 */

#define MAKE_MT_COMMAND( subtype ) (MT_COMMAND_PACKET|(subtype))

enum MTC_SUBTYPES {
    MTC_TEST,
    MTC_HOST_REQUEST_DATA,
    MTC_LOAD_AO,
    MTC_LOAD_DO,
    MTC_DEBUG_TEXT,
    MTC_HOST_REQUEST_X_DATA,
    MTC_REQUEST_STATUS_CHANGE_NOTIFICATION,
    MTC_HOST_REQUEST_DATA_HOSTBOUND,           /* ACQ to HOST */
    MTC_HOST_REQUEST_DATA_ACQBOUND,
    MTC_LAST
};


/* 
 * MTC_REQUEST_STATUS_CHANGE_NOTIFICATION
 */
 
typedef struct STATUS_CHANGE_NOTIFICATION_REQUEST {  
    unsigned request_mask;
    unsigned response_mask;
}
    StatusChangeNotificationRequest;
/*
 * Host Request Data - supply zero terminated list of records
 * to govern the transfer.
 * return individual records via I2O when transfer done
 *
 * For 1G, pci is the pci bus address.
 * Bit for 2G, pci is the host bus offset => 0 is valid!!
 */
typedef struct HOST_REQUEST_DATA_RECORD {
    unsigned pci;			       
    unsigned start;
    unsigned nsamples;        
    unsigned short chan;
    unsigned short stride;
}
    HostRequestDataRecord;

/*
 * optional flags for chan
 */

#define HRD_CHAN_IS_FILE 0x8000
#define HRD_CHAN_EOF     0x0400
#define HRD_CHECKED      0x4000    /* we have checked this one */
#define HRD_ABS_PCI	 0x2000    /* pci is absolute address  */
#define HRD_READ_NEXT	 0x1000	   /* ignore start, just return next data */

#define HRD_CHANNEL(hrd)  (((hrd)->chan)&0x0ff)
#define HRD_SPCLID(hrd)   (((hrd)->chan)&0x07f)
#define HRD_WAVETRACK    0x2000    /* track coded as {0x00T{7:3}C{4:5}} */

#define HRD_TRACK(chan)    ((chan>>5)&0x7)  /* index from 0 */
#define HRD_CHAN(chan)     (((chan)&0x1f)+1)    /* index from 0 */

/* special "channel ids" => bit 7 set */
#define HRD_SPCLID_BATCHTO 99          /* original batch in/out mech */
#define HRD_SPCLID_BATCHOUT 100
#define HRD_SPCLID_BATCHIO  101        /* batch in/out on same channel */
#define HRD_SPCLID_BATCHRSH 102        /* planned interactive rsh io   */

/* text acqcmd : monitor 128+109 = 237 */
#define HRD_SPCLID_ACQCMD   109

/*
 * Host Request Cross Sections Data - supply zero terminated list of records
 * to govern the transfer.
 * return individual records via I2O when transfer done
 */

#define REQUESTMAXBYTES( hrdr )    ((hrdr).nsamples)

#define START_LATEST    0xFFFFFFFF            // collect latest data from now

/*
 * AO data - data[0] is n data, data[1]..data[n] is the data to load
 *
 * DO data ??? the same ???
 */

typedef struct ACQ32_PULL_OUTPUT_DATA_RECORD {
    unsigned src_pci;
    unsigned func;                // {BP_FC_SET_DACS|BP_FC_SET_DO}
    unsigned start;
    unsigned nsamples;
    unsigned data[1];            // { 1..nsamples] is the rest of the data
}
    Acq32PullOutputDataRecord;

    
            
 
 
/*
 * TEST distributor data encoding
 */
/*
 * framed data for test:
 * aim is to encode entire sample info in 16 bits usng a frame structure
 *
 */
 
#define T_ISAMPLE_BIT   0x0040    // { d0, d1 ... d64 on successive subframes 
#define T_SUBFRAME_MASK 0x003f    // 64 sample subframe
#define T_CHANNEL_MASK  0x0f80    // {0..31} - channel ID
#define T_TRIGGER_BIT   0x8000    // set TRUE after trigger
#define T_RAMP_MASK     0x7000    // ramp major bits every 64 samples
 
/*
 * setRoute encoding MTC_ACQ32CPCI_SET_ROUTE - obsolete on 2G
 */

#define BP_FC_CPCI_SET_ROUTE fcy // <A0=func> <A1=route>

#define BP_FC_CPCI_GETROUTE_STATUS fcy // <A0=status>
// BEWARE: func, dest defs map DIRECTLY to HW bits ... danger!

// Store func as ONE of in A0

enum Q32C_SR_FUNC {
   Q32C_SR_FUNC_DI0,
   Q32C_SR_FUNC_DI1,
   Q32C_SR_FUNC_DI2,
   Q32C_SR_FUNC_DI3,
   Q32C_SR_FUNC_DI4,
   Q32C_SR_FUNC_DI5,
   
   Q32C_SR_FUNC_AICLK = 0,
   Q32C_SR_FUNC_AITRIG,
   Q32C_SR_FUNC_AOCLK,
   Q32C_SR_FUNC_AOTRIG,
   Q32C_SR_FUNC_DOCLK,
   Q32C_SR_FUNC_DOTRIG
};

// store dest as OR MASK of
enum Q32C_SR_DEST {
    
    Q32C_SR_DEST_NONE= 0x00,
    
    Q32C_SR_DEST_MIx = 0x01,
    Q32C_SR_DEST_MOx = 0x02,
    
    Q32C_SR_DEST_J50 = 0x04,
    Q32C_SR_DEST_J51 = 0x04,
    Q32C_SR_DEST_J52 = 0x04,
    Q32C_SR_DEST_J53 = 0x04,
    Q32C_SR_DEST_J34 = 0x04,
    Q32C_SR_DEST_J35 = 0x04,
    
    Q32C_SR_DEST_PXI_TRIG0 = 0x08, 
    Q32C_SR_DEST_PXI_TRIG1 = 0x08, 
    Q32C_SR_DEST_PXI_TRIG2 = 0x08, 
    Q32C_SR_DEST_PXI_TRIG3 = 0x08, 
    Q32C_SR_DEST_PXI_TRIG4 = 0x08,
    Q32C_SR_DEST_PXI_TRIG5 = 0x08,
    
    Q32C_SR_DEST_PXI_TRIG6 = 0x10, 
    Q32C_SR_DEST_PXI_TRIG7 = 0x10,
    Q32C_SR_DEST_PXI_STAR  = 0x10,
    Q32C_SR_DEST_PXI_CLK10 = 0x10
};


/*
 * PUSH model message passing using I2O
 */

// a message fits into 1K
 
#define MAKE_ID_DTACQ(n) (((n)&0x7f)<<8|0xDC)
#define IS_ID_DTACQ(id)  (((id)&0xff)==0xDC)
#define GET_IDN(id)      ((id)>>8)

typedef struct MESSAGE_HEADER {
    unsigned short id;          // %0x7fff  ID for the message
    unsigned short length;      // #bytes in data
    unsigned type;
}
        MessageHeader;

#define MID_NOT_VALID       0x8000  // host sets this to mark ID

#define MESSAGE_HEADER_SIZE (sizeof(MessageHeader))
#define MESSAGE_LEN         (0x400)
#define MESSAGE_DATA_LEN    (MESSAGE_LEN-MESSAGE_HEADER_SIZE)
#define HRDR_SZ             (sizeof(struct HOST_REQUEST_DATA_RECORD))
#define MESSAGE_HRDR_LEN    (MESSAGE_DATA_LEN/HRDR_SZ)

#define MESSAGE_ACTUAL_LEN(message) \
       (sizeof(MessageHeader) + (message)->header.length)

/** I2O message handling. */        
typedef struct MESSAGE { 
    MessageHeader header;
    union{
	    unsigned short sdata[MESSAGE_DATA_LEN/sizeof(unsigned short)];
	    unsigned       ldata[MESSAGE_DATA_LEN/sizeof(unsigned)];
	    char           cdata[MESSAGE_DATA_LEN];
	    struct HOST_REQUEST_DATA_RECORD hrdr[MESSAGE_HRDR_LEN];
    }
	    payload;
}
    Message;

#endif              /* _ACQ32BUSPROT_H_ */















