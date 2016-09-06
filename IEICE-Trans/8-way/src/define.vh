/***** DRAM Controller Parameter (Do not change)                                              *****/
/**************************************************************************************************/
`define DRAM_CMD_WRITE 3'b000
`define DRAM_CMD_READ  3'b001
`define DDR3_DATA      63:0
`define DDR3_ADDR      15:0
`define DDR3_CMD       2:0
`define APPDATA_WIDTH  512
`define APPADDR_WIDTH  32
`define APPMASK_WIDTH  (`APPDATA_WIDTH >> 3)
`define MEM_ADDR       28:0                  // 29bit address in 8byte for 4GB memory
`define MEM_LAST_ADDR  (({26{1'b1}}) << 3)   // 4096MB Write/Read
`define DRAM_REQ_READ  1
`define DRAM_REQ_WRITE 2

/******************************** ELEMS_PER_UNIT == 16 ***********************************************/
/** problem parameters and performance parameters for sorting datapath                              **/
/**  8-way : LastPhase 2 for 8K, 3 for 64K, 4 for 512K, 5 for 4M, 6 for 32M, 7 for 256M(1GB)        **/
/*****************************************************************************************************/
`define LAST_PHASE               7  // sorting last phase, P0, P1, ..., Pn
`define WAY_LOG                  3  // 3 for 8-way, 4->16-way, 5->32-way, 6->64-way, 7->128way ... 
                                    // sorting elements is calculated by WAY_LOG & LAST_PHASE
`define IB_SIZE                  7  // input  buffer FIFO size, 6->64entry, 7->128entry ...
`define OB_SIZE                  7  // output buffer FIFO size, 10->1024entry, 11->2048 entry ...  
`define DRAM_RBLOCKS            32  // the number of blocks per DRAM read  access, 4 blocks = 2KB
                                    // this must be 2^n where n is a natural number
`define DRAM_WBLOCKS            32  // the number of blocks per DRAM write access, 8 blocks = 4KB
                                    // this must be 2^n where n is a natural number
`define REQ_THRE                60  // read request threthold, may be (1<<IB_SIZE - RBLOCKS - 1)
`define ELEMS_PER_UNIT          16  // the number of elements in the first phase unit
                                    // 2 for tokuden board, 1 for VC707
`define PHASE_W                3:0  // bitwidth for phase, may be 4 bit (3:0)


/**************************************************************************************************/
`define SORTW                   32  // sort width in bit, may be 32bit
`define M_LOG                    2  // # of values emitted from sorter cell at once (LOG scale)
`define MERGW                  128  // merged value's width 
`define DRAMW                  512  // dram controller width in bit, 64 for tokuden, 512 for VC707
 
`define WW                    `SORTW-1:0

/***** do not change these parameters                                                         *****/
/**************************************************************************************************/
`define SORT_ELM     ((1<<((`LAST_PHASE+1)*`WAY_LOG))*`ELEMS_PER_UNIT)      //
`define SORT_WAY     (1<<`WAY_LOG)                                          // n-way of sorting 
`define SELM_PER_WAY ((`SORT_ELM<<2)>>`WAY_LOG)                             // the number of elements for one way
`define WAY_CNT      (((`SORT_ELM>>`WAY_LOG)/`DRAM_RBLOCKS)>>4)             // note!! the number of blocks for one way
`define WAY_CN_      (`WAY_CNT-1)                                           // the number of blocks for one way - 1
`define D_RS         (`DRAM_RBLOCKS<<3)                                     // DRAM_RBLOCKS * 8
`define D_WS         (`DRAM_WBLOCKS<<3)                                     // DRAM_WBLOCKS * 8
`define MAX_VALUE    {32'hffffffff,32'hffffffff,32'hffffffff,32'hffffffff}  // max value for synchronization
/**************************************************************************************************/

/***** simulation parameter                                                                   *****/
/**************************************************************************************************/
`define DRAM_SIZE      (8*1024*1024)   // size in block(64byte), 8M blocks = 512MB
`define INITTYPE       "xorshift"      // xorshift, reverse, sorted
