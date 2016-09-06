/****************************************************************************************/
/* ----- Do not change ----- */
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
`define SERIAL_WCNT 'd200                    //  1Mbaud for 200MHz Clock

/****************************************************************************************/
`define DATANUM         (256*1024*1024)  // in word size, 256M = 1GiB

`define DRAM_RBLOCKS                 32  // the number of blocks per DRAM read  access
`define DRAM_WBLOCKS                 32  // the number of blocks per DRAM write access

`define READ_ACCESS_CNT  ((`DATANUM>>4)/`DRAM_RBLOCKS)
`define WRITE_ACCESS_CNT ((`DATANUM>>4)/`DRAM_WBLOCKS)
