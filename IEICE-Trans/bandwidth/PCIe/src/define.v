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
`define CLKIN_PERIOD   5000  // 5000ps = 200MHz
  
/* ----- User Parameter ----- */
`define CLKFX_MULTIPLY      8
`define CLKFX_DIVIDE        1
`define USERCLK_DIVIDE      8
`define USERCLKMMCM_DIVIDE  2
`define DRAMCLK_DIVIDE (`USERCLK_DIVIDE >> 2)
`define FREQREF_DIVIDE (`USERCLK_DIVIDE >> 2)
`define SYNCPLS_DIVIDE (`USERCLK_DIVIDE << 2)

/****************************************************************************************/
`define MEM_ADDR 28:0                       // 29bit address in 8byte for 4GB memory
`define MEM_LAST_ADDR  (({26{1'b1}}) << 3)  // 4096MB Write/Read
/****************************************************************************************/
`define DRAM_REQ_READ  1
`define DRAM_REQ_WRITE 2
/****************************************************************************************/
`define SERIAL_WCNT 'd200  //  1Mbaud for 200MHz Clock

// FVCO = 200MHz(input clk freq) * CLKFBOUT_MULT_F / DIVCLK_DIVIDE
// CLK  = 200MHz(input clk freq) * CLKFBOUT_MULT_F / (DIVCLK_DIVIDE * CLKOUT(n)_DIVIDE)
// e.g.
// USERCLK = 200MHz(input clk freq) * CLKFX_MULTIPLY / (DIVCLK_DIVIDE * USERCLK_DIVIDE)

/* USERCLK */ 
/* 
 fabric clock freq ; either  half rate or quarter rate and is
 determined by  PLL parameters settings. 
*/ 
  
/* DRAMCLK */ 
/* 
 equal to  memory clock 
*/ 
  
/* FREQREF */ 
/* 
 freq above 400 MHz:  set freq_refclk = mem_refclk
 freq below 400 MHz:  set freq_refclk = 2* mem_refclk or 4* mem_refclk;
 to hard PHY for phaser
*/ 

/* SYNCPLS */
/*
exactly 1/16 of mem_refclk and the sync pulse is exactly 1 memref_clk wide
*/
