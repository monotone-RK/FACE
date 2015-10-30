/******************************************************************************/
/* FPGA Sort for VC707                                    ArchLab. TOKYO TECH */
/*                                                         Version 2014-11-26 */
/******************************************************************************/
`default_nettype none

`include "define.v"
`include "core.v"


/******************************************************************************/
module top_sim;
  reg CLK, RST;
  wire CLK100M = CLK;
  wire              d_busy;
  wire              d_w;
  wire [`DRAMW-1:0] d_din;
  wire [`DRAMW-1:0] d_dout;
  wire              d_douten;
  wire [1:0]        d_req;       // DRAM access request (read/write)
  wire [31:0]       d_initadr;   // dram initial address for the access
  wire [31:0]       d_blocks;    // the number of blocks per one access(read/write)
  wire initdone;
  wire sortdone;

  initial begin CLK=0; forever #50 CLK=~CLK; end
  initial begin RST=1; #400 RST=0; end
  
  reg [31:0] cnt;
  always @(posedge CLK) cnt <= (RST) ? 0 : cnt + 1;

  reg [31:0] lcnt;
  always @(posedge CLK) lcnt <= (RST) ? 0 : (c.last_phase && c.initdone) ? lcnt + 1 : lcnt;

  reg [31:0] cnt0_0, cnt1_0, cnt2_0, cnt3_0, cnt4_0, cnt5_0, cnt6_0, cnt7_0, cnt8_0;
  always @(posedge CLK) cnt0_0 <= (RST) ? 0 : (c.phase_a==0 && c.initdone) ? cnt0_0 + 1 : cnt0_0;
  always @(posedge CLK) cnt1_0 <= (RST) ? 0 : (c.phase_a==1 && c.initdone) ? cnt1_0 + 1 : cnt1_0;
  always @(posedge CLK) cnt2_0 <= (RST) ? 0 : (c.phase_a==2 && c.initdone) ? cnt2_0 + 1 : cnt2_0;
  always @(posedge CLK) cnt3_0 <= (RST) ? 0 : (c.phase_a==3 && c.initdone) ? cnt3_0 + 1 : cnt3_0;
  always @(posedge CLK) cnt4_0 <= (RST) ? 0 : (c.phase_a==4 && c.initdone) ? cnt4_0 + 1 : cnt4_0;
  always @(posedge CLK) cnt5_0 <= (RST) ? 0 : (c.phase_a==5 && c.initdone) ? cnt5_0 + 1 : cnt5_0;
  always @(posedge CLK) cnt6_0 <= (RST) ? 0 : (c.phase_a==6 && c.initdone) ? cnt6_0 + 1 : cnt6_0;    
  always @(posedge CLK) cnt7_0 <= (RST) ? 0 : (c.phase_a==7 && c.initdone) ? cnt7_0 + 1 : cnt7_0;    
  always @(posedge CLK) cnt8_0 <= (RST) ? 0 : (c.phase_a==8 && c.initdone) ? cnt8_0 + 1 : cnt8_0;    

  reg [31:0] cnt0_1, cnt1_1, cnt2_1, cnt3_1, cnt4_1, cnt5_1, cnt6_1, cnt7_1, cnt8_1;
  always @(posedge CLK) cnt0_1 <= (RST) ? 0 : (c.phase_b==0 && c.initdone) ? cnt0_1 + 1 : cnt0_1;
  always @(posedge CLK) cnt1_1 <= (RST) ? 0 : (c.phase_b==1 && c.initdone) ? cnt1_1 + 1 : cnt1_1;
  always @(posedge CLK) cnt2_1 <= (RST) ? 0 : (c.phase_b==2 && c.initdone) ? cnt2_1 + 1 : cnt2_1;
  always @(posedge CLK) cnt3_1 <= (RST) ? 0 : (c.phase_b==3 && c.initdone) ? cnt3_1 + 1 : cnt3_1;
  always @(posedge CLK) cnt4_1 <= (RST) ? 0 : (c.phase_b==4 && c.initdone) ? cnt4_1 + 1 : cnt4_1;
  always @(posedge CLK) cnt5_1 <= (RST) ? 0 : (c.phase_b==5 && c.initdone) ? cnt5_1 + 1 : cnt5_1;
  always @(posedge CLK) cnt6_1 <= (RST) ? 0 : (c.phase_b==6 && c.initdone) ? cnt6_1 + 1 : cnt6_1;    
  always @(posedge CLK) cnt7_1 <= (RST) ? 0 : (c.phase_b==7 && c.initdone) ? cnt7_1 + 1 : cnt7_1;    
  always @(posedge CLK) cnt8_1 <= (RST) ? 0 : (c.phase_b==8 && c.initdone) ? cnt8_1 + 1 : cnt8_1;    

  generate
    if (`INITTYPE=="reverse" || `INITTYPE=="sorted") begin
    // if (`INITTYPE=="reverse" || `INITTYPE=="sorted" || `INITTYPE=="xorshift") begin
      always @(posedge CLK) begin /// note
        if (c.initdone) begin

          $write("%d|%d|state(%d)", cnt[19:0], c.last_phase, c.state);
          $write("|");
          $write("P0%d(%d)|P1%d(%d)|P2%d(%d)|P3%d(%d)|P4%d(%d)|P5%d(%d)|P6%d(%d)|P7%d(%d) ", 
                 c.phase_a[2:0], c.pchange_a, c.phase_b[2:0], c.pchange_b, 
                 c.phase_c[2:0], c.pchange_c, c.phase_d[2:0], c.pchange_d,
                 c.phase_e[2:0], c.pchange_e, c.phase_f[2:0], c.pchange_f, 
                 c.phase_g[2:0], c.pchange_g, c.phase_h[2:0], c.pchange_h);
          
          if (c.F01_deq0) $write("%d ", c.F01_dot0); else $write("           ");
          if (c.F01_deq1) $write("%d ", c.F01_dot1); else $write("           ");
          if (c.F01_deq2) $write("%d ", c.F01_dot2); else $write("           ");
          if (c.F01_deq3) $write("%d ", c.F01_dot3); else $write("           ");
          if (c.F01_deq4) $write("%d ", c.F01_dot4); else $write("           ");
          if (c.F01_deq5) $write("%d ", c.F01_dot5); else $write("           ");
          if (c.F01_deq6) $write("%d ", c.F01_dot6); else $write("           ");
          if (c.F01_deq7) $write("%d ", c.F01_dot7); else $write("           ");
          $write(" |");
          if (d.app_wdf_wren) $write(" |M%d %d", d_din[63:32], d_din[31:0]);
          else $write("                        ");
          $write("|");
          $write(" %d %d %d %d %d %d %d %d ", c.ob0.OB_cnt, c.ob1.OB_cnt, c.ob2.OB_cnt, c.ob3.OB_cnt, c.ob4.OB_cnt, c.ob5.OB_cnt, c.ob6.OB_cnt, c.ob7.OB_cnt);
          $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqhalt_a_a, c.reqhalt_b_a, c.reqhalt_c_a, c.reqhalt_d_a, c.reqhalt_e_a, c.reqhalt_f_a, c.reqhalt_g_a, c.reqhalt_h_a);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqhalt_a_b, c.reqhalt_b_b, c.reqhalt_c_b, c.reqhalt_d_b, c.reqhalt_e_b, c.reqhalt_f_b, c.reqhalt_g_b, c.reqhalt_h_b);
          // $write("|");
          // $write(" %d %d", c.elem_c, c.elem_way_c);
          // $write("|");
          // $write(" %d %d %d %d", c.ob2.OB_cnt, c.w_block_c, c.OB_req_c, d_busy);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.r_block_a_c, c.r_block_b_c, c.r_block_c_c, c.r_block_d_c, c.r_block_e_c, c.r_block_f_c, c.r_block_g_c, c.r_block_h_c);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcnt_a_c, c.reqcnt_b_c, c.reqcnt_c_c, c.reqcnt_d_c, c.reqcnt_e_c, c.reqcnt_f_c, c.reqcnt_g_c, c.reqcnt_h_c);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcntmg_c.reqcnt_rsta, c.reqcntmg_c.reqcnt_rstb, c.reqcntmg_c.reqcnt_rstc, c.reqcntmg_c.reqcnt_rstd, 
          //                                     c.reqcntmg_c.reqcnt_rste, c.reqcntmg_c.reqcnt_rstf, c.reqcntmg_c.reqcnt_rstg, c.reqcntmg_c.reqcnt_rsth);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqhalt_a_c, c.reqhalt_b_c, c.reqhalt_c_c, c.reqhalt_d_c, c.reqhalt_e_c, c.reqhalt_f_c, c.reqhalt_g_c, c.reqhalt_h_c);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqhalt_a_d, c.reqhalt_b_d, c.reqhalt_c_d, c.reqhalt_d_d, c.reqhalt_e_d, c.reqhalt_f_d, c.reqhalt_g_d, c.reqhalt_h_d);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqhalt_a_e, c.reqhalt_b_e, c.reqhalt_c_e, c.reqhalt_d_e, c.reqhalt_e_e, c.reqhalt_f_e, c.reqhalt_g_e, c.reqhalt_h_e);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqhalt_a_f, c.reqhalt_b_f, c.reqhalt_c_f, c.reqhalt_d_f, c.reqhalt_e_f, c.reqhalt_f_f, c.reqhalt_g_f, c.reqhalt_h_f);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqhalt_a_g, c.reqhalt_b_g, c.reqhalt_c_g, c.reqhalt_d_g, c.reqhalt_e_g, c.reqhalt_f_g, c.reqhalt_g_g, c.reqhalt_h_g);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqhalt_a_h, c.reqhalt_b_h, c.reqhalt_c_h, c.reqhalt_d_h, c.reqhalt_e_h, c.reqhalt_f_h, c.reqhalt_g_h, c.reqhalt_h_h);
          // $write(" %d %d %d", c.ob0.OB_cnt, c.w_block_a, c.OB_req_a);
          // $write(" %d %d %d %d", c.ob0.OB_cnt, c.w_block_a, c.OB_req_a, d_busy);
          // $write(" %d %d %d %d", c.ob0.OB_cnt, c.w_block, c.OB_req_a, d_busy);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.r_block_a, c.r_block_b, c.r_block_c, c.r_block_d, c.r_block_e, c.r_block_f, c.r_block_g, c.r_block_h);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.readend_a, c.readend_b, c.readend_c, c.readend_d, c.readend_e, c.readend_f, c.readend_g, c.readend_h);
          // $write("|");
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.r_block_a_a, c.r_block_b_a, c.r_block_c_a, c.r_block_d_a, c.r_block_e_a, c.r_block_f_a, c.r_block_g_a, c.r_block_h_a);
          // $write("|");
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.radr_a_a, c.radr_b_a, c.radr_c_a, c.radr_d_a, c.radr_e_a, c.radr_f_a, c.radr_g_a, c.radr_h_a);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.r_endadr_a_a, c.r_endadr_b_a, c.r_endadr_c_a, c.r_endadr_d_a, c.r_endadr_e_a, c.r_endadr_f_a, c.r_endadr_g_a, c.r_endadr_h_a);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.readend_a_a, c.readend_b_a, c.readend_c_a, c.readend_d_a, c.readend_e_a, c.readend_f_a, c.readend_g_a, c.readend_h_a);
          // $write("|");
          
          // $write(" %d %d %d %d %d %d %d %d ", c.w_addr_a, c.w_addr_b, c.w_addr_c, c.w_addr_d, c.w_addr_e, c.w_addr_f, c.w_addr_g, c.w_addr_h);
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcnt_a, c.reqcnt_b, c.reqcnt_c, c.reqcnt_d, c.reqcnt_e, c.reqcnt_f, c.reqcnt_g, c.reqcnt_h);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcnt_a_a, c.reqcnt_b_a, c.reqcnt_c_a, c.reqcnt_d_a, c.reqcnt_e_a, c.reqcnt_f_a, c.reqcnt_g_a, c.reqcnt_h_a);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcnt_a_b, c.reqcnt_b_b, c.reqcnt_c_b, c.reqcnt_d_b, c.reqcnt_e_b, c.reqcnt_f_b, c.reqcnt_g_b, c.reqcnt_h_b);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcnt_a_c, c.reqcnt_b_c, c.reqcnt_c_c, c.reqcnt_d_c, c.reqcnt_e_c, c.reqcnt_f_c, c.reqcnt_g_c, c.reqcnt_h_c);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcnt_a_d, c.reqcnt_b_d, c.reqcnt_c_d, c.reqcnt_d_d, c.reqcnt_e_d, c.reqcnt_f_d, c.reqcnt_g_d, c.reqcnt_h_d);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcnt_a_e, c.reqcnt_b_e, c.reqcnt_c_e, c.reqcnt_d_e, c.reqcnt_e_e, c.reqcnt_f_e, c.reqcnt_g_e, c.reqcnt_h_e);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcnt_a_f, c.reqcnt_b_f, c.reqcnt_c_f, c.reqcnt_d_f, c.reqcnt_e_f, c.reqcnt_f_f, c.reqcnt_g_f, c.reqcnt_h_f);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcnt_a_g, c.reqcnt_b_g, c.reqcnt_c_g, c.reqcnt_d_g, c.reqcnt_e_g, c.reqcnt_f_g, c.reqcnt_g_g, c.reqcnt_h_g);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.reqcnt_a_h, c.reqcnt_b_h, c.reqcnt_c_h, c.reqcnt_d_h, c.reqcnt_e_h, c.reqcnt_f_h, c.reqcnt_g_h, c.reqcnt_h_h);
          // $write("|");

          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.im00_0.im_cnt, c.im01_0.im_cnt, c.im02_0.im_cnt, c.im03_0.im_cnt, c.im04_0.im_cnt, c.im05_0.im_cnt, c.im06_0.im_cnt, c.im07_0.im_cnt);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.im00_1.im_cnt, c.im01_1.im_cnt, c.im02_1.im_cnt, c.im03_1.im_cnt, c.im04_1.im_cnt, c.im05_1.im_cnt, c.im06_1.im_cnt, c.im07_1.im_cnt);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.im00_2.im_cnt, c.im01_2.im_cnt, c.im02_2.im_cnt, c.im03_2.im_cnt, c.im04_2.im_cnt, c.im05_2.im_cnt, c.im06_2.im_cnt, c.im07_2.im_cnt);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.im00_3.im_cnt, c.im01_3.im_cnt, c.im02_3.im_cnt, c.im03_3.im_cnt, c.im04_3.im_cnt, c.im05_3.im_cnt, c.im06_3.im_cnt, c.im07_3.im_cnt);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.im00_4.im_cnt, c.im01_4.im_cnt, c.im02_4.im_cnt, c.im03_4.im_cnt, c.im04_4.im_cnt, c.im05_4.im_cnt, c.im06_4.im_cnt, c.im07_4.im_cnt);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.im00_5.im_cnt, c.im01_5.im_cnt, c.im02_5.im_cnt, c.im03_5.im_cnt, c.im04_5.im_cnt, c.im05_5.im_cnt, c.im06_5.im_cnt, c.im07_5.im_cnt);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.im00_6.im_cnt, c.im01_6.im_cnt, c.im02_6.im_cnt, c.im03_6.im_cnt, c.im04_6.im_cnt, c.im05_6.im_cnt, c.im06_6.im_cnt, c.im07_6.im_cnt);
          // $write("|");
          // $write(" %d %d %d %d %d %d %d %d ", c.im00_7.im_cnt, c.im01_7.im_cnt, c.im02_7.im_cnt, c.im03_7.im_cnt, c.im04_7.im_cnt, c.im05_7.im_cnt, c.im06_7.im_cnt, c.im07_7.im_cnt);
          // $write("|");
          // $write("|");
          // $write(" %d ", c.decompressor.dmf_cnt);
          
          $write("\n");
          $fflush();

          // if (c.phase_b==2 && cnt[19:0]==500000) $finish(); // for debug
          // if ((c.phase_a==2) || (c.ob0.OB_cnt>128 || c.ob1.OB_cnt>128 || c.ob2.OB_cnt>128 || c.ob3.OB_cnt>128 || c.ob4.OB_cnt>128 || c.ob5.OB_cnt>128 || c.ob6.OB_cnt>128 || c.ob7.OB_cnt>128)) $finish(); // for debug

        end
      end
      
      always @(posedge CLK) begin
        if(c.sortdone) begin : simulation_finish
          $write("\nIt takes %d cycles\n", cnt);
          $write("last(%1d): %d cycles\n", `LAST_PHASE, lcnt);
          $write("phase0:  %d %d cycles\n", cnt0_0, cnt0_1);
          $write("phase1:  %d %d cycles\n", cnt1_0, cnt1_1);
          $write("phase2:  %d %d cycles\n", cnt2_0, cnt2_1);
          $write("phase3:  %d %d cycles\n", cnt3_0, cnt3_1);
          $write("phase4:  %d %d cycles\n", cnt4_0, cnt4_1);
          $write("phase5:  %d %d cycles\n", cnt5_0, cnt5_1);
          $write("phase6:  %d %d cycles\n", cnt6_0, cnt6_1);
          $write("phase7:  %d %d cycles\n", cnt7_0, cnt7_1);
          $write("phase8:  %d %d cycles\n", cnt8_0, cnt8_1);
          $write("Sorting finished!\n");
          $finish();
        end
      end
    end else if (`INITTYPE == "xorshift") begin
      integer fp;
      initial begin
        fp = $fopen("log.txt", "w");
      end
      always @(posedge CLK) begin /// note
        if (c.last_phase && c.F01_deq0) begin
          $write("%08x ", c.F01_dot0);
          $fwrite(fp, "%08x ", c.F01_dot0);
          $fflush();
        end
        if (c.sortdone) begin
          $fclose(fp);
          $finish();
        end
      end
    end
  endgenerate
  
  /***** DRAM Controller & DRAM Instantiation                                               *****/
  /**********************************************************************************************/
  DRAM d(CLK, RST, d_req, d_initadr, d_blocks, d_din, d_w, d_dout, d_douten, d_busy);

  wire ERROR;
  /***** Core Module Instantiation                                                          *****/
  /**********************************************************************************************/
  CORE c(CLK100M, RST, initdone, sortdone,
         d_busy, d_din, d_w, d_dout, d_douten, d_req, d_initadr, d_blocks, ERROR);
endmodule
/**************************************************************************************************/
/**************************************************************************************************/
module DRAM   (input  wire              CLK,       //
               input  wire              RST,     //
               input  wire [1:0]        D_REQ,     // dram request, load or store
               input  wire [31:0]       D_INITADR, // dram request, initial address
               input  wire [31:0]       D_ELEM,    // dram request, the number of elements
               input  wire [`DRAMW-1:0] D_DIN,     //
               output wire              D_W,       // 
               output reg  [`DRAMW-1:0] D_DOUT,    //
               output reg               D_DOUTEN,  //
               output wire              D_BUSY);   //

  /******* DRAM ******************************************************/
  localparam M_REQ   = 0;
  localparam M_WRITE = 1;
  localparam M_READ  = 2;
  ///////////////////////////////////////////////////////////////////////////////////
  reg [`DDR3_CMD]   app_cmd;
  reg               app_en;
  wire [`DRAMW-1:0] app_wdf_data;
  reg               app_wdf_wren;
  wire              app_wdf_end = app_wdf_wren;

  // outputs of u_dram
  wire [`DRAMW-1:0] app_rd_data;
  wire              app_rd_data_end;
  wire              app_rd_data_valid=1; // in simulation, always ready !!
  wire              app_rdy = 1;         // in simulation, always ready !!
  wire              app_wdf_rdy = 1;     // in simulation, always ready !!
  wire              ui_clk = CLK;
  
  reg [1:0]        mode;
  reg [`DRAMW-1:0] app_wdf_data_buf;
  reg [31:0]       caddr;           // check address
  reg [31:0]       remain, remain2; //
  reg [7:0]        req_state;       //
  ///////////////////////////////////////////////////////////////////////////////////
  reg [`DRAMW-1:0] mem [`DRAM_SIZE-1:0];
  reg [31:0]  app_addr;
  

  reg [31:0] dram_addr;
  always @(posedge CLK) dram_addr <= app_addr;

  always @(posedge CLK) begin  /***** DRAM WRITE *****/
    if (RST) begin end
    else if(app_wdf_wren) mem[dram_addr[27:3]] <= app_wdf_data;
  end

  assign app_rd_data = mem[app_addr[27:3]];
  assign app_wdf_data = D_DIN;
  
  assign D_BUSY  = (mode!=M_REQ);                             // DRAM busy  
  assign D_W     = (mode==M_WRITE && app_rdy && app_wdf_rdy); // store one element 
  
  ///// READ & WRITE PORT CONTROL (begin) ////////////////////////////////////////////
  always @(posedge ui_clk) begin
    if (RST) begin
      mode <= M_REQ;
      {app_addr, app_cmd, app_en, app_wdf_wren} <= 0;
      {D_DOUT, D_DOUTEN} <= 0;
      {caddr, remain, remain2, req_state} <= 0;
    end else begin
      case (mode)
        ///////////////////////////////////////////////////////////////// request
        M_REQ: begin
          D_DOUTEN <= 0;
          if(D_REQ==`DRAM_REQ_WRITE) begin  ///// WRITE or STORE request
            app_cmd      <= `DRAM_CMD_WRITE;
            mode         <= M_WRITE;   
            app_wdf_wren <= 0;
            app_en       <= 1;
            app_addr     <= D_INITADR; // param, initial address
            remain       <= D_ELEM;    // the number of blocks to be written
          end
          else if(D_REQ==`DRAM_REQ_READ) begin ///// READ or LOAD request
            app_cmd      <= `DRAM_CMD_READ;
            mode         <= M_READ;
            app_wdf_wren <= 0;
            app_en       <= 1;
            app_addr     <= D_INITADR; // param, initial address
            remain       <= D_ELEM;    // param, the number of blocks to be read
            remain2      <= D_ELEM;    // param, the number of blocks to be read
          end
          else begin
            app_wdf_wren <= 0; 
            app_en       <= 0;
          end
        end
        //////////////////////////////////////////////////////////////////// read
        M_READ: begin
          if (app_rdy) begin // read request is accepted.
            app_addr <= (app_addr==`MEM_LAST_ADDR) ? 0 : app_addr + 8;
            remain2 <= remain2 - 1;
            if(remain2==1) app_en <= 0;
          end
          
          D_DOUTEN <= app_rd_data_valid; // dram data_out enable
          
          if (app_rd_data_valid) begin
            D_DOUT <= app_rd_data;
            caddr   <= (caddr==`MEM_LAST_ADDR) ? 0 : caddr + 8; 
            remain <= remain - 1;
            if(remain==1) begin
              mode <= M_REQ;
            end
          end
        end
        /////////////////////////////////////////////////////////////////// write
        M_WRITE: begin
          if (app_rdy && app_wdf_rdy) begin
            //                       app_wdf_data <= D_DIN;
            app_wdf_wren <= 1;
            app_addr     <= (app_addr==`MEM_LAST_ADDR) ? 0 : app_addr + 8;
            remain       <= remain - 1;
            if(remain==1) begin
              mode   <= M_REQ;
              app_en <= 0;
            end
          end
          else app_wdf_wren <= 0;
        end
      endcase
    end
  end
  ///// READ & WRITE PORT CONTROL (end)   //////////////////////////////////////
endmodule    
/**************************************************************************************************/
`default_nettype wire
