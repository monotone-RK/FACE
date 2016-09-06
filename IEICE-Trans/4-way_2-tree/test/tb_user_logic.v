/******************************************************************************/
/* Test Bench for FPGA Sort on VC707                         Ryohei Kobayashi */
/*                                                                 2016-08-01 */
/******************************************************************************/
`default_nettype none

`include "define.vh"
`include "user_logic.v"
`include "sorter.v"

/******************************************************************************/
module tb_USER_LOGIC();
  reg CLK, RST;

  wire           chnl_rx_clk;
  wire           chnl_rx;
  wire           chnl_rx_ack;
  wire           chnl_rx_last; 
  wire [31:0]    chnl_rx_len;
  wire [30:0]    chnl_rx_off;
  wire [128-1:0] chnl_rx_data;
  wire           chnl_rx_data_valid; 
  wire           chnl_rx_data_ren;   
	       
  wire           chnl_tx_clk;
  wire           chnl_tx;
  wire           chnl_tx_ack; 
  wire           chnl_tx_last;
  wire [31:0]    chnl_tx_len;        
  wire [30:0]    chnl_tx_off;        
  wire [128-1:0] chnl_tx_data;
  wire           chnl_tx_data_vaild;
  wire           chnl_tx_data_ren = 1;   


  wire              d_busy;
  wire              d_w;
  wire [`DRAMW-1:0] d_din;
  wire [`DRAMW-1:0] d_dout;
  wire              d_douten;
  wire [1:0]        d_req;       // DRAM access request (read/write)
  wire [31:0]       d_initadr;   // dram initial address for the access
  wire [31:0]       d_blocks;    // the number of blocks per one access(read/write)
  
  reg sortdone;

  initial begin CLK=0; forever #50 CLK=~CLK; end
  initial begin RST=1; #400 RST=0; end
  
  reg [31:0] cnt;
  always @(posedge CLK) cnt <= (RST) ? 0 : cnt + 1;

  reg [31:0] cnt0, cnt1, cnt2, cnt3, cnt4, cnt5, cnt6, cnt7, cnt8, cnt9;
  always @(posedge CLK) cnt0 <= (RST) ? 0 : (u.core.phase_a==0) ? cnt0 + 1 : cnt0;
  always @(posedge CLK) cnt1 <= (RST) ? 0 : (u.core.phase_a==1) ? cnt1 + 1 : cnt1;
  always @(posedge CLK) cnt2 <= (RST) ? 0 : (u.core.phase_a==2) ? cnt2 + 1 : cnt2;
  always @(posedge CLK) cnt3 <= (RST) ? 0 : (u.core.phase_a==3) ? cnt3 + 1 : cnt3;
  always @(posedge CLK) cnt4 <= (RST) ? 0 : (u.core.phase_a==4) ? cnt4 + 1 : cnt4;
  always @(posedge CLK) cnt5 <= (RST) ? 0 : (u.core.phase_a==5) ? cnt5 + 1 : cnt5;
  always @(posedge CLK) cnt6 <= (RST) ? 0 : (u.core.phase_a==6) ? cnt6 + 1 : cnt6;    
  always @(posedge CLK) cnt7 <= (RST) ? 0 : (u.core.phase_a==7) ? cnt7 + 1 : cnt7;    
  always @(posedge CLK) cnt8 <= (RST) ? 0 : (u.core.phase_a==8) ? cnt8 + 1 : cnt8;    
  always @(posedge CLK) cnt9 <= (RST) ? 0 : (u.core.phase_a==9) ? cnt9 + 1 : cnt9;    

  reg [31:0] rslt_cnt;
  always @(posedge CLK) begin
    if (RST) begin
      rslt_cnt <= 0;
    end else begin
      if (chnl_tx_data_vaild) rslt_cnt <= rslt_cnt + 4;
    end
  end
  
  always @(posedge CLK) begin
    if      (RST)                   sortdone <= 0;
    else if (rslt_cnt == `SORT_ELM) sortdone <= 1;
  end

  // Debug Info
  always @(posedge CLK) begin 
    if (!RST) begin
      $write("%d|%d|P%d|%d%d%d|%d", cnt[19:0], u.core.elem_a, u.core.phase_a[2:0], u.core.iter_done_a, u.core.pchange_a, u.core.irst_a, u.core.ecnt_a);
      $write("|");
      if (d_douten) $write("%08x %08x ", d_dout[63:32], d_dout[31:0]); else $write("                  ");
      // $write("%d %d %x ", u.rState, u.rx_wait, u.core.req_pzero);
      // if (u.idata_valid) $write("%08x %08x ", u.idata[63:32], u.idata[31:0]); else $write("                  ");
      // $write("|");
      // if (u.core.doen_t) $write("%08x %08x ", u.core.dout_t[63:32], u.core.dout_t[31:0]); else $write("                  ");
      // $write("|");
      // if (u.core.doen_tc) $write("%08x %08x ", u.core.dout_tc[63:32], u.core.dout_tc[31:0]); else $write("                  ");
      $write("|");
      $write("(%d)", u.core.state);
      
      ///////////////// can be parameterized
      $write("| %d %d %d %d| %d %d %d %d|", 
             u.core.im00_a.imf.cnt, u.core.im01_a.imf.cnt, u.core.im02_a.imf.cnt, u.core.im03_a.imf.cnt,
             u.core.im00_b.imf.cnt, u.core.im01_b.imf.cnt, u.core.im02_b.imf.cnt, u.core.im03_b.imf.cnt);
      
      $write(" ");
      if (u.core.F01_deq_a) $write("%08x %08x %08x %08x ", u.core.F01_dot_a[127:96], u.core.F01_dot_a[95:64], u.core.F01_dot_a[63:32], u.core.F01_dot_a[31:0]); else $write("                                    ");
      if (u.core.F01_deq_b) $write("%08x %08x %08x %08x ", u.core.F01_dot_b[127:96], u.core.F01_dot_b[95:64], u.core.F01_dot_b[63:32], u.core.F01_dot_b[31:0]); else $write("                                    ");
      // $write("| ");
      // $write("%d", u.core.dcnt);
      if (d.app_wdf_wren) $write(" |M%d %d ", d_din[63:32], d_din[31:0]);
      $write("\n");
      $fflush();
    end
  end
  
  // checking the result
  generate
    if (`INITTYPE=="sorted" || `INITTYPE=="reverse") begin
      reg [`MERGW-1:0] check_cnt;
      always @(posedge CLK) begin
        if (RST) begin 
          check_cnt[31 : 0] <= 1; 
          check_cnt[63 :32] <= 2; 
          check_cnt[95 :64] <= 3; 
          check_cnt[127:96] <= 4; 
        end else begin
          if (chnl_tx_data_vaild) begin
            if (check_cnt != chnl_tx_data) begin
              $write("Error in sorter.v: %d %d\n", chnl_tx_data, check_cnt); // for simulation
              $finish();                                                     // for simulation
            end
            check_cnt[31 : 0] <= check_cnt[31 : 0] + 4; 
            check_cnt[63 :32] <= check_cnt[63 :32] + 4; 
            check_cnt[95 :64] <= check_cnt[95 :64] + 4; 
            check_cnt[127:96] <= check_cnt[127:96] + 4; 
          end
        end
      end
    end else if (`INITTYPE=="xorshift") begin
      integer fp;
      initial begin fp = $fopen("log.txt", "w"); end
      always @(posedge CLK) begin
        if (chnl_tx_data_vaild) begin
          $fwrite(fp, "%08x\n", chnl_tx_data[31:0]);
          $fwrite(fp, "%08x\n", chnl_tx_data[63:32]);
          $fwrite(fp, "%08x\n", chnl_tx_data[95:64]);
          $fwrite(fp, "%08x\n", chnl_tx_data[127:96]);
          $fflush();
        end
        if (sortdone) $fclose(fp);
      end
    end else begin
      always @(posedge CLK) begin
        $write("Error! INITTYPE is wrong.\n");  
        $write("Please make sure src/define.vh\n");  
        $finish();                                 
      end
    end
  endgenerate
      
  // Show the elapsed cycles
  always @(posedge CLK) begin
    if(sortdone) begin : simulation_finish
      $write("\nIt takes %d cycles\n", cnt);
      $write("phase0:  %d cycles\n", cnt0);
      $write("phase1:  %d cycles\n", cnt1);
      $write("phase2:  %d cycles\n", cnt2);
      $write("phase3:  %d cycles\n", cnt3);
      $write("phase4:  %d cycles\n", cnt4);
      $write("phase5:  %d cycles\n", cnt5);
      $write("phase6:  %d cycles\n", cnt6);
      $write("phase7:  %d cycles\n", cnt7);
      $write("phase8:  %d cycles\n", cnt8);
      $write("phase9:  %d cycles\n", cnt9);            
      $write("Sorting finished!\n");
      $finish();
    end
  end
  
  // Stub modules
  /**********************************************************************************************/
  Host_to_FPGA h2f(CLK, RST, chnl_rx_data_ren, chnl_rx, chnl_rx_data, chnl_rx_data_valid, chnl_rx_len);
  DRAM d(CLK, RST, d_req, d_initadr, d_blocks, d_din, d_w, d_dout, d_douten, d_busy);

  /***** Core Module Instantiation                                                          *****/
  /**********************************************************************************************/
  USER_LOGIC u(CLK, 
               RST, 
               chnl_rx_clk,        
               chnl_rx,            
               chnl_rx_ack,        
               chnl_rx_last,       
               chnl_rx_len,        
               chnl_rx_off,        
               chnl_rx_data,       
               chnl_rx_data_valid, 
               chnl_rx_data_ren,   
	       
               chnl_tx_clk,        
               chnl_tx,            
               chnl_tx_ack,        
               chnl_tx_last,       
               chnl_tx_len,        
               chnl_tx_off,        
               chnl_tx_data,       
               chnl_tx_data_vaild, 
               chnl_tx_data_ren,   
               
               d_busy,       // DRAM busy
               d_din,        // DRAM data in
               d_w,          // DRAM write flag
               d_dout,       // DRAM data out
               d_douten,     // DRAM data out enable
               d_req,        // DRAM REQ access request (read/write)
               d_initadr,    // DRAM REQ initial address for the access
               d_blocks      // DRAM REQ the number of blocks per one access
               );

endmodule
/**************************************************************************************************/

/***** Xorshift                                                                               *****/
/**************************************************************************************************/
module XORSHIFT #(parameter               WIDTH = 32,
                  parameter               SEED  = 1)
                 (input  wire             CLK,
                  input  wire             RST,
                  input  wire             EN,
                  output wire [WIDTH-1:0] RAND_VAL);

  reg  [WIDTH-1:0] x;
  reg  [WIDTH-1:0] y;
  reg  [WIDTH-1:0] z;
  reg  [WIDTH-1:0] w;
  wire [WIDTH-1:0] t = x^(x<<11);
  
  // Mask MSB for not generating the maximum value
  assign RAND_VAL = {1'b0, w[WIDTH-2:0]};
  
  reg ocen;
  always @(posedge CLK) ocen <= RST;

  always @(posedge CLK) begin
    if (RST) begin
      x <= 123456789;
      y <= 362436069;
      z <= 521288629;
      w <= 88675123 ^ SEED;
    end else begin
      if (EN || ocen) begin
        x <= y;
        y <= z;
        z <= w;
        w <= (w^(w>>19))^(t^(t>>8));
      end
    end
  end
endmodule


/**************************************************************************************************/
module Host_to_FPGA(input  wire              CLK,
                    input  wire              RST,
                    input  wire              ren,
                    output reg               chnl_rx,
                    output wire [`MERGW-1:0] dot,
                    output wire              doten,
                    output wire [31:0]       length);

  reg rst_buf;  always @(posedge CLK) rst_buf <= RST;
  
  wire              enq;
  wire              deq;
  wire [`MERGW-1:0] din;
  wire              emp;
  wire              ful;
  wire [4:0]        cnt;
  
  reg [`SORTW-1:0]  i_d,i_c,i_b,i_a;
  reg               onetime;
  reg [31:0]        enqcnt;
  reg               enqstop;
  
  wire [`SORTW-1:0] r15,r14,r13,r12,r11,r10,r09,r08,r07,r06,r05,r04,r03,r02,r01,r00;
  reg  [1:0]        selector;
  wire [`MERGW-1:0] din_xorshift = (selector == 0) ? {r03,r02,r01,r00} :
                                   (selector == 1) ? {r07,r06,r05,r04} :
                                   (selector == 2) ? {r11,r10,r09,r08} :
                                   (selector == 3) ? {r15,r14,r13,r12} : 0;
      
  SRL_FIFO #(4, `MERGW) fifo(CLK, rst_buf, enq, deq, din, dot, emp, ful, cnt);
  
  assign enq    = (!enqstop && !ful);
  assign deq    = (ren && !emp);
  assign din    = (`INITTYPE=="xorshift") ? din_xorshift : {i_d,i_c,i_b,i_a};
  assign doten  = deq;
  assign length = `SORT_ELM;
  
  always @(posedge CLK) begin
    if (rst_buf) begin
      chnl_rx <= 0;
      onetime <= 1;
    end else begin
      chnl_rx <= onetime;
      onetime <= 0;
    end
  end

  always @(posedge CLK) begin
    if      (rst_buf) enqcnt <= 0;
    else if (enq)     enqcnt <= enqcnt + 4;
  end
  
  always @(posedge CLK) begin
    if      (rst_buf)                        enqstop <= 0;
    else if (enq && (enqcnt == `SORT_ELM-4)) enqstop <= 1;
  end
  
  always @(posedge CLK) begin
    if      (rst_buf) selector <= 0;
    else if (enq)     selector <= selector + 1;
  end
  
  generate
    if (`INITTYPE=="sorted") begin
      always @(posedge CLK) begin
        if (rst_buf) begin
          i_a <= 1;   
          i_b <= 2;
          i_c <= 3;
          i_d <= 4;
        end else begin
          if (enq) begin
            i_a <= i_a+4;
            i_b <= i_b+4;
            i_c <= i_c+4;
            i_d <= i_d+4;
          end
        end
      end
    end else if (`INITTYPE=="reverse") begin
      always @(posedge CLK) begin
        if (rst_buf) begin
          i_a <= `SORT_ELM;   
          i_b <= `SORT_ELM-1;
          i_c <= `SORT_ELM-2;
          i_d <= `SORT_ELM-3;
        end else begin
          if (enq) begin
            i_a <= i_a-4;
            i_b <= i_b-4;
            i_c <= i_c-4;
            i_d <= i_d-4;
          end
        end
      end
    end else if (`INITTYPE=="xorshift") begin
      XORSHIFT #(`SORTW, 32'h00000001) xorshift00(CLK, RST, (enq && selector == 0), r00);
      XORSHIFT #(`SORTW, 32'h00000002) xorshift01(CLK, RST, (enq && selector == 0), r01);
      XORSHIFT #(`SORTW, 32'h00000004) xorshift02(CLK, RST, (enq && selector == 0), r02);
      XORSHIFT #(`SORTW, 32'h00000008) xorshift03(CLK, RST, (enq && selector == 0), r03);
      XORSHIFT #(`SORTW, 32'h00000010) xorshift04(CLK, RST, (enq && selector == 1), r04);
      XORSHIFT #(`SORTW, 32'h00000020) xorshift05(CLK, RST, (enq && selector == 1), r05);
      XORSHIFT #(`SORTW, 32'h00000040) xorshift06(CLK, RST, (enq && selector == 1), r06);
      XORSHIFT #(`SORTW, 32'h00000080) xorshift07(CLK, RST, (enq && selector == 1), r07);
      XORSHIFT #(`SORTW, 32'h00000100) xorshift08(CLK, RST, (enq && selector == 2), r08);
      XORSHIFT #(`SORTW, 32'h00000200) xorshift09(CLK, RST, (enq && selector == 2), r09);
      XORSHIFT #(`SORTW, 32'h00000400) xorshift10(CLK, RST, (enq && selector == 2), r10);
      XORSHIFT #(`SORTW, 32'h00000800) xorshift11(CLK, RST, (enq && selector == 2), r11);
      XORSHIFT #(`SORTW, 32'h00001000) xorshift12(CLK, RST, (enq && selector == 3), r12);
      XORSHIFT #(`SORTW, 32'h00002000) xorshift13(CLK, RST, (enq && selector == 3), r13);
      XORSHIFT #(`SORTW, 32'h00004000) xorshift14(CLK, RST, (enq && selector == 3), r14);
      XORSHIFT #(`SORTW, 32'h00008000) xorshift15(CLK, RST, (enq && selector == 3), r15);
    end
  endgenerate
  
endmodule


/**************************************************************************************************/
module DRAM(input  wire              CLK,       //
            input  wire              RST,       //
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
