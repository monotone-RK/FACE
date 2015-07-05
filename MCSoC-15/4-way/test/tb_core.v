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

    reg [31:0] cnt0, cnt1, cnt2, cnt3, cnt4, cnt5, cnt6, cnt7, cnt8, cnt9;
    always @(posedge CLK) cnt0 <= (RST) ? 0 : (c.phase==0 && c.initdone) ? cnt0 + 1 : cnt0;
    always @(posedge CLK) cnt1 <= (RST) ? 0 : (c.phase==1 && c.initdone) ? cnt1 + 1 : cnt1;
    always @(posedge CLK) cnt2 <= (RST) ? 0 : (c.phase==2 && c.initdone) ? cnt2 + 1 : cnt2;
    always @(posedge CLK) cnt3 <= (RST) ? 0 : (c.phase==3 && c.initdone) ? cnt3 + 1 : cnt3;
    always @(posedge CLK) cnt4 <= (RST) ? 0 : (c.phase==4 && c.initdone) ? cnt4 + 1 : cnt4;
    always @(posedge CLK) cnt5 <= (RST) ? 0 : (c.phase==5 && c.initdone) ? cnt5 + 1 : cnt5;
    always @(posedge CLK) cnt6 <= (RST) ? 0 : (c.phase==6 && c.initdone) ? cnt6 + 1 : cnt6;    
    always @(posedge CLK) cnt7 <= (RST) ? 0 : (c.phase==7 && c.initdone) ? cnt7 + 1 : cnt7;    
    always @(posedge CLK) cnt8 <= (RST) ? 0 : (c.phase==8 && c.initdone) ? cnt8 + 1 : cnt8;    
    always @(posedge CLK) cnt9 <= (RST) ? 0 : (c.phase==9 && c.initdone) ? cnt9 + 1 : cnt9;    


  generate
    if (`INITTYPE=="reverse" || `INITTYPE=="sorted") begin
      always @(posedge CLK) begin /// note
        if (c.initdone) begin
            
          $write("%d|%d|P%d|%d%d%d|%d", cnt[19:0], c.elem, c.phase[2:0], c.iter_done, c.pchange, c.irst, c.ecnt);
          
          
          $write("%d %d (%d) : ", d_dout[63:32], d_dout[31:0], d_douten);
          $write("[%d](%d)", c.req, c.state);
          
          $write("|");
          if(c.stree.F04_emp) $write("---------- "); else  $write("%d ", c.stree.F04_dot);
          if(c.stree.F05_emp) $write("---------- "); else  $write("%d ", c.stree.F05_dot);
          if(c.stree.F06_emp) $write("---------- "); else  $write("%d ", c.stree.F06_dot);
          if(c.stree.F07_emp) $write("---------- "); else  $write("%d ", c.stree.F07_dot);
          $write("|");            
          
          $write("| %d %d %d %d|", c.im00.imf.cnt, c.im01.imf.cnt, c.im02.imf.cnt, c.im03.imf.cnt);
          $write("| %d %d %d %d|", c.im00.im_deq, c.im01.im_deq, c.im02.im_deq, c.im03.im_deq);
          if(c.F01_deq) $write("%d", c.F01_dot); else $write("          ");
          if(d_w) $write(" |M%d %d ", d_din[63:32], d_din[31:0]);
          $write("\n");
          $fflush();
        end
      end
    
      always @(posedge CLK) begin
        if(c.sortdone) begin : simulation_finish
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
    end else if (`INITTYPE == "xorshift") begin
      integer fp;
      initial begin
        fp = $fopen("test.txt", "w");
      end
      always @(posedge CLK) begin /// note
        if (c.phase==`LAST_PHASE && c.F01_deq) begin
          $write("%08x ", c.F01_dot);
          $fwrite(fp, "%08x ", c.F01_dot);
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
        else if(d.app_wdf_wren) mem[dram_addr[27:3]] <= app_wdf_data;
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
