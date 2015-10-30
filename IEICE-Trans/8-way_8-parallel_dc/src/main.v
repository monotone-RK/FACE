/******************************************************************************/
/* Main Module for FPGA Sorting with VC707                ArchLab. TOKYO TECH */
/******************************************************************************/
`default_nettype none

`include "define.v"
  
module main(input  wire              CLK_P,
            input  wire              CLK_N,
            input  wire              RST_X_IN,
            output wire [7:0]        ULED,
            output wire              TXD,
            inout  wire [`DDR3_DATA] DDR3DQ,
            inout  wire [7:0]        DDR3DQS_N,
            inout  wire [7:0]        DDR3DQS_P,
            output wire [`DDR3_ADDR] DDR3ADDR,
            output wire [2:0]        DDR3BA,
            output wire              DDR3RAS_N,
            output wire              DDR3CAS_N,
            output wire              DDR3WE_N,
            output wire              DDR3RESET_N,
            output wire [0:0]        DDR3CK_P,
            output wire [0:0]        DDR3CK_N,
            output wire [0:0]        DDR3CKE,
            output wire [0:0]        DDR3CS_N,
            output wire [7:0]        DDR3DM,
            output wire [0:0]        DDR3ODT);    

  function mux1;
    input a;
    input b;
    input sel;
    begin
      case (sel)
        1'b0: mux1 = a;
        1'b1: mux1 = b;
      endcase
    end
  endfunction
  
  function [1:0] mux2;
    input [1:0] a;
    input [1:0] b;
    input       sel;
    begin
      case (sel)
        1'b0: mux2 = a;
        1'b1: mux2 = b;
      endcase
    end
  endfunction
  
  function [32-1:0] mux32;
    input [32-1:0] a;
    input [32-1:0] b;
    input          sel;
    begin
      case (sel)
        1'b0: mux32 = a;
        1'b1: mux32 = b;
      endcase
    end
  endfunction
  
  // Generated Clock and Reset from DRAMCON
  // ###########################################################################
  wire CLK100M;
  wire RST_O;
  wire CLK = CLK100M;
  wire RST = RST_O;
  
  // registers for performance counter logic
  // ###########################################################################
  reg [31:0] pcnt;
  reg        pcnt_halt;
  
  // Outputs from Core Module
  // ###########################################################################
  wire        sortdone;
  wire        initdone;
  wire [1:0]  core_req;      
  wire [31:0] core_initadr;  
  wire [31:0] core_blocks;
  wire        error;

  // (For Verification)
  //  Registers and Wires for State Machine of DRAM Read
  // ###########################################################################
  localparam WAIT      = 0;
  localparam READJUDGE = 1;
  localparam READREQ   = 2;
  localparam READING   = 3;

  wire [`PHASE_W]           last_phase = `LAST_PHASE;
  reg  [1:0]                state;
  reg  [`APPADDR_WIDTH-1:0] v_raddr;
  reg  [`APPADDR_WIDTH-1:0] v_cnt;
  reg                       v_c;
  reg  [1:0]                v_dreq;   
  reg  [31:0]               v_dblocks;
  reg  [`APPADDR_WIDTH-1:0] v_dinitadr;

  // Wires of DRAM Controller
  // ###########################################################################
  wire                      d_busy;
  wire [`APPDATA_WIDTH-1:0] d_din;
  wire                      d_w;
  wire [`APPDATA_WIDTH-1:0] d_dout;
  wire                      d_douten;
  wire [1:0]                d_req     = mux2(core_req, v_dreq, pcnt_halt);
  wire [31:0]               d_initadr = mux32(core_initadr, v_dinitadr, pcnt_halt);
  wire [31:0]               d_blocks  = mux32(core_blocks, v_dblocks, pcnt_halt);
  
  // Registers and Wires for FIFO Buffering Readed Data and LCD Controller
  // ###########################################################################
  wire                           dc_dinen = (d_douten && pcnt_halt);
  wire [`DRAMW-1:0]              dc_dout;
  wire [(1<<`P_LOG)+`SORT_WAY:0] dc_douten;
  wire                           dc_req;
  
  wire                      rdy;
  reg  [31:0]               lcd_cnt;
  reg  [3:0]                sfifo_scnt;
  reg                       sfifo_req;
  wire [`APPDATA_WIDTH-1:0] sfifo_dot;
  wire                      sfifo_emp, sfifo_full;
  reg                       sfifo_emp_r;
  wire                      sfifo_enq = dc_douten[0];
  wire [`VR_SIZE:0]         sfifo_cnt;
  wire                      lcd_we = (!sfifo_emp_r && rdy && pcnt_halt && (lcd_cnt < `SHOWNUM));
  wire                      sfifo_deq = (lcd_we && sfifo_scnt == 15);
  reg  [`APPDATA_WIDTH-1:0] sfifo_dot_t;
  wire [31:0]               lcd_data = mux32(sfifo_dot_t[31:0], sfifo_dot[31:0], (sfifo_scnt==0));
  reg                       pcnt_lcdwe;
  reg                       pcnt_send;
  
  /* DRAM Controller Instantiation                                              */
  /******************************************************************************/
  DRAMCON dramcon(.CLK_P(CLK_P),
                  .CLK_N(CLK_N),
                  .RST_X_IN(RST_X_IN),
                  ////////// User logic interface ports //////////
                  .D_REQ(d_req),
                  .D_INITADR(d_initadr),
                  .D_ELEM(d_blocks),
                  .D_BUSY(d_busy),
                  .D_DIN(d_din),
                  .D_W(d_w),
                  .D_DOUT(d_dout),
                  .D_DOUTEN(d_douten),
                  .USERCLK(CLK100M),
                  .RST_O(RST_O),
                  ////////// Memory interface ports //////////               
                  .DDR3DQ(DDR3DQ),
                  .DDR3DQS_N(DDR3DQS_N),
                  .DDR3DQS_P(DDR3DQS_P),
                  .DDR3ADDR(DDR3ADDR),
                  .DDR3BA(DDR3BA),
                  .DDR3RAS_N(DDR3RAS_N),
                  .DDR3CAS_N(DDR3CAS_N),
                  .DDR3WE_N(DDR3WE_N),
                  .DDR3RESET_N(DDR3RESET_N),
                  .DDR3CK_P(DDR3CK_P),
                  .DDR3CK_N(DDR3CK_N),
                  .DDR3CKE(DDR3CKE),
                  .DDR3CS_N(DDR3CS_N),
                  .DDR3DM(DDR3DM),
                  .DDR3ODT(DDR3ODT));

  /* Core Module Instantiation                                                  */
  /******************************************************************************/
  CORE core(CLK100M, RST, initdone, sortdone,
            d_busy, d_din, d_w, d_dout, d_douten, core_req, core_initadr, core_blocks, error);
  
  /* Decompressor Instantiation                                                  */
  /******************************************************************************/
  DECOMPRESSOR #(`VR_SIZE, `DRAM_VBLOCKS)
  decompressor(CLK, RST, {{((1<<`P_LOG)+`SORT_WAY){1'b0}},1'b1,d_dout}, dc_dinen, 
               dc_dout, dc_douten, dc_req);

  /* FIFO Instantiation                                                         */
  /******************************************************************************/
  BFIFO #(`VR_SIZE, `DRAMW) sfifo(.CLK(CLK), .RST(RST), .enq(sfifo_enq), .deq(sfifo_deq), 
                                  .din(dc_dout), .dot(sfifo_dot), .emp(sfifo_emp), .full(sfifo_full), 
                                  .cnt(sfifo_cnt));

  /* LCD Controller Instantiation                                               */
  /******************************************************************************/
  LCDCON lcdcon(CLK, 
                RST, 
                (mux32(lcd_data, pcnt, pcnt_lcdwe)), 
                (mux1(lcd_we, 1'b1, pcnt_lcdwe)), 
                TXD, 
                rdy);

  /* User Logic                                                                 */
  /******************************************************************************/
  // Outputs of main module (assign)
  assign ULED = {heartbeat[24], 
                 mux1(error, 1'b0, (`INITTYPE=="xorshift")), 
                 initdone, 
                 4'b0, 
                 sortdone};

  /* ========== heart beat ========================================= */
  reg [24:0] heartbeat;
  always @(posedge CLK) heartbeat <= heartbeat + 1;

  /* ========== Performance Counter ================================ */
  always @(posedge CLK) begin
    if (RST || !initdone) begin
      pcnt      <= 0;
      pcnt_halt <= 0;
    end else if (!pcnt_halt) begin
      if (!d_busy && sortdone) pcnt_halt <= 1;
      else                     pcnt      <= pcnt + 1;
    end
  end
  
  /* ========== FIFO buffering Readed Data and LCD Controller ====== */
  always @(posedge CLK) begin
    if (RST) begin
      pcnt_lcdwe <= 0;
      pcnt_send  <= 0;
    end else begin
      pcnt_lcdwe <= (!pcnt_lcdwe && rdy && (lcd_cnt >= `SHOWNUM) && !pcnt_send);
      if (pcnt_lcdwe) pcnt_send <= 1;
    end
  end
  
  always @(posedge CLK) begin
    case ({lcd_we, (sfifo_scnt == 0)})
      2'b10: sfifo_dot_t <= {32'b0, sfifo_dot_t[`APPDATA_WIDTH-1:32]};
      2'b11: sfifo_dot_t <= sfifo_dot[`APPDATA_WIDTH-1:32];
    endcase
  end
  
  always @(posedge CLK) begin
    if (RST) begin
      sfifo_scnt <= 0;
      lcd_cnt    <= 0;
    end else begin
      if (lcd_we) begin
        sfifo_scnt <= sfifo_scnt + 1;
        lcd_cnt    <= lcd_cnt + 1;
      end
    end
  end

  always @(posedge CLK) sfifo_req   <= (sfifo_cnt<`VR_REQ_THRE);
  always @(posedge CLK) sfifo_emp_r <= sfifo_emp;
  
  /* ========== State Machine of DRAM Read ========================= */
  always @(posedge CLK) begin
    if (RST) begin
      state      <= WAIT;
      v_raddr    <= mux32((`SORT_ELM>>1), 0, last_phase[0]);
      v_cnt      <= 0;
      v_c        <= 0;
      v_dreq     <= 0;
      v_dblocks  <= 0;
      v_dinitadr <= 0;
    end else begin
      case (state)
        WAIT: begin  // waiting for sorting termination
          if (pcnt_halt) state <= READJUDGE;
        end
        READJUDGE: begin 
          if (!d_busy) begin
            if (dc_req && sfifo_req && !v_c) state <= READREQ;
          end
        end
        READREQ: begin
          if (v_dreq != 0) begin
            v_dreq <= 0;
            state  <= READING;
          end else if (!d_busy) begin
            v_dreq     <= `DRAM_REQ_READ;
            v_dblocks  <= `DRAM_VBLOCKS;
            v_dinitadr <= v_raddr;
            v_raddr    <= v_raddr + (`D_VS);
            v_cnt      <= v_cnt + 1;
            v_c        <= (v_cnt >= ((`SORT_ELM>>5)/`DRAM_VBLOCKS)-1);
          end
        end
        READING: begin
          if (!d_busy) state <= READJUDGE;
        end
      endcase
    end
  end

endmodule
`default_nettype wire

