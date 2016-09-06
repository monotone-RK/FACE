/******************************************************************************/
/* Test Module of DRAM Controller for VC707            monotone-RK 2016.03.23 */
/******************************************************************************/
`default_nettype none

`include "define.v"
  
/******************************************************************************/
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

/******************************************************************************/
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


  // Generated Clock and Reset from DRAMCON
  // ###########################################################################
  wire CLK_O;
  wire RST_O;
  wire CLK = CLK_O;
  wire RST = RST_O;
  
  // Wires and Registers for State Machine of DRAM Write and Read
  // ###########################################################################
  localparam WRITE  = 0;
  localparam READ   = 1;
  localparam RESULT = 2;
  
  reg [2:0]  state;
  reg [31:0] w_cnt;
  reg [31:0] r_cnt;
  reg [31:0] w_cycle;
  reg [31:0] r_cycle;
  reg [1:0]  send_cnt;
  
  // Wires and Registers of DRAM Controller
  // ###########################################################################
  wire                      d_busy;
  wire [`APPDATA_WIDTH-1:0] d_din;
  wire                      d_w;
  wire [`APPDATA_WIDTH-1:0] d_dout;
  wire                      d_douten;
  reg  [1:0]                d_req;    
  reg  [31:0]               d_initadr;
  reg  [31:0]               d_blocks;
  
  // Wire of XORSHIFT Random Value Generator
  // ###########################################################################
  wire [31:0]               rand_addr;
  
  // Wires and Registers for LCD Controller
  // ###########################################################################
  wire                      rdy;
  reg  [31:0]               lcd_data;
  reg                       lcd_we;

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
                  .USERCLK(CLK_O),
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


  /* XORSHIFT Random Value Generator Instantiation                              */
  /******************************************************************************/
  XORSHIFT xorshift(CLK, RST, !RST, rand_addr);

  /* LCD Controller Instantiation                                               */
  /******************************************************************************/
  LCDCON lcdcon(CLK, RST, lcd_data, lcd_we, TXD, rdy);

  /* User Logic                                                                 */
  /******************************************************************************/
  // Outputs of main module (assign)
  assign ULED = {heartbeat[24], 1'b0, 1'b0, 4'b0, error};

  /* ========== heart beat ========================================= */
  reg [24:0] heartbeat;
  always @(posedge CLK) heartbeat <= heartbeat + 1;

  /* ====== WRITE : feed the initial data to be stored to DRAM ===== */
  reg [511:0] dram_wdata;  // DRAM write data
  assign d_din = dram_wdata;
  always @(posedge CLK) begin
    if      (RST) dram_wdata <= 0;
    else if (d_w) dram_wdata <= {480'h0, (dram_wdata[31:0]+32'h1)};
  end
  
  /* ====== READ data and ERROR check ============================== */
  reg [511:0] dram_rdata;  // DRAM read check data
  always @(posedge CLK) begin
    if      (RST)      dram_rdata <= 0;
    else if (d_douten) dram_rdata <= {480'h0, (dram_rdata[31:0]+32'h1)};
  end

  reg error;
  // always @(posedge CLK) begin
  //   if      (RST)                            error <= 0;
  //   else if (d_douten && d_dout!=dram_rdata) error <= 1;
  // end

  /* ========== State Machine of DRAM Write and Read =============== */
  always @(posedge CLK) begin
    if (RST) begin
      state     <= WRITE;
      w_cnt     <= 0;
      r_cnt     <= 0;
      w_cycle   <= 0;
      r_cycle   <= 0;
      send_cnt  <= 0;
      lcd_data  <= 0;
      lcd_we    <= 0;
      d_req     <= 0;
      d_initadr <= 0;
      d_blocks  <= 0;
    end else begin
      case (state)
        WRITE: begin
          if (d_req != 0) begin 
            d_req   <= 0;
            w_cycle <= w_cycle + 1;
          end else if (!d_busy) begin
            if (w_cnt == `WRITE_ACCESS_CNT) begin
              state <= READ;
            end else begin
              d_req     <= `DRAM_REQ_WRITE; 
              d_blocks  <= `DRAM_WBLOCKS;  // 64M blocks = 4096MB data
              d_initadr <= {3'b0,rand_addr[28:3],3'b0};               
              w_cnt     <= w_cnt + 1;
              w_cycle   <= w_cycle + 1;
            end
          end else begin
            w_cycle <= w_cycle + 1;
          end
        end
        READ: begin
          if (d_req != 0) begin
            d_req   <= 0;
            r_cycle <= r_cycle + 1;
          end else if (!d_busy) begin
            if (r_cnt == `READ_ACCESS_CNT) begin
              state <= RESULT;
            end else begin
              d_req     <= `DRAM_REQ_READ;
              d_blocks  <= `DRAM_RBLOCKS;  // 64M blocks = 4096MB data
              d_initadr <= {3'b0,rand_addr[28:3],3'b0};
              r_cnt     <= r_cnt + 1;
              r_cycle   <= r_cycle + 1;
            end
          end else begin
            r_cycle <= r_cycle + 1;
          end
        end
        RESULT: begin
          if (!lcd_we && rdy && send_cnt < 2) begin
            lcd_we   <= 1;
            lcd_data <= (send_cnt == 0) ? w_cycle : r_cycle;
            // lcd_data <= (send_cnt == 0) ? w_cycle : ({32{error}});
            send_cnt <= send_cnt + 1;
          end else begin
            lcd_we <= 0;
          end
        end
      endcase
    end
  end

endmodule
`default_nettype wire
