/******************************************************************************/
/* Top module of sorting logic                               Ryohei Kobayashi */
/*                                                                 2015-08-01 */
/******************************************************************************/

`default_nettype none

`include "define.vh"

/***** generating a 512-bits data from 128-bits input data from a host PC *****/
/******************************************************************************/
module DATAGEN(input  wire              CLK, 
               input  wire              RST, 
               input  wire              dinen, 
               input  wire [`MERGW-1:0] din, 
               output wire              doten,
               output wire [`DRAMW-1:0] dot);
  
  reg [1:0]        buf_t_cnt; // counter for temporary register
  reg              data_valid;
  reg [`DRAMW-1:0] buf_t;
  
  assign doten = data_valid;
  assign dot   = buf_t;
  
  always @(posedge CLK) begin
    if (dinen) buf_t <= {din, buf_t[`DRAMW-1:`MERGW]};
  end
  always @(posedge CLK) begin
    if (RST) begin
      buf_t_cnt <= 0;
    end else begin
      if (dinen) buf_t_cnt <= buf_t_cnt + 1;
    end
  end
  always @(posedge CLK) data_valid <= (dinen && buf_t_cnt == 3);
  
endmodule


/******************************************************************************/
module USER_LOGIC #(parameter C_PCI_DATA_WIDTH = 128)
                    (                          input  wire                        CLK,
                                               input  wire                        RST,
                                               output wire                        CHNL_RX_CLK, 
                     (* mark_debug = "true" *) input  wire                        CHNL_RX, 
                     (* mark_debug = "true" *) output wire                        CHNL_RX_ACK, 
                     (* mark_debug = "true" *) input  wire                        CHNL_RX_LAST, 
                     (* mark_debug = "true" *) input  wire [31:0]                 CHNL_RX_LEN, 
                     (* mark_debug = "true" *) input  wire [30:0]                 CHNL_RX_OFF, 
                     (* mark_debug = "true" *) input  wire [C_PCI_DATA_WIDTH-1:0] CHNL_RX_DATA, 
                     (* mark_debug = "true" *) input  wire                        CHNL_RX_DATA_VALID, 
                     (* mark_debug = "true" *) output wire                        CHNL_RX_DATA_REN,
	
                                               output wire                        CHNL_TX_CLK, 
                     (* mark_debug = "true" *) output wire                        CHNL_TX, 
                     (* mark_debug = "true" *) input  wire                        CHNL_TX_ACK, 
                     (* mark_debug = "true" *) output wire                        CHNL_TX_LAST, 
                     (* mark_debug = "true" *) output wire [31:0]                 CHNL_TX_LEN, 
                     (* mark_debug = "true" *) output wire [30:0]                 CHNL_TX_OFF, 
                     (* mark_debug = "true" *) output wire [C_PCI_DATA_WIDTH-1:0] CHNL_TX_DATA, 
                     (* mark_debug = "true" *) output wire                        CHNL_TX_DATA_VALID, 
                     (* mark_debug = "true" *) input  wire                        CHNL_TX_DATA_REN,
                                               
                                               input  wire                        d_busy,       // DRAM busy
                                               output wire [512-1:0]              d_din,        // DRAM data in
                                               input  wire                        d_w,          // DRAM write flag
                                               input  wire [512-1:0]              d_dout,       // DRAM data out
                                               input  wire                        d_douten,     // DRAM data out enable
                                               output wire  [1:0]                 d_req,        // DRAM REQ access request (read/write)
                                               output wire  [31:0]                d_initadr,    // DRAM REQ initial address for the access
                                               output wire  [31:0]                d_blocks      // DRAM REQ the number of blocks per one access
                                               );

  function [`DRAMW-1:0] mux;
    input [`DRAMW-1:0] a;
    input [`DRAMW-1:0] b;
    input              sel;
    begin
      case (sel)
        1'b0: mux = a;
        1'b1: mux = b;
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

                            reg [31:0]        rLen;
  (* mark_debug = "true" *) reg [31:0]        rCount;
  (* mark_debug = "true" *) reg [1:0]         rState;
                            reg [31:0]        tLen;
  (* mark_debug = "true" *) reg [31:0]        tCount;
  (* mark_debug = "true" *) reg [1:0]         tState;

  wire [`DRAMW-1:0]           idata;        // initdata
  wire                        idata_valid;  // initdata is valid
  wire                        rx_wait;      // wait flag for PCIe
  wire [C_PCI_DATA_WIDTH-1:0] core_dot;     // sorting result
  wire                        core_rdy;     // sorting result is ready
  
  DATAGEN datagen(CLK, RST, (CHNL_RX_DATA_REN && CHNL_RX_DATA_VALID), CHNL_RX_DATA, idata_valid, idata);
  CORE core(CLK, RST, 
            d_busy, d_din, d_w, d_dout, d_douten, d_req, d_initadr, d_blocks,  // DRAM interface
            idata, idata_valid, rx_wait,                                       // Interface for Host -> FPGA 
            CHNL_TX_DATA_REN, CHNL_TX_DATA_VALID, core_dot, core_rdy           // Interface for FPGA -> Host
            );

  assign CHNL_RX_CLK        = CLK;
  assign CHNL_RX_ACK        = (rState == 2'd1);
  assign CHNL_RX_DATA_REN   = (rState == 2'd1 && !rx_wait);

  assign CHNL_TX_CLK        = CLK;
  assign CHNL_TX            = (tState == 2'd1);
  assign CHNL_TX_LAST       = 1'd1;
  assign CHNL_TX_LEN        = tLen; // in words
  assign CHNL_TX_OFF        = 0;
  assign CHNL_TX_DATA       = core_dot;
  assign CHNL_TX_DATA_VALID = (tState == 2'd1 && core_rdy);
  
  // State machine for Host -> FPGA
  always @(posedge CLK) begin
    if (RST) begin
      rLen   <= 0;
      rCount <= 0;
      rState <= 0;
    end	else begin
      case (rState)
        2'd0: begin // Wait for start of RX, save length
          if (CHNL_RX) begin
            rLen   <= CHNL_RX_LEN;
            rCount <= 0;
            rState <= 2'd1;
          end
        end
        2'd1: begin // Wait for last data in RX, save value
          if (CHNL_RX_DATA_REN && CHNL_RX_DATA_VALID) rCount <= rCount + (C_PCI_DATA_WIDTH >> 5);
          if (rCount >= rLen)                         rState <= 2'd0;
        end
      endcase
    end
  end

  // State machine for FPGA -> Host
  always @(posedge CLK) begin
    if (RST) begin
      tLen   <= 0;
      tCount <= 0;
      tState <= 0;
    end else begin
      case (tState)
        2'd0: begin // Prepare for TX
          if (core_rdy) begin
            tLen   <= rLen;
            tCount <= 0;
            tState <= 2'd1;
          end
        end
        2'd1: begin // Start TX with save length and data value
          if (CHNL_TX_DATA_REN && CHNL_TX_DATA_VALID) tCount <= tCount + (C_PCI_DATA_WIDTH >> 5);
          if (tCount >= tLen)                         tState <= 2'd0;
        end
      endcase
    end
  end
  
endmodule
`default_nettype wire
