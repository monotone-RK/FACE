/******************************************************************************/
/* PCIe test module                               Ryohei Kobayashi 2015.07.12 */
/******************************************************************************/

`default_nettype none

/***** An SRL-based FIFO                                                  *****/
/******************************************************************************/
module SRL_FIFO #(parameter                    FIFO_SIZE  = 4,   // size in log scale, 4 for 16 entry
                  parameter                    FIFO_WIDTH = 64)  // fifo width in bit
                 (input  wire                  CLK,
                  input  wire                  RST,
                  input  wire                  enq,
                  input  wire                  deq,
                  input  wire [FIFO_WIDTH-1:0] din,
                  output wire [FIFO_WIDTH-1:0] dot,
                  output wire                  emp,
                  output wire                  full,
                  output reg  [FIFO_SIZE:0]    cnt);

  reg  [FIFO_SIZE-1:0]  head;
  reg  [FIFO_WIDTH-1:0] mem [(1<<FIFO_SIZE)-1:0];
  
  assign emp  = (cnt==0);
  assign full = (cnt==(1<<FIFO_SIZE));
  assign dot  = mem[head];
    
  always @(posedge CLK) begin
    if (RST) begin
      cnt  <= 0;
      head <= {(FIFO_SIZE){1'b1}};
    end else begin
      case ({enq, deq})
        2'b01: begin cnt <= cnt - 1; head <= head - 1; end
        2'b10: begin cnt <= cnt + 1; head <= head + 1; end
      endcase
    end
  end

  integer i;
  always @(posedge CLK) begin
    if (enq) begin
      mem[0] <= din;
      for (i=1; i<(1<<FIFO_SIZE); i=i+1) mem[i] <= mem[i-1];
    end
  end
  
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
                     (* mark_debug = "true" *) input  wire                        CHNL_TX_DATA_REN);


  //                           reg [31:0]                  rLen;
  // (* mark_debug = "true" *) reg [31:0]                  rCount;
  // (* mark_debug = "true" *) reg [1:0]                   rState;
  //                           reg [31:0]                  tLen;
  // (* mark_debug = "true" *) reg [31:0]                  tCount;
  // (* mark_debug = "true" *) reg [1:0]                   tState;

  //                           wire [C_PCI_DATA_WIDTH-1:0] fifo_dot;
  // (* mark_debug = "true" *) wire                        fifo_emp;
  // (* mark_debug = "true" *) wire                        fifo_ful;
  // (* mark_debug = "true" *) wire [4:0]                  fifo_cnt;
  // SRL_FIFO #(4, C_PCI_DATA_WIDTH) fifo(CLK, 
  //                                      RST, 
  //                                      (CHNL_RX_DATA_REN && CHNL_RX_DATA_VALID), 
  //                                      (CHNL_TX_DATA_REN && CHNL_TX_DATA_VALID), 
  //                                      CHNL_RX_DATA, 
  //                                      fifo_dot, 
  //                                      fifo_emp, 
  //                                      fifo_ful, 
  //                                      fifo_cnt);

  // assign CHNL_RX_CLK        = CLK;
  // assign CHNL_RX_ACK        = (rState == 2'd1);
  // assign CHNL_RX_DATA_REN   = (rState == 2'd1 && !fifo_ful);

  // assign CHNL_TX_CLK        = CLK;
  // assign CHNL_TX            = (tState == 2'd1);
  // assign CHNL_TX_LAST       = 1'd1;
  // assign CHNL_TX_LEN        = tLen; // in words
  // assign CHNL_TX_OFF        = 0;
  // assign CHNL_TX_DATA       = fifo_dot;
  // assign CHNL_TX_DATA_VALID = (tState == 2'd1 && !fifo_emp);
  
  // always @(posedge CLK) begin
  //   if (RST) begin
  //     rLen   <= 0;
  //     rCount <= 0;
  //     rState <= 0;
  //   end	else begin
  //     case (rState)
  //       2'd0: begin // Wait for start of RX, save length
  //         if (CHNL_RX) begin
  //           rLen   <= CHNL_RX_LEN;
  //           rCount <= 0;
  //           rState <= 2'd1;
  //         end
  //       end
  //       2'd1: begin // Wait for last data in RX, save value
  //         if (CHNL_RX_DATA_REN && CHNL_RX_DATA_VALID) rCount <= rCount + (C_PCI_DATA_WIDTH >> 5);
  //         if (rCount >= rLen)                         rState <= 2'd0;
  //       end
  //     endcase
  //   end
  // end

  // always @(posedge CLK) begin
  //   if (RST) begin
  //     tLen   <= 0;
  //     tCount <= 0;
  //     tState <= 0;
  //   end else begin
  //     case (tState)
  //       2'd0: begin // Prepare for TX
  //         if (rState == 2'd1) begin
  //           tLen   <= rLen;
  //           tCount <= 0;
  //           tState <= 2'd1;
  //         end
  //       end
  //       2'd1: begin // Start TX with save length and data value
  //         if (CHNL_TX_DATA_REN && CHNL_TX_DATA_VALID) tCount <= tCount + (C_PCI_DATA_WIDTH >> 5);
  //         if (tCount >= tLen)                         tState <= 2'd0;
  //       end
  //     endcase
  //   end
  // end
  
                            reg [C_PCI_DATA_WIDTH-1:0] rData;
                            reg [31:0]                 rLen;
  (* mark_debug = "true" *) reg [31:0]                 rCount;
  (* mark_debug = "true" *) reg [1:0]                  rState;

  assign CHNL_RX_CLK        = CLK;
  assign CHNL_RX_ACK        = (rState == 2'd1);
  assign CHNL_RX_DATA_REN   = (rState == 2'd1);

  assign CHNL_TX_CLK        = CLK;
  assign CHNL_TX            = (rState == 2'd3);
  assign CHNL_TX_LAST       = 1'd1;
  assign CHNL_TX_LEN        = rLen; // in words
  assign CHNL_TX_OFF        = 0;
  assign CHNL_TX_DATA       = rData;
  assign CHNL_TX_DATA_VALID = (rState == 2'd3);
  
  always @(posedge CLK) begin
    if (RST) begin
      rLen   <= 0;
      rCount <= 0;
      rState <= 0;
      rData  <= 0;
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
          if (CHNL_RX_DATA_VALID) begin
            rData  <= CHNL_RX_DATA;
            rCount <= rCount + (C_PCI_DATA_WIDTH/32);
          end
          if (rCount >= rLen) rState <= 2'd2;
        end
        2'd2: begin // Prepare for TX
          rCount <= (C_PCI_DATA_WIDTH/32);
          rState <= 2'd3;
        end
	2'd3: begin // Start TX with save length and data value
          if (CHNL_TX_DATA_REN & CHNL_TX_DATA_VALID) begin
            rData  <= {rCount+32'd4, rCount+32'd3, rCount+32'd2, rCount+32'd1};
            rCount <= rCount + (C_PCI_DATA_WIDTH/32);
            if (rCount >= rLen) rState <= 2'd0;
          end
        end
      endcase
    end
  end
  
endmodule
`default_nettype wire
