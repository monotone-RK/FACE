/******************************************************************************/
/* LCD Controller                                      monotone-RK 2014.12.01 */
/******************************************************************************/
`default_nettype none

`include "define.v"  

module LCDCON #(parameter                 DIGIT = 8)
               (input  wire               CLK, 
                input  wire               RST, 
                input  wire [DIGIT*4-1:0] DATA, 
                input  wire               WE, 
                output reg                TXD, 
                output reg                READY);
    
  reg [(DIGIT+1)*10-1:0] cmd;
  reg [11:0]             waitnum;
  reg [(DIGIT+3):0]      cnt;
  reg [DIGIT*4-1:0]      D;
  
  genvar i;
  generate
    wire [(DIGIT+1)*10-1:0] value;
    for (i=0; i<(DIGIT+1); i=i+1) begin: val
      if (i == DIGIT) begin
        assign value[10*(i+1)-1:10*i] = {8'h20, 2'b01};  // add space
      end else begin
        wire [7:0] data = (D[4*(DIGIT-i)-1:4*(DIGIT-(i+1))]<10) ? 8'd48+D[4*(DIGIT-i)-1:4*(DIGIT-(i+1))] : // 0 ~ 9
                                                                  8'd87+D[4*(DIGIT-i)-1:4*(DIGIT-(i+1))];  // a ~ f
        assign value[10*(i+1)-1:10*i] = {data, 2'b01};
      end
    end
  endgenerate

  always @(posedge CLK) begin
    if (RST) begin
      TXD       <= 1;
      READY     <= 1;
      cmd       <= {((DIGIT+1)*10){1'b1}};
      waitnum   <= 0;
      cnt       <= 0;
      D         <= 0;
    end else begin
      if (READY) begin
        TXD       <= 1;
        waitnum   <= 0;
        if (WE) begin
          READY <= 0;
          D     <= DATA;
          cnt   <= (DIGIT+1)*10+2;
        end
      end else if (cnt == (DIGIT+1)*10+2) begin
        cnt <= cnt - 1;
        cmd <= value;
      end else if (waitnum >= `SERIAL_WCNT) begin
        TXD       <= cmd[0];
        READY     <= (cnt == 1);
        cmd       <= {1'b1, cmd[(DIGIT+1)*10-1:1]};
        waitnum   <= 1;
        cnt       <= cnt - 1;
      end else begin
        waitnum   <= waitnum + 1;
      end
    end
  end
  
endmodule
`default_nettype wire
