`default_nettype none

module test;

  reg RCLK;
  reg WCLK;
  reg RST;

  initial begin // clock gen
    RCLK = 0; 
    forever #5 RCLK = ~RCLK; 
  end
  initial begin // clock gen
    WCLK = 0; 
    forever #10 WCLK = ~WCLK; 
  end
  initial begin // reset gen
    RST = 1; 
    #400;
    RST = 0;
  end

  reg  [31:0]     data_in;
  reg             data_ready;

  wire            write;
  wire            read;
  wire [31:0]     data_out;
  wire            full;
  wire            empty;

  assign write = !full;
  assign read  = !empty &&(cnt%10 > 5);

  always @(posedge WCLK) begin
    if      (RST) data_in <= 0;
    else if (!full)  data_in <= data_in + 1;
  end

  reg [31:0] cnt;
  always @(RCLK) begin
    if (RST) cnt <= 0;
    else        cnt <= cnt + 1;
  end
    
  always @(posedge RCLK) begin
    if (RST) data_ready <= 0;
    else        data_ready <= read;
  end
  
  async_fifo afifo(RST, WCLK, RCLK, data_in, write, read, data_out, full, empty);
  
endmodule

