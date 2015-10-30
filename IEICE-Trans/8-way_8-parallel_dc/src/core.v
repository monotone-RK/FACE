/**************************************************************************************************/
/* FPGA Sort for VC707                                                        ArchLab. TOKYO TECH */
/**************************************************************************************************/
`default_nettype none

`include "define.v"

/***** Sorter Cell                                                                            *****/
/**************************************************************************************************/
module SCELL(input  wire              valid1, 
             input  wire              valid2, 
             output wire              deq1, 
             output wire              deq2, 
             input  wire [`SORTW-1:0] din1, 
             input  wire [`SORTW-1:0] din2, 
             input  wire              full, 
             output wire [`SORTW-1:0] dout, 
             output wire              enq);
    

  wire cmp1 = (din1 < din2);

  function [`SORTW-1:0] mux;
    input [`SORTW-1:0] a;
    input [`SORTW-1:0] b;
    input              sel;
    begin
      case (sel)
        1'b0: mux = a;
        1'b1: mux = b;
      endcase
    end
  endfunction

  assign enq  = (!full && valid1 && valid2);
  assign deq1 = (enq &&  cmp1);
  assign deq2 = (enq && !cmp1);
  assign dout = mux(din2, din1, cmp1);
  
endmodule   

/***** FIFO of only two entries                                                               *****/
/**************************************************************************************************/
module MRE2 #(parameter                    FIFO_SIZE  =  1,  // dummy, just for portability
              parameter                    FIFO_WIDTH = 32)  // fifo width in bit
             (input  wire                  CLK, 
              input  wire                  RST, 
              input  wire                  enq, 
              input  wire                  deq, 
              input  wire [FIFO_WIDTH-1:0] din, 
              output wire [FIFO_WIDTH-1:0] dot, 
              output wire                  emp, 
              output wire                  full, 
              output reg [FIFO_SIZE:0]     cnt);
    
  reg head, tail;
  reg [FIFO_WIDTH-1:0] mem [(1<<FIFO_SIZE)-1:0];

  assign emp  = (cnt==0);
  assign full = (cnt==2);
  assign dot  = mem[head];

  always @(posedge CLK) begin
    if (RST) {cnt, head, tail} <= 0;
    else begin
      case ({enq, deq})
        2'b01: begin                 head<=~head;              cnt<=cnt-1; end
        2'b10: begin mem[tail]<=din;              tail<=~tail; cnt<=cnt+1; end
        2'b11: begin mem[tail]<=din; head<=~head; tail<=~tail;             end
      endcase
    end
  end
    
endmodule

/***** general FIFO (BRAM Version)                                                            *****/
/**************************************************************************************************/ // from here
module BFIFO #(parameter                    FIFO_SIZE  =  2, // size in log scale, 2 for 4 entry, 3 for 8 entry
               parameter                    FIFO_WIDTH = 32) // fifo width in bit
              (input  wire                  CLK, 
               input  wire                  RST, 
               input  wire                  enq, 
               input  wire                  deq, 
               input  wire [FIFO_WIDTH-1:0] din, 
               output reg  [FIFO_WIDTH-1:0] dot, 
               output wire                  emp, 
               output wire                  full, 
               output reg  [FIFO_SIZE:0]    cnt);
  
  reg [FIFO_SIZE-1:0]  head, tail;
  reg [FIFO_WIDTH-1:0] mem [(1<<FIFO_SIZE)-1:0];

  assign emp  = (cnt==0);
  assign full = (cnt==(1<<FIFO_SIZE));
    
  always @(posedge CLK) dot <= mem[head];
    
  always @(posedge CLK) begin
    if (RST) {cnt, head, tail} <= 0;
    else begin
      case ({enq, deq})
        2'b01: begin                 head<=head+1;               cnt<=cnt-1; end
        2'b10: begin mem[tail]<=din;               tail<=tail+1; cnt<=cnt+1; end
        2'b11: begin mem[tail]<=din; head<=head+1; tail<=tail+1;             end
      endcase
    end
  end
endmodule

/***** Input Module Pre                                                                       *****/
/**************************************************************************************************/
module INMOD2(input  wire              CLK, 
              input  wire              RST, 
              input  wire [`DRAMW-1:0] din,      // input data 
              input  wire              den,      // input data enable
              input  wire              IB_full,  // the next module is full ? 
              output wire [`SORTW-1:0] dot,      // this module's data output
              output wire              IB_enq,   // the next module's enqueue signal
              output wire              im_emp,   // BFIFO is empty
              output reg               im_req);  // DRAM data request
  
  wire req;
  reg  deq;
  wire [`DRAMW-1:0] im_dot;
  wire [`IB_SIZE:0] im_cnt;
  wire im_full;
  wire im_enq = den; // (!im_full && den); 
  wire im_deq = (req && !im_emp);
  
  always @(posedge CLK) im_req <= (im_cnt<`REQ_THRE);
  always @(posedge CLK) deq    <= im_deq;
  
  BFIFO #(`IB_SIZE, `DRAMW) // note, using BRAM
  imf(.CLK(CLK), .RST(RST), .enq(im_enq), .deq(im_deq), .din(din),
      .dot(im_dot), .emp(im_emp), .full(im_full), .cnt(im_cnt));
  
  INMOD inmod(.CLK(CLK), .RST(RST), .d_dout(im_dot), .d_douten(deq), 
              .IB_full(IB_full), .im_dot(dot), .IB_enq(IB_enq), .im_req(req));
endmodule

/***** Input Module                                                                           *****/
/**************************************************************************************************/
module INMOD(input  wire              CLK, 
             input  wire              RST, 
             input  wire [`DRAMW-1:0] d_dout,    // DRAM output
             input  wire              d_douten,  // DRAM output enable
             input  wire              IB_full,   // INBUF is full ? 
             output wire [`SORTW-1:0] im_dot,    // this module's data output
             output wire              IB_enq, 
             output wire              im_req);   // DRAM data request
  
  reg [`DRAMW-1:0] dot_t; // shift register to feed 32bit data
  reg  [3:0] cnte;        // the number of enqueued elements in one block
  reg cntez;              // cnte==0  ?
  reg cntef;              // cnte==15 ?
    
  wire [`DRAMW-1:0] dot;
  wire im_emp, im_full;
  wire im_enq = d_douten; // (!im_full && d_douten); 
  wire im_deq = (IB_enq && cntef); // old version may have a bug here!!

  function [`SORTW-1:0] mux;
    input [`SORTW-1:0] a;
    input [`SORTW-1:0] b;
    input              sel;
    begin
      case (sel)
        1'b0: mux = a;
        1'b1: mux = b;
      endcase
    end
  endfunction
  
  assign IB_enq = (!IB_full && !im_emp);              // enqueue signal for the next module
  assign im_req = (im_emp || im_deq);                 // note!!!    
  assign im_dot = mux(dot_t[31:0], dot[31:0], cntez);

  always @(posedge CLK) begin
    if (RST) begin
      cnte <= 0;
    end else begin
      if (IB_enq) cnte <= cnte + 1;
    end
  end
  always @(posedge CLK) begin
    if (RST) begin
      cntez <= 1;
    end else begin
      case ({IB_enq, (cnte==15)})
        2'b10: cntez <= 0;
        2'b11: cntez <= 1;
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RST) begin
      cntef <= 0;
    end else begin
      case ({IB_enq, (cnte==14)})
        2'b10: cntef <= 0;
        2'b11: cntef <= 1;
      endcase
    end
  end
  always @(posedge CLK) begin
    case ({IB_enq, cntez})
      2'b10: dot_t <= {32'b0, dot_t[`DRAMW-1:32]};
      2'b11: dot_t <= {32'b0, dot[`DRAMW-1:32]};
    endcase
  end

  MRE2 #(1, `DRAMW) imf(.CLK(CLK), .RST(RST), .enq(im_enq), .deq(im_deq), 
                        .din(d_dout), .dot(dot), .emp(im_emp), .full(im_full));
endmodule

/***** input buffer module                                                                    *****/
/**************************************************************************************************/
module INBUF(input  wire              CLK, 
             input  wire              RST, 
             output wire              ib_full,  // this module is full
             input  wire              full,     // next moldule's full
             output wire              enq,      // next module's enqueue
             input  wire [`SORTW-1:0] din,      // data in
             output wire [`SORTW-1:0] dot,      // data out
             input  wire              ib_enq,   // this module's enqueue
             input  wire [`PHASE_W]   phase,    // current phase
             input  wire              idone);   // iteration done, this module's enqueue
  
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
  
  function [`SORTW-1:0] mux32;
    input [`SORTW-1:0] a;
    input [`SORTW-1:0] b;
    input              sel;
    begin
      case (sel)
        1'b0: mux32 = a;
        1'b1: mux32 = b;
      endcase
    end
  endfunction
  
  /*****************************************/
  wire [`SORTW-1:0]  F_dout;
  wire        F_deq, F_emp;
  reg [31:0] ecnt;  // the number of elements in one iteration
  reg        ecntz; // ecnt==0 ?
  
  wire f_full;
  MRE2 #(1,`SORTW) F(.CLK(CLK), .RST(RST), .enq(ib_enq), .deq(F_deq),  // input buffer FIFO
                     .din(din), .dot(F_dout), .emp(F_emp), .full(f_full));

  assign ib_full = mux1(f_full, 0, F_deq); // INBUF back_pressure
  /*****************************************/
  assign enq = !full && (!F_emp || ecntz);  // enqueue for the next buffer
  assign F_deq = enq && (ecnt!=0);          //
  
  assign dot = mux32(F_dout, `MAX_VALUE, ecntz);
  
  always @(posedge CLK) begin
    if (RST || idone) begin
      ecnt  <= (`ELEMS_PER_UNIT << (phase * `WAY_LOG)); /// note
      ecntz <= 0;
    end else begin
      if (ecnt!=0 && enq) ecnt  <= ecnt - 1;
      if (ecnt==1 && enq) ecntz <= 1; // old version has a bug here!
    end
  end
endmodule

/**************************************************************************************************/
module STREE(input  wire                        CLK, 
             input  wire                        RST_in, 
             input  wire                        irst, 
             input  wire                        frst, 
             input  wire [`PHASE_W]             phase_in, 
             input  wire [`SORTW*`SORT_WAY-1:0] s_din,     // sorting-tree input data
             input  wire [`SORT_WAY-1:0]        enq,       // enqueue
             output wire [`SORT_WAY-1:0]        full,      // buffer is full ? 
             input  wire                        deq,       // dequeue
             output wire [`SORTW-1:0]           dot,       // output data 
             output wire                        emp);

  reg RST;
  always @(posedge CLK) RST <= RST_in;
    
  reg [`PHASE_W] phase;
  always @(posedge CLK) phase <= phase_in;
    
  wire [`SORTW-1:0] d00, d01, d02, d03, d04, d05, d06, d07;
  assign {d00, d01, d02, d03, d04, d05, d06, d07} = s_din;
    
  wire F01_enq, F01_deq, F01_emp, F01_full; wire [31:0] F01_din, F01_dot; wire [1:0] F01_cnt;
  wire F02_enq, F02_deq, F02_emp, F02_full; wire [31:0] F02_din, F02_dot; wire [1:0] F02_cnt;
  wire F03_enq, F03_deq, F03_emp, F03_full; wire [31:0] F03_din, F03_dot; wire [1:0] F03_cnt;
  wire F04_enq, F04_deq, F04_emp, F04_full; wire [31:0] F04_din, F04_dot; wire [1:0] F04_cnt;
  wire F05_enq, F05_deq, F05_emp, F05_full; wire [31:0] F05_din, F05_dot; wire [1:0] F05_cnt;
  wire F06_enq, F06_deq, F06_emp, F06_full; wire [31:0] F06_din, F06_dot; wire [1:0] F06_cnt;
  wire F07_enq, F07_deq, F07_emp, F07_full; wire [31:0] F07_din, F07_dot; wire [1:0] F07_cnt;
  wire F08_enq, F08_deq, F08_emp, F08_full; wire [31:0] F08_din, F08_dot; wire [1:0] F08_cnt;
  wire F09_enq, F09_deq, F09_emp, F09_full; wire [31:0] F09_din, F09_dot; wire [1:0] F09_cnt;
  wire F10_enq, F10_deq, F10_emp, F10_full; wire [31:0] F10_din, F10_dot; wire [1:0] F10_cnt;
  wire F11_enq, F11_deq, F11_emp, F11_full; wire [31:0] F11_din, F11_dot; wire [1:0] F11_cnt;
  wire F12_enq, F12_deq, F12_emp, F12_full; wire [31:0] F12_din, F12_dot; wire [1:0] F12_cnt;
  wire F13_enq, F13_deq, F13_emp, F13_full; wire [31:0] F13_din, F13_dot; wire [1:0] F13_cnt;
  wire F14_enq, F14_deq, F14_emp, F14_full; wire [31:0] F14_din, F14_dot; wire [1:0] F14_cnt;
  wire F15_enq, F15_deq, F15_emp, F15_full; wire [31:0] F15_din, F15_dot; wire [1:0] F15_cnt;

  INBUF IN08(CLK, RST, full[0], F08_full, F08_enq, d00, F08_din, enq[0], phase, irst);
  INBUF IN09(CLK, RST, full[1], F09_full, F09_enq, d01, F09_din, enq[1], phase, irst);
  INBUF IN10(CLK, RST, full[2], F10_full, F10_enq, d02, F10_din, enq[2], phase, irst);
  INBUF IN11(CLK, RST, full[3], F11_full, F11_enq, d03, F11_din, enq[3], phase, irst);
  INBUF IN12(CLK, RST, full[4], F12_full, F12_enq, d04, F12_din, enq[4], phase, irst);
  INBUF IN13(CLK, RST, full[5], F13_full, F13_enq, d05, F13_din, enq[5], phase, irst);
  INBUF IN14(CLK, RST, full[6], F14_full, F14_enq, d06, F14_din, enq[6], phase, irst);
  INBUF IN15(CLK, RST, full[7], F15_full, F15_enq, d07, F15_din, enq[7], phase, irst);

  MRE2 #(1,32) F01(CLK, frst, F01_enq, F01_deq, F01_din, F01_dot, F01_emp, F01_full, F01_cnt);
  MRE2 #(1,32) F02(CLK, frst, F02_enq, F02_deq, F02_din, F02_dot, F02_emp, F02_full, F02_cnt);
  MRE2 #(1,32) F03(CLK, frst, F03_enq, F03_deq, F03_din, F03_dot, F03_emp, F03_full, F03_cnt);
  MRE2 #(1,32) F04(CLK, frst, F04_enq, F04_deq, F04_din, F04_dot, F04_emp, F04_full, F04_cnt);
  MRE2 #(1,32) F05(CLK, frst, F05_enq, F05_deq, F05_din, F05_dot, F05_emp, F05_full, F05_cnt);
  MRE2 #(1,32) F06(CLK, frst, F06_enq, F06_deq, F06_din, F06_dot, F06_emp, F06_full, F06_cnt);
  MRE2 #(1,32) F07(CLK, frst, F07_enq, F07_deq, F07_din, F07_dot, F07_emp, F07_full, F07_cnt);
  MRE2 #(1,32) F08(CLK, frst, F08_enq, F08_deq, F08_din, F08_dot, F08_emp, F08_full, F08_cnt);
  MRE2 #(1,32) F09(CLK, frst, F09_enq, F09_deq, F09_din, F09_dot, F09_emp, F09_full, F09_cnt);
  MRE2 #(1,32) F10(CLK, frst, F10_enq, F10_deq, F10_din, F10_dot, F10_emp, F10_full, F10_cnt);
  MRE2 #(1,32) F11(CLK, frst, F11_enq, F11_deq, F11_din, F11_dot, F11_emp, F11_full, F11_cnt);
  MRE2 #(1,32) F12(CLK, frst, F12_enq, F12_deq, F12_din, F12_dot, F12_emp, F12_full, F12_cnt);
  MRE2 #(1,32) F13(CLK, frst, F13_enq, F13_deq, F13_din, F13_dot, F13_emp, F13_full, F13_cnt);
  MRE2 #(1,32) F14(CLK, frst, F14_enq, F14_deq, F14_din, F14_dot, F14_emp, F14_full, F14_cnt);
  MRE2 #(1,32) F15(CLK, frst, F15_enq, F15_deq, F15_din, F15_dot, F15_emp, F15_full, F15_cnt);

  SCELL S01(!F02_emp, !F03_emp, F02_deq, F03_deq, F02_dot, F03_dot, F01_full, F01_din, F01_enq);
  SCELL S02(!F04_emp, !F05_emp, F04_deq, F05_deq, F04_dot, F05_dot, F02_full, F02_din, F02_enq);
  SCELL S03(!F06_emp, !F07_emp, F06_deq, F07_deq, F06_dot, F07_dot, F03_full, F03_din, F03_enq);
  SCELL S04(!F08_emp, !F09_emp, F08_deq, F09_deq, F08_dot, F09_dot, F04_full, F04_din, F04_enq);
  SCELL S05(!F10_emp, !F11_emp, F10_deq, F11_deq, F10_dot, F11_dot, F05_full, F05_din, F05_enq);
  SCELL S06(!F12_emp, !F13_emp, F12_deq, F13_deq, F12_dot, F13_dot, F06_full, F06_din, F06_enq);
  SCELL S07(!F14_emp, !F15_emp, F14_deq, F15_deq, F14_dot, F15_dot, F07_full, F07_din, F07_enq);

  assign F01_deq = deq;
  assign dot = F01_dot;
  assign emp = F01_emp;

endmodule

/***** compressor                                                                             *****/
/**************************************************************************************************/
module COMPRESSOR(input  wire              CLK, 
                  input  wire              RST,
                  input  wire [`DRAMW-1:0] DIN,
                  input  wire              DIN_EN,
                  input  wire              BUF_FULL,
                  output wire [`DRAMW-1:0] DOUT,
                  output wire              DOUT_VALID);

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

  reg din_en;
  always @(posedge CLK) din_en <= DIN_EN;
  
  reg [31:0] block_cnt;
  always @(posedge CLK) begin
    if (RST) begin
      block_cnt <= 0;
    end else begin
      case ({(block_cnt == ((`SORT_ELM>>4)>>(`P_LOG+`WAY_LOG))), din_en})
        2'b10: block_cnt <= 0;
        2'b01: block_cnt <= block_cnt + 1;
      endcase
    end
  end
  
  // Base+Delta Compressor /////////////////////////////////////////////////
  wire [`SORTW-1:0] base    = DIN[31 : 0];
  wire [`SORTW-1:0] delta_a = DIN[63 : 32] - DIN[31 : 0];
  wire [`SORTW-1:0] delta_b = DIN[95 : 64] - DIN[63 : 32];
  wire [`SORTW-1:0] delta_c = DIN[127: 96] - DIN[95 : 64];
  wire [`SORTW-1:0] delta_d = DIN[159:128] - DIN[127: 96];
  wire [`SORTW-1:0] delta_e = DIN[191:160] - DIN[159:128];
  wire [`SORTW-1:0] delta_f = DIN[223:192] - DIN[191:160];
  wire [`SORTW-1:0] delta_g = DIN[255:224] - DIN[223:192];
  wire [`SORTW-1:0] delta_h = DIN[287:256] - DIN[255:224];
  wire [`SORTW-1:0] delta_i = DIN[319:288] - DIN[287:256];
  wire [`SORTW-1:0] delta_j = DIN[351:320] - DIN[319:288];
  wire [`SORTW-1:0] delta_k = DIN[383:352] - DIN[351:320];
  wire [`SORTW-1:0] delta_l = DIN[415:384] - DIN[383:352];
  wire [`SORTW-1:0] delta_m = DIN[447:416] - DIN[415:384];
  wire [`SORTW-1:0] delta_n = DIN[479:448] - DIN[447:416];
  wire [`SORTW-1:0] delta_o = DIN[511:480] - DIN[479:448];
  
  reg         c_cnt;
  reg         c_cflag;
  always @(posedge CLK) c_cflag  <= (delta_a<=13'h1fff) && (delta_b<=13'h1fff) && (delta_c<=13'h1fff) && (delta_d<=13'h1fff) &&
                                    (delta_e<=13'h1fff) && (delta_f<=13'h1fff) && (delta_g<=13'h1fff) && (delta_h<=13'h1fff) &&
                                    (delta_i<=13'h1fff) && (delta_j<=13'h1fff) && (delta_k<=13'h1fff) && (delta_l<=13'h1fff) &&
                                    (delta_m<=13'h1fff) && (delta_n<=13'h1fff) && (delta_o<=13'h1fff);
  wire        c_enable = (din_en && c_cflag);
  wire        c_cntrst;
  reg [226:0] c_data;
  always @(posedge CLK) c_data <= {delta_o[12:0], delta_n[12:0], delta_m[12:0], delta_l[12:0], delta_k[12:0], delta_j[12:0], delta_i[12:0], 
                                   delta_h[12:0], delta_g[12:0], delta_f[12:0], delta_e[12:0], delta_d[12:0], delta_c[12:0], delta_b[12:0], 
                                   delta_a[12:0], base};

  always @(posedge CLK) begin
    if (RST) begin
      c_cnt <= 0;
    end else begin
      case ({c_enable, c_cntrst})
        2'b10: c_cnt <= ~c_cnt;
        2'b01: c_cnt <= 0;
      endcase
    end
  end

  // Data Packer ///////////////////////////////////////////////////////////
  reg [226:0] data_buf;
  always @(posedge CLK) if (c_enable) data_buf <= c_data;
  
  reg              p_valid;  // packed data is valid
  reg [`DRAMW-1:0] packed_data;
  always @(posedge CLK) p_valid     <= c_enable && c_cnt;
  always @(posedge CLK) packed_data <= {{32'b0, 1'b1}, 25'b0, c_data, data_buf};

  // temp FIFO /////////////////////////////////////////////////////////////
  reg               deq_req;
  wire              tmp_emp;
  wire              tmp_rst = RST || p_valid;
  wire              tmp_enq = DIN_EN;
  wire              tmp_deq = !BUF_FULL && deq_req && !tmp_emp;
  wire [`DRAMW-1:0] tmp_dout;
  wire              tmp_full;
  
  MRE2 #(1, `DRAMW) tmp(.CLK(CLK), .RST(tmp_rst), .enq(tmp_enq), .deq(tmp_deq), 
                        .din(DIN), .dot(tmp_dout), .emp(tmp_emp), .full(tmp_full));
   
  assign c_cntrst = tmp_deq;
  
  always @(posedge CLK) begin
    if (RST) begin
      deq_req <= 0;
    end else begin
      if      ((din_en && !c_cflag) || 
               (din_en && (block_cnt==((`SORT_ELM>>4)>>(`P_LOG+`WAY_LOG))-1) && !c_cnt)) deq_req <= 1;
      else if (tmp_emp)                                                                  deq_req <= 0;
    end
  end
  
  // Output ////////////////////////////////////////////////////////////////
  assign DOUT       = mux(tmp_dout, packed_data, p_valid);
  assign DOUT_VALID = p_valid || tmp_deq;
  
endmodule

/***** decompressor                                                                           *****/
/**************************************************************************************************/
module DECOMPRESSOR #(parameter                                    SIZE   = 7, 
                      parameter                                    BLOCKS = 8)
                     (input  wire                                  CLK,
                      input  wire                                  RST,
                      input  wire [(1<<`P_LOG)+`SORT_WAY+`DRAMW:0] DIN,
                      input  wire                                  DIN_EN,
                      output reg  [`DRAMW-1:0]                     DOUT,
                      output reg  [(1<<`P_LOG)+`SORT_WAY:0]        DOUT_VALID,
                      output reg                                   DATA_REQ);

  function [227-1:0] mux227;
    input [227-1:0] a;
    input [227-1:0] b;
    input           sel;
    begin
      case (sel)
        1'b0: mux227 = a;
        1'b1: mux227 = b;
      endcase
    end
  endfunction
  
  function [512-1:0] mux512;
    input [512-1:0] a;
    input [512-1:0] b;
    input           sel;
    begin
      case (sel)
        1'b0: mux512 = a;
        1'b1: mux512 = b;
      endcase
    end
  endfunction
  
  // FIFO (Block RAM) //////////////////////////////////////////////////////
  wire                                  dmft_full;
  
  wire                                  dmf_emp;
  wire                                  dmf_enq = DIN_EN;
  wire                                  dmf_deq = !dmf_emp && !dmft_full;
  wire [(1<<`P_LOG)+`SORT_WAY+`DRAMW:0] dmf_din = DIN;
  wire                                  dmf_full;
  wire [(1<<`P_LOG)+`SORT_WAY+`DRAMW:0] dmf_dout;
  wire [SIZE:0]                         dmf_cnt;
  BFIFO #(SIZE, (1<<`P_LOG)+`SORT_WAY+1+`DRAMW) dmf(CLK, RST, dmf_enq, dmf_deq, dmf_din, dmf_dout, dmf_emp, dmf_full, dmf_cnt);
  
  reg dmf_dataen;
  always @(posedge CLK) dmf_dataen <= dmf_deq;
  
  always @(posedge CLK) DATA_REQ <= (dmf_cnt <= (1<<SIZE)-BLOCKS);
  
  // FIFO (two entries) ///////////////////////////////////////////////////
  wire                                  dmft_enq = dmf_dataen;
  wire [(1<<`P_LOG)+`SORT_WAY+`DRAMW:0] dmft_din = dmf_dout;
  wire [(1<<`P_LOG)+`SORT_WAY+`DRAMW:0] dmft_dout;
  wire                                  dmft_emp;
  wire                                  c_valid = (dmft_dout[`DRAMW-1:`DRAMW-33]=={32'b0,1'b1}) && dmft_dout[`DRAMW];  // check whether the data is compressed or not
  reg                                   c_sel;
  wire                                  dmft_deq = c_sel || (!c_valid && !dmft_emp);
  MRE2 #(1, (1<<`P_LOG)+`SORT_WAY+1+`DRAMW) dmft(.CLK(CLK), .RST(RST), .enq(dmft_enq), .deq(dmft_deq), 
                                                 .din(dmft_din), .dot(dmft_dout), .emp(dmft_emp), .full(dmft_full));
  
  // Base+Delta Decompressor ///////////////////////////////////////////////
  always @(posedge CLK) begin
    if (RST) begin
      c_sel <= 0;
    end else begin
      if  (c_valid && !dmft_emp) c_sel <= ~c_sel;
    end
  end

  wire [226:0] c_data = mux227(dmft_dout[226:0], dmft_dout[453:227], c_sel);
  
  // Stage A
  //////////////////////////////////////////////////////////////////////////////
  wire [`SORTW-1:0] a00 = c_data[31 : 0];
  wire [`SORTW-1:0] a01 = {19'b0, c_data[44 : 32]};
  wire [`SORTW-1:0] a02 = {19'b0, c_data[57 : 45]};
  wire [`SORTW-1:0] a03 = {19'b0, c_data[70 : 58]};
  wire [`SORTW-1:0] a04 = {19'b0, c_data[83 : 71]};
  wire [`SORTW-1:0] a05 = {19'b0, c_data[96 : 84]};
  wire [`SORTW-1:0] a06 = {19'b0, c_data[109: 97]};
  wire [`SORTW-1:0] a07 = {19'b0, c_data[122:110]};
  wire [`SORTW-1:0] a08 = {19'b0, c_data[135:123]};
  wire [`SORTW-1:0] a09 = {19'b0, c_data[148:136]};
  wire [`SORTW-1:0] a10 = {19'b0, c_data[161:149]};
  wire [`SORTW-1:0] a11 = {19'b0, c_data[174:162]};
  wire [`SORTW-1:0] a12 = {19'b0, c_data[187:175]};
  wire [`SORTW-1:0] a13 = {19'b0, c_data[200:188]};
  wire [`SORTW-1:0] a14 = {19'b0, c_data[213:201]};
  wire [`SORTW-1:0] a15 = {19'b0, c_data[226:214]};

  reg [511:0] pdA; // pipeline regester A for data
  always @(posedge CLK) pdA <= {a15,a14,a13,a12,a11,a10,a09,a08,a07,a06,a05,a04,(a03+a02+a01+a00),(a02+a01+a00),(a01+a00),a00};
  reg [`DRAMW-1:0] dmft_dout_A;
  always @(posedge CLK) dmft_dout_A <= dmft_dout[`DRAMW-1:0];
  reg  [(1<<`P_LOG)+`SORT_WAY+1:0] pcA;  // pipeline regester A for control
  always @(posedge CLK) pcA <= {dmft_dout[(1<<`P_LOG)+`SORT_WAY+`DRAMW:`DRAMW+1], (!dmft_emp), c_valid};
  
  // Stage B
  //////////////////////////////////////////////////////////////////////////////
  wire [`SORTW-1:0] b15,b14,b13,b12,b11,b10,b09,b08,b07,b06,b05,b04,b03,b02,b01,b00; // input
  assign {b15,b14,b13,b12,b11,b10,b09,b08,b07,b06,b05,b04,b03,b02,b01,b00} = pdA;

  reg [511:0] pdB; // pipeline regester B for data
  always @(posedge CLK) pdB <= {b15,b14,b13,b12,b11,b10,b09,b08,b07,(b06+b05+b04+b03),(b05+b04+b03),(b04+b03),b03,b02,b01,b00};
  reg [`DRAMW-1:0] dmft_dout_B;
  always @(posedge CLK) dmft_dout_B <= dmft_dout_A;
  reg  [(1<<`P_LOG)+`SORT_WAY+1:0] pcB;  // pipeline regester B for control
  always @(posedge CLK) pcB <= pcA;

  // Stage C
  //////////////////////////////////////////////////////////////////////////////
  wire [`SORTW-1:0] c15,c14,c13,c12,c11,c10,c09,c08,c07,c06,c05,c04,c03,c02,c01,c00; // input
  assign {c15,c14,c13,c12,c11,c10,c09,c08,c07,c06,c05,c04,c03,c02,c01,c00} = pdB;

  reg [511:0] pdC; // pipeline regester C for data
  always @(posedge CLK) pdC <= {c15,c14,c13,c12,c11,c10,(c09+c08+c07+c06),(c08+c07+c06),(c07+c06),c06,c05,c04,c03,c02,c01,c00};
  reg [`DRAMW-1:0] dmft_dout_C;
  always @(posedge CLK) dmft_dout_C <= dmft_dout_B;
  reg  [(1<<`P_LOG)+`SORT_WAY+1:0] pcC;  // pipeline regester C for control
  always @(posedge CLK) pcC <= pcB;

  // Stage D
  //////////////////////////////////////////////////////////////////////////////
  wire [`SORTW-1:0] d15,d14,d13,d12,d11,d10,d09,d08,d07,d06,d05,d04,d03,d02,d01,d00; // input
  assign {d15,d14,d13,d12,d11,d10,d09,d08,d07,d06,d05,d04,d03,d02,d01,d00} = pdC;

  reg [511:0] pdD; // pipeline regester D for data
  always @(posedge CLK) pdD <= {d15,d14,d13,(d12+d11+d10+d09),(d11+d10+d09),(d10+d09),d09,d08,d07,d06,d05,d04,d03,d02,d01,d00};
  reg [`DRAMW-1:0] dmft_dout_D;
  always @(posedge CLK) dmft_dout_D <= dmft_dout_C;
  reg  [(1<<`P_LOG)+`SORT_WAY+1:0] pcD;  // pipeline regester D for control
  always @(posedge CLK) pcD <= pcC;

  // Stage E
  //////////////////////////////////////////////////////////////////////////////
  wire [`SORTW-1:0] e15,e14,e13,e12,e11,e10,e09,e08,e07,e06,e05,e04,e03,e02,e01,e00; // input
  assign {e15,e14,e13,e12,e11,e10,e09,e08,e07,e06,e05,e04,e03,e02,e01,e00} = pdD;

  reg [511:0] pdE; // pipeline regester E for data
  always @(posedge CLK) pdE <= {(e15+e14+e13+e12),(e14+e13+e12),(e13+e12),e12,e11,e10,e09,e08,e07,e06,e05,e04,e03,e02,e01,e00};
  reg [`DRAMW-1:0] dmft_dout_E;
  always @(posedge CLK) dmft_dout_E <= dmft_dout_D;
  reg  [(1<<`P_LOG)+`SORT_WAY+1:0] pcE;  // pipeline regester E for control
  always @(posedge CLK) pcE <= pcD;

  // Decompression Result
  //////////////////////////////////////////////////////////////////////////////
  wire [`DRAMW-1:0] dc_data  = pdE;
  wire [`DRAMW-1:0] dmft_dot = dmft_dout_E;
  wire              c_vld    = pcE[0];
  wire              dataen   = pcE[1];
  
  // Output ////////////////////////////////////////////////////////////////
  always @(posedge CLK) if (dataen) DOUT <= mux512(dmft_dot, dc_data, c_vld);
  
  always @(posedge CLK) DOUT_VALID <= pcE[(1<<`P_LOG)+`SORT_WAY+1:1];
  
endmodule

/***** Output Module                                                                          *****/
/**************************************************************************************************/
module OTMOD(input  wire              CLK, 
             input  wire              RST,
             input  wire              d_busy, 
             input  wire [31:0]       w_block,
             input  wire              F01_deq, 
             input  wire [`SORTW-1:0] F01_dot, 
             input  wire              OB_deq, 
             output wire [`DRAMW-1:0] OB_dot, 
             output wire              OB_full, 
             output reg               OB_req);
  
  // 512-bit shift register ////////////////////////////////////////////////
  reg [3:0]        buf_t_cnt; // counter for temporary register
  reg              buf_t_en;
  reg [`DRAMW-1:0] buf_t;

  always @(posedge CLK) begin
    if (F01_deq) buf_t <= {F01_dot, buf_t[`DRAMW-1:32]};
  end
  always @(posedge CLK) begin
    if (RST) begin
      buf_t_cnt <= 0;
    end else begin
      if (F01_deq) buf_t_cnt <= buf_t_cnt + 1;
    end
  end
  always @(posedge CLK) buf_t_en <= (F01_deq && buf_t_cnt == 15);

  // Compressor ////////////////////////////////////////////////////////////
  wire [`DRAMW-1:0] c_din   = buf_t;
  wire              c_dinen = buf_t_en;
  wire [`DRAMW-1:0] c_dout;
  wire              c_douten;
  COMPRESSOR compressor(CLK, RST, c_din, c_dinen, OB_full, c_dout, c_douten);
  
  // Output Buffer /////////////////////////////////////////////////////////
  wire [`DRAMW-1:0] OB_din = c_dout;
  wire              OB_enq = c_douten;
  wire [`OB_SIZE:0] OB_cnt;
  BFIFO #(`OB_SIZE, `DRAMW) OB(.CLK(CLK), .RST(RST), .enq(OB_enq), .deq(OB_deq), 
                               .din(OB_din), .dot(OB_dot), .full(OB_full), .cnt(OB_cnt));

  always @(posedge CLK) OB_req <= ((OB_cnt>=w_block) && !d_busy);

endmodule

/**************************************************************************************************/
module COMPARATOR #(parameter               WIDTH = 32)
                   (input  wire [WIDTH-1:0] DIN0,
                    input  wire [WIDTH-1:0] DIN1,
                    output wire [WIDTH-1:0] DOUT0,
                    output wire [WIDTH-1:0] DOUT1);
    
  wire comp_rslt = (DIN0 < DIN1);
  
  function [WIDTH-1:0] mux;
    input [WIDTH-1:0] a;
    input [WIDTH-1:0] b;
    input             sel;
    begin
      case (sel)
        1'b0: mux = a;
        1'b1: mux = b;
      endcase
    end
  endfunction

  assign DOUT0 = mux(DIN1, DIN0, comp_rslt);
  assign DOUT1 = mux(DIN0, DIN1, comp_rslt);
endmodule 

/**************************************************************************************************/
module SORTINGNETWORK(input  wire                           CLK,
                      input  wire                           RST_IN,
                      input  wire [(1<<`P_LOG)+`SORT_WAY:0] DATAEN_IN,
                      input  wire [511:0]                   DIN_T,
                      output reg  [511:0]                   DOUT,
                      output reg  [(1<<`P_LOG)+`SORT_WAY:0] DATAEN_OUT);

  reg                           RST;
  reg [511:0]                   DIN;
  reg [(1<<`P_LOG)+`SORT_WAY:0] DATAEN;
  always @(posedge CLK) RST    <= RST_IN;
  always @(posedge CLK) DIN    <= DIN_T;
  always @(posedge CLK) DATAEN <= DATAEN_IN;
  
  // Stage A
  ////////////////////////////////////////////////////////////////////////////////////////////////
  wire [`WW] A15,A14,A13,A12,A11,A10,A09,A08,A07,A06,A05,A04,A03,A02,A01,A00; // output
  wire [`WW] a15,a14,a13,a12,a11,a10,a09,a08,a07,a06,a05,a04,a03,a02,a01,a00; // input
  assign {a15,a14,a13,a12,a11,a10,a09,a08,a07,a06,a05,a04,a03,a02,a01,a00} = DIN;
  
  COMPARATOR comp00(a00, a01, A00, A01);
  COMPARATOR comp01(a02, a03, A02, A03);
  COMPARATOR comp02(a04, a05, A04, A05);
  COMPARATOR comp03(a06, a07, A06, A07);
  COMPARATOR comp04(a08, a09, A08, A09);
  COMPARATOR comp05(a10, a11, A10, A11);
  COMPARATOR comp06(a12, a13, A12, A13);
  COMPARATOR comp07(a14, a15, A14, A15);
  
  reg [511:0]       pdA; // pipeline regester A for data
  reg [(1<<`P_LOG)+`SORT_WAY:0] pcA; // pipeline regester A for control
  always @(posedge CLK) pdA <= {A15,A14,A13,A12,A11,A10,A09,A08,A07,A06,A05,A04,A03,A02,A01,A00};
  always @(posedge CLK) pcA <= DATAEN;
  
  // Stage B
  ////////////////////////////////////////////////////////////////////////////////////////////////
  wire [`WW] B15,B14,B13,B12,B11,B10,B09,B08,B07,B06,B05,B04,B03,B02,B01,B00; // output
  wire [`WW] b15,b14,b13,b12,b11,b10,b09,b08,b07,b06,b05,b04,b03,b02,b01,b00; // input
  assign {b15,b14,b13,b12,b11,b10,b09,b08,b07,b06,b05,b04,b03,b02,b01,b00} = pdA;
  
  COMPARATOR comp10(b00, b02, B00, B02);
  COMPARATOR comp11(b04, b06, B04, B06);
  COMPARATOR comp12(b08, b10, B08, B10);
  COMPARATOR comp13(b12, b14, B12, B14);
  COMPARATOR comp14(b01, b03, B01, B03);
  COMPARATOR comp15(b05, b07, B05, B07);
  COMPARATOR comp16(b09, b11, B09, B11);
  COMPARATOR comp17(b13, b15, B13, B15);
  
  reg [511:0]       pdB; // pipeline regester A for data
  reg [(1<<`P_LOG)+`SORT_WAY:0] pcB; // pipeline regester A for control
  always @(posedge CLK) pdB <= {B15,B14,B13,B12,B11,B10,B09,B08,B07,B06,B05,B04,B03,B02,B01,B00};
  always @(posedge CLK) pcB <= pcA;
  
  // Stage C
  ////////////////////////////////////////////////////////////////////////////////////////////////
  wire [`WW] C15,C14,C13,C12,C11,C10,C09,C08,C07,C06,C05,C04,C03,C02,C01,C00; // output
  wire [`WW] c15,c14,c13,c12,c11,c10,c09,c08,c07,c06,c05,c04,c03,c02,c01,c00; // input
  assign {c15,c14,c13,c12,c11,c10,c09,c08,c07,c06,c05,c04,c03,c02,c01,c00} = pdB;
  
  assign {C00,C03,C04,C07,C08,C11,C12,C15} = {c00,c03,c04,c07,c08,c11,c12,c15};
  COMPARATOR comp20(c01, c02, C01, C02);
  COMPARATOR comp21(c05, c06, C05, C06);
  COMPARATOR comp22(c09, c10, C09, C10);
  COMPARATOR comp23(c13, c14, C13, C14);
  
  reg [511:0]       pdC; // pipeline regester A for data
  reg [(1<<`P_LOG)+`SORT_WAY:0] pcC; // pipeline regester A for control
  always @(posedge CLK) pdC <= {C15,C14,C13,C12,C11,C10,C09,C08,C07,C06,C05,C04,C03,C02,C01,C00};
  always @(posedge CLK) pcC <= pcB;
  
  // Stage D
  ////////////////////////////////////////////////////////////////////////////////////////////////
  wire [`WW] D15,D14,D13,D12,D11,D10,D09,D08,D07,D06,D05,D04,D03,D02,D01,D00; // output
  wire [`WW] d15,d14,d13,d12,d11,d10,d09,d08,d07,d06,d05,d04,d03,d02,d01,d00; // input
  assign {d15,d14,d13,d12,d11,d10,d09,d08,d07,d06,d05,d04,d03,d02,d01,d00} = pdC;
  
  COMPARATOR comp30(d00, d04, D00,  D04);
  COMPARATOR comp31(d08, d12, D08,  D12);
  COMPARATOR comp32(d01, d05, D01,  D05);
  COMPARATOR comp33(d09, d13, D09,  D13);
  COMPARATOR comp34(d02, d06, D02,  D06);
  COMPARATOR comp35(d10, d14, D10,  D14);
  COMPARATOR comp36(d03, d07, D03,  D07);
  COMPARATOR comp37(d11, d15, D11,  D15);
  
  reg [511:0]       pdD; // pipeline regester A for data
  reg [(1<<`P_LOG)+`SORT_WAY:0] pcD; // pipeline regester A for control
  always @(posedge CLK) pdD <= {D15,D14,D13,D12,D11,D10,D09,D08,D07,D06,D05,D04,D03,D02,D01,D00};
  always @(posedge CLK) pcD <= pcC;
  
  // Stage E
  ////////////////////////////////////////////////////////////////////////////////////////////////
  wire [`WW] E15,E14,E13,E12,E11,E10,E09,E08,E07,E06,E05,E04,E03,E02,E01,E00; // output
  wire [`WW] e15,e14,e13,e12,e11,e10,e09,e08,e07,e06,e05,e04,e03,e02,e01,e00; // input
  assign {e15,e14,e13,e12,e11,e10,e09,e08,e07,e06,e05,e04,e03,e02,e01,e00} = pdD;
  
  assign {E00,E01,E06,E07,E08,E09,E14,E15} = {e00,e01,e06,e07,e08,e09,e14,e15};
  COMPARATOR comp40(e02, e04, E02, E04);
  COMPARATOR comp41(e10, e12, E10, E12);
  COMPARATOR comp42(e03, e05, E03, E05);
  COMPARATOR comp43(e11, e13, E11, E13);
  
  reg [511:0]       pdE; // pipeline regester A for data
  reg [(1<<`P_LOG)+`SORT_WAY:0] pcE; // pipeline regester A for control
  always @(posedge CLK) pdE <= {E15,E14,E13,E12,E11,E10,E09,E08,E07,E06,E05,E04,E03,E02,E01,E00};
  always @(posedge CLK) pcE <= pcD;
  
  // Stage F
  ////////////////////////////////////////////////////////////////////////////////////////////////
  wire [`WW] F15,F14,F13,F12,F11,F10,F09,F08,F07,F06,F05,F04,F03,F02,F01,F00; // output
  wire [`WW] f15,f14,f13,f12,f11,f10,f09,f08,f07,f06,f05,f04,f03,f02,f01,f00; // input
  assign {f15,f14,f13,f12,f11,f10,f09,f08,f07,f06,f05,f04,f03,f02,f01,f00} = pdE;
  
  assign {F00,F07,F08,F15} = {f00,f07,f08,f15};
  COMPARATOR comp50(f01, f02, F01, F02);
  COMPARATOR comp51(f03, f04, F03, F04);
  COMPARATOR comp52(f05, f06, F05, F06);
  COMPARATOR comp53(f09, f10, F09, F10);
  COMPARATOR comp54(f11, f12, F11, F12);
  COMPARATOR comp55(f13, f14, F13, F14);
  
  reg [511:0]       pdF; // pipeline regester A for data
  reg [(1<<`P_LOG)+`SORT_WAY:0] pcF; // pipeline regester A for control
  always @(posedge CLK) pdF <= {F15,F14,F13,F12,F11,F10,F09,F08,F07,F06,F05,F04,F03,F02,F01,F00};
  always @(posedge CLK) pcF <= pcE;
  
  // Stage G
  ////////////////////////////////////////////////////////////////////////////////////////////////
  wire [`WW] G15,G14,G13,G12,G11,G10,G09,G08,G07,G06,G05,G04,G03,G02,G01,G00; // output
  wire [`WW] g15,g14,g13,g12,g11,g10,g09,g08,g07,g06,g05,g04,g03,g02,g01,g00; // input
  assign {g15,g14,g13,g12,g11,g10,g09,g08,g07,g06,g05,g04,g03,g02,g01,g00} = pdF;
  
  COMPARATOR comp60(g00, g08, G00, G08);
  COMPARATOR comp61(g01, g09, G01, G09);
  COMPARATOR comp62(g02, g10, G02, G10);
  COMPARATOR comp63(g03, g11, G03, G11);
  COMPARATOR comp64(g04, g12, G04, G12);
  COMPARATOR comp65(g05, g13, G05, G13);
  COMPARATOR comp66(g06, g14, G06, G14);
  COMPARATOR comp67(g07, g15, G07, G15);
  
  reg [511:0]       pdG; // pipeline regester A for data
  reg [(1<<`P_LOG)+`SORT_WAY:0] pcG; // pipeline regester A for control
  always @(posedge CLK) pdG <= {G15,G14,G13,G12,G11,G10,G09,G08,G07,G06,G05,G04,G03,G02,G01,G00};
  always @(posedge CLK) pcG <= pcF;
  
  // Stage H
  ////////////////////////////////////////////////////////////////////////////////////////////////
  wire [`WW] H15,H14,H13,H12,H11,H10,H09,H08,H07,H06,H05,H04,H03,H02,H01,H00; // output
  wire [`WW] h15,h14,h13,h12,h11,h10,h09,h08,h07,h06,h05,h04,h03,h02,h01,h00; // input
  assign {h15,h14,h13,h12,h11,h10,h09,h08,h07,h06,h05,h04,h03,h02,h01,h00} = pdG;
  
  assign {H00,H01,H02,H03,H12,H13,H14,H15} = {h00,h01,h02,h03,h12,h13,h14,h15};
  COMPARATOR comp70(h04, h08, H04, H08);
  COMPARATOR comp71(h05, h09, H05, H09);
  COMPARATOR comp72(h06, h10, H06, H10);
  COMPARATOR comp73(h07, h11, H07, H11);
  
  reg [511:0]       pdH; // pipeline regester A for data
  reg [(1<<`P_LOG)+`SORT_WAY:0] pcH; // pipeline regester A for control
  always @(posedge CLK) pdH <= {H15,H14,H13,H12,H11,H10,H09,H08,H07,H06,H05,H04,H03,H02,H01,H00};
  always @(posedge CLK) pcH <= pcG;
  
  // Stage I
  ////////////////////////////////////////////////////////////////////////////////////////////////
  wire [`WW] I15,I14,I13,I12,I11,I10,I09,I08,I07,I06,I05,I04,I03,I02,I01,I00; // output
  wire [`WW] i15,i14,i13,i12,i11,i10,i09,i08,i07,i06,i05,i04,i03,i02,i01,i00; // input
  assign {i15,i14,i13,i12,i11,i10,i09,i08,i07,i06,i05,i04,i03,i02,i01,i00} = pdH;
  
  assign {I00,I01,I14,I15} = {i00,i01,i14,i15};
  COMPARATOR comp80(i02, i04, I02, I04);
  COMPARATOR comp81(i06, i08, I06, I08);
  COMPARATOR comp82(i10, i12, I10, I12);
  COMPARATOR comp83(i03, i05, I03, I05);
  COMPARATOR comp84(i07, i09, I07, I09);
  COMPARATOR comp85(i11, i13, I11, I13);
  
  reg [511:0]       pdI; // pipeline regester A for data
  reg [(1<<`P_LOG)+`SORT_WAY:0] pcI; // pipeline regester A for control
  always @(posedge CLK) pdI <= {I15,I14,I13,I12,I11,I10,I09,I08,I07,I06,I05,I04,I03,I02,I01,I00};
  always @(posedge CLK) pcI <= pcH;
    
  // Stage J
  ////////////////////////////////////////////////////////////////////////////////////////////////
  wire [`WW] J15,J14,J13,J12,J11,J10,J09,J08,J07,J06,J05,J04,J03,J02,J01,J00; // output
  wire [`WW] j15,j14,j13,j12,j11,j10,j09,j08,j07,j06,j05,j04,j03,j02,j01,j00; // input
  assign {j15,j14,j13,j12,j11,j10,j09,j08,j07,j06,j05,j04,j03,j02,j01,j00} = pdI;
    
  assign {J00,J15} = {j00,j15};
  COMPARATOR comp90(j01, j02, J01, J02);
  COMPARATOR comp91(j03, j04, J03, J04);
  COMPARATOR comp92(j05, j06, J05, J06);
  COMPARATOR comp93(j07, j08, J07, J08);
  COMPARATOR comp94(j09, j10, J09, J10);
  COMPARATOR comp95(j11, j12, J11, J12);
  COMPARATOR comp96(j13, j14, J13, J14);
  
  always @(posedge CLK) DOUT <= {J15,J14,J13,J12,J11,J10,J09,J08,J07,J06,J05,J04,J03,J02,J01,J00};
  always @(posedge CLK) DATAEN_OUT <= pcI;
endmodule

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

// /***** Initial Data Generator                                                                 *****/
// /**************************************************************************************************/
// module INITDATAGEN(input  wire              CLK,
//                    input  wire              RST,
//                    input  wire              d_w,
//                    output wire [`DRAMW-1:0] INITDATA);

//   function [`SORTW-1:0] mux;
//     input [`SORTW-1:0] a;
//     input [`SORTW-1:0] b;
//     input              sel;
//     begin
//       case (sel)
//         1'b0: mux = a;
//         1'b1: mux = b;
//       endcase
//     end
//   endfunction
  
//   reg RST_INI; // reset signal for value initialization module
//   always @(posedge CLK) RST_INI <= RST;

//   reg [`SORTW-1:0] i_p,i_o,i_n,i_m,i_l,i_k,i_j,i_i,i_h,i_g,i_f,i_e,i_d,i_c,i_b,i_a;
//   assign INITDATA = {i_p,i_o,i_n,i_m,i_l,i_k,i_j,i_i,i_h,i_g,i_f,i_e,i_d,i_c,i_b,i_a};

//   generate
//     if (`INITTYPE == "xorshift") begin
//       wire [`SORTW-1:0] r15,r14,r13,r12,r11,r10,r09,r08,r07,r06,r05,r04,r03,r02,r01,r00;
//       XORSHIFT #(`SORTW, 32'h00000001) xorshift00(CLK, RST_INI, d_w, r00);
//       XORSHIFT #(`SORTW, 32'h00000002) xorshift01(CLK, RST_INI, d_w, r01);
//       XORSHIFT #(`SORTW, 32'h00000004) xorshift02(CLK, RST_INI, d_w, r02);
//       XORSHIFT #(`SORTW, 32'h00000008) xorshift03(CLK, RST_INI, d_w, r03);
//       XORSHIFT #(`SORTW, 32'h00000010) xorshift04(CLK, RST_INI, d_w, r04);
//       XORSHIFT #(`SORTW, 32'h00000020) xorshift05(CLK, RST_INI, d_w, r05);
//       XORSHIFT #(`SORTW, 32'h00000040) xorshift06(CLK, RST_INI, d_w, r06);
//       XORSHIFT #(`SORTW, 32'h00000080) xorshift07(CLK, RST_INI, d_w, r07);
//       XORSHIFT #(`SORTW, 32'h00000100) xorshift08(CLK, RST_INI, d_w, r08);
//       XORSHIFT #(`SORTW, 32'h00000200) xorshift09(CLK, RST_INI, d_w, r09);
//       XORSHIFT #(`SORTW, 32'h00000400) xorshift10(CLK, RST_INI, d_w, r10);
//       XORSHIFT #(`SORTW, 32'h00000800) xorshift11(CLK, RST_INI, d_w, r11);
//       XORSHIFT #(`SORTW, 32'h00001000) xorshift12(CLK, RST_INI, d_w, r12);
//       XORSHIFT #(`SORTW, 32'h00002000) xorshift13(CLK, RST_INI, d_w, r13);
//       XORSHIFT #(`SORTW, 32'h00004000) xorshift14(CLK, RST_INI, d_w, r14);
//       XORSHIFT #(`SORTW, 32'h00008000) xorshift15(CLK, RST_INI, d_w, r15);
//       always @(posedge CLK) begin
//         i_a <= r00 % 65536;
//         i_b <= r01 % 65536;
//         i_c <= r02;
//         i_d <= r03;
//         i_e <= r04 % 65536;
//         i_f <= r05;
//         i_g <= r06 % 65536;
//         i_h <= r07;
//         i_i <= r08 % 65536;
//         i_j <= r09;
//         i_k <= r10 % 65536;
//         i_l <= r11;
//         i_m <= r12 % 65536;
//         i_n <= r13;
//         i_o <= r14 % 65536;
//         i_p <= r15 % 65536;
//       end
//     end else if (`INITTYPE == "reverse") begin
//       always @(posedge CLK) begin
//         if (RST_INI) begin
//           i_a <= `SORT_ELM+16;   
//           i_b <= `SORT_ELM+16-1;
//           i_c <= `SORT_ELM+16-2;
//           i_d <= `SORT_ELM+16-3;
//           i_e <= `SORT_ELM+16-4;
//           i_f <= `SORT_ELM+16-5;
//           i_g <= `SORT_ELM+16-6;
//           i_h <= `SORT_ELM+16-7;
//           i_i <= `SORT_ELM+16-8;
//           i_j <= `SORT_ELM+16-9;
//           i_k <= `SORT_ELM+16-10;
//           i_l <= `SORT_ELM+16-11;
//           i_m <= `SORT_ELM+16-12;
//           i_n <= `SORT_ELM+16-13;
//           i_o <= `SORT_ELM+16-14;
//           i_p <= `SORT_ELM+16-15;
//         end else begin
//           if (d_w) begin
//             i_a <= i_a-16;
//             i_b <= i_b-16;
//             i_c <= i_c-16;
//             i_d <= i_d-16;
//             i_e <= i_e-16;
//             i_f <= i_f-16;
//             i_g <= i_g-16;
//             i_h <= i_h-16;
//             i_i <= i_i-16;
//             i_j <= i_j-16;
//             i_k <= i_k-16;
//             i_l <= i_l-16;
//             i_m <= i_m-16;
//             i_n <= i_n-16;
//             i_o <= i_o-16;
//             i_p <= i_p-16;
//           end
//         end
//       end
//     end else if (`INITTYPE == "sorted") begin
//       reg ocen;
//       always @(posedge CLK) begin
//         if (RST_INI) begin
//           ocen <= 0;
//           i_a  <= 1;   
//           i_b  <= 2;
//           i_c  <= 3;
//           i_d  <= 4;
//           i_e  <= 5;
//           i_f  <= 6;
//           i_g  <= 7;
//           i_h  <= 8;
//           i_i  <= 9;
//           i_j  <= 10;
//           i_k  <= 11;
//           i_l  <= 12;
//           i_m  <= 13;
//           i_n  <= 14;
//           i_o  <= 15;
//           i_p  <= 16;
//         end else begin
//           if (d_w) begin
//             ocen <= 1;
//             i_a  <= mux(i_a, i_a+16, ocen);
//             i_b  <= mux(i_b, i_b+16, ocen);
//             i_c  <= mux(i_c, i_c+16, ocen);
//             i_d  <= mux(i_d, i_d+16, ocen);
//             i_e  <= mux(i_e, i_e+16, ocen);
//             i_f  <= mux(i_f, i_f+16, ocen);
//             i_g  <= mux(i_g, i_g+16, ocen);
//             i_h  <= mux(i_h, i_h+16, ocen);
//             i_i  <= mux(i_i, i_i+16, ocen);
//             i_j  <= mux(i_j, i_j+16, ocen);
//             i_k  <= mux(i_k, i_k+16, ocen);
//             i_l  <= mux(i_l, i_l+16, ocen);
//             i_m  <= mux(i_m, i_m+16, ocen);
//             i_n  <= mux(i_n, i_n+16, ocen);
//             i_o  <= mux(i_o, i_o+16, ocen);
//             i_p  <= mux(i_p, i_p+16, ocen);
//           end
//         end
//       end
//     end
//   endgenerate

// endmodule

/***** Initial Data Generator                                                                 *****/
/**************************************************************************************************/
module INITDATAGEN(input  wire              CLK,
                   input  wire              RST,
                   input  wire              d_w,
                   output wire [`DRAMW-1:0] INITDATA);

  function [`SORTW-1:0] mux;
    input [`SORTW-1:0] a;
    input [`SORTW-1:0] b;
    input              sel;
    begin
      case (sel)
        1'b0: mux = a;
        1'b1: mux = b;
      endcase
    end
  endfunction
  
  reg RST_INI; // reset signal for value initialization module
  always @(posedge CLK) RST_INI <= RST;

  reg [`SORTW-1:0] i_p,i_o,i_n,i_m,i_l,i_k,i_j,i_i,i_h,i_g,i_f,i_e,i_d,i_c,i_b,i_a;
  assign INITDATA = {i_p,i_o,i_n,i_m,i_l,i_k,i_j,i_i,i_h,i_g,i_f,i_e,i_d,i_c,i_b,i_a};

  generate
    if (`INITTYPE == "xorshift") begin
      wire [`SORTW-1:0] r15,r14,r13,r12,r11,r10,r09,r08,r07,r06,r05,r04,r03,r02,r01,r00;
      XORSHIFT #(`SORTW, 32'h00000001) xorshift00(CLK, RST_INI, d_w, r00);
      XORSHIFT #(`SORTW, 32'h00000002) xorshift01(CLK, RST_INI, d_w, r01);
      XORSHIFT #(`SORTW, 32'h00000004) xorshift02(CLK, RST_INI, d_w, r02);
      XORSHIFT #(`SORTW, 32'h00000008) xorshift03(CLK, RST_INI, d_w, r03);
      XORSHIFT #(`SORTW, 32'h00000010) xorshift04(CLK, RST_INI, d_w, r04);
      XORSHIFT #(`SORTW, 32'h00000020) xorshift05(CLK, RST_INI, d_w, r05);
      XORSHIFT #(`SORTW, 32'h00000040) xorshift06(CLK, RST_INI, d_w, r06);
      XORSHIFT #(`SORTW, 32'h00000080) xorshift07(CLK, RST_INI, d_w, r07);
      XORSHIFT #(`SORTW, 32'h00000100) xorshift08(CLK, RST_INI, d_w, r08);
      XORSHIFT #(`SORTW, 32'h00000200) xorshift09(CLK, RST_INI, d_w, r09);
      XORSHIFT #(`SORTW, 32'h00000400) xorshift10(CLK, RST_INI, d_w, r10);
      XORSHIFT #(`SORTW, 32'h00000800) xorshift11(CLK, RST_INI, d_w, r11);
      XORSHIFT #(`SORTW, 32'h00001000) xorshift12(CLK, RST_INI, d_w, r12);
      XORSHIFT #(`SORTW, 32'h00002000) xorshift13(CLK, RST_INI, d_w, r13);
      XORSHIFT #(`SORTW, 32'h00004000) xorshift14(CLK, RST_INI, d_w, r14);
      XORSHIFT #(`SORTW, 32'h00008000) xorshift15(CLK, RST_INI, d_w, r15);
      always @(posedge CLK) begin
        i_a <= r00;
        i_b <= r01;
        i_c <= r02;
        i_d <= r03;
        i_e <= r04;
        i_f <= r05;
        i_g <= r06;
        i_h <= r07;
        i_i <= r08;
        i_j <= r09;
        i_k <= r10;
        i_l <= r11;
        i_m <= r12;
        i_n <= r13;
        i_o <= r14;
        i_p <= r15;
      end
    end else if (`INITTYPE == "reverse") begin
      always @(posedge CLK) begin
        if (RST_INI) begin
          i_a <= `SORT_ELM+16;   
          i_b <= `SORT_ELM+16-1;
          i_c <= `SORT_ELM+16-2;
          i_d <= `SORT_ELM+16-3;
          i_e <= `SORT_ELM+16-4;
          i_f <= `SORT_ELM+16-5;
          i_g <= `SORT_ELM+16-6;
          i_h <= `SORT_ELM+16-7;
          i_i <= `SORT_ELM+16-8;
          i_j <= `SORT_ELM+16-9;
          i_k <= `SORT_ELM+16-10;
          i_l <= `SORT_ELM+16-11;
          i_m <= `SORT_ELM+16-12;
          i_n <= `SORT_ELM+16-13;
          i_o <= `SORT_ELM+16-14;
          i_p <= `SORT_ELM+16-15;
        end else begin
          if (d_w) begin
            i_a <= i_a-16;
            i_b <= i_b-16;
            i_c <= i_c-16;
            i_d <= i_d-16;
            i_e <= i_e-16;
            i_f <= i_f-16;
            i_g <= i_g-16;
            i_h <= i_h-16;
            i_i <= i_i-16;
            i_j <= i_j-16;
            i_k <= i_k-16;
            i_l <= i_l-16;
            i_m <= i_m-16;
            i_n <= i_n-16;
            i_o <= i_o-16;
            i_p <= i_p-16;
          end
        end
      end
    end else if (`INITTYPE == "sorted") begin
      reg ocen;
      always @(posedge CLK) begin
        if (RST_INI) begin
          ocen <= 0;
          i_a  <= 1;   
          i_b  <= 2;
          i_c  <= 3;
          i_d  <= 4;
          i_e  <= 5;
          i_f  <= 6;
          i_g  <= 7;
          i_h  <= 8;
          i_i  <= 9;
          i_j  <= 10;
          i_k  <= 11;
          i_l  <= 12;
          i_m  <= 13;
          i_n  <= 14;
          i_o  <= 15;
          i_p  <= 16;
        end else begin
          if (d_w) begin
            ocen <= 1;
            i_a  <= mux(i_a, i_a+16, ocen);
            i_b  <= mux(i_b, i_b+16, ocen);
            i_c  <= mux(i_c, i_c+16, ocen);
            i_d  <= mux(i_d, i_d+16, ocen);
            i_e  <= mux(i_e, i_e+16, ocen);
            i_f  <= mux(i_f, i_f+16, ocen);
            i_g  <= mux(i_g, i_g+16, ocen);
            i_h  <= mux(i_h, i_h+16, ocen);
            i_i  <= mux(i_i, i_i+16, ocen);
            i_j  <= mux(i_j, i_j+16, ocen);
            i_k  <= mux(i_k, i_k+16, ocen);
            i_l  <= mux(i_l, i_l+16, ocen);
            i_m  <= mux(i_m, i_m+16, ocen);
            i_n  <= mux(i_n, i_n+16, ocen);
            i_o  <= mux(i_o, i_o+16, ocen);
            i_p  <= mux(i_p, i_p+16, ocen);
          end
        end
      end
    end
  endgenerate

endmodule

/***** request counter manager                                                                *****/
/**************************************************************************************************/
module REQCNTMG(input  wire                 CLK, 
                input  wire                 RST,
                input  wire                 DRIVE,
                input  wire [`SORT_WAY-1:0] req,
                input  wire [`SORT_WAY-1:0] im_enq,
                input  wire [`SORT_WAY-1:0] im_emp,
                output reg                  reqcnt_a,  
                output reg                  reqcnt_b,  // note!!! bit width
                output reg                  reqcnt_c,  // note!!! bit width
                output reg                  reqcnt_d,  // note!!! bit width
                output reg                  reqcnt_e,  // note!!! bit width
                output reg                  reqcnt_f,  // note!!! bit width
                output reg                  reqcnt_g,  // note!!! bit width
                output reg                  reqcnt_h); // note!!! bit width

  reg reqcnt_rsta; 
  reg reqcnt_rstb; 
  reg reqcnt_rstc; 
  reg reqcnt_rstd; 
  reg reqcnt_rste; 
  reg reqcnt_rstf; 
  reg reqcnt_rstg; 
  reg reqcnt_rsth;

  // request counter manager
  always @(posedge CLK) begin
    if (RST) begin
      {reqcnt_a, reqcnt_b, reqcnt_c, reqcnt_d, reqcnt_e, reqcnt_f, reqcnt_g, reqcnt_h} <= 0;
      {reqcnt_rsta, reqcnt_rstb, reqcnt_rstc, reqcnt_rstd, reqcnt_rste, reqcnt_rstf, reqcnt_rstg, reqcnt_rsth} <= 0;
    end else begin
      if (DRIVE) begin
        case (req)
          8'h01: reqcnt_a <= 1;
          8'h02: reqcnt_b <= 1;
          8'h04: reqcnt_c <= 1;
          8'h08: reqcnt_d <= 1;
          8'h10: reqcnt_e <= 1;
          8'h20: reqcnt_f <= 1;
          8'h40: reqcnt_g <= 1;
          8'h80: reqcnt_h <= 1;
        endcase
      end
      if (|im_enq) begin
        case (im_enq)
          8'h01: begin
            reqcnt_rsta <= 1;
            {reqcnt_rstb, reqcnt_rstc, reqcnt_rstd, reqcnt_rste, reqcnt_rstf, reqcnt_rstg, reqcnt_rsth} <= 0;
          end
          8'h02: begin
            reqcnt_rstb <= 1;
            {reqcnt_rsta, reqcnt_rstc, reqcnt_rstd, reqcnt_rste, reqcnt_rstf, reqcnt_rstg, reqcnt_rsth} <= 0;
          end
          8'h04: begin
            reqcnt_rstc <= 1;
            {reqcnt_rsta, reqcnt_rstb, reqcnt_rstd, reqcnt_rste, reqcnt_rstf, reqcnt_rstg, reqcnt_rsth} <= 0;
          end
          8'h08: begin
            reqcnt_rstd <= 1;
            {reqcnt_rsta, reqcnt_rstb, reqcnt_rstc, reqcnt_rste, reqcnt_rstf, reqcnt_rstg, reqcnt_rsth} <= 0;
          end
          8'h10: begin
            reqcnt_rste <= 1;
            {reqcnt_rsta, reqcnt_rstb, reqcnt_rstc, reqcnt_rstd, reqcnt_rstf, reqcnt_rstg, reqcnt_rsth} <= 0;
          end
          8'h20: begin
            reqcnt_rstf <= 1;
            {reqcnt_rsta, reqcnt_rstb, reqcnt_rstc, reqcnt_rstd, reqcnt_rste, reqcnt_rstg, reqcnt_rsth} <= 0;
          end
          8'h40: begin
            reqcnt_rstg <= 1;
            {reqcnt_rsta, reqcnt_rstb, reqcnt_rstc, reqcnt_rstd, reqcnt_rste, reqcnt_rstf, reqcnt_rsth} <= 0;
          end
          8'h80: begin
            reqcnt_rsth <= 1;
            {reqcnt_rsta, reqcnt_rstb, reqcnt_rstc, reqcnt_rstd, reqcnt_rste, reqcnt_rstf, reqcnt_rstg} <= 0;
          end
        endcase
      end else begin
        if (reqcnt_rsta && im_emp[0]) reqcnt_rsta <= 0;
        if (reqcnt_rstb && im_emp[1]) reqcnt_rstb <= 0;
        if (reqcnt_rstc && im_emp[2]) reqcnt_rstc <= 0;
        if (reqcnt_rstd && im_emp[3]) reqcnt_rstd <= 0;
        if (reqcnt_rste && im_emp[4]) reqcnt_rste <= 0;
        if (reqcnt_rstf && im_emp[5]) reqcnt_rstf <= 0;
        if (reqcnt_rstg && im_emp[6]) reqcnt_rstg <= 0;
        if (reqcnt_rsth && im_emp[7]) reqcnt_rsth <= 0;
      end
      if (reqcnt_rsta && ((|im_enq[`SORT_WAY-1:1])                || im_emp[0])) reqcnt_a <= 0;
      if (reqcnt_rstb && ((|{im_enq[`SORT_WAY-1:2], im_enq[0]})   || im_emp[1])) reqcnt_b <= 0;
      if (reqcnt_rstc && ((|{im_enq[`SORT_WAY-1:3], im_enq[1:0]}) || im_emp[2])) reqcnt_c <= 0;
      if (reqcnt_rstd && ((|{im_enq[`SORT_WAY-1:4], im_enq[2:0]}) || im_emp[3])) reqcnt_d <= 0;
      if (reqcnt_rste && ((|{im_enq[`SORT_WAY-1:5], im_enq[3:0]}) || im_emp[4])) reqcnt_e <= 0;
      if (reqcnt_rstf && ((|{im_enq[`SORT_WAY-1:6], im_enq[4:0]}) || im_emp[5])) reqcnt_f <= 0;
      if (reqcnt_rstg && ((|{im_enq[`SORT_WAY-1],   im_enq[5:0]}) || im_emp[6])) reqcnt_g <= 0;
      if (reqcnt_rsth && ((|im_enq[6:0])                          || im_emp[7])) reqcnt_h <= 0;
    end
  end
  
endmodule

/***** write manager                                                                          *****/
/**************************************************************************************************/
module WRITEMG #(parameter          SORTELM_WAY = (`SORT_ELM>>`WAY_LOG))
                (input  wire        CLK,
                 input  wire        RST,
                 input  wire        pchange,
                 input  wire        p_last,
                 input  wire        mgdrive,
                 input  wire [31:0] elem,
                 input  wire [31:0] elem_way,
                 input  wire [31:0] w_addr, 
                 output reg  [31:0] w_block, 
                 output reg  [31:0] r_endadr_a, 
                 output reg  [31:0] r_endadr_b, 
                 output reg  [31:0] r_endadr_c, 
                 output reg  [31:0] r_endadr_d, 
                 output reg  [31:0] r_endadr_e, 
                 output reg  [31:0] r_endadr_f, 
                 output reg  [31:0] r_endadr_g, 
                 output reg  [31:0] r_endadr_h);
  
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
  
  reg [31:0] adr_a, adr_b, adr_c, adr_d, adr_e, adr_f, adr_g, adr_h;
  reg        reduce_flag;
  
  always @(posedge CLK) begin
    if (RST || pchange) begin
      if (RST) {adr_a, adr_b, adr_c, adr_d, adr_e, adr_f, adr_g, adr_h} <= 0;
      w_block     <= `DRAM_WBLOCKS;
      reduce_flag <= 0;
      r_endadr_a  <= adr_a;
      r_endadr_b  <= adr_b;
      r_endadr_c  <= adr_c;
      r_endadr_d  <= adr_d;
      r_endadr_e  <= adr_e;
      r_endadr_f  <= adr_f;
      r_endadr_g  <= adr_g;
      r_endadr_h  <= adr_h;
    end else begin
      case (p_last)
        1'b0: begin
          if (elem_way >= SORTELM_WAY-(`DRAM_WBLOCKS<<7)) reduce_flag <= 1;
          if (mgdrive && reduce_flag)                     w_block     <= mux32((w_block>>1), 1, (w_block==1));
          case (elem)
            SORTELM_WAY*1: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_a <= w_addr; end
            end
            SORTELM_WAY*2: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_b <= w_addr; end
            end
            SORTELM_WAY*3: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_c <= w_addr; end
            end
            SORTELM_WAY*4: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_d <= w_addr; end
            end
            SORTELM_WAY*5: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_e <= w_addr; end
            end
            SORTELM_WAY*6: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_f <= w_addr; end
            end
            SORTELM_WAY*7: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_g <= w_addr; end
            end
            SORTELM_WAY*8: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_h <= w_addr; end
            end
          endcase
        end
        1'b1: begin
          if (elem >= (SORTELM_WAY*8)-(`DRAM_WBLOCKS<<7)) reduce_flag <= 1;
          if (mgdrive && reduce_flag)                     w_block     <= mux32((w_block>>1), 1, (w_block==1));
        end
      endcase
    end     
  end

endmodule

/***** write manager for last phase                                                           *****/
/**************************************************************************************************/
module WRITEMG_LAST(input  wire        CLK,
                    input  wire        RST,
                    input  wire        mgdrive,
                    input  wire [31:0] elem,
                    input  wire [31:0] w_addr, 
                    output reg  [31:0] w_block, 
                    output reg  [31:0] r_endadr);
  
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
  
  reg reduce_flag;
  always @(posedge CLK) begin
    if (RST) begin
      w_block     <= `DRAM_WBLOCKS;
      reduce_flag <= 0;
      r_endadr    <= 0;
    end else begin
      if (elem >= `SORT_ELM-(`DRAM_WBLOCKS<<7)) reduce_flag <= 1;
      if (mgdrive && reduce_flag)               w_block     <= mux32((w_block>>1), 1, (w_block==1));
      if (elem==`SORT_ELM)                      r_endadr    <= w_addr;
    end     
  end

endmodule

/***** read manager                                                                           *****/
/**************************************************************************************************/
module READMG(input  wire                 CLK,
              input  wire                 RST,
              input  wire                 mgdrive,
              input  wire [`SORT_WAY-1:0] req,
              input  wire                 phase_lsb, 
              input  wire [31:0]          radr_a,
              input  wire [31:0]          radr_b,
              input  wire [31:0]          radr_c,
              input  wire [31:0]          radr_d,
              input  wire [31:0]          radr_e,
              input  wire [31:0]          radr_f,
              input  wire [31:0]          radr_g,
              input  wire [31:0]          radr_h,
              input  wire [31:0]          r_endadr_a, 
              input  wire [31:0]          r_endadr_b, 
              input  wire [31:0]          r_endadr_c, 
              input  wire [31:0]          r_endadr_d, 
              input  wire [31:0]          r_endadr_e, 
              input  wire [31:0]          r_endadr_f, 
              input  wire [31:0]          r_endadr_g, 
              input  wire [31:0]          r_endadr_h, 
              output reg  [31:0]          r_block_a,
              output reg  [31:0]          r_block_b,
              output reg  [31:0]          r_block_c,
              output reg  [31:0]          r_block_d,
              output reg  [31:0]          r_block_e,
              output reg  [31:0]          r_block_f,
              output reg  [31:0]          r_block_g,
              output reg  [31:0]          r_block_h,
              output reg                  readend_a,
              output reg                  readend_b,
              output reg                  readend_c,
              output reg                  readend_d,
              output reg                  readend_e,
              output reg                  readend_f,
              output reg                  readend_g,
              output reg                  readend_h);

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
  
  wire [31:0] way0_radr = mux32(radr_a, (radr_a + (`SORT_ELM>>1)), phase_lsb);
  wire [31:0] way1_radr = mux32(radr_b, (radr_b + (`SORT_ELM>>1)), phase_lsb);
  wire [31:0] way2_radr = mux32(radr_c, (radr_c + (`SORT_ELM>>1)), phase_lsb);
  wire [31:0] way3_radr = mux32(radr_d, (radr_d + (`SORT_ELM>>1)), phase_lsb);
  wire [31:0] way4_radr = mux32(radr_e, (radr_e + (`SORT_ELM>>1)), phase_lsb);
  wire [31:0] way5_radr = mux32(radr_f, (radr_f + (`SORT_ELM>>1)), phase_lsb);
  wire [31:0] way6_radr = mux32(radr_g, (radr_g + (`SORT_ELM>>1)), phase_lsb);
  wire [31:0] way7_radr = mux32(radr_h, (radr_h + (`SORT_ELM>>1)), phase_lsb);
      
  reg reduce_flag_a, reduce_flag_b, reduce_flag_c, reduce_flag_d, reduce_flag_e, reduce_flag_f, reduce_flag_g, reduce_flag_h;
  
  always @(posedge CLK) begin
    if (RST) begin
      r_block_a <= `DRAM_RBLOCKS;
      r_block_b <= `DRAM_RBLOCKS;
      r_block_c <= `DRAM_RBLOCKS;
      r_block_d <= `DRAM_RBLOCKS;
      r_block_e <= `DRAM_RBLOCKS;
      r_block_f <= `DRAM_RBLOCKS;
      r_block_g <= `DRAM_RBLOCKS;
      r_block_h <= `DRAM_RBLOCKS;
      {readend_a,readend_b,readend_c,readend_d,readend_e,readend_f,readend_g,readend_h} <= 0;
      {reduce_flag_a,reduce_flag_b,reduce_flag_c,reduce_flag_d,reduce_flag_e,reduce_flag_f,reduce_flag_g,reduce_flag_h} <= 0;
    end else begin
      readend_a <= (r_endadr_a == way0_radr);
      readend_b <= (r_endadr_b == way1_radr);
      readend_c <= (r_endadr_c == way2_radr);
      readend_d <= (r_endadr_d == way3_radr);
      readend_e <= (r_endadr_e == way4_radr);
      readend_f <= (r_endadr_f == way5_radr);
      readend_g <= (r_endadr_g == way6_radr);
      readend_h <= (r_endadr_h == way7_radr);
      if (r_endadr_a-((`D_RS)<<2) <= way0_radr) reduce_flag_a <= 1;
      if (r_endadr_b-((`D_RS)<<2) <= way1_radr) reduce_flag_b <= 1;
      if (r_endadr_c-((`D_RS)<<2) <= way2_radr) reduce_flag_c <= 1;
      if (r_endadr_d-((`D_RS)<<2) <= way3_radr) reduce_flag_d <= 1;
      if (r_endadr_e-((`D_RS)<<2) <= way4_radr) reduce_flag_e <= 1;
      if (r_endadr_f-((`D_RS)<<2) <= way5_radr) reduce_flag_f <= 1;
      if (r_endadr_g-((`D_RS)<<2) <= way6_radr) reduce_flag_g <= 1;
      if (r_endadr_h-((`D_RS)<<2) <= way7_radr) reduce_flag_h <= 1;
      if (mgdrive) begin
        case (req)
          8'h01: if (reduce_flag_a) r_block_a <= mux32((r_block_a>>1), 1, (r_block_a==1));
          8'h02: if (reduce_flag_b) r_block_b <= mux32((r_block_b>>1), 1, (r_block_b==1));
          8'h04: if (reduce_flag_c) r_block_c <= mux32((r_block_c>>1), 1, (r_block_c==1));
          8'h08: if (reduce_flag_d) r_block_d <= mux32((r_block_d>>1), 1, (r_block_d==1));
          8'h10: if (reduce_flag_e) r_block_e <= mux32((r_block_e>>1), 1, (r_block_e==1));
          8'h20: if (reduce_flag_f) r_block_f <= mux32((r_block_f>>1), 1, (r_block_f==1));
          8'h40: if (reduce_flag_g) r_block_g <= mux32((r_block_g>>1), 1, (r_block_g==1));
          8'h80: if (reduce_flag_h) r_block_h <= mux32((r_block_h>>1), 1, (r_block_h==1));
        endcase
      end
    end
  end
  
endmodule

/***** dummy logic                                                                            *****/
/**************************************************************************************************/
module CORE_W(input  wire            CLK,            // clock
              input  wire              RST_in,       // reset
              output reg               initdone,     // dram initialize is done
              output reg               sortdone,     // sort is finished
              input  wire              d_busy_in,    // DRAM busy              
              input  wire [1:0]        d_mode_in,    // DRAM mode
              input  wire              din_bit,      // DRAM data out
              input  wire              din_en_in,    // DRAM data out enable
              output reg [3:0]         data_out,     // DRAM data in
              input  wire              d_w_in,       // DRAM write flag
              output reg [1:0]         d_req,        // DRAM REQ access request (read/write)
              output reg [31:0]        d_initadr,    // DRAM REQ initial address for the access
              output reg [31:0]        d_blocks,     // DRAM REQ the number of blocks per one access
              output wire              ERROR);       //
              
  reg RST;            always @(posedge CLK) RST <= RST_in;
  wire initdone_w;      always @(posedge CLK) initdone <= initdone_w;
  wire sortdone_w;      always @(posedge CLK) sortdone <= sortdone_w;
  reg d_busy;           always @(posedge CLK) d_busy <= d_busy_in;
  reg [1:0] d_mode;     always @(posedge CLK) d_mode <= d_mode_in;
  reg [`DRAMW-1:0] din; always @(posedge CLK) din <= (RST) ? 0 : {din[`DRAMW-2:0], din_bit};
  reg din_en;           always @(posedge CLK) din_en <= din_en_in;
  wire [1:0] d_req_w;   always @(posedge CLK) d_req <= d_req_w;

  wire dout_en;
  wire [`DRAMW-1:0] dout;
  reg  [`DRAMW-1:0] dout_r;
  always @(posedge CLK) dout_r <= dout;

  reg d_w;   always @(posedge CLK) d_w <= d_w_in;
  always @(posedge CLK) data_out <= {^dout_r[127:0], ^dout_r[128+127:128], 
                                     ^dout_r[256+127:256],  ^dout_r[384+127:384]};
  
  wire [31:0] d_initadr_w, d_blocks_w;
  always @(posedge CLK) d_initadr <= d_initadr_w;
  always @(posedge CLK) d_blocks  <= d_blocks_w;
  
  CORE core(CLK, RST, initdone_w, sortdone_w,
            d_busy, dout, d_w, din, din_en, d_req_w, d_initadr_w, d_blocks_w, ERROR);
endmodule

/***** Core User Logic                                                                        *****/
/**************************************************************************************************/
module CORE(input  wire              CLK,          // clock
            input  wire              RST_IN,       // reset
            output reg               initdone,     // dram initialize is done
            output reg               sortdone,     // sort is finished
            input  wire              d_busy,       // DRAM busy
            output wire [`DRAMW-1:0] d_din,        // DRAM data in
            input  wire              d_w,          // DRAM write flag
            input  wire [`DRAMW-1:0] d_dout,       // DRAM data out
            input  wire              d_douten,     // DRAM data out enable
            output reg  [1:0]        d_req,        // DRAM REQ access request (read/write)
            output reg  [31:0]       d_initadr,    // DRAM REQ initial address for the access
            output reg  [31:0]       d_blocks,     // DRAM REQ the number of blocks per one access
            output reg               ERROR);       // Sorting value ERROR ?
           
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
  
  function [4-1:0] mux8in4;
    input [4-1:0] a;
    input [4-1:0] b;
    input [4-1:0] c;
    input [4-1:0] d;
    input [4-1:0] e;
    input [4-1:0] f;
    input [4-1:0] g;
    input [4-1:0] h;
    input [7:0]   sel;
    begin
      case (sel)
        8'h01: mux8in4 = a;
        8'h02: mux8in4 = b;
        8'h04: mux8in4 = c; 
        8'h08: mux8in4 = d;
        8'h10: mux8in4 = e;
        8'h20: mux8in4 = f; 
        8'h40: mux8in4 = g;
        8'h80: mux8in4 = h;
      endcase
    end
  endfunction
  
  function [`SORT_WAY-1:0] mux8insortway;
    input [`SORT_WAY-1:0] a;
    input [`SORT_WAY-1:0] b;
    input [`SORT_WAY-1:0] c;
    input [`SORT_WAY-1:0] d;
    input [`SORT_WAY-1:0] e;
    input [`SORT_WAY-1:0] f;
    input [`SORT_WAY-1:0] g;
    input [`SORT_WAY-1:0] h;
    input [7:0]           sel;
    begin
      case (sel)
        8'h01: mux8insortway = a;
        8'h02: mux8insortway = b;
        8'h04: mux8insortway = c; 
        8'h08: mux8insortway = d;
        8'h10: mux8insortway = e;
        8'h20: mux8insortway = f; 
        8'h40: mux8insortway = g;
        8'h80: mux8insortway = h;
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
  
  function [32-1:0] mux8in32;
    input [32-1:0] a;
    input [32-1:0] b;
    input [32-1:0] c;
    input [32-1:0] d; 
    input [32-1:0] e; 
    input [32-1:0] f; 
    input [32-1:0] g; 
    input [32-1:0] h; 
    input [7:0]   sel;
    begin
      case (sel)
        8'h01: mux8in32 = a;
        8'h02: mux8in32 = b;
        8'h04: mux8in32 = c; 
        8'h08: mux8in32 = d;
        8'h10: mux8in32 = e;
        8'h20: mux8in32 = f; 
        8'h40: mux8in32 = g;
        8'h80: mux8in32 = h;
      endcase
    end
  endfunction
  
  /**********************************************************************************************/
  reg idone_a;
  reg idone_b; 
  reg idone_c;
  reg idone_d; 
  reg idone_e;
  reg idone_f; 
  reg idone_g;
  reg idone_h; 

  wire [`DRAMW-1:0] OB_dot0;  
  wire [`DRAMW-1:0] OB_dot1;  
  wire [`DRAMW-1:0] OB_dot2;  
  wire [`DRAMW-1:0] OB_dot3;  
  wire [`DRAMW-1:0] OB_dot4;  
  wire [`DRAMW-1:0] OB_dot5;  
  wire [`DRAMW-1:0] OB_dot6;  
  wire [`DRAMW-1:0] OB_dot7;  
  
  wire OB_req_a;  
  wire OB_req_b;  
  wire OB_req_c;  
  wire OB_req_d;  
  wire OB_req_e;  
  wire OB_req_f;  
  wire OB_req_g;  
  wire OB_req_h;  
  
  wire OB_full0; 
  wire OB_full1; 
  wire OB_full2; 
  wire OB_full3; 
  wire OB_full4; 
  wire OB_full5; 
  wire OB_full6; 
  wire OB_full7; 
    
  reg  OB_deq_ta;
  reg  OB_deq_tb;
  reg  OB_deq_tc;
  reg  OB_deq_td;
  reg  OB_deq_te;
  reg  OB_deq_tf;
  reg  OB_deq_tg;
  reg  OB_deq_th;
  
  wire [7:0] OB_dot_sel ={OB_deq_th, OB_deq_tg, OB_deq_tf, OB_deq_te, 
                          OB_deq_td, OB_deq_tc, OB_deq_tb, OB_deq_ta};
  
  wire OB_deq0 = d_w && OB_deq_ta;
  wire OB_deq1 = d_w && OB_deq_tb;
  wire OB_deq2 = d_w && OB_deq_tc;
  wire OB_deq3 = d_w && OB_deq_td;
  wire OB_deq4 = d_w && OB_deq_te;
  wire OB_deq5 = d_w && OB_deq_tf;
  wire OB_deq6 = d_w && OB_deq_tg;
  wire OB_deq7 = d_w && OB_deq_th;
  
  reg  OB_dataen_a;
  reg  OB_dataen_b;
  reg  OB_dataen_c;
  reg  OB_dataen_d;
  reg  OB_dataen_e;
  reg  OB_dataen_f;
  reg  OB_dataen_g;
  reg  OB_dataen_h;
  
  wire [`PHASE_W] l_phase = `LAST_PHASE;
  
  reg [`DRAMW-1:0] dout_t;  
  reg [`DRAMW-1:0] dout_tta, dout_ttb, dout_ttc, dout_ttd, dout_tte, dout_ttf;
  reg [`DRAMW-1:0] dout_t0_a, dout_t1_a, dout_t2_a, dout_t3_a, dout_t4_a, dout_t5_a, dout_t6_a;
  reg [`DRAMW-1:0] dout_t0_b, dout_t1_b, dout_t2_b, dout_t3_b, dout_t4_b, dout_t5_b, dout_t6_b;
  reg [`DRAMW-1:0] dout_t0_c, dout_t1_c, dout_t2_c, dout_t3_c, dout_t4_c, dout_t5_c, dout_t6_c;
  reg [`DRAMW-1:0] dout_t0_d, dout_t1_d, dout_t2_d, dout_t3_d, dout_t4_d, dout_t5_d, dout_t6_d;
  reg [`DRAMW-1:0] dout_t0_e, dout_t1_e, dout_t2_e, dout_t3_e, dout_t4_e, dout_t5_e, dout_t6_e;
  reg [`DRAMW-1:0] dout_t0_f, dout_t1_f, dout_t2_f, dout_t3_f, dout_t4_f, dout_t5_f, dout_t6_f;
  reg [`DRAMW-1:0] dout_t0_g, dout_t1_g, dout_t2_g, dout_t3_g, dout_t4_g, dout_t5_g, dout_t6_g;
  reg [`DRAMW-1:0] dout_t0_h, dout_t1_h, dout_t2_h, dout_t3_h, dout_t4_h, dout_t5_h, dout_t6_h;
  reg [`DRAMW-1:0] dout_ta_a, dout_tb_a, dout_tc_a, dout_td_a, dout_te_a, dout_tf_a, dout_tg_a, dout_th_a;
  reg [`DRAMW-1:0] dout_ta_b, dout_tb_b, dout_tc_b, dout_td_b, dout_te_b, dout_tf_b, dout_tg_b, dout_th_b;
  reg [`DRAMW-1:0] dout_ta_c, dout_tb_c, dout_tc_c, dout_td_c, dout_te_c, dout_tf_c, dout_tg_c, dout_th_c;
  reg [`DRAMW-1:0] dout_ta_d, dout_tb_d, dout_tc_d, dout_td_d, dout_te_d, dout_tf_d, dout_tg_d, dout_th_d;
  reg [`DRAMW-1:0] dout_ta_e, dout_tb_e, dout_tc_e, dout_td_e, dout_te_e, dout_tf_e, dout_tg_e, dout_th_e;
  reg [`DRAMW-1:0] dout_ta_f, dout_tb_f, dout_tc_f, dout_td_f, dout_te_f, dout_tf_f, dout_tg_f, dout_th_f;
  reg [`DRAMW-1:0] dout_ta_g, dout_tb_g, dout_tc_g, dout_td_g, dout_te_g, dout_tf_g, dout_tg_g, dout_th_g;
  reg [`DRAMW-1:0] dout_ta_h, dout_tb_h, dout_tc_h, dout_td_h, dout_te_h, dout_tf_h, dout_tg_h, dout_th_h;
  
  reg              doen_t;
  reg              doen_tta, doen_ttb, doen_ttc, doen_ttd, doen_tte, doen_ttf;  
  reg              doen_t0_a, doen_t1_a, doen_t2_a, doen_t3_a, doen_t4_a, doen_t5_a, doen_t6_a;
  reg              doen_t0_b, doen_t1_b, doen_t2_b, doen_t3_b, doen_t4_b, doen_t5_b, doen_t6_b;
  reg              doen_t0_c, doen_t1_c, doen_t2_c, doen_t3_c, doen_t4_c, doen_t5_c, doen_t6_c;
  reg              doen_t0_d, doen_t1_d, doen_t2_d, doen_t3_d, doen_t4_d, doen_t5_d, doen_t6_d;
  reg              doen_t0_e, doen_t1_e, doen_t2_e, doen_t3_e, doen_t4_e, doen_t5_e, doen_t6_e;
  reg              doen_t0_f, doen_t1_f, doen_t2_f, doen_t3_f, doen_t4_f, doen_t5_f, doen_t6_f;
  reg              doen_t0_g, doen_t1_g, doen_t2_g, doen_t3_g, doen_t4_g, doen_t5_g, doen_t6_g;
  reg              doen_t0_h, doen_t1_h, doen_t2_h, doen_t3_h, doen_t4_h, doen_t5_h, doen_t6_h;
  reg              doen_ta_a, doen_tb_a, doen_tc_a, doen_td_a, doen_te_a, doen_tf_a, doen_tg_a, doen_th_a;
  reg              doen_ta_b, doen_tb_b, doen_tc_b, doen_td_b, doen_te_b, doen_tf_b, doen_tg_b, doen_th_b;
  reg              doen_ta_c, doen_tb_c, doen_tc_c, doen_td_c, doen_te_c, doen_tf_c, doen_tg_c, doen_th_c;
  reg              doen_ta_d, doen_tb_d, doen_tc_d, doen_td_d, doen_te_d, doen_tf_d, doen_tg_d, doen_th_d;
  reg              doen_ta_e, doen_tb_e, doen_tc_e, doen_td_e, doen_te_e, doen_tf_e, doen_tg_e, doen_th_e;
  reg              doen_ta_f, doen_tb_f, doen_tc_f, doen_td_f, doen_te_f, doen_tf_f, doen_tg_f, doen_th_f;
  reg              doen_ta_g, doen_tb_g, doen_tc_g, doen_td_g, doen_te_g, doen_tf_g, doen_tg_g, doen_th_g;
  reg              doen_ta_h, doen_tb_h, doen_tc_h, doen_td_h, doen_te_h, doen_tf_h, doen_tg_h, doen_th_h;

  reg [`SORT_WAY-1:0] req_tt0_a, req_tt1_a, req_tt2_a, req_tt3_a, req_tt4_a, req_tt5_a;    
  reg [`SORT_WAY-1:0] req_tt0_b, req_tt1_b, req_tt2_b, req_tt3_b, req_tt4_b, req_tt5_b;    
  reg [`SORT_WAY-1:0] req_tt0_c, req_tt1_c, req_tt2_c, req_tt3_c, req_tt4_c, req_tt5_c;    
  reg [`SORT_WAY-1:0] req_tt0_d, req_tt1_d, req_tt2_d, req_tt3_d, req_tt4_d, req_tt5_d;    
  reg [`SORT_WAY-1:0] req_tt0_e, req_tt1_e, req_tt2_e, req_tt3_e, req_tt4_e, req_tt5_e;    
  reg [`SORT_WAY-1:0] req_tt0_f, req_tt1_f, req_tt2_f, req_tt3_f, req_tt4_f, req_tt5_f;    
  reg [`SORT_WAY-1:0] req_tt0_g, req_tt1_g, req_tt2_g, req_tt3_g, req_tt4_g, req_tt5_g;    
  reg [`SORT_WAY-1:0] req_tt0_h, req_tt1_h, req_tt2_h, req_tt3_h, req_tt4_h, req_tt5_h;    

  reg [`SORT_WAY-1:0] req_ta;     
  reg [`SORT_WAY-1:0] req_tb;     
  reg [`SORT_WAY-1:0] req_tc;     
  reg [`SORT_WAY-1:0] req_td;
  reg [`SORT_WAY-1:0] req_te;     
  reg [`SORT_WAY-1:0] req_tf;     
  reg [`SORT_WAY-1:0] req_tg;     
  reg [`SORT_WAY-1:0] req_th;
  
  reg [(1<<`P_LOG)-1:0] req_g_sel;
  reg req_gga, req_ggb, req_ggc, req_ggd, req_gge, req_ggf, req_ggg, req_ggh;
  reg req_ga, req_gb, req_gc, req_gd, req_ge, req_gf, req_gg, req_gh;
  reg [`SORT_WAY-1:0] req_a, req_b, req_c, req_d, req_e, req_f, req_g, req_h;
  reg [`SORT_WAY-1:0] req;

  reg [31:0]          elem;      
  reg [31:0]          elem_a;      
  reg [31:0]          elem_b;      
  reg [31:0]          elem_c;      
  reg [31:0]          elem_d;      
  reg [31:0]          elem_e;      
  reg [31:0]          elem_f;      
  reg [31:0]          elem_g;      
  reg [31:0]          elem_h;      

  reg [31:0]          elem_way_a;      
  reg [31:0]          elem_way_b;      
  reg [31:0]          elem_way_c;      
  reg [31:0]          elem_way_d;      
  reg [31:0]          elem_way_e;      
  reg [31:0]          elem_way_f;      
  reg [31:0]          elem_way_g;      
  reg [31:0]          elem_way_h;

  reg                 elem_en_a;
  reg                 elem_en_b;
  reg                 elem_en_c;
  reg                 elem_en_d;
  reg                 elem_en_e;
  reg                 elem_en_f;
  reg                 elem_en_g;
  reg                 elem_en_h;

  reg [`PHASE_W]        phase;
  reg [`PHASE_W]        phase_a;     
  reg [`PHASE_W]        phase_b;     
  reg [`PHASE_W]        phase_c;     
  reg [`PHASE_W]        phase_d;     
  reg [`PHASE_W]        phase_e;     
  reg [`PHASE_W]        phase_f;     
  reg [`PHASE_W]        phase_g;     
  reg [`PHASE_W]        phase_h;     

  reg                 last_phase;
  reg                 last_phase_a;
  reg                 last_phase_b;
  reg                 last_phase_c;
  reg                 last_phase_d;
  reg                 last_phase_e;
  reg                 last_phase_f;

  reg                 pchange_a;   
  reg                 pchange_b;   
  reg                 pchange_c;   
  reg                 pchange_d;   
  reg                 pchange_e;   
  reg                 pchange_f;   
  reg                 pchange_g;   
  reg                 pchange_h;   

  reg                 iter_done_a; 
  reg                 iter_done_b; 
  reg                 iter_done_c; 
  reg                 iter_done_d; 
  reg                 iter_done_e; 
  reg                 iter_done_f; 
  reg                 iter_done_g; 
  reg                 iter_done_h; 

  reg [31:0]          ecnt;      
  reg [31:0]          ecnt_a;      
  reg [31:0]          ecnt_b;      
  reg [31:0]          ecnt_c;      
  reg [31:0]          ecnt_d;
  reg [31:0]          ecnt_e;      
  reg [31:0]          ecnt_f;      
  reg [31:0]          ecnt_g;      
  reg [31:0]          ecnt_h;
  
  reg                 irst_a;      
  reg                 irst_b;      
  reg                 irst_c;      
  reg                 irst_d;      
  reg                 irst_e;      
  reg                 irst_f;      
  reg                 irst_g;      
  reg                 irst_h;      

  reg                 frst_a;    
  reg                 frst_b;    
  reg                 frst_c;    
  reg                 frst_d;    
  reg                 frst_e;    
  reg                 frst_f;    
  reg                 frst_g;    
  reg                 frst_h;    

  reg                 pexe_done_a;
  reg                 pexe_done_b;
  reg                 pexe_done_c;
  reg                 pexe_done_d;
  reg                 pexe_done_e;
  reg                 pexe_done_f;
  reg                 pexe_done_g;
  reg                 pexe_done_h;
  
  reg                 pexe_done_a_p;
  reg                 pexe_done_b_p;
  reg                 pexe_done_c_p;
  reg                 pexe_done_d_p;
  reg                 pexe_done_e_p;
  reg                 pexe_done_f_p;
  reg                 pexe_done_g_p;
  reg                 pexe_done_h_p;
  
  reg RSTa;
  always @(posedge CLK) RSTa <= RST_IN;
  reg RSTb;
  always @(posedge CLK) RSTb <= RST_IN;
  reg RSTc;
  always @(posedge CLK) RSTc <= RST_IN;
  reg RSTd;
  always @(posedge CLK) RSTd <= RST_IN;
  reg RSTe;
  always @(posedge CLK) RSTe <= RST_IN;
  reg RSTf;
  always @(posedge CLK) RSTf <= RST_IN;
  reg RSTg;
  always @(posedge CLK) RSTg <= RST_IN;
  reg RSTh;
  always @(posedge CLK) RSTh <= RST_IN;

  /**********************************************************************************************/
  wire [`SORTW-1:0] d00_0, d01_0, d02_0, d03_0, d04_0, d05_0, d06_0, d07_0;
  wire [`SORTW-1:0] d00_1, d01_1, d02_1, d03_1, d04_1, d05_1, d06_1, d07_1;
  wire [`SORTW-1:0] d00_2, d01_2, d02_2, d03_2, d04_2, d05_2, d06_2, d07_2;
  wire [`SORTW-1:0] d00_3, d01_3, d02_3, d03_3, d04_3, d05_3, d06_3, d07_3;
  wire [`SORTW-1:0] d00_4, d01_4, d02_4, d03_4, d04_4, d05_4, d06_4, d07_4;
  wire [`SORTW-1:0] d00_5, d01_5, d02_5, d03_5, d04_5, d05_5, d06_5, d07_5;
  wire [`SORTW-1:0] d00_6, d01_6, d02_6, d03_6, d04_6, d05_6, d06_6, d07_6;
  wire [`SORTW-1:0] d00_7, d01_7, d02_7, d03_7, d04_7, d05_7, d06_7, d07_7;
  
  wire ib00_req_a, ib01_req_a, ib02_req_a, ib03_req_a, ib04_req_a, ib05_req_a, ib06_req_a, ib07_req_a;
  wire ib00_req_b, ib01_req_b, ib02_req_b, ib03_req_b, ib04_req_b, ib05_req_b, ib06_req_b, ib07_req_b;
  wire ib00_req_c, ib01_req_c, ib02_req_c, ib03_req_c, ib04_req_c, ib05_req_c, ib06_req_c, ib07_req_c;
  wire ib00_req_d, ib01_req_d, ib02_req_d, ib03_req_d, ib04_req_d, ib05_req_d, ib06_req_d, ib07_req_d;
  wire ib00_req_e, ib01_req_e, ib02_req_e, ib03_req_e, ib04_req_e, ib05_req_e, ib06_req_e, ib07_req_e;
  wire ib00_req_f, ib01_req_f, ib02_req_f, ib03_req_f, ib04_req_f, ib05_req_f, ib06_req_f, ib07_req_f;
  wire ib00_req_g, ib01_req_g, ib02_req_g, ib03_req_g, ib04_req_g, ib05_req_g, ib06_req_g, ib07_req_g;
  wire ib00_req_h, ib01_req_h, ib02_req_h, ib03_req_h, ib04_req_h, ib05_req_h, ib06_req_h, ib07_req_h;

  wire F01_emp0;
  wire F01_emp1;
  wire F01_emp2;
  wire F01_emp3;
  wire F01_emp4;
  wire F01_emp5;
  wire F01_emp6;
  wire F01_emp7;
  
  wire F01_deq0 = !F01_emp0 && !OB_full0;
  wire F01_deq1 = !F01_emp1 && !OB_full1;
  wire F01_deq2 = !F01_emp2 && !OB_full2;
  wire F01_deq3 = !F01_emp3 && !OB_full3;
  wire F01_deq4 = !F01_emp4 && !OB_full4;
  wire F01_deq5 = !F01_emp5 && !OB_full5;
  wire F01_deq6 = !F01_emp6 && !OB_full6;
  wire F01_deq7 = !F01_emp7 && !OB_full7;
  
  wire [`SORTW-1:0] F01_dot0;
  wire [`SORTW-1:0] F01_dot1;
  wire [`SORTW-1:0] F01_dot2;
  wire [`SORTW-1:0] F01_dot3;
  wire [`SORTW-1:0] F01_dot4;
  wire [`SORTW-1:0] F01_dot5;
  wire [`SORTW-1:0] F01_dot6;
  wire [`SORTW-1:0] F01_dot7;
  
  wire [`SORTW*`SORT_WAY-1:0] s_din0 = {d00_0, d01_0, d02_0, d03_0, d04_0, d05_0, d06_0, d07_0};
  wire [`SORTW*`SORT_WAY-1:0] s_din1 = {d00_1, d01_1, d02_1, d03_1, d04_1, d05_1, d06_1, d07_1};
  wire [`SORTW*`SORT_WAY-1:0] s_din2 = {d00_2, d01_2, d02_2, d03_2, d04_2, d05_2, d06_2, d07_2};
  wire [`SORTW*`SORT_WAY-1:0] s_din3 = {d00_3, d01_3, d02_3, d03_3, d04_3, d05_3, d06_3, d07_3};
  wire [`SORTW*`SORT_WAY-1:0] s_din4 = {d00_4, d01_4, d02_4, d03_4, d04_4, d05_4, d06_4, d07_4};
  wire [`SORTW*`SORT_WAY-1:0] s_din5 = {d00_5, d01_5, d02_5, d03_5, d04_5, d05_5, d06_5, d07_5};
  wire [`SORTW*`SORT_WAY-1:0] s_din6 = {d00_6, d01_6, d02_6, d03_6, d04_6, d05_6, d06_6, d07_6};
  wire [`SORTW*`SORT_WAY-1:0] s_din7 = {d00_7, d01_7, d02_7, d03_7, d04_7, d05_7, d06_7, d07_7};
  
  wire [`SORT_WAY-1:0] enq0; 
  wire [`SORT_WAY-1:0] enq1; 
  wire [`SORT_WAY-1:0] enq2; 
  wire [`SORT_WAY-1:0] enq3;
  wire [`SORT_WAY-1:0] enq4; 
  wire [`SORT_WAY-1:0] enq5; 
  wire [`SORT_WAY-1:0] enq6; 
  wire [`SORT_WAY-1:0] enq7;
  
  wire [`SORT_WAY-1:0] s_ful0;
  wire [`SORT_WAY-1:0] s_ful1;
  wire [`SORT_WAY-1:0] s_ful2;
  wire [`SORT_WAY-1:0] s_ful3;
  wire [`SORT_WAY-1:0] s_ful4;
  wire [`SORT_WAY-1:0] s_ful5;
  wire [`SORT_WAY-1:0] s_ful6;
  wire [`SORT_WAY-1:0] s_ful7;

  
  wire [`SORT_WAY-1:0]                  dc_req_t = mux8insortway(req_ta, req_tb, req_tc, req_td, req_te, req_tf, req_tg, req_th, req_g_sel);
  wire [`PHASE_W]                       dc_phase = mux8in4(phase_a, phase_b, phase_c, phase_d, phase_e, phase_f, phase_g, phase_h, req_g_sel); 
  wire [(1<<`P_LOG)+`SORT_WAY+`DRAMW:0] dc_din = {req_g_sel, dc_req_t, (dc_phase!=0), d_dout};
  wire [`DRAMW-1:0]                     dc_dout;
  wire [(1<<`P_LOG)+`SORT_WAY:0]        dc_douten;
  wire                                  dc_req;
 
  DECOMPRESSOR #(`IB_SIZE, `DRAM_RBLOCKS)
  decompressor(CLK, RSTa, dc_din, d_douten, dc_dout, dc_douten, dc_req);
  
  wire [`DRAMW-1:0]              stnet_dout;
  wire [(1<<`P_LOG)+`SORT_WAY:0] stnet_douten;
  SORTINGNETWORK sortingnetwork(CLK, RSTa, dc_douten, dc_dout, stnet_dout, stnet_douten);

  ////////////////////////////////////////////////
  wire im00_enq0 = doen_ta_a & req_tt5_a[0];
  wire im01_enq0 = doen_tb_a & req_tt5_a[1];
  wire im02_enq0 = doen_tc_a & req_tt5_a[2];
  wire im03_enq0 = doen_td_a & req_tt5_a[3];
  wire im04_enq0 = doen_te_a & req_tt5_a[4];
  wire im05_enq0 = doen_tf_a & req_tt5_a[5];
  wire im06_enq0 = doen_tg_a & req_tt5_a[6];
  wire im07_enq0 = doen_th_a & req_tt5_a[7];

  wire im00_emp0;
  wire im01_emp0;
  wire im02_emp0;
  wire im03_emp0;
  wire im04_emp0;
  wire im05_emp0;
  wire im06_emp0;
  wire im07_emp0;

  INMOD2 im00_0(CLK, RSTa, dout_ta_a, im00_enq0, s_ful0[0], d00_0, enq0[0], im00_emp0, ib00_req_a);
  INMOD2 im01_0(CLK, RSTa, dout_tb_a, im01_enq0, s_ful0[1], d01_0, enq0[1], im01_emp0, ib01_req_a);
  INMOD2 im02_0(CLK, RSTa, dout_tc_a, im02_enq0, s_ful0[2], d02_0, enq0[2], im02_emp0, ib02_req_a);
  INMOD2 im03_0(CLK, RSTa, dout_td_a, im03_enq0, s_ful0[3], d03_0, enq0[3], im03_emp0, ib03_req_a);
  INMOD2 im04_0(CLK, RSTa, dout_te_a, im04_enq0, s_ful0[4], d04_0, enq0[4], im04_emp0, ib04_req_a);
  INMOD2 im05_0(CLK, RSTa, dout_tf_a, im05_enq0, s_ful0[5], d05_0, enq0[5], im05_emp0, ib05_req_a);
  INMOD2 im06_0(CLK, RSTa, dout_tg_a, im06_enq0, s_ful0[6], d06_0, enq0[6], im06_emp0, ib06_req_a);
  INMOD2 im07_0(CLK, RSTa, dout_th_a, im07_enq0, s_ful0[7], d07_0, enq0[7], im07_emp0, ib07_req_a);

  ////////////////////////////////////////////////
  wire im00_enq1 = doen_ta_b & req_tt5_b[0];
  wire im01_enq1 = doen_tb_b & req_tt5_b[1];
  wire im02_enq1 = doen_tc_b & req_tt5_b[2];
  wire im03_enq1 = doen_td_b & req_tt5_b[3];
  wire im04_enq1 = doen_te_b & req_tt5_b[4];
  wire im05_enq1 = doen_tf_b & req_tt5_b[5];
  wire im06_enq1 = doen_tg_b & req_tt5_b[6];
  wire im07_enq1 = doen_th_b & req_tt5_b[7];
  
  wire im00_emp1;
  wire im01_emp1;
  wire im02_emp1;
  wire im03_emp1;
  wire im04_emp1;
  wire im05_emp1;
  wire im06_emp1;
  wire im07_emp1;
  
  INMOD2 im00_1(CLK, RSTb, dout_ta_b, im00_enq1, s_ful1[0], d00_1, enq1[0], im00_emp1, ib00_req_b);
  INMOD2 im01_1(CLK, RSTb, dout_tb_b, im01_enq1, s_ful1[1], d01_1, enq1[1], im01_emp1, ib01_req_b);
  INMOD2 im02_1(CLK, RSTb, dout_tc_b, im02_enq1, s_ful1[2], d02_1, enq1[2], im02_emp1, ib02_req_b);
  INMOD2 im03_1(CLK, RSTb, dout_td_b, im03_enq1, s_ful1[3], d03_1, enq1[3], im03_emp1, ib03_req_b);
  INMOD2 im04_1(CLK, RSTb, dout_te_b, im04_enq1, s_ful1[4], d04_1, enq1[4], im04_emp1, ib04_req_b);
  INMOD2 im05_1(CLK, RSTb, dout_tf_b, im05_enq1, s_ful1[5], d05_1, enq1[5], im05_emp1, ib05_req_b);
  INMOD2 im06_1(CLK, RSTb, dout_tg_b, im06_enq1, s_ful1[6], d06_1, enq1[6], im06_emp1, ib06_req_b);
  INMOD2 im07_1(CLK, RSTb, dout_th_b, im07_enq1, s_ful1[7], d07_1, enq1[7], im07_emp1, ib07_req_b);

  ////////////////////////////////////////////////
  wire im00_enq2 = doen_ta_c & req_tt5_c[0];
  wire im01_enq2 = doen_tb_c & req_tt5_c[1];
  wire im02_enq2 = doen_tc_c & req_tt5_c[2];
  wire im03_enq2 = doen_td_c & req_tt5_c[3];
  wire im04_enq2 = doen_te_c & req_tt5_c[4];
  wire im05_enq2 = doen_tf_c & req_tt5_c[5];
  wire im06_enq2 = doen_tg_c & req_tt5_c[6];
  wire im07_enq2 = doen_th_c & req_tt5_c[7];

  wire im00_emp2;
  wire im01_emp2;
  wire im02_emp2;
  wire im03_emp2;
  wire im04_emp2;
  wire im05_emp2;
  wire im06_emp2;
  wire im07_emp2;
  
  INMOD2 im00_2(CLK, RSTc, dout_ta_c, im00_enq2, s_ful2[0], d00_2, enq2[0], im00_emp2, ib00_req_c);
  INMOD2 im01_2(CLK, RSTc, dout_tb_c, im01_enq2, s_ful2[1], d01_2, enq2[1], im01_emp2, ib01_req_c);
  INMOD2 im02_2(CLK, RSTc, dout_tc_c, im02_enq2, s_ful2[2], d02_2, enq2[2], im02_emp2, ib02_req_c);
  INMOD2 im03_2(CLK, RSTc, dout_td_c, im03_enq2, s_ful2[3], d03_2, enq2[3], im03_emp2, ib03_req_c);
  INMOD2 im04_2(CLK, RSTc, dout_te_c, im04_enq2, s_ful2[4], d04_2, enq2[4], im04_emp2, ib04_req_c);
  INMOD2 im05_2(CLK, RSTc, dout_tf_c, im05_enq2, s_ful2[5], d05_2, enq2[5], im05_emp2, ib05_req_c);
  INMOD2 im06_2(CLK, RSTc, dout_tg_c, im06_enq2, s_ful2[6], d06_2, enq2[6], im06_emp2, ib06_req_c);
  INMOD2 im07_2(CLK, RSTc, dout_th_c, im07_enq2, s_ful2[7], d07_2, enq2[7], im07_emp2, ib07_req_c);

  ////////////////////////////////////////////////
  wire im00_enq3 = doen_ta_d & req_tt5_d[0];
  wire im01_enq3 = doen_tb_d & req_tt5_d[1];
  wire im02_enq3 = doen_tc_d & req_tt5_d[2];
  wire im03_enq3 = doen_td_d & req_tt5_d[3];
  wire im04_enq3 = doen_te_d & req_tt5_d[4];
  wire im05_enq3 = doen_tf_d & req_tt5_d[5];
  wire im06_enq3 = doen_tg_d & req_tt5_d[6];
  wire im07_enq3 = doen_th_d & req_tt5_d[7];

  wire im00_emp3;
  wire im01_emp3;
  wire im02_emp3;
  wire im03_emp3;
  wire im04_emp3;
  wire im05_emp3;
  wire im06_emp3;
  wire im07_emp3;
  
  INMOD2 im00_3(CLK, RSTd, dout_ta_d, im00_enq3, s_ful3[0], d00_3, enq3[0], im00_emp3, ib00_req_d);
  INMOD2 im01_3(CLK, RSTd, dout_tb_d, im01_enq3, s_ful3[1], d01_3, enq3[1], im01_emp3, ib01_req_d);
  INMOD2 im02_3(CLK, RSTd, dout_tc_d, im02_enq3, s_ful3[2], d02_3, enq3[2], im02_emp3, ib02_req_d);
  INMOD2 im03_3(CLK, RSTd, dout_td_d, im03_enq3, s_ful3[3], d03_3, enq3[3], im03_emp3, ib03_req_d);
  INMOD2 im04_3(CLK, RSTd, dout_te_d, im04_enq3, s_ful3[4], d04_3, enq3[4], im04_emp3, ib04_req_d);
  INMOD2 im05_3(CLK, RSTd, dout_tf_d, im05_enq3, s_ful3[5], d05_3, enq3[5], im05_emp3, ib05_req_d);
  INMOD2 im06_3(CLK, RSTd, dout_tg_d, im06_enq3, s_ful3[6], d06_3, enq3[6], im06_emp3, ib06_req_d);
  INMOD2 im07_3(CLK, RSTd, dout_th_d, im07_enq3, s_ful3[7], d07_3, enq3[7], im07_emp3, ib07_req_d);
  
  ////////////////////////////////////////////////
  wire im00_enq4 = doen_ta_e & req_tt5_e[0];
  wire im01_enq4 = doen_tb_e & req_tt5_e[1];
  wire im02_enq4 = doen_tc_e & req_tt5_e[2];
  wire im03_enq4 = doen_td_e & req_tt5_e[3];
  wire im04_enq4 = doen_te_e & req_tt5_e[4];
  wire im05_enq4 = doen_tf_e & req_tt5_e[5];
  wire im06_enq4 = doen_tg_e & req_tt5_e[6];
  wire im07_enq4 = doen_th_e & req_tt5_e[7];

  wire im00_emp4;
  wire im01_emp4;
  wire im02_emp4;
  wire im03_emp4;
  wire im04_emp4;
  wire im05_emp4;
  wire im06_emp4;
  wire im07_emp4;

  INMOD2 im00_4(CLK, RSTe, dout_ta_e, im00_enq4, s_ful4[0], d00_4, enq4[0], im00_emp4, ib00_req_e);
  INMOD2 im01_4(CLK, RSTe, dout_tb_e, im01_enq4, s_ful4[1], d01_4, enq4[1], im01_emp4, ib01_req_e);
  INMOD2 im02_4(CLK, RSTe, dout_tc_e, im02_enq4, s_ful4[2], d02_4, enq4[2], im02_emp4, ib02_req_e);
  INMOD2 im03_4(CLK, RSTe, dout_td_e, im03_enq4, s_ful4[3], d03_4, enq4[3], im03_emp4, ib03_req_e);
  INMOD2 im04_4(CLK, RSTe, dout_te_e, im04_enq4, s_ful4[4], d04_4, enq4[4], im04_emp4, ib04_req_e);
  INMOD2 im05_4(CLK, RSTe, dout_tf_e, im05_enq4, s_ful4[5], d05_4, enq4[5], im05_emp4, ib05_req_e);
  INMOD2 im06_4(CLK, RSTe, dout_tg_e, im06_enq4, s_ful4[6], d06_4, enq4[6], im06_emp4, ib06_req_e);
  INMOD2 im07_4(CLK, RSTe, dout_th_e, im07_enq4, s_ful4[7], d07_4, enq4[7], im07_emp4, ib07_req_e);

  ////////////////////////////////////////////////
  wire im00_enq5 = doen_ta_f & req_tt5_f[0];
  wire im01_enq5 = doen_tb_f & req_tt5_f[1];
  wire im02_enq5 = doen_tc_f & req_tt5_f[2];
  wire im03_enq5 = doen_td_f & req_tt5_f[3];
  wire im04_enq5 = doen_te_f & req_tt5_f[4];
  wire im05_enq5 = doen_tf_f & req_tt5_f[5];
  wire im06_enq5 = doen_tg_f & req_tt5_f[6];
  wire im07_enq5 = doen_th_f & req_tt5_f[7];

  wire im00_emp5;
  wire im01_emp5;
  wire im02_emp5;
  wire im03_emp5;
  wire im04_emp5;
  wire im05_emp5;
  wire im06_emp5;
  wire im07_emp5;

  INMOD2 im00_5(CLK, RSTf, dout_ta_f, im00_enq5, s_ful5[0], d00_5, enq5[0], im00_emp5, ib00_req_f);
  INMOD2 im01_5(CLK, RSTf, dout_tb_f, im01_enq5, s_ful5[1], d01_5, enq5[1], im01_emp5, ib01_req_f);
  INMOD2 im02_5(CLK, RSTf, dout_tc_f, im02_enq5, s_ful5[2], d02_5, enq5[2], im02_emp5, ib02_req_f);
  INMOD2 im03_5(CLK, RSTf, dout_td_f, im03_enq5, s_ful5[3], d03_5, enq5[3], im03_emp5, ib03_req_f);
  INMOD2 im04_5(CLK, RSTf, dout_te_f, im04_enq5, s_ful5[4], d04_5, enq5[4], im04_emp5, ib04_req_f);
  INMOD2 im05_5(CLK, RSTf, dout_tf_f, im05_enq5, s_ful5[5], d05_5, enq5[5], im05_emp5, ib05_req_f);
  INMOD2 im06_5(CLK, RSTf, dout_tg_f, im06_enq5, s_ful5[6], d06_5, enq5[6], im06_emp5, ib06_req_f);
  INMOD2 im07_5(CLK, RSTf, dout_th_f, im07_enq5, s_ful5[7], d07_5, enq5[7], im07_emp5, ib07_req_f);

  ////////////////////////////////////////////////
  wire im00_enq6 = doen_ta_g & req_tt5_g[0];
  wire im01_enq6 = doen_tb_g & req_tt5_g[1];
  wire im02_enq6 = doen_tc_g & req_tt5_g[2];
  wire im03_enq6 = doen_td_g & req_tt5_g[3];
  wire im04_enq6 = doen_te_g & req_tt5_g[4];
  wire im05_enq6 = doen_tf_g & req_tt5_g[5];
  wire im06_enq6 = doen_tg_g & req_tt5_g[6];
  wire im07_enq6 = doen_th_g & req_tt5_g[7];

  wire im00_emp6;
  wire im01_emp6;
  wire im02_emp6;
  wire im03_emp6;
  wire im04_emp6;
  wire im05_emp6;
  wire im06_emp6;
  wire im07_emp6;

  INMOD2 im00_6(CLK, RSTg, dout_ta_g, im00_enq6, s_ful6[0], d00_6, enq6[0], im00_emp6, ib00_req_g);
  INMOD2 im01_6(CLK, RSTg, dout_tb_g, im01_enq6, s_ful6[1], d01_6, enq6[1], im01_emp6, ib01_req_g);
  INMOD2 im02_6(CLK, RSTg, dout_tc_g, im02_enq6, s_ful6[2], d02_6, enq6[2], im02_emp6, ib02_req_g);
  INMOD2 im03_6(CLK, RSTg, dout_td_g, im03_enq6, s_ful6[3], d03_6, enq6[3], im03_emp6, ib03_req_g);
  INMOD2 im04_6(CLK, RSTg, dout_te_g, im04_enq6, s_ful6[4], d04_6, enq6[4], im04_emp6, ib04_req_g);
  INMOD2 im05_6(CLK, RSTg, dout_tf_g, im05_enq6, s_ful6[5], d05_6, enq6[5], im05_emp6, ib05_req_g);
  INMOD2 im06_6(CLK, RSTg, dout_tg_g, im06_enq6, s_ful6[6], d06_6, enq6[6], im06_emp6, ib06_req_g);
  INMOD2 im07_6(CLK, RSTg, dout_th_g, im07_enq6, s_ful6[7], d07_6, enq6[7], im07_emp6, ib07_req_g);

  ////////////////////////////////////////////////
  wire im00_enq7 = doen_ta_h & req_tt5_h[0];
  wire im01_enq7 = doen_tb_h & req_tt5_h[1];
  wire im02_enq7 = doen_tc_h & req_tt5_h[2];
  wire im03_enq7 = doen_td_h & req_tt5_h[3];
  wire im04_enq7 = doen_te_h & req_tt5_h[4];
  wire im05_enq7 = doen_tf_h & req_tt5_h[5];
  wire im06_enq7 = doen_tg_h & req_tt5_h[6];
  wire im07_enq7 = doen_th_h & req_tt5_h[7];

  wire im00_emp7;
  wire im01_emp7;
  wire im02_emp7;
  wire im03_emp7;
  wire im04_emp7;
  wire im05_emp7;
  wire im06_emp7;
  wire im07_emp7;

  INMOD2 im00_7(CLK, RSTh, dout_ta_h, im00_enq7, s_ful7[0], d00_7, enq7[0], im00_emp7, ib00_req_h);
  INMOD2 im01_7(CLK, RSTh, dout_tb_h, im01_enq7, s_ful7[1], d01_7, enq7[1], im01_emp7, ib01_req_h);
  INMOD2 im02_7(CLK, RSTh, dout_tc_h, im02_enq7, s_ful7[2], d02_7, enq7[2], im02_emp7, ib02_req_h);
  INMOD2 im03_7(CLK, RSTh, dout_td_h, im03_enq7, s_ful7[3], d03_7, enq7[3], im03_emp7, ib03_req_h);
  INMOD2 im04_7(CLK, RSTh, dout_te_h, im04_enq7, s_ful7[4], d04_7, enq7[4], im04_emp7, ib04_req_h);
  INMOD2 im05_7(CLK, RSTh, dout_tf_h, im05_enq7, s_ful7[5], d05_7, enq7[5], im05_emp7, ib05_req_h);
  INMOD2 im06_7(CLK, RSTh, dout_tg_h, im06_enq7, s_ful7[6], d06_7, enq7[6], im06_emp7, ib06_req_h);
  INMOD2 im07_7(CLK, RSTh, dout_th_h, im07_enq7, s_ful7[7], d07_7, enq7[7], im07_emp7, ib07_req_h);
  
  ////////////////////////////////////////////////
  STREE stree0(CLK, RSTa, irst_a, frst_a, phase_a, s_din0, enq0, s_ful0, F01_deq0, F01_dot0, F01_emp0);
  STREE stree1(CLK, RSTb, irst_b, frst_b, phase_b, s_din1, enq1, s_ful1, F01_deq1, F01_dot1, F01_emp1);
  STREE stree2(CLK, RSTc, irst_c, frst_c, phase_c, s_din2, enq2, s_ful2, F01_deq2, F01_dot2, F01_emp2);
  STREE stree3(CLK, RSTd, irst_d, frst_d, phase_d, s_din3, enq3, s_ful3, F01_deq3, F01_dot3, F01_emp3);
  STREE stree4(CLK, RSTe, irst_e, frst_e, phase_e, s_din4, enq4, s_ful4, F01_deq4, F01_dot4, F01_emp4);
  STREE stree5(CLK, RSTf, irst_f, frst_f, phase_f, s_din5, enq5, s_ful5, F01_deq5, F01_dot5, F01_emp5);
  STREE stree6(CLK, RSTg, irst_g, frst_g, phase_g, s_din6, enq6, s_ful6, F01_deq6, F01_dot6, F01_emp6);
  STREE stree7(CLK, RSTh, irst_h, frst_h, phase_h, s_din7, enq7, s_ful7, F01_deq7, F01_dot7, F01_emp7);

  ////////////////////////////////////////////////
 
  // ----- for dram READ/WRITE controller -----
  reg [31:0] w_addr;   // for last phase
  reg [31:0] w_addr_a; // 
  reg [31:0] w_addr_b; // 
  reg [31:0] w_addr_c; // 
  reg [31:0] w_addr_d; // 
  reg [31:0] w_addr_e; // 
  reg [31:0] w_addr_f; // 
  reg [31:0] w_addr_g; // 
  reg [31:0] w_addr_h; // 
  reg [3:0]  state;    // state
  
  reg [31:0] radr_a, radr_b, radr_c, radr_d, radr_e, radr_f, radr_g, radr_h;
  reg [31:0] radr_a_a, radr_b_a, radr_c_a, radr_d_a, radr_e_a, radr_f_a, radr_g_a, radr_h_a;
  reg [31:0] radr_a_b, radr_b_b, radr_c_b, radr_d_b, radr_e_b, radr_f_b, radr_g_b, radr_h_b;
  reg [31:0] radr_a_c, radr_b_c, radr_c_c, radr_d_c, radr_e_c, radr_f_c, radr_g_c, radr_h_c;
  reg [31:0] radr_a_d, radr_b_d, radr_c_d, radr_d_d, radr_e_d, radr_f_d, radr_g_d, radr_h_d;
  reg [31:0] radr_a_e, radr_b_e, radr_c_e, radr_d_e, radr_e_e, radr_f_e, radr_g_e, radr_h_e;
  reg [31:0] radr_a_f, radr_b_f, radr_c_f, radr_d_f, radr_e_f, radr_f_f, radr_g_f, radr_h_f;
  reg [31:0] radr_a_g, radr_b_g, radr_c_g, radr_d_g, radr_e_g, radr_f_g, radr_g_g, radr_h_g;
  reg [31:0] radr_a_h, radr_b_h, radr_c_h, radr_d_h, radr_e_h, radr_f_h, radr_g_h, radr_h_h;
  
  reg [27:0] cnt_a_a, cnt_b_a, cnt_c_a, cnt_d_a, cnt_e_a, cnt_f_a, cnt_g_a, cnt_h_a;
  reg [27:0] cnt_a_b, cnt_b_b, cnt_c_b, cnt_d_b, cnt_e_b, cnt_f_b, cnt_g_b, cnt_h_b;
  reg [27:0] cnt_a_c, cnt_b_c, cnt_c_c, cnt_d_c, cnt_e_c, cnt_f_c, cnt_g_c, cnt_h_c;
  reg [27:0] cnt_a_d, cnt_b_d, cnt_c_d, cnt_d_d, cnt_e_d, cnt_f_d, cnt_g_d, cnt_h_d;
  reg [27:0] cnt_a_e, cnt_b_e, cnt_c_e, cnt_d_e, cnt_e_e, cnt_f_e, cnt_g_e, cnt_h_e;
  reg [27:0] cnt_a_f, cnt_b_f, cnt_c_f, cnt_d_f, cnt_e_f, cnt_f_f, cnt_g_f, cnt_h_f;
  reg [27:0] cnt_a_g, cnt_b_g, cnt_c_g, cnt_d_g, cnt_e_g, cnt_f_g, cnt_g_g, cnt_h_g;
  reg [27:0] cnt_a_h, cnt_b_h, cnt_c_h, cnt_d_h, cnt_e_h, cnt_f_h, cnt_g_h, cnt_h_h;
  
  reg        c_a_a, c_b_a, c_c_a, c_d_a, c_e_a, c_f_a, c_g_a, c_h_a; 
  reg        c_a_b, c_b_b, c_c_b, c_d_b, c_e_b, c_f_b, c_g_b, c_h_b;
  reg        c_a_c, c_b_c, c_c_c, c_d_c, c_e_c, c_f_c, c_g_c, c_h_c; 
  reg        c_a_d, c_b_d, c_c_d, c_d_d, c_e_d, c_f_d, c_g_d, c_h_d;
  reg        c_a_e, c_b_e, c_c_e, c_d_e, c_e_e, c_f_e, c_g_e, c_h_e; 
  reg        c_a_f, c_b_f, c_c_f, c_d_f, c_e_f, c_f_f, c_g_f, c_h_f;
  reg        c_a_g, c_b_g, c_c_g, c_d_g, c_e_g, c_f_g, c_g_g, c_h_g; 
  reg        c_a_h, c_b_h, c_c_h, c_d_h, c_e_h, c_f_h, c_g_h, c_h_h;
  
  // ----- request counter manager -----
  // input
  wire mgdrive   = (state==4 && d_req!=0);
  wire mgdrive_a = (state==3 && d_req!=0 && req_g_sel[0]);
  wire mgdrive_b = (state==3 && d_req!=0 && req_g_sel[1]);
  wire mgdrive_c = (state==3 && d_req!=0 && req_g_sel[2]);
  wire mgdrive_d = (state==3 && d_req!=0 && req_g_sel[3]);
  wire mgdrive_e = (state==3 && d_req!=0 && req_g_sel[4]);
  wire mgdrive_f = (state==3 && d_req!=0 && req_g_sel[5]);
  wire mgdrive_g = (state==3 && d_req!=0 && req_g_sel[6]);
  wire mgdrive_h = (state==3 && d_req!=0 && req_g_sel[7]);

  wire [`SORT_WAY-1:0] im_enq_a = {im07_enq0,im06_enq0,im05_enq0,im04_enq0,im03_enq0,im02_enq0,im01_enq0,im00_enq0};
  wire [`SORT_WAY-1:0] im_enq_b = {im07_enq1,im06_enq1,im05_enq1,im04_enq1,im03_enq1,im02_enq1,im01_enq1,im00_enq1};
  wire [`SORT_WAY-1:0] im_enq_c = {im07_enq2,im06_enq2,im05_enq2,im04_enq2,im03_enq2,im02_enq2,im01_enq2,im00_enq2};
  wire [`SORT_WAY-1:0] im_enq_d = {im07_enq3,im06_enq3,im05_enq3,im04_enq3,im03_enq3,im02_enq3,im01_enq3,im00_enq3};
  wire [`SORT_WAY-1:0] im_enq_e = {im07_enq4,im06_enq4,im05_enq4,im04_enq4,im03_enq4,im02_enq4,im01_enq4,im00_enq4};
  wire [`SORT_WAY-1:0] im_enq_f = {im07_enq5,im06_enq5,im05_enq5,im04_enq5,im03_enq5,im02_enq5,im01_enq5,im00_enq5};
  wire [`SORT_WAY-1:0] im_enq_g = {im07_enq6,im06_enq6,im05_enq6,im04_enq6,im03_enq6,im02_enq6,im01_enq6,im00_enq6};
  wire [`SORT_WAY-1:0] im_enq_h = {im07_enq7,im06_enq7,im05_enq7,im04_enq7,im03_enq7,im02_enq7,im01_enq7,im00_enq7};
  
  wire [`SORT_WAY-1:0] im_emp_a = {im07_emp0,im06_emp0,im05_emp0,im04_emp0,im03_emp0,im02_emp0,im01_emp0,im00_emp0};
  wire [`SORT_WAY-1:0] im_emp_b = {im07_emp1,im06_emp1,im05_emp1,im04_emp1,im03_emp1,im02_emp1,im01_emp1,im00_emp1};
  wire [`SORT_WAY-1:0] im_emp_c = {im07_emp2,im06_emp2,im05_emp2,im04_emp2,im03_emp2,im02_emp2,im01_emp2,im00_emp2};
  wire [`SORT_WAY-1:0] im_emp_d = {im07_emp3,im06_emp3,im05_emp3,im04_emp3,im03_emp3,im02_emp3,im01_emp3,im00_emp3};
  wire [`SORT_WAY-1:0] im_emp_e = {im07_emp4,im06_emp4,im05_emp4,im04_emp4,im03_emp4,im02_emp4,im01_emp4,im00_emp4};
  wire [`SORT_WAY-1:0] im_emp_f = {im07_emp5,im06_emp5,im05_emp5,im04_emp5,im03_emp5,im02_emp5,im01_emp5,im00_emp5};
  wire [`SORT_WAY-1:0] im_emp_g = {im07_emp6,im06_emp6,im05_emp6,im04_emp6,im03_emp6,im02_emp6,im01_emp6,im00_emp6};
  wire [`SORT_WAY-1:0] im_emp_h = {im07_emp7,im06_emp7,im05_emp7,im04_emp7,im03_emp7,im02_emp7,im01_emp7,im00_emp7};
  
  // output
  wire reqcnt_a, reqcnt_b, reqcnt_c, reqcnt_d, reqcnt_e, reqcnt_f, reqcnt_g, reqcnt_h;
  wire reqcnt_a_a, reqcnt_b_a, reqcnt_c_a, reqcnt_d_a, reqcnt_e_a, reqcnt_f_a, reqcnt_g_a, reqcnt_h_a;
  wire reqcnt_a_b, reqcnt_b_b, reqcnt_c_b, reqcnt_d_b, reqcnt_e_b, reqcnt_f_b, reqcnt_g_b, reqcnt_h_b;
  wire reqcnt_a_c, reqcnt_b_c, reqcnt_c_c, reqcnt_d_c, reqcnt_e_c, reqcnt_f_c, reqcnt_g_c, reqcnt_h_c;
  wire reqcnt_a_d, reqcnt_b_d, reqcnt_c_d, reqcnt_d_d, reqcnt_e_d, reqcnt_f_d, reqcnt_g_d, reqcnt_h_d;
  wire reqcnt_a_e, reqcnt_b_e, reqcnt_c_e, reqcnt_d_e, reqcnt_e_e, reqcnt_f_e, reqcnt_g_e, reqcnt_h_e;
  wire reqcnt_a_f, reqcnt_b_f, reqcnt_c_f, reqcnt_d_f, reqcnt_e_f, reqcnt_f_f, reqcnt_g_f, reqcnt_h_f;
  wire reqcnt_a_g, reqcnt_b_g, reqcnt_c_g, reqcnt_d_g, reqcnt_e_g, reqcnt_f_g, reqcnt_g_g, reqcnt_h_g;
  wire reqcnt_a_h, reqcnt_b_h, reqcnt_c_h, reqcnt_d_h, reqcnt_e_h, reqcnt_f_h, reqcnt_g_h, reqcnt_h_h;
      
  REQCNTMG reqcntmg(CLK, RSTa, mgdrive, req_ta, im_enq_a, im_emp_a, reqcnt_a, reqcnt_b, reqcnt_c, reqcnt_d, reqcnt_e, reqcnt_f, reqcnt_g, reqcnt_h);
  REQCNTMG reqcntmg_a(CLK, RSTa, mgdrive_a, req_ta, im_enq_a, im_emp_a, reqcnt_a_a, reqcnt_b_a, reqcnt_c_a, reqcnt_d_a, reqcnt_e_a, reqcnt_f_a, reqcnt_g_a, reqcnt_h_a);
  REQCNTMG reqcntmg_b(CLK, RSTb, mgdrive_b, req_tb, im_enq_b, im_emp_b, reqcnt_a_b, reqcnt_b_b, reqcnt_c_b, reqcnt_d_b, reqcnt_e_b, reqcnt_f_b, reqcnt_g_b, reqcnt_h_b);
  REQCNTMG reqcntmg_c(CLK, RSTc, mgdrive_c, req_tc, im_enq_c, im_emp_c, reqcnt_a_c, reqcnt_b_c, reqcnt_c_c, reqcnt_d_c, reqcnt_e_c, reqcnt_f_c, reqcnt_g_c, reqcnt_h_c);
  REQCNTMG reqcntmg_d(CLK, RSTd, mgdrive_d, req_td, im_enq_d, im_emp_d, reqcnt_a_d, reqcnt_b_d, reqcnt_c_d, reqcnt_d_d, reqcnt_e_d, reqcnt_f_d, reqcnt_g_d, reqcnt_h_d);
  REQCNTMG reqcntmg_e(CLK, RSTe, mgdrive_e, req_te, im_enq_e, im_emp_e, reqcnt_a_e, reqcnt_b_e, reqcnt_c_e, reqcnt_d_e, reqcnt_e_e, reqcnt_f_e, reqcnt_g_e, reqcnt_h_e);
  REQCNTMG reqcntmg_f(CLK, RSTf, mgdrive_f, req_tf, im_enq_f, im_emp_f, reqcnt_a_f, reqcnt_b_f, reqcnt_c_f, reqcnt_d_f, reqcnt_e_f, reqcnt_f_f, reqcnt_g_f, reqcnt_h_f);
  REQCNTMG reqcntmg_g(CLK, RSTg, mgdrive_g, req_tg, im_enq_g, im_emp_g, reqcnt_a_g, reqcnt_b_g, reqcnt_c_g, reqcnt_d_g, reqcnt_e_g, reqcnt_f_g, reqcnt_g_g, reqcnt_h_g);
  REQCNTMG reqcntmg_h(CLK, RSTh, mgdrive_h, req_th, im_enq_h, im_emp_h, reqcnt_a_h, reqcnt_b_h, reqcnt_c_h, reqcnt_d_h, reqcnt_e_h, reqcnt_f_h, reqcnt_g_h, reqcnt_h_h);
  
  // // ----- write manager -----
  // // output
  wire [31:0] w_block;
  wire [31:0] w_block_a;
  wire [31:0] w_block_b;
  wire [31:0] w_block_c;
  wire [31:0] w_block_d;
  wire [31:0] w_block_e;
  wire [31:0] w_block_f;
  wire [31:0] w_block_g;
  wire [31:0] w_block_h;
  wire [31:0] r_endadr;
  wire [31:0] r_endadr_a_a, r_endadr_b_a, r_endadr_c_a, r_endadr_d_a, r_endadr_e_a, r_endadr_f_a, r_endadr_g_a, r_endadr_h_a;
  wire [31:0] r_endadr_a_b, r_endadr_b_b, r_endadr_c_b, r_endadr_d_b, r_endadr_e_b, r_endadr_f_b, r_endadr_g_b, r_endadr_h_b;
  wire [31:0] r_endadr_a_c, r_endadr_b_c, r_endadr_c_c, r_endadr_d_c, r_endadr_e_c, r_endadr_f_c, r_endadr_g_c, r_endadr_h_c;
  wire [31:0] r_endadr_a_d, r_endadr_b_d, r_endadr_c_d, r_endadr_d_d, r_endadr_e_d, r_endadr_f_d, r_endadr_g_d, r_endadr_h_d;
  wire [31:0] r_endadr_a_e, r_endadr_b_e, r_endadr_c_e, r_endadr_d_e, r_endadr_e_e, r_endadr_f_e, r_endadr_g_e, r_endadr_h_e;
  wire [31:0] r_endadr_a_f, r_endadr_b_f, r_endadr_c_f, r_endadr_d_f, r_endadr_e_f, r_endadr_f_f, r_endadr_g_f, r_endadr_h_f;
  wire [31:0] r_endadr_a_g, r_endadr_b_g, r_endadr_c_g, r_endadr_d_g, r_endadr_e_g, r_endadr_f_g, r_endadr_g_g, r_endadr_h_g;
  wire [31:0] r_endadr_a_h, r_endadr_b_h, r_endadr_c_h, r_endadr_d_h, r_endadr_e_h, r_endadr_f_h, r_endadr_g_h, r_endadr_h_h;
  
  WRITEMG_LAST writemg_last(CLK, !last_phase, (state==13 && d_req!=0), elem, w_addr, w_block, r_endadr);
  WRITEMG #((`SORT_ELM>>(`P_LOG+`WAY_LOG)))
  writemg_a(CLK, RSTa, pchange_a, pexe_done_a_p, (state==5 && d_req!=0), elem_a, elem_way_a, w_addr_a, 
            w_block_a, r_endadr_a_a, r_endadr_b_a, r_endadr_c_a, r_endadr_d_a, r_endadr_e_a, r_endadr_f_a, r_endadr_g_a, r_endadr_h_a);
  WRITEMG #((`SORT_ELM>>(`P_LOG+`WAY_LOG)))
  writemg_b(CLK, RSTb, pchange_b, pexe_done_b_p, (state==6 && d_req!=0), elem_b, elem_way_b, w_addr_b, 
            w_block_b, r_endadr_a_b, r_endadr_b_b, r_endadr_c_b, r_endadr_d_b, r_endadr_e_b, r_endadr_f_b, r_endadr_g_b, r_endadr_h_b);
  WRITEMG #((`SORT_ELM>>(`P_LOG+`WAY_LOG)))
  writemg_c(CLK, RSTc, pchange_c, pexe_done_c_p, (state==7 && d_req!=0), elem_c, elem_way_c, w_addr_c, 
            w_block_c, r_endadr_a_c, r_endadr_b_c, r_endadr_c_c, r_endadr_d_c, r_endadr_e_c, r_endadr_f_c, r_endadr_g_c, r_endadr_h_c);
  WRITEMG #((`SORT_ELM>>(`P_LOG+`WAY_LOG)))
  writemg_d(CLK, RSTd, pchange_d, pexe_done_d_p, (state==8 && d_req!=0), elem_d, elem_way_d, w_addr_d, 
            w_block_d, r_endadr_a_d, r_endadr_b_d, r_endadr_c_d, r_endadr_d_d, r_endadr_e_d, r_endadr_f_d, r_endadr_g_d, r_endadr_h_d);
  WRITEMG #((`SORT_ELM>>(`P_LOG+`WAY_LOG)))
  writemg_e(CLK, RSTe, pchange_e, pexe_done_e_p, (state==9 && d_req!=0), elem_e, elem_way_e, w_addr_e, 
            w_block_e, r_endadr_a_e, r_endadr_b_e, r_endadr_c_e, r_endadr_d_e, r_endadr_e_e, r_endadr_f_e, r_endadr_g_e, r_endadr_h_e);
  WRITEMG #((`SORT_ELM>>(`P_LOG+`WAY_LOG)))
  writemg_f(CLK, RSTf, pchange_f, pexe_done_f_p, (state==10 && d_req!=0), elem_f, elem_way_f, w_addr_f, 
            w_block_f, r_endadr_a_f, r_endadr_b_f, r_endadr_c_f, r_endadr_d_f, r_endadr_e_f, r_endadr_f_f, r_endadr_g_f, r_endadr_h_f);
  WRITEMG #((`SORT_ELM>>(`P_LOG+`WAY_LOG)))
  writemg_g(CLK, RSTg, pchange_g, pexe_done_g_p, (state==11 && d_req!=0), elem_g, elem_way_g, w_addr_g, 
            w_block_g, r_endadr_a_g, r_endadr_b_g, r_endadr_c_g, r_endadr_d_g, r_endadr_e_g, r_endadr_f_g, r_endadr_g_g, r_endadr_h_g);
  WRITEMG #((`SORT_ELM>>(`P_LOG+`WAY_LOG)))
  writemg_h(CLK, RSTh, pchange_h, pexe_done_h_p, (state==12 && d_req!=0), elem_h, elem_way_h, w_addr_h, 
            w_block_h, r_endadr_a_h, r_endadr_b_h, r_endadr_c_h, r_endadr_d_h, r_endadr_e_h, r_endadr_f_h, r_endadr_g_h, r_endadr_h_h);
  
  // // ----- read manager -----
  // // output
  wire [31:0] r_block_a, r_block_b, r_block_c, r_block_d, r_block_e, r_block_f, r_block_g, r_block_h;
  wire [31:0] r_block_a_a, r_block_b_a, r_block_c_a, r_block_d_a, r_block_e_a, r_block_f_a, r_block_g_a, r_block_h_a;
  wire [31:0] r_block_a_b, r_block_b_b, r_block_c_b, r_block_d_b, r_block_e_b, r_block_f_b, r_block_g_b, r_block_h_b;
  wire [31:0] r_block_a_c, r_block_b_c, r_block_c_c, r_block_d_c, r_block_e_c, r_block_f_c, r_block_g_c, r_block_h_c;
  wire [31:0] r_block_a_d, r_block_b_d, r_block_c_d, r_block_d_d, r_block_e_d, r_block_f_d, r_block_g_d, r_block_h_d;
  wire [31:0] r_block_a_e, r_block_b_e, r_block_c_e, r_block_d_e, r_block_e_e, r_block_f_e, r_block_g_e, r_block_h_e;
  wire [31:0] r_block_a_f, r_block_b_f, r_block_c_f, r_block_d_f, r_block_e_f, r_block_f_f, r_block_g_f, r_block_h_f;
  wire [31:0] r_block_a_g, r_block_b_g, r_block_c_g, r_block_d_g, r_block_e_g, r_block_f_g, r_block_g_g, r_block_h_g;
  wire [31:0] r_block_a_h, r_block_b_h, r_block_c_h, r_block_d_h, r_block_e_h, r_block_f_h, r_block_g_h, r_block_h_h;
  wire        readend_a, readend_b, readend_c, readend_d, readend_e, readend_f, readend_g, readend_h;
  wire        readend_a_a, readend_b_a, readend_c_a, readend_d_a, readend_e_a, readend_f_a, readend_g_a, readend_h_a;
  wire        readend_a_b, readend_b_b, readend_c_b, readend_d_b, readend_e_b, readend_f_b, readend_g_b, readend_h_b;
  wire        readend_a_c, readend_b_c, readend_c_c, readend_d_c, readend_e_c, readend_f_c, readend_g_c, readend_h_c;
  wire        readend_a_d, readend_b_d, readend_c_d, readend_d_d, readend_e_d, readend_f_d, readend_g_d, readend_h_d;
  wire        readend_a_e, readend_b_e, readend_c_e, readend_d_e, readend_e_e, readend_f_e, readend_g_e, readend_h_e;
  wire        readend_a_f, readend_b_f, readend_c_f, readend_d_f, readend_e_f, readend_f_f, readend_g_f, readend_h_f;
  wire        readend_a_g, readend_b_g, readend_c_g, readend_d_g, readend_e_g, readend_f_g, readend_g_g, readend_h_g;
  wire        readend_a_h, readend_b_h, readend_c_h, readend_d_h, readend_e_h, readend_f_h, readend_g_h, readend_h_h;

  READMG readmg(CLK, !last_phase, (state==4 && d_req!=0), req_ta, l_phase[0], 
                radr_a, radr_b, radr_c, radr_d, radr_e, radr_f, radr_g, radr_h, 
                w_addr_a, w_addr_b, w_addr_c, w_addr_d, w_addr_e, w_addr_f, w_addr_g, w_addr_h, 
                r_block_a, r_block_b, r_block_c, r_block_d, r_block_e, r_block_f, r_block_g, r_block_h, 
                readend_a, readend_b, readend_c, readend_d, readend_e, readend_f, readend_g, readend_h);
  READMG readmg_a(CLK, (RSTa || pchange_a), (state==3 && d_req!=0 && req_g_sel[0]), req_ta, phase_a[0], 
                  radr_a_a, radr_b_a, radr_c_a, radr_d_a, radr_e_a, radr_f_a, radr_g_a, radr_h_a, 
                  r_endadr_a_a, r_endadr_b_a, r_endadr_c_a, r_endadr_d_a, r_endadr_e_a, r_endadr_f_a, r_endadr_g_a, r_endadr_h_a, 
                  r_block_a_a, r_block_b_a, r_block_c_a, r_block_d_a, r_block_e_a, r_block_f_a, r_block_g_a, r_block_h_a, 
                  readend_a_a, readend_b_a, readend_c_a, readend_d_a, readend_e_a, readend_f_a, readend_g_a, readend_h_a);
  READMG readmg_b(CLK, (RSTb || pchange_b), (state==3 && d_req!=0 && req_g_sel[1]), req_tb, phase_b[0], 
                  radr_a_b, radr_b_b, radr_c_b, radr_d_b, radr_e_b, radr_f_b, radr_g_b, radr_h_b, 
                  r_endadr_a_b, r_endadr_b_b, r_endadr_c_b, r_endadr_d_b, r_endadr_e_b, r_endadr_f_b, r_endadr_g_b, r_endadr_h_b, 
                  r_block_a_b, r_block_b_b, r_block_c_b, r_block_d_b, r_block_e_b, r_block_f_b, r_block_g_b, r_block_h_b, 
                  readend_a_b, readend_b_b, readend_c_b, readend_d_b, readend_e_b, readend_f_b, readend_g_b, readend_h_b);
  READMG readmg_c(CLK, (RSTc || pchange_c), (state==3 && d_req!=0 && req_g_sel[2]), req_tc, phase_c[0], 
                  radr_a_c, radr_b_c, radr_c_c, radr_d_c, radr_e_c, radr_f_c, radr_g_c, radr_h_c, 
                  r_endadr_a_c, r_endadr_b_c, r_endadr_c_c, r_endadr_d_c, r_endadr_e_c, r_endadr_f_c, r_endadr_g_c, r_endadr_h_c, 
                  r_block_a_c, r_block_b_c, r_block_c_c, r_block_d_c, r_block_e_c, r_block_f_c, r_block_g_c, r_block_h_c, 
                  readend_a_c, readend_b_c, readend_c_c, readend_d_c, readend_e_c, readend_f_c, readend_g_c, readend_h_c);
  READMG readmg_d(CLK, (RSTd || pchange_d), (state==3 && d_req!=0 && req_g_sel[3]), req_td, phase_d[0], 
                  radr_a_d, radr_b_d, radr_c_d, radr_d_d, radr_e_d, radr_f_d, radr_g_d, radr_h_d, 
                  r_endadr_a_d, r_endadr_b_d, r_endadr_c_d, r_endadr_d_d, r_endadr_e_d, r_endadr_f_d, r_endadr_g_d, r_endadr_h_d, 
                  r_block_a_d, r_block_b_d, r_block_c_d, r_block_d_d, r_block_e_d, r_block_f_d, r_block_g_d, r_block_h_d, 
                  readend_a_d, readend_b_d, readend_c_d, readend_d_d, readend_e_d, readend_f_d, readend_g_d, readend_h_d);
  READMG readmg_e(CLK, (RSTe || pchange_e), (state==3 && d_req!=0 && req_g_sel[4]), req_te, phase_e[0], 
                  radr_a_e, radr_b_e, radr_c_e, radr_d_e, radr_e_e, radr_f_e, radr_g_e, radr_h_e, 
                  r_endadr_a_e, r_endadr_b_e, r_endadr_c_e, r_endadr_d_e, r_endadr_e_e, r_endadr_f_e, r_endadr_g_e, r_endadr_h_e, 
                  r_block_a_e, r_block_b_e, r_block_c_e, r_block_d_e, r_block_e_e, r_block_f_e, r_block_g_e, r_block_h_e, 
                  readend_a_e, readend_b_e, readend_c_e, readend_d_e, readend_e_e, readend_f_e, readend_g_e, readend_h_e);
  READMG readmg_f(CLK, (RSTf || pchange_f), (state==3 && d_req!=0 && req_g_sel[5]), req_tf, phase_f[0], 
                  radr_a_f, radr_b_f, radr_c_f, radr_d_f, radr_e_f, radr_f_f, radr_g_f, radr_h_f, 
                  r_endadr_a_f, r_endadr_b_f, r_endadr_c_f, r_endadr_d_f, r_endadr_e_f, r_endadr_f_f, r_endadr_g_f, r_endadr_h_f, 
                  r_block_a_f, r_block_b_f, r_block_c_f, r_block_d_f, r_block_e_f, r_block_f_f, r_block_g_f, r_block_h_f, 
                  readend_a_f, readend_b_f, readend_c_f, readend_d_f, readend_e_f, readend_f_f, readend_g_f, readend_h_f);
  READMG readmg_g(CLK, (RSTg || pchange_g), (state==3 && d_req!=0 && req_g_sel[6]), req_tg, phase_g[0], 
                  radr_a_g, radr_b_g, radr_c_g, radr_d_g, radr_e_g, radr_f_g, radr_g_g, radr_h_g, 
                  r_endadr_a_g, r_endadr_b_g, r_endadr_c_g, r_endadr_d_g, r_endadr_e_g, r_endadr_f_g, r_endadr_g_g, r_endadr_h_g, 
                  r_block_a_g, r_block_b_g, r_block_c_g, r_block_d_g, r_block_e_g, r_block_f_g, r_block_g_g, r_block_h_g, 
                  readend_a_g, readend_b_g, readend_c_g, readend_d_g, readend_e_g, readend_f_g, readend_g_g, readend_h_g);
  READMG readmg_h(CLK, (RSTh || pchange_h), (state==3 && d_req!=0 && req_g_sel[7]), req_th, phase_h[0], 
                  radr_a_h, radr_b_h, radr_c_h, radr_d_h, radr_e_h, radr_f_h, radr_g_h, radr_h_h, 
                  r_endadr_a_h, r_endadr_b_h, r_endadr_c_h, r_endadr_d_h, r_endadr_e_h, r_endadr_f_h, r_endadr_g_h, r_endadr_h_h, 
                  r_block_a_h, r_block_b_h, r_block_c_h, r_block_d_h, r_block_e_h, r_block_f_h, r_block_g_h, r_block_h_h, 
                  readend_a_h, readend_b_h, readend_c_h, readend_d_h, readend_e_h, readend_f_h, readend_g_h, readend_h_h);

  // ----- output buffer -----
  reg OB_stopreq_a;
  reg OB_stopreq_b;
  reg OB_stopreq_c;
  reg OB_stopreq_d;
  reg OB_stopreq_e;
  reg OB_stopreq_f;
  reg OB_stopreq_g;
  reg OB_stopreq_h;
  always @(posedge CLK) OB_stopreq_a <= (d_busy || OB_dataen_a || elem_en_a || (elem_way_a==(`SORT_ELM>>(`P_LOG+`WAY_LOG))));
  always @(posedge CLK) OB_stopreq_b <= (d_busy || OB_dataen_b || elem_en_b || (elem_way_b==(`SORT_ELM>>(`P_LOG+`WAY_LOG))));
  always @(posedge CLK) OB_stopreq_c <= (d_busy || OB_dataen_c || elem_en_c || (elem_way_c==(`SORT_ELM>>(`P_LOG+`WAY_LOG))));
  always @(posedge CLK) OB_stopreq_d <= (d_busy || OB_dataen_d || elem_en_d || (elem_way_d==(`SORT_ELM>>(`P_LOG+`WAY_LOG))));
  always @(posedge CLK) OB_stopreq_e <= (d_busy || OB_dataen_e || elem_en_e || (elem_way_e==(`SORT_ELM>>(`P_LOG+`WAY_LOG))));
  always @(posedge CLK) OB_stopreq_f <= (d_busy || OB_dataen_f || elem_en_f || (elem_way_f==(`SORT_ELM>>(`P_LOG+`WAY_LOG))));
  always @(posedge CLK) OB_stopreq_g <= (d_busy || OB_dataen_g || elem_en_g || (elem_way_g==(`SORT_ELM>>(`P_LOG+`WAY_LOG))));
  always @(posedge CLK) OB_stopreq_h <= (d_busy || OB_dataen_h || elem_en_h || (elem_way_h==(`SORT_ELM>>(`P_LOG+`WAY_LOG))));
  
  always @(posedge CLK) OB_dataen_a <= OB_deq0;
  always @(posedge CLK) OB_dataen_b <= OB_deq1;
  always @(posedge CLK) OB_dataen_c <= OB_deq2;
  always @(posedge CLK) OB_dataen_d <= OB_deq3;
  always @(posedge CLK) OB_dataen_e <= OB_deq4;
  always @(posedge CLK) OB_dataen_f <= OB_deq5;
  always @(posedge CLK) OB_dataen_g <= OB_deq6;
  always @(posedge CLK) OB_dataen_h <= OB_deq7;
  OTMOD ob0(CLK, RSTa, OB_stopreq_a, mux32(w_block_a, w_block, last_phase), F01_deq0, F01_dot0, OB_deq0, OB_dot0, OB_full0, OB_req_a);
  OTMOD ob1(CLK, RSTb, OB_stopreq_b, w_block_b, F01_deq1, F01_dot1, OB_deq1, OB_dot1, OB_full1, OB_req_b);
  OTMOD ob2(CLK, RSTc, OB_stopreq_c, w_block_c, F01_deq2, F01_dot2, OB_deq2, OB_dot2, OB_full2, OB_req_c);
  OTMOD ob3(CLK, RSTd, OB_stopreq_d, w_block_d, F01_deq3, F01_dot3, OB_deq3, OB_dot3, OB_full3, OB_req_d);
  OTMOD ob4(CLK, RSTe, OB_stopreq_e, w_block_e, F01_deq4, F01_dot4, OB_deq4, OB_dot4, OB_full4, OB_req_e);
  OTMOD ob5(CLK, RSTf, OB_stopreq_f, w_block_f, F01_deq5, F01_dot5, OB_deq5, OB_dot5, OB_full5, OB_req_f);
  OTMOD ob6(CLK, RSTg, OB_stopreq_g, w_block_g, F01_deq6, F01_dot6, OB_deq6, OB_dot6, OB_full6, OB_req_g);
  OTMOD ob7(CLK, RSTh, OB_stopreq_h, w_block_h, F01_deq7, F01_dot7, OB_deq7, OB_dot7, OB_full7, OB_req_h);

  /********************************** Error Check ***********************************************/
  generate
    if (`INITTYPE=="reverse" || `INITTYPE=="sorted") begin
      reg [`SORTW-1:0] check_cnt;
      always @(posedge CLK) begin
        if (RSTa) begin check_cnt<=1; ERROR<=0; end
        if (last_phase && F01_deq0) begin
          if (check_cnt != F01_dot0) begin
            ERROR <= 1;
            $write("Error in core.v: %d %d\n", F01_dot0, check_cnt); // for simulation
            $finish();                                               // for simulation
          end
          check_cnt <= check_cnt + 1;
        end
      end
    end else if (`INITTYPE != "xorshift") begin
      always @(posedge CLK) begin
        ERROR <= 1;
        // for simulation
        $write("Error! INITTYPE is wrong.\n");  
        $write("Please make sure src/define.v\n");  
        $finish();
      end
    end
  endgenerate

  /***** dram READ/WRITE controller                                                         *****/
  /**********************************************************************************************/
  wire reqhalt_a_a = mux1((readend_a_a || (phase_a==`LAST_PHASE)), c_a_a, (phase_a==0));
  wire reqhalt_b_a = mux1((readend_b_a || (phase_a==`LAST_PHASE)), c_b_a, (phase_a==0));
  wire reqhalt_c_a = mux1((readend_c_a || (phase_a==`LAST_PHASE)), c_c_a, (phase_a==0));
  wire reqhalt_d_a = mux1((readend_d_a || (phase_a==`LAST_PHASE)), c_d_a, (phase_a==0));
  wire reqhalt_e_a = mux1((readend_e_a || (phase_a==`LAST_PHASE)), c_e_a, (phase_a==0));
  wire reqhalt_f_a = mux1((readend_f_a || (phase_a==`LAST_PHASE)), c_f_a, (phase_a==0));
  wire reqhalt_g_a = mux1((readend_g_a || (phase_a==`LAST_PHASE)), c_g_a, (phase_a==0));
  wire reqhalt_h_a = mux1((readend_h_a || (phase_a==`LAST_PHASE)), c_h_a, (phase_a==0));
  
  wire reqhalt_a_b = mux1((readend_a_b || (phase_b==`LAST_PHASE)), c_a_b, (phase_b==0));
  wire reqhalt_b_b = mux1((readend_b_b || (phase_b==`LAST_PHASE)), c_b_b, (phase_b==0));
  wire reqhalt_c_b = mux1((readend_c_b || (phase_b==`LAST_PHASE)), c_c_b, (phase_b==0));
  wire reqhalt_d_b = mux1((readend_d_b || (phase_b==`LAST_PHASE)), c_d_b, (phase_b==0));
  wire reqhalt_e_b = mux1((readend_e_b || (phase_b==`LAST_PHASE)), c_e_b, (phase_b==0));
  wire reqhalt_f_b = mux1((readend_f_b || (phase_b==`LAST_PHASE)), c_f_b, (phase_b==0));
  wire reqhalt_g_b = mux1((readend_g_b || (phase_b==`LAST_PHASE)), c_g_b, (phase_b==0));
  wire reqhalt_h_b = mux1((readend_h_b || (phase_b==`LAST_PHASE)), c_h_b, (phase_b==0));
  
  wire reqhalt_a_c = mux1((readend_a_c || (phase_c==`LAST_PHASE)), c_a_c, (phase_c==0));
  wire reqhalt_b_c = mux1((readend_b_c || (phase_c==`LAST_PHASE)), c_b_c, (phase_c==0));
  wire reqhalt_c_c = mux1((readend_c_c || (phase_c==`LAST_PHASE)), c_c_c, (phase_c==0));
  wire reqhalt_d_c = mux1((readend_d_c || (phase_c==`LAST_PHASE)), c_d_c, (phase_c==0));
  wire reqhalt_e_c = mux1((readend_e_c || (phase_c==`LAST_PHASE)), c_e_c, (phase_c==0));
  wire reqhalt_f_c = mux1((readend_f_c || (phase_c==`LAST_PHASE)), c_f_c, (phase_c==0));
  wire reqhalt_g_c = mux1((readend_g_c || (phase_c==`LAST_PHASE)), c_g_c, (phase_c==0));
  wire reqhalt_h_c = mux1((readend_h_c || (phase_c==`LAST_PHASE)), c_h_c, (phase_c==0));
  
  wire reqhalt_a_d = mux1((readend_a_d || (phase_d==`LAST_PHASE)), c_a_d, (phase_d==0));
  wire reqhalt_b_d = mux1((readend_b_d || (phase_d==`LAST_PHASE)), c_b_d, (phase_d==0));
  wire reqhalt_c_d = mux1((readend_c_d || (phase_d==`LAST_PHASE)), c_c_d, (phase_d==0));
  wire reqhalt_d_d = mux1((readend_d_d || (phase_d==`LAST_PHASE)), c_d_d, (phase_d==0));
  wire reqhalt_e_d = mux1((readend_e_d || (phase_d==`LAST_PHASE)), c_e_d, (phase_d==0));
  wire reqhalt_f_d = mux1((readend_f_d || (phase_d==`LAST_PHASE)), c_f_d, (phase_d==0));
  wire reqhalt_g_d = mux1((readend_g_d || (phase_d==`LAST_PHASE)), c_g_d, (phase_d==0));
  wire reqhalt_h_d = mux1((readend_h_d || (phase_d==`LAST_PHASE)), c_h_d, (phase_d==0));
  
  wire reqhalt_a_e = mux1((readend_a_e || (phase_e==`LAST_PHASE)), c_a_e, (phase_e==0));
  wire reqhalt_b_e = mux1((readend_b_e || (phase_e==`LAST_PHASE)), c_b_e, (phase_e==0));
  wire reqhalt_c_e = mux1((readend_c_e || (phase_e==`LAST_PHASE)), c_c_e, (phase_e==0));
  wire reqhalt_d_e = mux1((readend_d_e || (phase_e==`LAST_PHASE)), c_d_e, (phase_e==0));
  wire reqhalt_e_e = mux1((readend_e_e || (phase_e==`LAST_PHASE)), c_e_e, (phase_e==0));
  wire reqhalt_f_e = mux1((readend_f_e || (phase_e==`LAST_PHASE)), c_f_e, (phase_e==0));
  wire reqhalt_g_e = mux1((readend_g_e || (phase_e==`LAST_PHASE)), c_g_e, (phase_e==0));
  wire reqhalt_h_e = mux1((readend_h_e || (phase_e==`LAST_PHASE)), c_h_e, (phase_e==0));
  
  wire reqhalt_a_f = mux1((readend_a_f || (phase_f==`LAST_PHASE)), c_a_f, (phase_f==0));
  wire reqhalt_b_f = mux1((readend_b_f || (phase_f==`LAST_PHASE)), c_b_f, (phase_f==0));
  wire reqhalt_c_f = mux1((readend_c_f || (phase_f==`LAST_PHASE)), c_c_f, (phase_f==0));
  wire reqhalt_d_f = mux1((readend_d_f || (phase_f==`LAST_PHASE)), c_d_f, (phase_f==0));
  wire reqhalt_e_f = mux1((readend_e_f || (phase_f==`LAST_PHASE)), c_e_f, (phase_f==0));
  wire reqhalt_f_f = mux1((readend_f_f || (phase_f==`LAST_PHASE)), c_f_f, (phase_f==0));
  wire reqhalt_g_f = mux1((readend_g_f || (phase_f==`LAST_PHASE)), c_g_f, (phase_f==0));
  wire reqhalt_h_f = mux1((readend_h_f || (phase_f==`LAST_PHASE)), c_h_f, (phase_f==0));
  
  wire reqhalt_a_g = mux1((readend_a_g || (phase_g==`LAST_PHASE)), c_a_g, (phase_g==0));
  wire reqhalt_b_g = mux1((readend_b_g || (phase_g==`LAST_PHASE)), c_b_g, (phase_g==0));
  wire reqhalt_c_g = mux1((readend_c_g || (phase_g==`LAST_PHASE)), c_c_g, (phase_g==0));
  wire reqhalt_d_g = mux1((readend_d_g || (phase_g==`LAST_PHASE)), c_d_g, (phase_g==0));
  wire reqhalt_e_g = mux1((readend_e_g || (phase_g==`LAST_PHASE)), c_e_g, (phase_g==0));
  wire reqhalt_f_g = mux1((readend_f_g || (phase_g==`LAST_PHASE)), c_f_g, (phase_g==0));
  wire reqhalt_g_g = mux1((readend_g_g || (phase_g==`LAST_PHASE)), c_g_g, (phase_g==0));
  wire reqhalt_h_g = mux1((readend_h_g || (phase_g==`LAST_PHASE)), c_h_g, (phase_g==0));
  
  wire reqhalt_a_h = mux1((readend_a_h || (phase_h==`LAST_PHASE)), c_a_h, (phase_h==0));
  wire reqhalt_b_h = mux1((readend_b_h || (phase_h==`LAST_PHASE)), c_b_h, (phase_h==0));
  wire reqhalt_c_h = mux1((readend_c_h || (phase_h==`LAST_PHASE)), c_c_h, (phase_h==0));
  wire reqhalt_d_h = mux1((readend_d_h || (phase_h==`LAST_PHASE)), c_d_h, (phase_h==0));
  wire reqhalt_e_h = mux1((readend_e_h || (phase_h==`LAST_PHASE)), c_e_h, (phase_h==0));
  wire reqhalt_f_h = mux1((readend_f_h || (phase_h==`LAST_PHASE)), c_f_h, (phase_h==0));
  wire reqhalt_g_h = mux1((readend_g_h || (phase_h==`LAST_PHASE)), c_g_h, (phase_h==0));
  wire reqhalt_h_h = mux1((readend_h_h || (phase_h==`LAST_PHASE)), c_h_h, (phase_h==0));
  
  // state machine for dram READ/WRITE controller //////////////////////////
  always @(posedge CLK) begin
    if (RSTa || pchange_a || pchange_b || pchange_c || pchange_d || 
        pchange_e || pchange_f || pchange_g || pchange_h) begin
      if (RSTa) {initdone, state} <= 0;
      if (RSTa) {d_req, d_initadr, d_blocks} <= 0;
      if (RSTa) {req_a, req_b, req_c, req_d, req_e, req_f, req_g, req_h} <= 0;
      if (RSTa) {req_ta, req_tb, req_tc, req_td, req_te, req_tf, req_tg, req_th} <= 0;
      if (RSTa) {req_ga, req_gb, req_gc, req_gd, req_ge, req_gf, req_gg, req_gh} <= 0;
      if (RSTa) {req_gga, req_ggb, req_ggc, req_ggd, req_gge, req_ggf, req_ggg, req_ggh} <= 0;
      if (RSTa) req_g_sel <= 0;
      
      req     <= 0;
      w_addr  <= mux32((`SORT_ELM>>1), 0, l_phase[0]);
      radr_a  <= ((`SELM_PER_WAY>>3)*0);
      radr_b  <= ((`SELM_PER_WAY>>3)*1);
      radr_c  <= ((`SELM_PER_WAY>>3)*2);
      radr_d  <= ((`SELM_PER_WAY>>3)*3);
      radr_e  <= ((`SELM_PER_WAY>>3)*4);
      radr_f  <= ((`SELM_PER_WAY>>3)*5);
      radr_g  <= ((`SELM_PER_WAY>>3)*6);
      radr_h  <= ((`SELM_PER_WAY>>3)*7);

      if ((RSTa || pchange_a) && !pexe_done_a_p) begin
        w_addr_a  <= mux32((`SORT_ELM>>1), 0, phase_a[0]);
        radr_a_a  <= ((`SELM_PER_WAY>>(`P_LOG+3))*0);
        radr_b_a  <= ((`SELM_PER_WAY>>(`P_LOG+3))*1);
        radr_c_a  <= ((`SELM_PER_WAY>>(`P_LOG+3))*2);
        radr_d_a  <= ((`SELM_PER_WAY>>(`P_LOG+3))*3);
        radr_e_a  <= ((`SELM_PER_WAY>>(`P_LOG+3))*4);
        radr_f_a  <= ((`SELM_PER_WAY>>(`P_LOG+3))*5);
        radr_g_a  <= ((`SELM_PER_WAY>>(`P_LOG+3))*6);
        radr_h_a  <= ((`SELM_PER_WAY>>(`P_LOG+3))*7);
        {cnt_a_a, cnt_b_a, cnt_c_a, cnt_d_a, cnt_e_a, cnt_f_a, cnt_g_a, cnt_h_a} <= 0;
        {c_a_a, c_b_a, c_c_a, c_d_a, c_e_a, c_f_a, c_g_a, c_h_a} <= 0;
        OB_deq_ta <= 0;
      end
      
      if ((RSTa || pchange_b) && !pexe_done_b_p) begin
        w_addr_b  <= mux32(((`SORT_ELM>>4) | (`SORT_ELM>>1)), (`SORT_ELM>>4), phase_b[0]);
        radr_a_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*0) | (`SORT_ELM>>4);
        radr_b_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>4);
        radr_c_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>4);
        radr_d_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>4);
        radr_e_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>4);
        radr_f_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>4);
        radr_g_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>4);
        radr_h_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>4);
        {cnt_a_b, cnt_b_b, cnt_c_b, cnt_d_b, cnt_e_b, cnt_f_b, cnt_g_b, cnt_h_b} <= 0;
        {c_a_b, c_b_b, c_c_b, c_d_b, c_e_b, c_f_b, c_g_b, c_h_b} <= 0;
        OB_deq_tb <= 0;
      end
      
      if ((RSTa || pchange_c) && !pexe_done_c_p) begin
        w_addr_c  <= mux32(((`SORT_ELM>>3) | (`SORT_ELM>>1)), (`SORT_ELM>>3), phase_c[0]);
        radr_a_c  <= ((`SELM_PER_WAY>>(`P_LOG+3))*0) | (`SORT_ELM>>3);
        radr_b_c  <= ((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>3);
        radr_c_c  <= ((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>3);
        radr_d_c  <= ((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>3);
        radr_e_c  <= ((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>3);
        radr_f_c  <= ((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>3);
        radr_g_c  <= ((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>3);
        radr_h_c  <= ((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>3);
        {cnt_a_c, cnt_b_c, cnt_c_c, cnt_d_c, cnt_e_c, cnt_f_c, cnt_g_c, cnt_h_c} <= 0;
        {c_a_c, c_b_c, c_c_c, c_d_c, c_e_c, c_f_c, c_g_c, c_h_c} <= 0;
        OB_deq_tc <= 0;
      end
      
      if ((RSTa || pchange_d) && !pexe_done_d_p) begin
        w_addr_d  <= mux32((((`SORT_ELM>>4) | (`SORT_ELM>>3)) | (`SORT_ELM>>1)), ((`SORT_ELM>>4) | (`SORT_ELM>>3)), phase_d[0]);
        radr_a_d  <= ((`SELM_PER_WAY>>(`P_LOG+3))*0) | ((`SORT_ELM>>4) | (`SORT_ELM>>3));
        radr_b_d  <= ((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>4) | (`SORT_ELM>>3));
        radr_c_d  <= ((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>4) | (`SORT_ELM>>3));
        radr_d_d  <= ((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>4) | (`SORT_ELM>>3));
        radr_e_d  <= ((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>4) | (`SORT_ELM>>3));
        radr_f_d  <= ((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>4) | (`SORT_ELM>>3));
        radr_g_d  <= ((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>4) | (`SORT_ELM>>3));
        radr_h_d  <= ((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>4) | (`SORT_ELM>>3));
        {cnt_a_d, cnt_b_d, cnt_c_d, cnt_d_d, cnt_e_d, cnt_f_d, cnt_g_d, cnt_h_d} <= 0;
        {c_a_d, c_b_d, c_c_d, c_d_d, c_e_d, c_f_d, c_g_d, c_h_d} <= 0;
        OB_deq_td <= 0;
      end
      
      if ((RSTa || pchange_e) && !pexe_done_e_p) begin
        w_addr_e  <= mux32(((`SORT_ELM>>2) | (`SORT_ELM>>1)), (`SORT_ELM>>2), phase_e[0]);
        radr_a_e  <= ((`SELM_PER_WAY>>(`P_LOG+3))*0) | (`SORT_ELM>>2);
        radr_b_e  <= ((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>2);
        radr_c_e  <= ((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>2);
        radr_d_e  <= ((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>2);
        radr_e_e  <= ((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>2);
        radr_f_e  <= ((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>2);
        radr_g_e  <= ((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>2);
        radr_h_e  <= ((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>2);
        {cnt_a_e, cnt_b_e, cnt_c_e, cnt_d_e, cnt_e_e, cnt_f_e, cnt_g_e, cnt_h_e} <= 0;
        {c_a_e, c_b_e, c_c_e, c_d_e, c_e_e, c_f_e, c_g_e, c_h_e} <= 0;
        OB_deq_te <= 0;
      end
      
      if ((RSTa || pchange_f) && !pexe_done_f_p) begin
        w_addr_f  <= mux32((((`SORT_ELM>>4) | (`SORT_ELM>>2)) | (`SORT_ELM>>1)), ((`SORT_ELM>>4) | (`SORT_ELM>>2)), phase_f[0]);
        radr_a_f  <= ((`SELM_PER_WAY>>(`P_LOG+3))*0) | ((`SORT_ELM>>4) | (`SORT_ELM>>2));
        radr_b_f  <= ((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>4) | (`SORT_ELM>>2));
        radr_c_f  <= ((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>4) | (`SORT_ELM>>2));
        radr_d_f  <= ((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>4) | (`SORT_ELM>>2));
        radr_e_f  <= ((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>4) | (`SORT_ELM>>2));
        radr_f_f  <= ((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>4) | (`SORT_ELM>>2));
        radr_g_f  <= ((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>4) | (`SORT_ELM>>2));
        radr_h_f  <= ((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>4) | (`SORT_ELM>>2));
        {cnt_a_f, cnt_b_f, cnt_c_f, cnt_d_f, cnt_e_f, cnt_f_f, cnt_g_f, cnt_h_f} <= 0;
        {c_a_f, c_b_f, c_c_f, c_d_f, c_e_f, c_f_f, c_g_f, c_h_f} <= 0;
        OB_deq_tf <= 0;
      end
      
      if ((RSTa || pchange_g) && !pexe_done_g_p) begin
        w_addr_g  <= mux32((((`SORT_ELM>>3) | (`SORT_ELM>>2)) | (`SORT_ELM>>1)), ((`SORT_ELM>>3) | (`SORT_ELM>>2)), phase_g[0]);
        radr_a_g  <= ((`SELM_PER_WAY>>(`P_LOG+3))*0) | ((`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_b_g  <= ((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_c_g  <= ((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_d_g  <= ((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_e_g  <= ((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_f_g  <= ((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_g_g  <= ((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_h_g  <= ((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>3) | (`SORT_ELM>>2));
        {cnt_a_g, cnt_b_g, cnt_c_g, cnt_d_g, cnt_e_g, cnt_f_g, cnt_g_g, cnt_h_g} <= 0;
        {c_a_g, c_b_g, c_c_g, c_d_g, c_e_g, c_f_g, c_g_g, c_h_g} <= 0;
        OB_deq_tg <= 0;
      end
      
      if ((RSTa || pchange_h) && !pexe_done_h_p) begin
        w_addr_h  <= mux32((((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2)) | (`SORT_ELM>>1)), ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2)), phase_h[0]);
        radr_a_h  <= ((`SELM_PER_WAY>>(`P_LOG+3))*0) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_b_h  <= ((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_c_h  <= ((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_d_h  <= ((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_e_h  <= ((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_f_h  <= ((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_g_h  <= ((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2));
        radr_h_h  <= ((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2));
        {cnt_a_h, cnt_b_h, cnt_c_h, cnt_d_h, cnt_e_h, cnt_f_h, cnt_g_h, cnt_h_h} <= 0;
        {c_a_h, c_b_h, c_c_h, c_d_h, c_e_h, c_f_h, c_g_h, c_h_h} <= 0;
        OB_deq_th <= 0;
      end
      
    end else begin
      case (state)
        ////////////////////////////////////////////////////////////////////////////////////////
        0: begin ///// Initialize memory, write data to DRAM
          if (d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE; //
            d_blocks  <= (`SORT_ELM>>4);  // 16word/block for VC_H07, 2word/b for Tokuden
            d_initadr <= 0;               //
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        1: begin ///// request arbitration 
          if (!d_busy) begin
            initdone <= 1;
            OB_deq_ta <= 0;
            OB_deq_tb <= 0;
            OB_deq_tc <= 0;
            OB_deq_td <= 0;
            OB_deq_te <= 0;
            OB_deq_tf <= 0;
            OB_deq_tg <= 0;
            OB_deq_th <= 0;
            case (last_phase)
              1'b0: begin
                if      (dc_req && ib00_req_a && !reqhalt_a_a && (reqcnt_a_a==0)) begin req_a <= 8'h01; req_gga <= 1; end // 
                else if (dc_req && ib01_req_a && !reqhalt_b_a && (reqcnt_b_a==0)) begin req_a <= 8'h02; req_gga <= 1; end // 
                else if (dc_req && ib02_req_a && !reqhalt_c_a && (reqcnt_c_a==0)) begin req_a <= 8'h04; req_gga <= 1; end // 
                else if (dc_req && ib03_req_a && !reqhalt_d_a && (reqcnt_d_a==0)) begin req_a <= 8'h08; req_gga <= 1; end // 
                else if (dc_req && ib04_req_a && !reqhalt_e_a && (reqcnt_e_a==0)) begin req_a <= 8'h10; req_gga <= 1; end // 
                else if (dc_req && ib05_req_a && !reqhalt_f_a && (reqcnt_f_a==0)) begin req_a <= 8'h20; req_gga <= 1; end // 
                else if (dc_req && ib06_req_a && !reqhalt_g_a && (reqcnt_g_a==0)) begin req_a <= 8'h40; req_gga <= 1; end // 
                else if (dc_req && ib07_req_a && !reqhalt_h_a && (reqcnt_h_a==0)) begin req_a <= 8'h80; req_gga <= 1; end //

                if      (dc_req && ib00_req_b && !reqhalt_a_b && (reqcnt_a_b==0)) begin req_b <= 8'h01; req_ggb <= 1; end // 
                else if (dc_req && ib01_req_b && !reqhalt_b_b && (reqcnt_b_b==0)) begin req_b <= 8'h02; req_ggb <= 1; end // 
                else if (dc_req && ib02_req_b && !reqhalt_c_b && (reqcnt_c_b==0)) begin req_b <= 8'h04; req_ggb <= 1; end // 
                else if (dc_req && ib03_req_b && !reqhalt_d_b && (reqcnt_d_b==0)) begin req_b <= 8'h08; req_ggb <= 1; end // 
                else if (dc_req && ib04_req_b && !reqhalt_e_b && (reqcnt_e_b==0)) begin req_b <= 8'h10; req_ggb <= 1; end // 
                else if (dc_req && ib05_req_b && !reqhalt_f_b && (reqcnt_f_b==0)) begin req_b <= 8'h20; req_ggb <= 1; end // 
                else if (dc_req && ib06_req_b && !reqhalt_g_b && (reqcnt_g_b==0)) begin req_b <= 8'h40; req_ggb <= 1; end //
                else if (dc_req && ib07_req_b && !reqhalt_h_b && (reqcnt_h_b==0)) begin req_b <= 8'h80; req_ggb <= 1; end //
                
                if      (dc_req && ib00_req_c && !reqhalt_a_c && (reqcnt_a_c==0)) begin req_c <= 8'h01; req_ggc <= 1; end // 
                else if (dc_req && ib01_req_c && !reqhalt_b_c && (reqcnt_b_c==0)) begin req_c <= 8'h02; req_ggc <= 1; end // 
                else if (dc_req && ib02_req_c && !reqhalt_c_c && (reqcnt_c_c==0)) begin req_c <= 8'h04; req_ggc <= 1; end // 
                else if (dc_req && ib03_req_c && !reqhalt_d_c && (reqcnt_d_c==0)) begin req_c <= 8'h08; req_ggc <= 1; end // 
                else if (dc_req && ib04_req_c && !reqhalt_e_c && (reqcnt_e_c==0)) begin req_c <= 8'h10; req_ggc <= 1; end // 
                else if (dc_req && ib05_req_c && !reqhalt_f_c && (reqcnt_f_c==0)) begin req_c <= 8'h20; req_ggc <= 1; end // 
                else if (dc_req && ib06_req_c && !reqhalt_g_c && (reqcnt_g_c==0)) begin req_c <= 8'h40; req_ggc <= 1; end //
                else if (dc_req && ib07_req_c && !reqhalt_h_c && (reqcnt_h_c==0)) begin req_c <= 8'h80; req_ggc <= 1; end //

                if      (dc_req && ib00_req_d && !reqhalt_a_d && (reqcnt_a_d==0)) begin req_d <= 8'h01; req_ggd <= 1; end // 
                else if (dc_req && ib01_req_d && !reqhalt_b_d && (reqcnt_b_d==0)) begin req_d <= 8'h02; req_ggd <= 1; end // 
                else if (dc_req && ib02_req_d && !reqhalt_c_d && (reqcnt_c_d==0)) begin req_d <= 8'h04; req_ggd <= 1; end // 
                else if (dc_req && ib03_req_d && !reqhalt_d_d && (reqcnt_d_d==0)) begin req_d <= 8'h08; req_ggd <= 1; end // 
                else if (dc_req && ib04_req_d && !reqhalt_e_d && (reqcnt_e_d==0)) begin req_d <= 8'h10; req_ggd <= 1; end // 
                else if (dc_req && ib05_req_d && !reqhalt_f_d && (reqcnt_f_d==0)) begin req_d <= 8'h20; req_ggd <= 1; end // 
                else if (dc_req && ib06_req_d && !reqhalt_g_d && (reqcnt_g_d==0)) begin req_d <= 8'h40; req_ggd <= 1; end //
                else if (dc_req && ib07_req_d && !reqhalt_h_d && (reqcnt_h_d==0)) begin req_d <= 8'h80; req_ggd <= 1; end //
                
                if      (dc_req && ib00_req_e && !reqhalt_a_e && (reqcnt_a_e==0)) begin req_e <= 8'h01; req_gge <= 1; end // 
                else if (dc_req && ib01_req_e && !reqhalt_b_e && (reqcnt_b_e==0)) begin req_e <= 8'h02; req_gge <= 1; end // 
                else if (dc_req && ib02_req_e && !reqhalt_c_e && (reqcnt_c_e==0)) begin req_e <= 8'h04; req_gge <= 1; end // 
                else if (dc_req && ib03_req_e && !reqhalt_d_e && (reqcnt_d_e==0)) begin req_e <= 8'h08; req_gge <= 1; end // 
                else if (dc_req && ib04_req_e && !reqhalt_e_e && (reqcnt_e_e==0)) begin req_e <= 8'h10; req_gge <= 1; end // 
                else if (dc_req && ib05_req_e && !reqhalt_f_e && (reqcnt_f_e==0)) begin req_e <= 8'h20; req_gge <= 1; end // 
                else if (dc_req && ib06_req_e && !reqhalt_g_e && (reqcnt_g_e==0)) begin req_e <= 8'h40; req_gge <= 1; end // 
                else if (dc_req && ib07_req_e && !reqhalt_h_e && (reqcnt_h_e==0)) begin req_e <= 8'h80; req_gge <= 1; end //

                if      (dc_req && ib00_req_f && !reqhalt_a_f && (reqcnt_a_f==0)) begin req_f <= 8'h01; req_ggf <= 1; end // 
                else if (dc_req && ib01_req_f && !reqhalt_b_f && (reqcnt_b_f==0)) begin req_f <= 8'h02; req_ggf <= 1; end // 
                else if (dc_req && ib02_req_f && !reqhalt_c_f && (reqcnt_c_f==0)) begin req_f <= 8'h04; req_ggf <= 1; end // 
                else if (dc_req && ib03_req_f && !reqhalt_d_f && (reqcnt_d_f==0)) begin req_f <= 8'h08; req_ggf <= 1; end // 
                else if (dc_req && ib04_req_f && !reqhalt_e_f && (reqcnt_e_f==0)) begin req_f <= 8'h10; req_ggf <= 1; end // 
                else if (dc_req && ib05_req_f && !reqhalt_f_f && (reqcnt_f_f==0)) begin req_f <= 8'h20; req_ggf <= 1; end // 
                else if (dc_req && ib06_req_f && !reqhalt_g_f && (reqcnt_g_f==0)) begin req_f <= 8'h40; req_ggf <= 1; end //
                else if (dc_req && ib07_req_f && !reqhalt_h_f && (reqcnt_h_f==0)) begin req_f <= 8'h80; req_ggf <= 1; end //
                
                if      (dc_req && ib00_req_g && !reqhalt_a_g && (reqcnt_a_g==0)) begin req_g <= 8'h01; req_ggg <= 1; end // 
                else if (dc_req && ib01_req_g && !reqhalt_b_g && (reqcnt_b_g==0)) begin req_g <= 8'h02; req_ggg <= 1; end // 
                else if (dc_req && ib02_req_g && !reqhalt_c_g && (reqcnt_c_g==0)) begin req_g <= 8'h04; req_ggg <= 1; end // 
                else if (dc_req && ib03_req_g && !reqhalt_d_g && (reqcnt_d_g==0)) begin req_g <= 8'h08; req_ggg <= 1; end // 
                else if (dc_req && ib04_req_g && !reqhalt_e_g && (reqcnt_e_g==0)) begin req_g <= 8'h10; req_ggg <= 1; end // 
                else if (dc_req && ib05_req_g && !reqhalt_f_g && (reqcnt_f_g==0)) begin req_g <= 8'h20; req_ggg <= 1; end // 
                else if (dc_req && ib06_req_g && !reqhalt_g_g && (reqcnt_g_g==0)) begin req_g <= 8'h40; req_ggg <= 1; end //
                else if (dc_req && ib07_req_g && !reqhalt_h_g && (reqcnt_h_g==0)) begin req_g <= 8'h80; req_ggg <= 1; end //

                if      (dc_req && ib00_req_h && !reqhalt_a_h && (reqcnt_a_h==0)) begin req_h <= 8'h01; req_ggh <= 1; end // 
                else if (dc_req && ib01_req_h && !reqhalt_b_h && (reqcnt_b_h==0)) begin req_h <= 8'h02; req_ggh <= 1; end // 
                else if (dc_req && ib02_req_h && !reqhalt_c_h && (reqcnt_c_h==0)) begin req_h <= 8'h04; req_ggh <= 1; end // 
                else if (dc_req && ib03_req_h && !reqhalt_d_h && (reqcnt_d_h==0)) begin req_h <= 8'h08; req_ggh <= 1; end // 
                else if (dc_req && ib04_req_h && !reqhalt_e_h && (reqcnt_e_h==0)) begin req_h <= 8'h10; req_ggh <= 1; end // 
                else if (dc_req && ib05_req_h && !reqhalt_f_h && (reqcnt_f_h==0)) begin req_h <= 8'h20; req_ggh <= 1; end // 
                else if (dc_req && ib06_req_h && !reqhalt_g_h && (reqcnt_g_h==0)) begin req_h <= 8'h40; req_ggh <= 1; end //
                else if (dc_req && ib07_req_h && !reqhalt_h_h && (reqcnt_h_h==0)) begin req_h <= 8'h80; req_ggh <= 1; end //
                
                state <= 2;
              end
              1'b1: begin
                if      (dc_req && ib00_req_a && !readend_a && (reqcnt_a==0)) begin req<=8'h01;     state<=4;  end // 
                else if (dc_req && ib01_req_a && !readend_b && (reqcnt_b==0)) begin req<=8'h02;     state<=4;  end // 
                else if (dc_req && ib02_req_a && !readend_c && (reqcnt_c==0)) begin req<=8'h04;     state<=4;  end // 
                else if (dc_req && ib03_req_a && !readend_d && (reqcnt_d==0)) begin req<=8'h08;     state<=4;  end // 
                else if (dc_req && ib04_req_a && !readend_e && (reqcnt_e==0)) begin req<=8'h10;     state<=4;  end // 
                else if (dc_req && ib05_req_a && !readend_f && (reqcnt_f==0)) begin req<=8'h20;     state<=4;  end // 
                else if (dc_req && ib06_req_a && !readend_g && (reqcnt_g==0)) begin req<=8'h40;     state<=4;  end // 
                else if (dc_req && ib07_req_a && !readend_h && (reqcnt_h==0)) begin req<=8'h80;     state<=4;  end //
                else if (OB_req_a && !OB_stopreq_a)                           begin OB_deq_ta <= 1; state<=13; end // WRITE
              end
            endcase
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        2: begin ///// request arbitration 
          if (!d_busy) begin

            if (!pexe_done_a_p) begin
              case (elem_a)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*1: w_addr_a <= mux32(((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>(`P_LOG+3))*1), phase_a[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_a <= mux32(((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>(`P_LOG+3))*2), phase_a[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*3: w_addr_a <= mux32(((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>(`P_LOG+3))*3), phase_a[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_a <= mux32(((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>(`P_LOG+3))*4), phase_a[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*5: w_addr_a <= mux32(((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>(`P_LOG+3))*5), phase_a[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_a <= mux32(((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>(`P_LOG+3))*6), phase_a[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*7: w_addr_a <= mux32(((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>(`P_LOG+3))*7), phase_a[0]);
              endcase
            end
            if (!pexe_done_b_p) begin
              case (elem_b)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*1: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>4)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>4)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>4)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>4)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*3: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>4)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>4)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>4)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>4)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*5: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>4)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>4)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>4)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>4)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*7: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>4)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>4)), phase_b[0]);
              endcase
            end
            if (!pexe_done_c_p) begin
              case (elem_c)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*1: w_addr_c <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>3)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>3)), phase_c[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_c <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>3)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>3)), phase_c[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*3: w_addr_c <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>3)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>3)), phase_c[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_c <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>3)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>3)), phase_c[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*5: w_addr_c <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>3)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>3)), phase_c[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_c <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>3)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>3)), phase_c[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*7: w_addr_c <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>3)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>3)), phase_c[0]);
              endcase
            end
            if (!pexe_done_d_p) begin
              case (elem_d)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*1: w_addr_d <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))), phase_d[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_d <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))), phase_d[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*3: w_addr_d <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))), phase_d[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_d <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))), phase_d[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*5: w_addr_d <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))), phase_d[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_d <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))), phase_d[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*7: w_addr_d <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>4) | (`SORT_ELM>>3))), phase_d[0]);
              endcase
            end
            if (!pexe_done_e_p) begin
              case (elem_e)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*1: w_addr_e <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>2)), phase_e[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_e <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>2)), phase_e[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*3: w_addr_e <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>2)), phase_e[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_e <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>2)), phase_e[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*5: w_addr_e <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>2)), phase_e[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_e <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>2)), phase_e[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*7: w_addr_e <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>2)), phase_e[0]);
              endcase
            end
            if (!pexe_done_f_p) begin
              case (elem_f)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*1: w_addr_f <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))), phase_f[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_f <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))), phase_f[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*3: w_addr_f <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))), phase_f[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_f <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))), phase_f[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*5: w_addr_f <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))), phase_f[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_f <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))), phase_f[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*7: w_addr_f <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>4) | (`SORT_ELM>>2))), phase_f[0]);
              endcase
            end
            if (!pexe_done_g_p) begin
              case (elem_g)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*1: w_addr_g <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_g[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_g <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_g[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*3: w_addr_g <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_g[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_g <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_g[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*5: w_addr_g <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_g[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_g <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_g[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*7: w_addr_g <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_g[0]);
              endcase
            end
            if (!pexe_done_h_p) begin
              case (elem_h)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*1: w_addr_h <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*1) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_h[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_h <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*2) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_h[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*3: w_addr_h <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*3) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_h[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_h <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*4) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_h[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*5: w_addr_h <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*5) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_h[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_h <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*6) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_h[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*7: w_addr_h <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*7) | ((`SORT_ELM>>4) | (`SORT_ELM>>3) | (`SORT_ELM>>2))), phase_h[0]);
              endcase
            end
            if      (req_gga)  begin req_ga <= 1;     state <= 3;  end
            else if (req_ggb)  begin req_gb <= 1;     state <= 3;  end
            else if (req_ggc)  begin req_gc <= 1;     state <= 3;  end
            else if (req_ggd)  begin req_gd <= 1;     state <= 3;  end
            else if (req_gge)  begin req_ge <= 1;     state <= 3;  end
            else if (req_ggf)  begin req_gf <= 1;     state <= 3;  end
            else if (req_ggg)  begin req_gg <= 1;     state <= 3;  end
            else if (req_ggh)  begin req_gh <= 1;     state <= 3;  end
            else if (OB_req_a) begin OB_deq_ta <= 1;  state <= 5;  end // WRITE
            else if (OB_req_b) begin OB_deq_tb <= 1;  state <= 6;  end // WRITE
            else if (OB_req_c) begin OB_deq_tc <= 1;  state <= 7;  end // WRITE
            else if (OB_req_d) begin OB_deq_td <= 1;  state <= 8;  end // WRITE
            else if (OB_req_e) begin OB_deq_te <= 1;  state <= 9;  end // WRITE
            else if (OB_req_f) begin OB_deq_tf <= 1;  state <= 10; end // WRITE
            else if (OB_req_g) begin OB_deq_tg <= 1;  state <= 11; end // WRITE
            else if (OB_req_h) begin OB_deq_th <= 1;  state <= 12; end // WRITE
            else                                      state <= 1;
            {req_gga, req_ggb, req_ggc, req_ggd, req_gge, req_ggf, req_ggg, req_ggh} <= 0;
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        3: begin ///// READ data from DRAM
          if (d_req!=0) begin 
            d_req <= 0; 
            state <= 1;
          end else if (!d_busy) begin
            d_req <= `DRAM_REQ_READ;
            case ({req_gh, req_gg, req_gf, req_ge, req_gd, req_gc, req_gb, req_ga})
              8'h01: begin
                req_g_sel <= 8'h01;
                req_ga    <= 0;
                req_ta    <= req_a;
                case (req_a)
                  8'h01: begin
                    d_initadr <= mux32(radr_a_a, (radr_a_a | (`SORT_ELM>>1)), phase_a[0]);
                    radr_a_a  <= mux32((radr_a_a+(r_block_a_a<<3)), radr_a_a+(`D_RS), (phase_a==0)); 
                    cnt_a_a   <= cnt_a_a+1; 
                    c_a_a     <= (cnt_a_a>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_a_a, `DRAM_RBLOCKS, (phase_a==0));
                  end
                  8'h02: begin
                    d_initadr <= mux32(radr_b_a, (radr_b_a | (`SORT_ELM>>1)), phase_a[0]);
                    radr_b_a  <= mux32((radr_b_a+(r_block_b_a<<3)), radr_b_a+(`D_RS), (phase_a==0)); 
                    cnt_b_a   <= cnt_b_a+1; 
                    c_b_a     <= (cnt_b_a>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_b_a, `DRAM_RBLOCKS, (phase_a==0));
                  end
                  8'h04: begin
                    d_initadr <= mux32(radr_c_a, (radr_c_a | (`SORT_ELM>>1)), phase_a[0]);
                    radr_c_a  <= mux32((radr_c_a+(r_block_c_a<<3)), radr_c_a+(`D_RS), (phase_a==0)); 
                    cnt_c_a   <= cnt_c_a+1; 
                    c_c_a     <= (cnt_c_a>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_c_a, `DRAM_RBLOCKS, (phase_a==0));
                  end
                  8'h08: begin
                    d_initadr <= mux32(radr_d_a, (radr_d_a | (`SORT_ELM>>1)), phase_a[0]);
                    radr_d_a  <= mux32((radr_d_a+(r_block_d_a<<3)), radr_d_a+(`D_RS), (phase_a==0)); 
                    cnt_d_a   <= cnt_d_a+1; 
                    c_d_a     <= (cnt_d_a>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_d_a, `DRAM_RBLOCKS, (phase_a==0));
                  end
                  8'h10: begin
                    d_initadr <= mux32(radr_e_a, (radr_e_a | (`SORT_ELM>>1)), phase_a[0]);
                    radr_e_a  <= mux32((radr_e_a+(r_block_e_a<<3)), radr_e_a+(`D_RS), (phase_a==0));  
                    cnt_e_a   <= cnt_e_a+1; 
                    c_e_a     <= (cnt_e_a>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_e_a, `DRAM_RBLOCKS, (phase_a==0));
                  end
                  8'h20: begin
                    d_initadr <= mux32(radr_f_a, (radr_f_a | (`SORT_ELM>>1)), phase_a[0]);
                    radr_f_a  <= mux32((radr_f_a+(r_block_f_a<<3)), radr_f_a+(`D_RS), (phase_a==0));  
                    cnt_f_a   <= cnt_f_a+1; 
                    c_f_a     <= (cnt_f_a>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_f_a, `DRAM_RBLOCKS, (phase_a==0));
                  end
                  8'h40: begin
                    d_initadr <= mux32(radr_g_a, (radr_g_a | (`SORT_ELM>>1)), phase_a[0]);
                    radr_g_a  <= mux32((radr_g_a+(r_block_g_a<<3)), radr_g_a+(`D_RS), (phase_a==0));  
                    cnt_g_a   <= cnt_g_a+1; 
                    c_g_a     <= (cnt_g_a>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_g_a, `DRAM_RBLOCKS, (phase_a==0));
                  end
                  8'h80: begin
                    d_initadr <= mux32(radr_h_a, (radr_h_a | (`SORT_ELM>>1)), phase_a[0]);
                    radr_h_a  <= mux32((radr_h_a+(r_block_h_a<<3)), radr_h_a+(`D_RS), (phase_a==0));  
                    cnt_h_a   <= cnt_h_a+1; 
                    c_h_a     <= (cnt_h_a>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_h_a, `DRAM_RBLOCKS, (phase_a==0));
                  end
                endcase
              end
              8'h02: begin
                req_g_sel <= 8'h02;
                req_gb    <= 0;
                req_tb    <= req_b;
                case (req_b)
                  8'h01: begin
                    d_initadr <= mux32(radr_a_b, (radr_a_b | (`SORT_ELM>>1)), phase_b[0]);
                    radr_a_b  <= mux32((radr_a_b+(r_block_a_b<<3)), radr_a_b+(`D_RS), (phase_b==0)); 
                    cnt_a_b   <= cnt_a_b+1; 
                    c_a_b     <= (cnt_a_b>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_a_b, `DRAM_RBLOCKS, (phase_b==0));
                  end
                  8'h02: begin
                    d_initadr <= mux32(radr_b_b, (radr_b_b | (`SORT_ELM>>1)), phase_b[0]);
                    radr_b_b  <= mux32((radr_b_b+(r_block_b_b<<3)), radr_b_b+(`D_RS), (phase_b==0)); 
                    cnt_b_b   <= cnt_b_b+1; 
                    c_b_b     <= (cnt_b_b>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_b_b, `DRAM_RBLOCKS, (phase_b==0));
                  end
                  8'h04: begin
                    d_initadr <= mux32(radr_c_b, (radr_c_b | (`SORT_ELM>>1)), phase_b[0]);
                    radr_c_b  <= mux32((radr_c_b+(r_block_c_b<<3)), radr_c_b+(`D_RS), (phase_b==0)); 
                    cnt_c_b   <= cnt_c_b+1; 
                    c_c_b     <= (cnt_c_b>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_c_b, `DRAM_RBLOCKS, (phase_b==0));
                  end
                  8'h08: begin
                    d_initadr <= mux32(radr_d_b, (radr_d_b | (`SORT_ELM>>1)), phase_b[0]);
                    radr_d_b  <= mux32((radr_d_b+(r_block_d_b<<3)), radr_d_b+(`D_RS), (phase_b==0)); 
                    cnt_d_b   <= cnt_d_b+1; 
                    c_d_b     <= (cnt_d_b>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_d_b, `DRAM_RBLOCKS, (phase_b==0));
                  end
                  8'h10: begin
                    d_initadr <= mux32(radr_e_b, (radr_e_b | (`SORT_ELM>>1)), phase_b[0]);
                    radr_e_b  <= mux32((radr_e_b+(r_block_e_b<<3)), radr_e_b+(`D_RS), (phase_b==0)); 
                    cnt_e_b   <= cnt_e_b+1; 
                    c_e_b     <= (cnt_e_b>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_e_b, `DRAM_RBLOCKS, (phase_b==0));
                  end
                  8'h20: begin
                    d_initadr <= mux32(radr_f_b, (radr_f_b | (`SORT_ELM>>1)), phase_b[0]);
                    radr_f_b  <= mux32((radr_f_b+(r_block_f_b<<3)), radr_f_b+(`D_RS), (phase_b==0)); 
                    cnt_f_b   <= cnt_f_b+1; 
                    c_f_b     <= (cnt_f_b>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_f_b, `DRAM_RBLOCKS, (phase_b==0));
                  end
                  8'h40: begin
                    d_initadr <= mux32(radr_g_b, (radr_g_b | (`SORT_ELM>>1)), phase_b[0]);
                    radr_g_b  <= mux32((radr_g_b+(r_block_g_b<<3)), radr_g_b+(`D_RS), (phase_b==0)); 
                    cnt_g_b   <= cnt_g_b+1; 
                    c_g_b     <= (cnt_g_b>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_g_b, `DRAM_RBLOCKS, (phase_b==0));
                  end
                  8'h80: begin
                    d_initadr <= mux32(radr_h_b, (radr_h_b | (`SORT_ELM>>1)), phase_b[0]);
                    radr_h_b  <= mux32((radr_h_b+(r_block_h_b<<3)), radr_h_b+(`D_RS), (phase_b==0)); 
                    cnt_h_b   <= cnt_h_b+1; 
                    c_h_b     <= (cnt_h_b>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_h_b, `DRAM_RBLOCKS, (phase_b==0));
                  end
                endcase
              end
              8'h04: begin
                req_g_sel <= 8'h04;
                req_gc    <= 0;
                req_tc    <= req_c;
                case (req_c)
                  8'h01: begin
                    d_initadr <= mux32(radr_a_c, (radr_a_c | (`SORT_ELM>>1)), phase_c[0]);
                    radr_a_c  <= mux32((radr_a_c+(r_block_a_c<<3)), radr_a_c+(`D_RS), (phase_c==0)); 
                    cnt_a_c   <= cnt_a_c+1; 
                    c_a_c     <= (cnt_a_c>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_a_c, `DRAM_RBLOCKS, (phase_c==0));
                  end
                  8'h02: begin
                    d_initadr <= mux32(radr_b_c, (radr_b_c | (`SORT_ELM>>1)), phase_c[0]);
                    radr_b_c  <= mux32((radr_b_c+(r_block_b_c<<3)), radr_b_c+(`D_RS), (phase_c==0)); 
                    cnt_b_c   <= cnt_b_c+1; 
                    c_b_c     <= (cnt_b_c>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_b_c, `DRAM_RBLOCKS, (phase_c==0));
                  end
                  8'h04: begin
                    d_initadr <= mux32(radr_c_c, (radr_c_c | (`SORT_ELM>>1)), phase_c[0]);
                    radr_c_c  <= mux32((radr_c_c+(r_block_c_c<<3)), radr_c_c+(`D_RS), (phase_c==0)); 
                    cnt_c_c   <= cnt_c_c+1; 
                    c_c_c     <= (cnt_c_c>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_c_c, `DRAM_RBLOCKS, (phase_c==0));
                  end
                  8'h08: begin
                    d_initadr <= mux32(radr_d_c, (radr_d_c | (`SORT_ELM>>1)), phase_c[0]);
                    radr_d_c  <= mux32((radr_d_c+(r_block_d_c<<3)), radr_d_c+(`D_RS), (phase_c==0)); 
                    cnt_d_c   <= cnt_d_c+1; 
                    c_d_c     <= (cnt_d_c>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_d_c, `DRAM_RBLOCKS, (phase_c==0));
                  end
                  8'h10: begin
                    d_initadr <= mux32(radr_e_c, (radr_e_c | (`SORT_ELM>>1)), phase_c[0]);
                    radr_e_c  <= mux32((radr_e_c+(r_block_e_c<<3)), radr_e_c+(`D_RS), (phase_c==0)); 
                    cnt_e_c   <= cnt_e_c+1; 
                    c_e_c     <= (cnt_e_c>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_e_c, `DRAM_RBLOCKS, (phase_c==0));
                  end
                  8'h20: begin
                    d_initadr <= mux32(radr_f_c, (radr_f_c | (`SORT_ELM>>1)), phase_c[0]);
                    radr_f_c  <= mux32((radr_f_c+(r_block_f_c<<3)), radr_f_c+(`D_RS), (phase_c==0)); 
                    cnt_f_c   <= cnt_f_c+1; 
                    c_f_c     <= (cnt_f_c>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_f_c, `DRAM_RBLOCKS, (phase_c==0));
                  end
                  8'h40: begin
                    d_initadr <= mux32(radr_g_c, (radr_g_c | (`SORT_ELM>>1)), phase_c[0]);
                    radr_g_c  <= mux32((radr_g_c+(r_block_g_c<<3)), radr_g_c+(`D_RS), (phase_c==0)); 
                    cnt_g_c   <= cnt_g_c+1; 
                    c_g_c     <= (cnt_g_c>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_g_c, `DRAM_RBLOCKS, (phase_c==0));
                  end
                  8'h80: begin
                    d_initadr <= mux32(radr_h_c, (radr_h_c | (`SORT_ELM>>1)), phase_c[0]);
                    radr_h_c  <= mux32((radr_h_c+(r_block_h_c<<3)), radr_h_c+(`D_RS), (phase_c==0)); 
                    cnt_h_c   <= cnt_h_c+1; 
                    c_h_c     <= (cnt_h_c>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_h_c, `DRAM_RBLOCKS, (phase_c==0));
                  end
                endcase
              end
              8'h08: begin
                req_g_sel <= 8'h08;
                req_gd    <= 0;
                req_td    <= req_d;
                case (req_d)
                  8'h01: begin
                    d_initadr <= mux32(radr_a_d, (radr_a_d | (`SORT_ELM>>1)), phase_d[0]);
                    radr_a_d  <= mux32((radr_a_d+(r_block_a_d<<3)), radr_a_d+(`D_RS), (phase_d==0)); 
                    cnt_a_d   <= cnt_a_d+1; 
                    c_a_d     <= (cnt_a_d>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_a_d, `DRAM_RBLOCKS, (phase_d==0));
                  end
                  8'h02: begin
                    d_initadr <= mux32(radr_b_d, (radr_b_d | (`SORT_ELM>>1)), phase_d[0]);
                    radr_b_d  <= mux32((radr_b_d+(r_block_b_d<<3)), radr_b_d+(`D_RS), (phase_d==0)); 
                    cnt_b_d   <= cnt_b_d+1; 
                    c_b_d     <= (cnt_b_d>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_b_d, `DRAM_RBLOCKS, (phase_d==0));
                  end
                  8'h04: begin
                    d_initadr <= mux32(radr_c_d, (radr_c_d | (`SORT_ELM>>1)), phase_d[0]);
                    radr_c_d  <= mux32((radr_c_d+(r_block_c_d<<3)), radr_c_d+(`D_RS), (phase_d==0)); 
                    cnt_c_d   <= cnt_c_d+1; 
                    c_c_d     <= (cnt_c_d>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_c_d, `DRAM_RBLOCKS, (phase_d==0));
                  end
                  8'h08: begin
                    d_initadr <= mux32(radr_d_d, (radr_d_d | (`SORT_ELM>>1)), phase_d[0]);
                    radr_d_d  <= mux32((radr_d_d+(r_block_d_d<<3)), radr_d_d+(`D_RS), (phase_d==0)); 
                    cnt_d_d   <= cnt_d_d+1; 
                    c_d_d     <= (cnt_d_d>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_d_d, `DRAM_RBLOCKS, (phase_d==0));
                  end
                  8'h10: begin
                    d_initadr <= mux32(radr_e_d, (radr_e_d | (`SORT_ELM>>1)), phase_d[0]);
                    radr_e_d  <= mux32((radr_e_d+(r_block_e_d<<3)), radr_e_d+(`D_RS), (phase_d==0)); 
                    cnt_e_d   <= cnt_e_d+1; 
                    c_e_d     <= (cnt_e_d>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_e_d, `DRAM_RBLOCKS, (phase_d==0));
                  end
                  8'h20: begin
                    d_initadr <= mux32(radr_f_d, (radr_f_d | (`SORT_ELM>>1)), phase_d[0]);
                    radr_f_d  <= mux32((radr_f_d+(r_block_f_d<<3)), radr_f_d+(`D_RS), (phase_d==0)); 
                    cnt_f_d   <= cnt_f_d+1; 
                    c_f_d     <= (cnt_f_d>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_f_d, `DRAM_RBLOCKS, (phase_d==0));
                  end
                  8'h40: begin
                    d_initadr <= mux32(radr_g_d, (radr_g_d | (`SORT_ELM>>1)), phase_d[0]);
                    radr_g_d  <= mux32((radr_g_d+(r_block_g_d<<3)), radr_g_d+(`D_RS), (phase_d==0)); 
                    cnt_g_d   <= cnt_g_d+1; 
                    c_g_d     <= (cnt_g_d>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_g_d, `DRAM_RBLOCKS, (phase_d==0));
                  end
                  8'h80: begin
                    d_initadr <= mux32(radr_h_d, (radr_h_d | (`SORT_ELM>>1)), phase_d[0]);
                    radr_h_d  <= mux32((radr_h_d+(r_block_h_d<<3)), radr_h_d+(`D_RS), (phase_d==0)); 
                    cnt_h_d   <= cnt_h_d+1; 
                    c_h_d     <= (cnt_h_d>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_h_d, `DRAM_RBLOCKS, (phase_d==0));
                  end
                endcase
              end
              8'h10: begin
                req_g_sel <= 8'h10;
                req_ge    <= 0;
                req_te    <= req_e;
                case (req_e)
                  8'h01: begin
                    d_initadr <= mux32(radr_a_e, (radr_a_e | (`SORT_ELM>>1)), phase_e[0]);
                    radr_a_e  <= mux32((radr_a_e+(r_block_a_e<<3)), radr_a_e+(`D_RS), (phase_e==0)); 
                    cnt_a_e   <= cnt_a_e+1; 
                    c_a_e     <= (cnt_a_e>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_a_e, `DRAM_RBLOCKS, (phase_e==0));
                  end
                  8'h02: begin
                    d_initadr <= mux32(radr_b_e, (radr_b_e | (`SORT_ELM>>1)), phase_e[0]);
                    radr_b_e  <= mux32((radr_b_e+(r_block_b_e<<3)), radr_b_e+(`D_RS), (phase_e==0)); 
                    cnt_b_e   <= cnt_b_e+1; 
                    c_b_e     <= (cnt_b_e>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_b_e, `DRAM_RBLOCKS, (phase_e==0));
                  end
                  8'h04: begin
                    d_initadr <= mux32(radr_c_e, (radr_c_e | (`SORT_ELM>>1)), phase_e[0]);
                    radr_c_e  <= mux32((radr_c_e+(r_block_c_e<<3)), radr_c_e+(`D_RS), (phase_e==0)); 
                    cnt_c_e   <= cnt_c_e+1; 
                    c_c_e     <= (cnt_c_e>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_c_e, `DRAM_RBLOCKS, (phase_e==0));
                  end
                  8'h08: begin
                    d_initadr <= mux32(radr_d_e, (radr_d_e | (`SORT_ELM>>1)), phase_e[0]);
                    radr_d_e  <= mux32((radr_d_e+(r_block_d_e<<3)), radr_d_e+(`D_RS), (phase_e==0)); 
                    cnt_d_e   <= cnt_d_e+1; 
                    c_d_e     <= (cnt_d_e>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_d_e, `DRAM_RBLOCKS, (phase_e==0));
                  end
                  8'h10: begin
                    d_initadr <= mux32(radr_e_e, (radr_e_e | (`SORT_ELM>>1)), phase_e[0]);
                    radr_e_e  <= mux32((radr_e_e+(r_block_e_e<<3)), radr_e_e+(`D_RS), (phase_e==0)); 
                    cnt_e_e   <= cnt_e_e+1; 
                    c_e_e     <= (cnt_e_e>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_e_e, `DRAM_RBLOCKS, (phase_e==0));
                  end
                  8'h20: begin
                    d_initadr <= mux32(radr_f_e, (radr_f_e | (`SORT_ELM>>1)), phase_e[0]);
                    radr_f_e  <= mux32((radr_f_e+(r_block_f_e<<3)), radr_f_e+(`D_RS), (phase_e==0)); 
                    cnt_f_e   <= cnt_f_e+1; 
                    c_f_e     <= (cnt_f_e>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_f_e, `DRAM_RBLOCKS, (phase_e==0));
                  end
                  8'h40: begin
                    d_initadr <= mux32(radr_g_e, (radr_g_e | (`SORT_ELM>>1)), phase_e[0]);
                    radr_g_e  <= mux32((radr_g_e+(r_block_g_e<<3)), radr_g_e+(`D_RS), (phase_e==0)); 
                    cnt_g_e   <= cnt_g_e+1; 
                    c_g_e     <= (cnt_g_e>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_g_e, `DRAM_RBLOCKS, (phase_e==0));
                  end
                  8'h80: begin
                    d_initadr <= mux32(radr_h_e, (radr_h_e | (`SORT_ELM>>1)), phase_e[0]);
                    radr_h_e  <= mux32((radr_h_e+(r_block_h_e<<3)), radr_h_e+(`D_RS), (phase_e==0)); 
                    cnt_h_e   <= cnt_h_e+1; 
                    c_h_e     <= (cnt_h_e>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_h_e, `DRAM_RBLOCKS, (phase_e==0));
                  end
                endcase
              end
              8'h20: begin
                req_g_sel <= 8'h20;
                req_gf    <= 0;
                req_tf    <= req_f;
                case (req_f)
                  8'h01: begin
                    d_initadr <= mux32(radr_a_f, (radr_a_f | (`SORT_ELM>>1)), phase_f[0]);
                    radr_a_f  <= mux32((radr_a_f+(r_block_a_f<<3)), radr_a_f+(`D_RS), (phase_f==0)); 
                    cnt_a_f   <= cnt_a_f+1; 
                    c_a_f     <= (cnt_a_f>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_a_f, `DRAM_RBLOCKS, (phase_f==0));
                  end
                  8'h02: begin
                    d_initadr <= mux32(radr_b_f, (radr_b_f | (`SORT_ELM>>1)), phase_f[0]);
                    radr_b_f  <= mux32((radr_b_f+(r_block_b_f<<3)), radr_b_f+(`D_RS), (phase_f==0)); 
                    cnt_b_f   <= cnt_b_f+1; 
                    c_b_f     <= (cnt_b_f>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_b_f, `DRAM_RBLOCKS, (phase_f==0));
                  end
                  8'h04: begin
                    d_initadr <= mux32(radr_c_f, (radr_c_f | (`SORT_ELM>>1)), phase_f[0]);
                    radr_c_f  <= mux32((radr_c_f+(r_block_c_f<<3)), radr_c_f+(`D_RS), (phase_f==0)); 
                    cnt_c_f   <= cnt_c_f+1; 
                    c_c_f     <= (cnt_c_f>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_c_f, `DRAM_RBLOCKS, (phase_f==0));
                  end
                  8'h08: begin
                    d_initadr <= mux32(radr_d_f, (radr_d_f | (`SORT_ELM>>1)), phase_f[0]);
                    radr_d_f  <= mux32((radr_d_f+(r_block_d_f<<3)), radr_d_f+(`D_RS), (phase_f==0)); 
                    cnt_d_f   <= cnt_d_f+1; 
                    c_d_f     <= (cnt_d_f>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_d_f, `DRAM_RBLOCKS, (phase_f==0));
                  end
                  8'h10: begin
                    d_initadr <= mux32(radr_e_f, (radr_e_f | (`SORT_ELM>>1)), phase_f[0]);
                    radr_e_f  <= mux32((radr_e_f+(r_block_e_f<<3)), radr_e_f+(`D_RS), (phase_f==0)); 
                    cnt_e_f   <= cnt_e_f+1; 
                    c_e_f     <= (cnt_e_f>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_e_f, `DRAM_RBLOCKS, (phase_f==0));
                  end
                  8'h20: begin
                    d_initadr <= mux32(radr_f_f, (radr_f_f | (`SORT_ELM>>1)), phase_f[0]);
                    radr_f_f  <= mux32((radr_f_f+(r_block_f_f<<3)), radr_f_f+(`D_RS), (phase_f==0)); 
                    cnt_f_f   <= cnt_f_f+1; 
                    c_f_f     <= (cnt_f_f>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_f_f, `DRAM_RBLOCKS, (phase_f==0));
                  end
                  8'h40: begin
                    d_initadr <= mux32(radr_g_f, (radr_g_f | (`SORT_ELM>>1)), phase_f[0]);
                    radr_g_f  <= mux32((radr_g_f+(r_block_g_f<<3)), radr_g_f+(`D_RS), (phase_f==0)); 
                    cnt_g_f   <= cnt_g_f+1; 
                    c_g_f     <= (cnt_g_f>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_g_f, `DRAM_RBLOCKS, (phase_f==0));
                  end
                  8'h80: begin
                    d_initadr <= mux32(radr_h_f, (radr_h_f | (`SORT_ELM>>1)), phase_f[0]);
                    radr_h_f  <= mux32((radr_h_f+(r_block_h_f<<3)), radr_h_f+(`D_RS), (phase_f==0)); 
                    cnt_h_f   <= cnt_h_f+1; 
                    c_h_f     <= (cnt_h_f>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_h_f, `DRAM_RBLOCKS, (phase_f==0));
                  end
                endcase
              end
              8'h40: begin
                req_g_sel <= 8'h40;
                req_gg    <= 0;
                req_tg    <= req_g;
                case (req_g)
                  8'h01: begin
                    d_initadr <= mux32(radr_a_g, (radr_a_g | (`SORT_ELM>>1)), phase_g[0]);
                    radr_a_g  <= mux32((radr_a_g+(r_block_a_g<<3)), radr_a_g+(`D_RS), (phase_g==0)); 
                    cnt_a_g   <= cnt_a_g+1; 
                    c_a_g     <= (cnt_a_g>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_a_g, `DRAM_RBLOCKS, (phase_g==0));
                  end
                  8'h02: begin
                    d_initadr <= mux32(radr_b_g, (radr_b_g | (`SORT_ELM>>1)), phase_g[0]);
                    radr_b_g  <= mux32((radr_b_g+(r_block_b_g<<3)), radr_b_g+(`D_RS), (phase_g==0)); 
                    cnt_b_g   <= cnt_b_g+1; 
                    c_b_g     <= (cnt_b_g>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_b_g, `DRAM_RBLOCKS, (phase_g==0));
                  end
                  8'h04: begin
                    d_initadr <= mux32(radr_c_g, (radr_c_g | (`SORT_ELM>>1)), phase_g[0]);
                    radr_c_g  <= mux32((radr_c_g+(r_block_c_g<<3)), radr_c_g+(`D_RS), (phase_g==0)); 
                    cnt_c_g   <= cnt_c_g+1; 
                    c_c_g     <= (cnt_c_g>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_c_g, `DRAM_RBLOCKS, (phase_g==0));
                  end
                  8'h08: begin
                    d_initadr <= mux32(radr_d_g, (radr_d_g | (`SORT_ELM>>1)), phase_g[0]);
                    radr_d_g  <= mux32((radr_d_g+(r_block_d_g<<3)), radr_d_g+(`D_RS), (phase_g==0)); 
                    cnt_d_g   <= cnt_d_g+1; 
                    c_d_g     <= (cnt_d_g>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_d_g, `DRAM_RBLOCKS, (phase_g==0));
                  end
                  8'h10: begin
                    d_initadr <= mux32(radr_e_g, (radr_e_g | (`SORT_ELM>>1)), phase_g[0]);
                    radr_e_g  <= mux32((radr_e_g+(r_block_e_g<<3)), radr_e_g+(`D_RS), (phase_g==0)); 
                    cnt_e_g   <= cnt_e_g+1; 
                    c_e_g     <= (cnt_e_g>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_e_g, `DRAM_RBLOCKS, (phase_g==0));
                  end
                  8'h20: begin
                    d_initadr <= mux32(radr_f_g, (radr_f_g | (`SORT_ELM>>1)), phase_g[0]);
                    radr_f_g  <= mux32((radr_f_g+(r_block_f_g<<3)), radr_f_g+(`D_RS), (phase_g==0)); 
                    cnt_f_g   <= cnt_f_g+1; 
                    c_f_g     <= (cnt_f_g>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_f_g, `DRAM_RBLOCKS, (phase_g==0));
                  end
                  8'h40: begin
                    d_initadr <= mux32(radr_g_g, (radr_g_g | (`SORT_ELM>>1)), phase_g[0]);
                    radr_g_g  <= mux32((radr_g_g+(r_block_g_g<<3)), radr_g_g+(`D_RS), (phase_g==0)); 
                    cnt_g_g   <= cnt_g_g+1; 
                    c_g_g     <= (cnt_g_g>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_g_g, `DRAM_RBLOCKS, (phase_g==0));
                  end
                  8'h80: begin
                    d_initadr <= mux32(radr_h_g, (radr_h_g | (`SORT_ELM>>1)), phase_g[0]);
                    radr_h_g  <= mux32((radr_h_g+(r_block_h_g<<3)), radr_h_g+(`D_RS), (phase_g==0)); 
                    cnt_h_g   <= cnt_h_g+1; 
                    c_h_g     <= (cnt_h_g>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_h_g, `DRAM_RBLOCKS, (phase_g==0));
                  end
                endcase
              end
              8'h80: begin
                req_g_sel <= 8'h80;
                req_gh    <= 0;
                req_th    <= req_h;
                case (req_h)
                  8'h01: begin
                    d_initadr <= mux32(radr_a_h, (radr_a_h | (`SORT_ELM>>1)), phase_h[0]);
                    radr_a_h  <= mux32((radr_a_h+(r_block_a_h<<3)), radr_a_h+(`D_RS), (phase_h==0)); 
                    cnt_a_h   <= cnt_a_h+1; 
                    c_a_h     <= (cnt_a_h>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_a_h, `DRAM_RBLOCKS, (phase_h==0));
                  end
                  8'h02: begin
                    d_initadr <= mux32(radr_b_h, (radr_b_h | (`SORT_ELM>>1)), phase_h[0]);
                    radr_b_h  <= mux32((radr_b_h+(r_block_b_h<<3)), radr_b_h+(`D_RS), (phase_h==0)); 
                    cnt_b_h   <= cnt_b_h+1; 
                    c_b_h     <= (cnt_b_h>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_b_h, `DRAM_RBLOCKS, (phase_h==0));
                  end
                  8'h04: begin
                    d_initadr <= mux32(radr_c_h, (radr_c_h | (`SORT_ELM>>1)), phase_h[0]);
                    radr_c_h  <= mux32((radr_c_h+(r_block_c_h<<3)), radr_c_h+(`D_RS), (phase_h==0)); 
                    cnt_c_h   <= cnt_c_h+1; 
                    c_c_h     <= (cnt_c_h>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_c_h, `DRAM_RBLOCKS, (phase_h==0));
                  end
                  8'h08: begin
                    d_initadr <= mux32(radr_d_h, (radr_d_h | (`SORT_ELM>>1)), phase_h[0]);
                    radr_d_h  <= mux32((radr_d_h+(r_block_d_h<<3)), radr_d_h+(`D_RS), (phase_h==0)); 
                    cnt_d_h   <= cnt_d_h+1; 
                    c_d_h     <= (cnt_d_h>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_d_h, `DRAM_RBLOCKS, (phase_h==0));
                  end
                  8'h10: begin
                    d_initadr <= mux32(radr_e_h, (radr_e_h | (`SORT_ELM>>1)), phase_h[0]);
                    radr_e_h  <= mux32((radr_e_h+(r_block_e_h<<3)), radr_e_h+(`D_RS), (phase_h==0)); 
                    cnt_e_h   <= cnt_e_h+1; 
                    c_e_h     <= (cnt_e_h>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_e_h, `DRAM_RBLOCKS, (phase_h==0));
                  end
                  8'h20: begin
                    d_initadr <= mux32(radr_f_h, (radr_f_h | (`SORT_ELM>>1)), phase_h[0]);
                    radr_f_h  <= mux32((radr_f_h+(r_block_f_h<<3)), radr_f_h+(`D_RS), (phase_h==0)); 
                    cnt_f_h   <= cnt_f_h+1; 
                    c_f_h     <= (cnt_f_h>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_f_h, `DRAM_RBLOCKS, (phase_h==0));
                  end
                  8'h40: begin
                    d_initadr <= mux32(radr_g_h, (radr_g_h | (`SORT_ELM>>1)), phase_h[0]);
                    radr_g_h  <= mux32((radr_g_h+(r_block_g_h<<3)), radr_g_h+(`D_RS), (phase_h==0)); 
                    cnt_g_h   <= cnt_g_h+1; 
                    c_g_h     <= (cnt_g_h>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_g_h, `DRAM_RBLOCKS, (phase_h==0));
                  end
                  8'h80: begin
                    d_initadr <= mux32(radr_h_h, (radr_h_h | (`SORT_ELM>>1)), phase_h[0]);
                    radr_h_h  <= mux32((radr_h_h+(r_block_h_h<<3)), radr_h_h+(`D_RS), (phase_h==0)); 
                    cnt_h_h   <= cnt_h_h+1; 
                    c_h_h     <= (cnt_h_h>=`WAYP_CN_);
                    d_blocks  <= mux32(r_block_h_h, `DRAM_RBLOCKS, (phase_h==0));
                  end
                endcase
              end
            endcase
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        4: begin
          if (d_req!=0) begin 
            d_req <= 0; 
            state <= 1;
          end else if (!d_busy) begin
            case (req)
              8'h01: begin
                d_initadr <= mux32(radr_a, (radr_a | (`SORT_ELM>>1)), l_phase[0]);
                radr_a    <= radr_a+(r_block_a<<3); 
                d_blocks  <= r_block_a;
              end
              8'h02: begin
                d_initadr <= mux32(radr_b, (radr_b | (`SORT_ELM>>1)), l_phase[0]);
                radr_b    <= radr_b+(r_block_b<<3); 
                d_blocks  <= r_block_b;
              end
              8'h04: begin
                d_initadr <= mux32(radr_c, (radr_c | (`SORT_ELM>>1)), l_phase[0]);
                radr_c    <= radr_c+(r_block_c<<3); 
                d_blocks  <= r_block_c;
              end
              8'h08: begin
                d_initadr <= mux32(radr_d, (radr_d | (`SORT_ELM>>1)), l_phase[0]);
                radr_d    <= radr_d+(r_block_d<<3); 
                d_blocks  <= r_block_d;
              end
              8'h10: begin
                d_initadr <= mux32(radr_e, (radr_e | (`SORT_ELM>>1)), l_phase[0]);
                radr_e     <= radr_e+(r_block_e<<3); 
                d_blocks  <= r_block_e;
              end
              8'h20: begin
                d_initadr <= mux32(radr_f, (radr_f | (`SORT_ELM>>1)), l_phase[0]);
                radr_f    <= radr_f+(r_block_f<<3); 
                d_blocks  <= r_block_f;
              end
              8'h40: begin
                d_initadr <= mux32(radr_g, (radr_g | (`SORT_ELM>>1)), l_phase[0]);
                radr_g    <= radr_g+(r_block_g<<3); 
                d_blocks  <= r_block_g;
              end
              8'h80: begin
                d_initadr <= mux32(radr_h, (radr_h | (`SORT_ELM>>1)), l_phase[0]);
                radr_h    <= radr_h+(r_block_h<<3); 
                d_blocks  <= r_block_h;
              end
            endcase
            d_req     <= `DRAM_REQ_READ;
            req_ta    <= req;
            req_g_sel <= 8'h01;
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        5: begin ///// WRITE data to DRAM
          if (d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= w_block_a;
            d_initadr <= w_addr_a;
            w_addr_a  <= w_addr_a + (w_block_a<<3);
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        6: begin ///// WRITE data to DRAM
          if(d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= w_block_b;
            d_initadr <= w_addr_b;
            w_addr_b  <= w_addr_b + (w_block_b<<3);
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        7: begin ///// WRITE data to DRAM
          if(d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= w_block_c;
            d_initadr <= w_addr_c;
            w_addr_c  <= w_addr_c + (w_block_c<<3);
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        8: begin ///// WRITE data to DRAM
          if(d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= w_block_d;
            d_initadr <= w_addr_d;
            w_addr_d  <= w_addr_d + (w_block_d<<3);
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        9: begin ///// WRITE data to DRAM
          if(d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= w_block_e;
            d_initadr <= w_addr_e;
            w_addr_e  <= w_addr_e + (w_block_e<<3);
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        10: begin ///// WRITE data to DRAM
          if(d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= w_block_f;
            d_initadr <= w_addr_f;
            w_addr_f  <= w_addr_f + (w_block_f<<3);
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        11: begin ///// WRITE data to DRAM
          if(d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= w_block_g;
            d_initadr <= w_addr_g;
            w_addr_g  <= w_addr_g + (w_block_g<<3);
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        12: begin ///// WRITE data to DRAM
          if(d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= w_block_h;
            d_initadr <= w_addr_h;
            w_addr_h  <= w_addr_h + (w_block_h<<3);
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        13: begin ///// WRITE data to DRAM
          if (d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= w_block;
            d_initadr <= w_addr;
            w_addr    <= w_addr + (w_block<<3);  // address for the next write
          end
        end
      endcase
    end
  end
  
  /***** WRITE : feed the initial data to be stored to DRAM                                 *****/
  /**********************************************************************************************/
  always @(posedge CLK) idone_a <= initdone;
  always @(posedge CLK) idone_b <= initdone;
  always @(posedge CLK) idone_c <= initdone;
  always @(posedge CLK) idone_d <= initdone;
  always @(posedge CLK) idone_e <= initdone;
  always @(posedge CLK) idone_f <= initdone;
  always @(posedge CLK) idone_g <= initdone;
  always @(posedge CLK) idone_h <= initdone;
  
  wire [`DRAMW-1:0] initdata;
  INITDATAGEN initdatagen(CLK, RSTa, d_w, initdata);
  
  assign d_din[31:0] = mux32(initdata[31:0],
                             mux8in32(OB_dot0[31:0], OB_dot1[31:0], OB_dot2[31:0], OB_dot3[31:0], 
                                      OB_dot4[31:0], OB_dot5[31:0], OB_dot6[31:0], OB_dot7[31:0], OB_dot_sel),
                             idone_a);
  assign d_din[63:32] = mux32(initdata[63:32],
                              mux8in32(OB_dot0[63:32], OB_dot1[63:32], OB_dot2[63:32], OB_dot3[63:32], 
                                       OB_dot4[63:32], OB_dot5[63:32], OB_dot6[63:32], OB_dot7[63:32], OB_dot_sel),
                              idone_a);
  assign d_din[95:64] = mux32(initdata[95:64],
                              mux8in32(OB_dot0[95:64], OB_dot1[95:64], OB_dot2[95:64], OB_dot3[95:64], 
                                       OB_dot4[95:64], OB_dot5[95:64], OB_dot6[95:64], OB_dot7[95:64], OB_dot_sel),
                              idone_b);
  assign d_din[127:96] = mux32(initdata[127:96],
                               mux8in32(OB_dot0[127:96], OB_dot1[127:96], OB_dot2[127:96], OB_dot3[127:96], 
                                        OB_dot4[127:96], OB_dot5[127:96], OB_dot6[127:96], OB_dot7[127:96], OB_dot_sel),
                               idone_b);
  assign d_din[159:128] = mux32(initdata[159:128],
                                mux8in32(OB_dot0[159:128], OB_dot1[159:128], OB_dot2[159:128], OB_dot3[159:128], 
                                         OB_dot4[159:128], OB_dot5[159:128], OB_dot6[159:128], OB_dot7[159:128], OB_dot_sel),
                                idone_c);
  assign d_din[191:160] = mux32(initdata[191:160],
                                mux8in32(OB_dot0[191:160], OB_dot1[191:160], OB_dot2[191:160], OB_dot3[191:160], 
                                         OB_dot4[191:160], OB_dot5[191:160], OB_dot6[191:160], OB_dot7[191:160], OB_dot_sel),
                                idone_c);
  assign d_din[223:192] = mux32(initdata[223:192],
                                mux8in32(OB_dot0[223:192], OB_dot1[223:192], OB_dot2[223:192], OB_dot3[223:192], 
                                         OB_dot4[223:192], OB_dot5[223:192], OB_dot6[223:192], OB_dot7[223:192], OB_dot_sel),
                                idone_d);
  assign d_din[255:224] = mux32(initdata[255:224],
                                mux8in32(OB_dot0[255:224], OB_dot1[255:224], OB_dot2[255:224], OB_dot3[255:224], 
                                         OB_dot4[255:224], OB_dot5[255:224], OB_dot6[255:224], OB_dot7[255:224], OB_dot_sel),
                                idone_d);
  assign d_din[287:256] = mux32(initdata[287:256],
                                mux8in32(OB_dot0[287:256], OB_dot1[287:256], OB_dot2[287:256], OB_dot3[287:256], 
                                         OB_dot4[287:256], OB_dot5[287:256], OB_dot6[287:256], OB_dot7[287:256], OB_dot_sel),
                                idone_e);
  assign d_din[319:288] = mux32(initdata[319:288],
                                mux8in32(OB_dot0[319:288], OB_dot1[319:288], OB_dot2[319:288], OB_dot3[319:288], 
                                         OB_dot4[319:288], OB_dot5[319:288], OB_dot6[319:288], OB_dot7[319:288], OB_dot_sel),
                                idone_e);
  assign d_din[351:320] = mux32(initdata[351:320],
                                mux8in32(OB_dot0[351:320], OB_dot1[351:320], OB_dot2[351:320], OB_dot3[351:320], 
                                         OB_dot4[351:320], OB_dot5[351:320], OB_dot6[351:320], OB_dot7[351:320], OB_dot_sel),
                                idone_f);
  assign d_din[383:352] = mux32(initdata[383:352],
                                mux8in32(OB_dot0[383:352], OB_dot1[383:352], OB_dot2[383:352], OB_dot3[383:352], 
                                         OB_dot4[383:352], OB_dot5[383:352], OB_dot6[383:352], OB_dot7[383:352], OB_dot_sel),
                                idone_f);
  assign d_din[415:384] = mux32(initdata[415:384],
                                mux8in32(OB_dot0[415:384], OB_dot1[415:384], OB_dot2[415:384], OB_dot3[415:384], 
                                         OB_dot4[415:384], OB_dot5[415:384], OB_dot6[415:384], OB_dot7[415:384], OB_dot_sel),
                                idone_g);
  assign d_din[447:416] = mux32(initdata[447:416],
                                mux8in32(OB_dot0[447:416], OB_dot1[447:416], OB_dot2[447:416], OB_dot3[447:416], 
                                         OB_dot4[447:416], OB_dot5[447:416], OB_dot6[447:416], OB_dot7[447:416], OB_dot_sel),
                                idone_g);
  assign d_din[479:448] = mux32(initdata[479:448],
                                mux8in32(OB_dot0[479:448], OB_dot1[479:448], OB_dot2[479:448], OB_dot3[479:448], 
                                         OB_dot4[479:448], OB_dot5[479:448], OB_dot6[479:448], OB_dot7[479:448], OB_dot_sel),
                                idone_h);
  assign d_din[511:480] = mux32(initdata[511:480],
                                mux8in32(OB_dot0[511:480], OB_dot1[511:480], OB_dot2[511:480], OB_dot3[511:480], 
                                         OB_dot4[511:480], OB_dot5[511:480], OB_dot6[511:480], OB_dot7[511:480], OB_dot_sel),
                                idone_h);
  
  /**********************************************************************************************/    
  always @(posedge CLK) begin
    // Stage 0
    ////////////////////////////////////
    dout_tta  <= stnet_dout;
    dout_ttb  <= stnet_dout;
    
    doen_tta  <= stnet_douten[0];
    doen_ttb  <= stnet_douten[0];
    
    case (stnet_douten[(1<<`P_LOG)+`SORT_WAY:`SORT_WAY+1])
      8'h01: begin
        req_tt0_a <= stnet_douten[`SORT_WAY:1];
        req_tt0_b <= 0;
        req_tt0_c <= 0;
        req_tt0_d <= 0;
        req_tt0_e <= 0;
        req_tt0_f <= 0;
        req_tt0_g <= 0;
        req_tt0_h <= 0;
      end
      8'h02: begin
        req_tt0_a <= 0;
        req_tt0_b <= stnet_douten[`SORT_WAY:1];
        req_tt0_c <= 0;
        req_tt0_d <= 0;
        req_tt0_e <= 0;
        req_tt0_f <= 0;
        req_tt0_g <= 0;
        req_tt0_h <= 0;
      end
      8'h04: begin
        req_tt0_a <= 0;
        req_tt0_b <= 0;
        req_tt0_c <= stnet_douten[`SORT_WAY:1];
        req_tt0_d <= 0;
        req_tt0_e <= 0;
        req_tt0_f <= 0;
        req_tt0_g <= 0;
        req_tt0_h <= 0;
      end
      8'h08: begin
        req_tt0_a <= 0;
        req_tt0_b <= 0;
        req_tt0_c <= 0;
        req_tt0_d <= stnet_douten[`SORT_WAY:1];
        req_tt0_e <= 0;
        req_tt0_f <= 0;
        req_tt0_g <= 0;
        req_tt0_h <= 0;
      end
      8'h10: begin
        req_tt0_a <= 0;
        req_tt0_b <= 0;
        req_tt0_c <= 0;
        req_tt0_d <= 0;
        req_tt0_e <= stnet_douten[`SORT_WAY:1];
        req_tt0_f <= 0;
        req_tt0_g <= 0;
        req_tt0_h <= 0;
      end
      8'h20: begin
        req_tt0_a <= 0;
        req_tt0_b <= 0;
        req_tt0_c <= 0;
        req_tt0_d <= 0;
        req_tt0_e <= 0;
        req_tt0_f <= stnet_douten[`SORT_WAY:1];
        req_tt0_g <= 0;
        req_tt0_h <= 0;
      end
      8'h40: begin
        req_tt0_a <= 0;
        req_tt0_b <= 0;
        req_tt0_c <= 0;
        req_tt0_d <= 0;
        req_tt0_e <= 0;
        req_tt0_g <= stnet_douten[`SORT_WAY:1];
        req_tt0_f <= 0;
        req_tt0_h <= 0;
      end
      8'h80: begin
        req_tt0_a <= 0;
        req_tt0_b <= 0;
        req_tt0_c <= 0;
        req_tt0_d <= 0;
        req_tt0_e <= 0;
        req_tt0_f <= 0;
        req_tt0_g <= 0;
        req_tt0_h <= stnet_douten[`SORT_WAY:1];
      end
    endcase
    
    // Stage 1
    ////////////////////////////////////
    dout_ttc  <= dout_tta;
    dout_ttd  <= dout_tta;
    dout_tte  <= dout_ttb;
    dout_ttf  <= dout_ttb;  

    doen_ttc  <= doen_tta;
    doen_ttd  <= doen_tta;
    doen_tte  <= doen_ttb;
    doen_ttf  <= doen_ttb;
    
    req_tt1_a <= req_tt0_a;
    req_tt1_b <= req_tt0_b;
    req_tt1_c <= req_tt0_c;
    req_tt1_d <= req_tt0_d;
    req_tt1_e <= req_tt0_e;
    req_tt1_f <= req_tt0_f;
    req_tt1_g <= req_tt0_g;
    req_tt1_h <= req_tt0_h;
    
    // Stage 2
    ////////////////////////////////////
    dout_t0_a <= dout_ttc;
    dout_t0_b <= dout_ttc;
    dout_t0_c <= dout_ttd;
    dout_t0_d <= dout_ttd;
    dout_t0_e <= dout_tte;
    dout_t0_f <= dout_tte;
    dout_t0_g <= dout_ttf;
    dout_t0_h <= dout_ttf;
    
    doen_t0_a <= doen_ttc;
    doen_t0_b <= doen_ttc;
    doen_t0_c <= doen_ttd;
    doen_t0_d <= doen_ttd;
    doen_t0_e <= doen_tte;
    doen_t0_f <= doen_tte;
    doen_t0_g <= doen_ttf;
    doen_t0_h <= doen_ttf;
    
    req_tt2_a <= req_tt1_a;
    req_tt2_b <= req_tt1_b;
    req_tt2_c <= req_tt1_c;
    req_tt2_d <= req_tt1_d;
    req_tt2_e <= req_tt1_e;
    req_tt2_f <= req_tt1_f;
    req_tt2_g <= req_tt1_g;
    req_tt2_h <= req_tt1_h;
    
    // Stage 3
    ////////////////////////////////////
    dout_t1_a <= dout_t0_a;
    dout_t2_a <= dout_t0_a;
    dout_t1_b <= dout_t0_b;
    dout_t2_b <= dout_t0_b;
    dout_t1_c <= dout_t0_c;
    dout_t2_c <= dout_t0_c;
    dout_t1_d <= dout_t0_d;
    dout_t2_d <= dout_t0_d;
    dout_t1_e <= dout_t0_e;
    dout_t2_e <= dout_t0_e;
    dout_t1_f <= dout_t0_f;
    dout_t2_f <= dout_t0_f;
    dout_t1_g <= dout_t0_g;
    dout_t2_g <= dout_t0_g;
    dout_t1_h <= dout_t0_h;
    dout_t2_h <= dout_t0_h;
    
    doen_t1_a <= doen_t0_a;
    doen_t2_a <= doen_t0_a;
    doen_t1_b <= doen_t0_b;
    doen_t2_b <= doen_t0_b;
    doen_t1_c <= doen_t0_c;
    doen_t2_c <= doen_t0_c;
    doen_t1_d <= doen_t0_d;
    doen_t2_d <= doen_t0_d;
    doen_t1_e <= doen_t0_e;
    doen_t2_e <= doen_t0_e;
    doen_t1_f <= doen_t0_f;
    doen_t2_f <= doen_t0_f;
    doen_t1_g <= doen_t0_g;
    doen_t2_g <= doen_t0_g;
    doen_t1_h <= doen_t0_h;
    doen_t2_h <= doen_t0_h;
    
    req_tt3_a <= req_tt2_a;
    req_tt3_b <= req_tt2_b;
    req_tt3_c <= req_tt2_c;
    req_tt3_d <= req_tt2_d;
    req_tt3_e <= req_tt2_e;
    req_tt3_f <= req_tt2_f;
    req_tt3_g <= req_tt2_g;
    req_tt3_h <= req_tt2_h;

    // Stage 4
    ////////////////////////////////////
    dout_t3_a <= dout_t1_a;
    dout_t4_a <= dout_t1_a;
    dout_t5_a <= dout_t2_a;
    dout_t6_a <= dout_t2_a;
    dout_t3_b <= dout_t1_b;
    dout_t4_b <= dout_t1_b;
    dout_t5_b <= dout_t2_b;
    dout_t6_b <= dout_t2_b;
    dout_t3_c <= dout_t1_c;
    dout_t4_c <= dout_t1_c;
    dout_t5_c <= dout_t2_c;
    dout_t6_c <= dout_t2_c;
    dout_t3_d <= dout_t1_d;
    dout_t4_d <= dout_t1_d;
    dout_t5_d <= dout_t2_d;
    dout_t6_d <= dout_t2_d;
    dout_t3_e <= dout_t1_e;
    dout_t4_e <= dout_t1_e;
    dout_t5_e <= dout_t2_e;
    dout_t6_e <= dout_t2_e;
    dout_t3_f <= dout_t1_f;
    dout_t4_f <= dout_t1_f;
    dout_t5_f <= dout_t2_f;
    dout_t6_f <= dout_t2_f;
    dout_t3_g <= dout_t1_g;
    dout_t4_g <= dout_t1_g;
    dout_t5_g <= dout_t2_g;
    dout_t6_g <= dout_t2_g;
    dout_t3_h <= dout_t1_h;
    dout_t4_h <= dout_t1_h;
    dout_t5_h <= dout_t2_h;
    dout_t6_h <= dout_t2_h;
    
    doen_t3_a <= doen_t1_a;
    doen_t4_a <= doen_t1_a;
    doen_t5_a <= doen_t2_a;
    doen_t6_a <= doen_t2_a;
    doen_t3_b <= doen_t1_b;
    doen_t4_b <= doen_t1_b;
    doen_t5_b <= doen_t2_b;
    doen_t6_b <= doen_t2_b;
    doen_t3_c <= doen_t1_c;
    doen_t4_c <= doen_t1_c;
    doen_t5_c <= doen_t2_c;
    doen_t6_c <= doen_t2_c;
    doen_t3_d <= doen_t1_d;
    doen_t4_d <= doen_t1_d;
    doen_t5_d <= doen_t2_d;
    doen_t6_d <= doen_t2_d;
    doen_t3_e <= doen_t1_e;
    doen_t4_e <= doen_t1_e;
    doen_t5_e <= doen_t2_e;
    doen_t6_e <= doen_t2_e;
    doen_t3_f <= doen_t1_f;
    doen_t4_f <= doen_t1_f;
    doen_t5_f <= doen_t2_f;
    doen_t6_f <= doen_t2_f;
    doen_t3_g <= doen_t1_g;
    doen_t4_g <= doen_t1_g;
    doen_t5_g <= doen_t2_g;
    doen_t6_g <= doen_t2_g;
    doen_t3_h <= doen_t1_h;
    doen_t4_h <= doen_t1_h;
    doen_t5_h <= doen_t2_h;
    doen_t6_h <= doen_t2_h;

    req_tt4_a <= req_tt3_a;
    req_tt4_b <= req_tt3_b;
    req_tt4_c <= req_tt3_c;
    req_tt4_d <= req_tt3_d;
    req_tt4_e <= req_tt3_e;
    req_tt4_f <= req_tt3_f;
    req_tt4_g <= req_tt3_g;
    req_tt4_h <= req_tt3_h;

    // Stage 5
    ////////////////////////////////////
    dout_ta_a <= dout_t3_a;
    dout_tb_a <= dout_t3_a;
    dout_tc_a <= dout_t4_a;
    dout_td_a <= dout_t4_a;
    dout_te_a <= dout_t5_a;
    dout_tf_a <= dout_t5_a;
    dout_tg_a <= dout_t6_a;
    dout_th_a <= dout_t6_a;
    dout_ta_b <= dout_t3_b;
    dout_tb_b <= dout_t3_b;
    dout_tc_b <= dout_t4_b;
    dout_td_b <= dout_t4_b;
    dout_te_b <= dout_t5_b;
    dout_tf_b <= dout_t5_b;
    dout_tg_b <= dout_t6_b;
    dout_th_b <= dout_t6_b;
    dout_ta_c <= dout_t3_c;
    dout_tb_c <= dout_t3_c;
    dout_tc_c <= dout_t4_c;
    dout_td_c <= dout_t4_c;
    dout_te_c <= dout_t5_c;
    dout_tf_c <= dout_t5_c;
    dout_tg_c <= dout_t6_c;
    dout_th_c <= dout_t6_c;
    dout_ta_d <= dout_t3_d;
    dout_tb_d <= dout_t3_d;
    dout_tc_d <= dout_t4_d;
    dout_td_d <= dout_t4_d;
    dout_te_d <= dout_t5_d;
    dout_tf_d <= dout_t5_d;
    dout_tg_d <= dout_t6_d;
    dout_th_d <= dout_t6_d;
    dout_ta_e <= dout_t3_e;
    dout_tb_e <= dout_t3_e;
    dout_tc_e <= dout_t4_e;
    dout_td_e <= dout_t4_e;
    dout_te_e <= dout_t5_e;
    dout_tf_e <= dout_t5_e;
    dout_tg_e <= dout_t6_e;
    dout_th_e <= dout_t6_e;
    dout_ta_f <= dout_t3_f;
    dout_tb_f <= dout_t3_f;
    dout_tc_f <= dout_t4_f;
    dout_td_f <= dout_t4_f;
    dout_te_f <= dout_t5_f;
    dout_tf_f <= dout_t5_f;
    dout_tg_f <= dout_t6_f;
    dout_th_f <= dout_t6_f;
    dout_ta_g <= dout_t3_g;
    dout_tb_g <= dout_t3_g;
    dout_tc_g <= dout_t4_g;
    dout_td_g <= dout_t4_g;
    dout_te_g <= dout_t5_g;
    dout_tf_g <= dout_t5_g;
    dout_tg_g <= dout_t6_g;
    dout_th_g <= dout_t6_g;
    dout_ta_h <= dout_t3_h;
    dout_tb_h <= dout_t3_h;
    dout_tc_h <= dout_t4_h;
    dout_td_h <= dout_t4_h;
    dout_te_h <= dout_t5_h;
    dout_tf_h <= dout_t5_h;
    dout_tg_h <= dout_t6_h;
    dout_th_h <= dout_t6_h;
    
    doen_ta_a <= doen_t3_a;
    doen_tb_a <= doen_t3_a;
    doen_tc_a <= doen_t4_a;
    doen_td_a <= doen_t4_a;
    doen_te_a <= doen_t5_a;
    doen_tf_a <= doen_t5_a;
    doen_tg_a <= doen_t6_a;
    doen_th_a <= doen_t6_a;
    doen_ta_b <= doen_t3_b;
    doen_tb_b <= doen_t3_b;
    doen_tc_b <= doen_t4_b;
    doen_td_b <= doen_t4_b;
    doen_te_b <= doen_t5_b;
    doen_tf_b <= doen_t5_b;
    doen_tg_b <= doen_t6_b;
    doen_th_b <= doen_t6_b;
    doen_ta_c <= doen_t3_c;
    doen_tb_c <= doen_t3_c;
    doen_tc_c <= doen_t4_c;
    doen_td_c <= doen_t4_c;
    doen_te_c <= doen_t5_c;
    doen_tf_c <= doen_t5_c;
    doen_tg_c <= doen_t6_c;
    doen_th_c <= doen_t6_c;
    doen_ta_d <= doen_t3_d;
    doen_tb_d <= doen_t3_d;
    doen_tc_d <= doen_t4_d;
    doen_td_d <= doen_t4_d;
    doen_te_d <= doen_t5_d;
    doen_tf_d <= doen_t5_d;
    doen_tg_d <= doen_t6_d;
    doen_th_d <= doen_t6_d;
    doen_ta_e <= doen_t3_e;
    doen_tb_e <= doen_t3_e;
    doen_tc_e <= doen_t4_e;
    doen_td_e <= doen_t4_e;
    doen_te_e <= doen_t5_e;
    doen_tf_e <= doen_t5_e;
    doen_tg_e <= doen_t6_e;
    doen_th_e <= doen_t6_e;
    doen_ta_f <= doen_t3_f;
    doen_tb_f <= doen_t3_f;
    doen_tc_f <= doen_t4_f;
    doen_td_f <= doen_t4_f;
    doen_te_f <= doen_t5_f;
    doen_tf_f <= doen_t5_f;
    doen_tg_f <= doen_t6_f;
    doen_th_f <= doen_t6_f;
    doen_ta_g <= doen_t3_g;
    doen_tb_g <= doen_t3_g;
    doen_tc_g <= doen_t4_g;
    doen_td_g <= doen_t4_g;
    doen_te_g <= doen_t5_g;
    doen_tf_g <= doen_t5_g;
    doen_tg_g <= doen_t6_g;
    doen_th_g <= doen_t6_g;
    doen_ta_h <= doen_t3_h;
    doen_tb_h <= doen_t3_h;
    doen_tc_h <= doen_t4_h;
    doen_td_h <= doen_t4_h;
    doen_te_h <= doen_t5_h;
    doen_tf_h <= doen_t5_h;
    doen_tg_h <= doen_t6_h;
    doen_th_h <= doen_t6_h;
    
    req_tt5_a <= req_tt4_a;
    req_tt5_b <= req_tt4_b;
    req_tt5_c <= req_tt4_c;
    req_tt5_d <= req_tt4_d;
    req_tt5_e <= req_tt4_e;
    req_tt5_f <= req_tt4_f;
    req_tt5_g <= req_tt4_g;
    req_tt5_h <= req_tt4_h;
  end

  // for last_phase
  // ###########################################################################
  always @(posedge CLK) begin
    if (RSTa) begin
      last_phase <= 0;
    end else begin
      if (last_phase_a && last_phase_b) last_phase <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTa) begin
      last_phase_a <= 0;
    end else begin
      if (last_phase_c && last_phase_d) last_phase_a <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTb) begin
      last_phase_b <= 0;
    end else begin
      if (last_phase_e && last_phase_f) last_phase_b <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTc) begin
      last_phase_c <= 0;
    end else begin
      if (pexe_done_a && pexe_done_b) last_phase_c <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTd) begin
      last_phase_d <= 0;
    end else begin
      if (pexe_done_c && pexe_done_d) last_phase_d <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTe) begin
      last_phase_e <= 0;
    end else begin
      if (pexe_done_e && pexe_done_f) last_phase_e <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTf) begin
      last_phase_f <= 0;
    end else begin
      if (pexe_done_g && pexe_done_h) last_phase_f <= 1;
    end
  end

  // for phase
  // ###########################################################################
  always @(posedge CLK) begin
    if (RSTa) begin
      phase <= `LAST_PHASE;
    end else begin 
      if (elem==`SORT_ELM) phase <= phase+1;
    end
  end
  always @(posedge CLK) begin
    if (RSTa) begin
      phase_a <= 0;
    end else begin 
      if (elem_a==`SRTP_ELM) phase_a <= phase_a+1;
    end
  end
  always @(posedge CLK) begin
    if (RSTb) begin
      phase_b <= 0;
    end else begin 
      if (elem_b==`SRTP_ELM) phase_b <= phase_b+1;
    end
  end
  always @(posedge CLK) begin
    if (RSTc) begin
      phase_c <= 0;
    end else begin 
      if (elem_c==`SRTP_ELM) phase_c <= phase_c+1;
    end
  end
  always @(posedge CLK) begin
    if (RSTd) begin
      phase_d <= 0;
    end else begin 
      if (elem_d==`SRTP_ELM) phase_d <= phase_d+1;
    end
  end
  always @(posedge CLK) begin
    if (RSTe) begin
      phase_e <= 0;
    end else begin 
      if (elem_e==`SRTP_ELM) phase_e <= phase_e+1;
    end
  end
  always @(posedge CLK) begin
    if (RSTf) begin
      phase_f <= 0;
    end else begin 
      if (elem_f==`SRTP_ELM) phase_f <= phase_f+1;
    end
  end
  always @(posedge CLK) begin
    if (RSTg) begin
      phase_g <= 0;
    end else begin 
      if (elem_g==`SRTP_ELM) phase_g <= phase_g+1;
    end
  end
  always @(posedge CLK) begin
    if (RSTh) begin
      phase_h <= 0;
    end else begin 
      if (elem_h==`SRTP_ELM) phase_h <= phase_h+1;
    end
  end
  
  // for pexe_done
  // ###########################################################################
  always @(posedge CLK) begin
    if (RSTa) begin
      pexe_done_a <= 0;
    end else begin
      if (phase_a==`LAST_PHASE) pexe_done_a <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTb) begin
      pexe_done_b <= 0;
    end else begin
      if (phase_b==`LAST_PHASE) pexe_done_b <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTc) begin
      pexe_done_c <= 0;
    end else begin
      if (phase_c==`LAST_PHASE) pexe_done_c <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTd) begin
      pexe_done_d <= 0;
    end else begin
      if (phase_d==`LAST_PHASE) pexe_done_d <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTe) begin
      pexe_done_e <= 0;
    end else begin
      if (phase_e==`LAST_PHASE) pexe_done_e <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTf) begin
      pexe_done_f <= 0;
    end else begin
      if (phase_f==`LAST_PHASE) pexe_done_f <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTg) begin
      pexe_done_g <= 0;
    end else begin
      if (phase_g==`LAST_PHASE) pexe_done_g <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTh) begin
      pexe_done_h <= 0;
    end else begin
      if (phase_h==`LAST_PHASE) pexe_done_h <= 1;
    end
  end
  
  // for pexe_done_p
  // ###########################################################################
  always @(posedge CLK) begin
    if (RSTa) begin
      pexe_done_a_p <= 0;
    end else begin
      if (phase_a==`LAST_PHASE-1) pexe_done_a_p <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTb) begin
      pexe_done_b_p <= 0;
    end else begin
      if (phase_b==`LAST_PHASE-1) pexe_done_b_p <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTc) begin
      pexe_done_c_p <= 0;
    end else begin
      if (phase_c==`LAST_PHASE-1) pexe_done_c_p <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTd) begin
      pexe_done_d_p <= 0;
    end else begin
      if (phase_d==`LAST_PHASE-1) pexe_done_d_p <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTe) begin
      pexe_done_e_p <= 0;
    end else begin
      if (phase_e==`LAST_PHASE-1) pexe_done_e_p <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTf) begin
      pexe_done_f_p <= 0;
    end else begin
      if (phase_f==`LAST_PHASE-1) pexe_done_f_p <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTg) begin
      pexe_done_g_p <= 0;
    end else begin
      if (phase_g==`LAST_PHASE-1) pexe_done_g_p <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTh) begin
      pexe_done_h_p <= 0;
    end else begin
      if (phase_h==`LAST_PHASE-1) pexe_done_h_p <= 1;
    end
  end

  // for elem
  // ###########################################################################
  reg c_valid_a;  // check whether the data is compressed or not
  always @(posedge CLK) c_valid_a <= (OB_dot0[`DRAMW-1:`DRAMW-33] == {32'b0,1'b1});
  always @(posedge CLK) elem_en_a <= OB_dataen_a;
  
  always @(posedge CLK) begin
    if (RSTa) begin
      elem  <= 0;
      elem_a <= 0;
    end else begin
      case (last_phase)
        1'b0: begin
          case ({elem_en_a, (elem_a==`SRTP_ELM)})
            2'b01: elem_a <= 0;
            2'b10: elem_a <= mux32(elem_a+16, elem_a+32, c_valid_a);
          endcase
        end
        1'b1: begin
          case ({elem_en_a, (elem==`SORT_ELM)})
            2'b01: elem <= 0;
            2'b10: elem <= mux32(elem+16, elem+32, c_valid_a);
          endcase
        end
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RSTa) begin
      elem_way_a <= 0;
    end else begin
      case ({elem_en_a, (elem_way_a==(`SORT_ELM>>(`P_LOG+`WAY_LOG)))})
        2'b01: elem_way_a <= 0;
        2'b10: elem_way_a <= mux32(elem_way_a+16, elem_way_a+32, c_valid_a);
      endcase
    end
  end

  reg c_valid_b;  // check whether the data is compressed or not
  always @(posedge CLK) c_valid_b <= (OB_dot1[`DRAMW-1:`DRAMW-33] == {32'b0,1'b1});
  always @(posedge CLK) elem_en_b <= OB_dataen_b;
  
  always @(posedge CLK) begin
    if (RSTb) begin
      elem_b <= 0;
    end else begin
      case ({elem_en_b, (elem_b==`SRTP_ELM)})
        2'b01: elem_b <= 0;
        2'b10: elem_b <= mux32(elem_b+16, elem_b+32, c_valid_b);
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RSTb) begin
      elem_way_b <= 0;
    end else begin
      case ({elem_en_b, (elem_way_b==(`SORT_ELM>>(`P_LOG+`WAY_LOG)))})
        2'b01: elem_way_b <= 0;
        2'b10: elem_way_b <= mux32(elem_way_b+16, elem_way_b+32, c_valid_b);
      endcase
    end
  end

  reg c_valid_c;  // check whether the data is compressed or not
  always @(posedge CLK) c_valid_c <= (OB_dot2[`DRAMW-1:`DRAMW-33] == {32'b0,1'b1});
  always @(posedge CLK) elem_en_c <= OB_dataen_c;
  
  always @(posedge CLK) begin
    if (RSTc) begin
      elem_c <= 0;
    end else begin
      case ({elem_en_c, (elem_c==`SRTP_ELM)})
        2'b01: elem_c <= 0;
        2'b10: elem_c <= mux32(elem_c+16, elem_c+32, c_valid_c);
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RSTc) begin
      elem_way_c <= 0;
    end else begin
      case ({elem_en_c, (elem_way_c==(`SORT_ELM>>(`P_LOG+`WAY_LOG)))})
        2'b01: elem_way_c <= 0;
        2'b10: elem_way_c <= mux32(elem_way_c+16, elem_way_c+32, c_valid_c);
      endcase
    end
  end

  reg c_valid_d;  // check whether the data is compressed or not
  always @(posedge CLK) c_valid_d <= (OB_dot3[`DRAMW-1:`DRAMW-33] == {32'b0,1'b1});
  always @(posedge CLK) elem_en_d <= OB_dataen_d;

  always @(posedge CLK) begin
    if (RSTd) begin
      elem_d <= 0;
    end else begin
      case ({elem_en_d, (elem_d==`SRTP_ELM)})
        2'b01: elem_d <= 0;
        2'b10: elem_d <= mux32(elem_d+16, elem_d+32, c_valid_d);
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RSTd) begin
      elem_way_d <= 0;
    end else begin
      case ({elem_en_d, (elem_way_d==(`SORT_ELM>>(`P_LOG+`WAY_LOG)))})
        2'b01: elem_way_d <= 0;
        2'b10: elem_way_d <= mux32(elem_way_d+16, elem_way_d+32, c_valid_d);
      endcase
    end
  end

  reg c_valid_e;  // check whether the data is compressed or not
  always @(posedge CLK) c_valid_e <= (OB_dot4[`DRAMW-1:`DRAMW-33] == {32'b0,1'b1});
  always @(posedge CLK) elem_en_e <= OB_dataen_e;
  
  always @(posedge CLK) begin
    if (RSTe) begin
      elem_e <= 0;
    end else begin
      case ({elem_en_e, (elem_e==`SRTP_ELM)})
        2'b01: elem_e <= 0;
        2'b10: elem_e <= mux32(elem_e+16, elem_e+32, c_valid_e);
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RSTe) begin
      elem_way_e <= 0;
    end else begin
      case ({elem_en_e, (elem_way_e==(`SORT_ELM>>(`P_LOG+`WAY_LOG)))})
        2'b01: elem_way_e <= 0;
        2'b10: elem_way_e <= mux32(elem_way_e+16, elem_way_e+32, c_valid_e);
      endcase
    end
  end

  reg c_valid_f;  // check whether the data is compressed or not
  always @(posedge CLK) c_valid_f <= (OB_dot5[`DRAMW-1:`DRAMW-33] == {32'b0,1'b1});
  always @(posedge CLK) elem_en_f <= OB_dataen_f;
  
  always @(posedge CLK) begin
    if (RSTf) begin
      elem_f <= 0;
    end else begin
      case ({elem_en_f, (elem_f==`SRTP_ELM)})
        2'b01: elem_f <= 0;
        2'b10: elem_f <= mux32(elem_f+16, elem_f+32, c_valid_f);
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RSTf) begin
      elem_way_f <= 0;
    end else begin
      case ({elem_en_f, (elem_way_f==(`SORT_ELM>>(`P_LOG+`WAY_LOG)))})
        2'b01: elem_way_f <= 0;
        2'b10: elem_way_f <= mux32(elem_way_f+16, elem_way_f+32, c_valid_f);
      endcase
    end
  end

  reg c_valid_g;  // check whether the data is compressed or not
  always @(posedge CLK) c_valid_g <= (OB_dot6[`DRAMW-1:`DRAMW-33] == {32'b0,1'b1});
  always @(posedge CLK) elem_en_g <= OB_dataen_g;

  always @(posedge CLK) begin
    if (RSTg) begin
      elem_g <= 0;
    end else begin
      case ({elem_en_g, (elem_g==`SRTP_ELM)})
        2'b01: elem_g <= 0;
        2'b10: elem_g <= mux32(elem_g+16, elem_g+32, c_valid_g);
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RSTg) begin
      elem_way_g <= 0;
    end else begin
      case ({elem_en_g, (elem_way_g==(`SORT_ELM>>(`P_LOG+`WAY_LOG)))})
        2'b01: elem_way_g <= 0;
        2'b10: elem_way_g <= mux32(elem_way_g+16, elem_way_g+32, c_valid_g);
      endcase
    end
  end

  reg c_valid_h;  // check whether the data is compressed or not
  always @(posedge CLK) c_valid_h <= (OB_dot7[`DRAMW-1:`DRAMW-33] == {32'b0,1'b1});
  always @(posedge CLK) elem_en_h <= OB_dataen_h;

  always @(posedge CLK) begin
    if (RSTh) begin
      elem_h <= 0;
    end else begin
      case ({elem_en_h, (elem_h==`SRTP_ELM)})
        2'b01: elem_h <= 0;
        2'b10: elem_h <= mux32(elem_h+16, elem_h+32, c_valid_h);
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RSTh) begin
      elem_way_h <= 0;
    end else begin
      case ({elem_en_h, (elem_way_h==(`SORT_ELM>>(`P_LOG+`WAY_LOG)))})
        2'b01: elem_way_h <= 0;
        2'b10: elem_way_h <= mux32(elem_way_h+16, elem_way_h+32, c_valid_h);
      endcase
    end
  end
  
  // for iter_done
  // ###########################################################################
  always @(posedge CLK) iter_done_a <= (ecnt_a==2);
  always @(posedge CLK) iter_done_b <= (ecnt_b==2);
  always @(posedge CLK) iter_done_c <= (ecnt_c==2);
  always @(posedge CLK) iter_done_d <= (ecnt_d==2);
  always @(posedge CLK) iter_done_e <= (ecnt_e==2);
  always @(posedge CLK) iter_done_f <= (ecnt_f==2);
  always @(posedge CLK) iter_done_g <= (ecnt_g==2);
  always @(posedge CLK) iter_done_h <= (ecnt_h==2);
  
  // for pchange
  // ###########################################################################
  always @(posedge CLK) pchange_a <= (elem_a==`SRTP_ELM);
  always @(posedge CLK) pchange_b <= (elem_b==`SRTP_ELM);
  always @(posedge CLK) pchange_c <= (elem_c==`SRTP_ELM);
  always @(posedge CLK) pchange_d <= (elem_d==`SRTP_ELM);
  always @(posedge CLK) pchange_e <= (elem_e==`SRTP_ELM);
  always @(posedge CLK) pchange_f <= (elem_f==`SRTP_ELM);
  always @(posedge CLK) pchange_g <= (elem_g==`SRTP_ELM);
  always @(posedge CLK) pchange_h <= (elem_h==`SRTP_ELM);
  
  // for irst
  // ###########################################################################
  always @(posedge CLK) irst_a <= mux1(((ecnt_a==2) || pchange_a), (ecnt==2), last_phase);
  always @(posedge CLK) irst_b <= (ecnt_b==2) || pchange_b;
  always @(posedge CLK) irst_c <= (ecnt_c==2) || pchange_c;
  always @(posedge CLK) irst_d <= (ecnt_d==2) || pchange_d;
  always @(posedge CLK) irst_e <= (ecnt_e==2) || pchange_e;
  always @(posedge CLK) irst_f <= (ecnt_f==2) || pchange_f;
  always @(posedge CLK) irst_g <= (ecnt_g==2) || pchange_g;
  always @(posedge CLK) irst_h <= (ecnt_h==2) || pchange_h;
  
  // for frst
  // ###########################################################################
  always @(posedge CLK) frst_a <= mux1((RSTa || (ecnt_a==2) || (elem_a==`SRTP_ELM)), (ecnt==2), last_phase);
  always @(posedge CLK) frst_b <= RSTb || (ecnt_b==2) || (elem_b==`SRTP_ELM);
  always @(posedge CLK) frst_c <= RSTc || (ecnt_c==2) || (elem_c==`SRTP_ELM);
  always @(posedge CLK) frst_d <= RSTd || (ecnt_d==2) || (elem_d==`SRTP_ELM);
  always @(posedge CLK) frst_e <= RSTe || (ecnt_e==2) || (elem_e==`SRTP_ELM);
  always @(posedge CLK) frst_f <= RSTf || (ecnt_f==2) || (elem_f==`SRTP_ELM);
  always @(posedge CLK) frst_g <= RSTg || (ecnt_g==2) || (elem_g==`SRTP_ELM);
  always @(posedge CLK) frst_h <= RSTh || (ecnt_h==2) || (elem_h==`SRTP_ELM);
  
  // for ecnt
  // ###########################################################################
  always @(posedge CLK) begin
    if (RSTa) begin
      ecnt <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase * `WAY_LOG));
    end else begin
      if (ecnt!=0 && F01_deq0 && last_phase) ecnt <= ecnt - 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTa || iter_done_a || pchange_a) begin
      ecnt_a <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase_a * `WAY_LOG));
    end else begin
      if (ecnt_a!=0 && F01_deq0 && !pexe_done_a) ecnt_a <= ecnt_a - 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTb || iter_done_b || pchange_b) begin
      ecnt_b <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase_b * `WAY_LOG));
    end else begin
      if (ecnt_b!=0 && F01_deq1 && !pexe_done_b) ecnt_b <= ecnt_b - 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTc || iter_done_c || pchange_c) begin
      ecnt_c <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase_c * `WAY_LOG));
    end else begin
      if (ecnt_c!=0 && F01_deq2 && !pexe_done_c) ecnt_c <= ecnt_c - 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTd || iter_done_d || pchange_d) begin
      ecnt_d <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase_d * `WAY_LOG));
    end else begin
      if (ecnt_d!=0 && F01_deq3 && !pexe_done_d) ecnt_d <= ecnt_d - 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTe || iter_done_e || pchange_e) begin
      ecnt_e <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase_e * `WAY_LOG));
    end else begin
      if (ecnt_e!=0 && F01_deq4 && !pexe_done_e) ecnt_e <= ecnt_e - 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTf || iter_done_f || pchange_f) begin
      ecnt_f <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase_f * `WAY_LOG));
    end else begin
      if (ecnt_f!=0 && F01_deq5 && !pexe_done_f) ecnt_f <= ecnt_f - 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTg || iter_done_g || pchange_g) begin
      ecnt_g <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase_g * `WAY_LOG));
    end else begin
      if (ecnt_g!=0 && F01_deq6 && !pexe_done_g) ecnt_g <= ecnt_g - 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTh || iter_done_h || pchange_h) begin
      ecnt_h <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase_h * `WAY_LOG));
    end else begin
      if (ecnt_h!=0 && F01_deq7 && !pexe_done_h) ecnt_h <= ecnt_h - 1;
    end
  end

  // for sortdone
  // ###########################################################################
  always @(posedge CLK) begin
    if (RSTa) begin
      sortdone <= 0;
    end else begin
      if (phase==(`LAST_PHASE+1)) sortdone <= 1;
    end
  end
  
endmodule 
/**************************************************************************************************/

`default_nettype wire
