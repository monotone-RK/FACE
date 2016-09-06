/******************************************************************************/
/* FPGA Sort on VC707                                        Ryohei Kobayashi */
/*                                                                 2016-08-01 */
/******************************************************************************/
`default_nettype none

`include "define.vh"

/***** Comparator                                                                             *****/
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

/***** Sorter cell emitting multiple values at once                                           *****/
/**************************************************************************************************/
module SCELL #(parameter                        SORTW = 32,
               parameter                        M_LOG = 2)
              (input  wire                      CLK,
               input  wire                      RST,
               input  wire                      valid1, 
               input  wire                      valid2, 
               output wire                      deq1, 
               output wire                      deq2, 
               input  wire [(SORTW<<M_LOG)-1:0] din1, 
               input  wire [(SORTW<<M_LOG)-1:0] din2, 
               input  wire                      full, 
               output wire [(SORTW<<M_LOG)-1:0] dout, 
               output wire                      enq);
    
  function [(SORTW<<M_LOG)-1:0] mux;
    input [(SORTW<<M_LOG)-1:0] a;
    input [(SORTW<<M_LOG)-1:0] b;
    input                      sel;
    begin
      case (sel)
        1'b0: mux = a;
        1'b1: mux = b;
      endcase
    end
  endfunction

  wire                      cmp      = (din1[SORTW-1:0] < din2[SORTW-1:0]);
  wire [(SORTW<<M_LOG)-1:0] cmp_dout = mux(din2, din1, cmp);
  
  wire                      F_enq;
  wire                      F_deq; 
  wire                      F_emp; 
  wire                      F_full; 
  wire [(SORTW<<M_LOG)-1:0] F_dot;
  MRE2 #(1,(SORTW<<M_LOG)) F(.CLK(CLK), .RST(RST), .enq(F_enq), .deq(F_deq), 
                             .din(cmp_dout), .dot(F_dot), .emp(F_emp), .full(F_full));
  assign F_enq = &{~F_full,valid1,valid2};  // assign F_enq = (!F_full && valid1 && valid2);
  assign F_deq = ~|{full,F_emp};  // assign F_deq = !full && !F_emp;
  
  reg [(SORTW<<M_LOG)-1:0] fbdata;
  reg [(SORTW<<M_LOG)-1:0] fbdata_a;  // duplicated register
  reg [(SORTW<<M_LOG)-1:0] fbdata_b;  // duplicated register
  reg                      fbinvoke;

  assign enq  = (F_deq && fbinvoke);
  assign deq1 = (F_enq &&  cmp);
  assign deq2 = (F_enq && !cmp);
  
  localparam P_DATAWIDTH = 32;
  wire [P_DATAWIDTH-1:0] a, b, c, d, e, f, g, h;
  wire [P_DATAWIDTH-1:0] e_a, f_a, g_a, h_a;  // for duplicated register
  wire [P_DATAWIDTH-1:0] e_b, f_b, g_b, h_b;  // for duplicated register
  assign a = F_dot[ 31: 0];
  assign b = F_dot[ 63:32];
  assign c = F_dot[ 95:64];
  assign d = F_dot[127:96];
  assign e = fbdata[ 31: 0];
  assign f = fbdata[ 63:32];
  assign g = fbdata[ 95:64];
  assign h = fbdata[127:96];
  assign e_a = fbdata_a[ 31: 0];
  assign f_a = fbdata_a[ 63:32];
  assign g_a = fbdata_a[ 95:64];
  assign h_a = fbdata_a[127:96];
  assign e_b = fbdata_b[ 31: 0];
  assign f_b = fbdata_b[ 63:32];
  assign g_b = fbdata_b[ 95:64];
  assign h_b = fbdata_b[127:96];

  wire t0_c0 = (a < h);
  wire t0_c1 = (b < g);
  wire t0_c2 = (c < f);
  wire t0_c3 = (d < e);

  wire t0_x0 = t0_c0 ^ t0_c1;
  wire t0_x1 = t0_c2 ^ t0_c3;

  wire t0 = t0_x0 ^ t0_x1;
  
  wire s2_c0 = (b < e);
  wire s2_c1 = (a < f);

  wire s3_c0 = (c < h);
  wire s3_c1 = (d < g);

  wire s4_c0 = (a < g);
  wire s4_c1 = (b < f);
  wire s4_c2 = (c < e);

  wire s5_c0 = (d < f);
  wire s5_c1 = (c < g);
  wire s5_c2 = (b < h);

  wire s0 = (a < e);
  wire s1 = (d < h);
  wire [1:0] s2 = {s0, (s2_c0 ^ s2_c1)};
  wire [1:0] s3 = {s1, (s3_c0 ^ s3_c1)};
  wire [2:0] s4 = {s2, (s4_c0 ^ s4_c1 ^ s4_c2)};
  wire [2:0] s5 = {s3, (s5_c0 ^ s5_c1 ^ s5_c2)};
  wire [3:0] s6 = {s4, t0};
  wire [3:0] s7 = {s5, t0};

  wire [P_DATAWIDTH-1:0] m0, m1, m2, m3, m4, m5, m6, m7;

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
  
  function [32-1:0] mux4in32;
    input [32-1:0] a;
    input [32-1:0] b;
    input [32-1:0] c;
    input [32-1:0] d;
    input [1:0]    sel;
    begin
      case (sel)
        2'b00: mux4in32 = a;
        2'b01: mux4in32 = b;
        2'b10: mux4in32 = c;
        2'b11: mux4in32 = d;
      endcase
    end
  endfunction
  
  function [32-1:0] mux6in32;
    input [32-1:0] a;
    input [32-1:0] b;
    input [32-1:0] c;
    input [32-1:0] d;
    input [32-1:0] e;
    input [32-1:0] f;
    input [2:0]    sel;
    begin
      casex (sel)
        3'b000: mux6in32 = a;
        3'b001: mux6in32 = b;
        3'b100: mux6in32 = c;
        3'b101: mux6in32 = d;
        3'bx10: mux6in32 = e;
        3'bx11: mux6in32 = f;
      endcase
    end
  endfunction
  
  function [32-1:0] mux12in32;
    input [32-1:0] a;
    input [32-1:0] b;
    input [32-1:0] c;
    input [32-1:0] d;
    input [32-1:0] e;
    input [32-1:0] f;
    input [32-1:0] g;
    input [32-1:0] h;
    input [32-1:0] i;
    input [32-1:0] j;
    input [32-1:0] k;
    input [32-1:0] l;
    input [3:0]    sel;
    begin
      casex (sel)
        4'b0000: mux12in32 = a;
        4'b0001: mux12in32 = b;
        4'b0010: mux12in32 = c;
        4'b0011: mux12in32 = d;
        4'b1000: mux12in32 = e;
        4'b1001: mux12in32 = f;
        4'b1010: mux12in32 = g;
        4'b1011: mux12in32 = h;
        4'bx100: mux12in32 = i;
        4'bx101: mux12in32 = j;
        4'bx110: mux12in32 = k;
        4'bx111: mux12in32 = l;
      endcase
    end
  endfunction
  
  assign m0 = mux32(e, a, s0);
  assign m1 = mux32(d, h, s1);
  assign m2 = mux4in32(f, a, b, e, s2);
  assign m3 = mux4in32(c, h, g, d, s3);
  assign m4 = mux6in32(g, a, e, c, b, f, s4);
  assign m5 = mux6in32(b, h, d, f, g, c, s5);
  // using duplicated registers
  assign m6 = mux12in32(h_a, a, b, g_a, f_a, c, d, e_a, f_a, c, b, g_a, s6);
  assign m7 = mux12in32(a, h_b, g_b, b, c, f_b, e_b, d, c, f_b, g_b, b, s7);

  // output and feedback
  //////////////////////////////////////////////////////////
  assign dout = {m6,m4,m2,m0};  // output
  always @(posedge CLK) begin   // feedback
    if (RST) begin
      fbdata   <= 0;
      fbdata_a <= 0;
      fbdata_b <= 0;
      fbinvoke <= 0;
    end else begin
      if (F_deq) begin
        fbdata   <= {m1,m3,m5,m7};
        fbdata_a <= {m1,m3,m5,m7};
        fbdata_b <= {m1,m3,m5,m7};
        fbinvoke <= 1;
      end
    end
  end
  
endmodule


/***** general FIFO (BRAM Version)                                                            *****/
/**************************************************************************************************/
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
              input  wire [`DRAMW-1:0] din,     // input data 
              input  wire              den,     // input data enable
              input  wire              IB_full, // the next module is full ?
              output wire              rx_wait,
              output wire [`MERGW-1:0] dot,     // this module's data output
              output wire              IB_enq,  // the next module's enqueue signal
              output reg [1:0]         im_req); // DRAM data request
  
  wire req;
  reg  deq;
  wire [`DRAMW-1:0] im_dot;
  (* mark_debug = "true" *) wire [`IB_SIZE:0] im_cnt;
  wire im_full, im_emp;
  wire im_enq = den; 
  wire im_deq = (req && !im_emp);
  
  assign rx_wait = im_cnt[`IB_SIZE-1];
  
  always @(posedge CLK) im_req <= (im_cnt==0) ? 3 : (im_cnt<`REQ_THRE);
  always @(posedge CLK) deq <= im_deq;
  
  BFIFO #(`IB_SIZE, `DRAMW) // note, using BRAM
  imf(.CLK(CLK), .RST(RST), .enq(im_enq), .deq(im_deq), .din(din),
      .dot(im_dot), .emp(im_emp), .full(im_full), .cnt(im_cnt));
  
  INMOD inmod(.CLK(CLK), .RST(RST), .d_dout(im_dot), .d_douten(deq), 
              .IB_full(IB_full), .im_dot(dot), .IB_enq(IB_enq), .im_req(req));
endmodule

/***** Input Module                                                                           *****/
/**************************************************************************************************/ // todo
module INMOD(input  wire              CLK, 
             input  wire              RST, 
             input  wire [`DRAMW-1:0] d_dout,    // DRAM output
             input  wire              d_douten,  // DRAM output enable
             input  wire              IB_full,   // INBUF is full ? 
             output wire [`MERGW-1:0] im_dot,    // this module's data output
             output wire              IB_enq, 
             output wire              im_req);   // DRAM data request
  
  reg [`DRAMW-1:0] dot_t; // shift register to feed 32bit data
  reg  [1:0] cnte;        // the number of enqueued elements in one block
  reg cntez;              // cnte==0  ?
  reg cntef;              // cnte==15 ?
  
  wire [`DRAMW-1:0] dot;
  wire im_emp, im_full;
  wire im_enq = d_douten; // (!im_full && d_douten); 
  wire im_deq = (IB_enq && cntef); // old version may have a bug here!!

  function [`MERGW-1:0] mux;
    input [`MERGW-1:0] a;
    input [`MERGW-1:0] b;
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
  assign im_dot = mux(dot_t[`MERGW-1:0], dot[`MERGW-1:0], cntez);

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
      case ({IB_enq, (cnte==3)})
        2'b10: cntez <= 0;
        2'b11: cntez <= 1;
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RST) begin
      cntef <= 0;
    end else begin
      case ({IB_enq, (cnte==2)})
        2'b10: cntef <= 0;
        2'b11: cntef <= 1;
      endcase
    end
  end
  always @(posedge CLK) begin
    case ({IB_enq, cntez})
      2'b10: dot_t <= {`MERGW'b0, dot_t[`DRAMW-1:`MERGW]};
      2'b11: dot_t <= {`MERGW'b0, dot[`DRAMW-1:`MERGW]};
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
             input  wire [`MERGW-1:0] din,      // data in
             output wire [`MERGW-1:0] dot,      // data out
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
  
  function [`MERGW-1:0] mux128;
    input [`MERGW-1:0] a;
    input [`MERGW-1:0] b;
    input              sel;
    begin
      case (sel)
        1'b0: mux128 = a;
        1'b1: mux128 = b;
      endcase
    end
  endfunction
  
  /*****************************************/
  wire [`MERGW-1:0]  F_dout;
  wire        F_deq, F_emp;
  reg [31:0] ecnt;  // the number of elements in one iteration
  reg        ecntz; // ecnt==0 ?
  
  wire f_full;
  MRE2 #(1,`MERGW) F(.CLK(CLK), .RST(RST), .enq(ib_enq), .deq(F_deq),  // input buffer FIFO
                     .din(din), .dot(F_dout), .emp(F_emp), .full(f_full));

  assign ib_full = mux1(f_full, 0, F_deq); // INBUF back_pressure
  /*****************************************/
  assign enq = !full && (!F_emp || ecntz);  // enqueue for the next buffer
  assign F_deq = enq && (ecnt!=0);          //
  
  assign dot = mux128(F_dout, `MAX_VALUE, ecntz);
  
  always @(posedge CLK) begin
    if (RST || idone) begin
      ecnt  <= (`ELEMS_PER_UNIT << (phase * `WAY_LOG)); /// note
      ecntz <= 0;
    end else begin
      if (ecnt!=0 && enq) ecnt  <= ecnt - 4;
      if (ecnt==4 && enq) ecntz <= 1; // old version has a bug here!
    end
  end
endmodule

/**************************************************************************************************/
module STREE(input  wire                        CLK, 
             input  wire                        RST_in, 
             input  wire                        irst, 
             input  wire                        frst, 
             input  wire [`PHASE_W]             phase_in, 
             input  wire [`MERGW*`SORT_WAY-1:0] s_din,     // sorting-tree input data
             input  wire [`SORT_WAY-1:0]        enq,       // enqueue
             output wire [`SORT_WAY-1:0]        full,      // buffer is full ? 
             input  wire                        deq,       // dequeue
             output wire [`MERGW-1:0]           dot,       // output data 
             output wire                        emp);

  reg RST;
  always @(posedge CLK) RST <= RST_in;
  
  reg [`PHASE_W] phase;
  always @(posedge CLK) phase <= phase_in;
  
  wire [`MERGW-1:0] d00, d01, d02, d03, d04, d05, d06, d07;
  assign {d00, d01, d02, d03, d04, d05, d06, d07} = s_din;
  
  wire F01_enq, F01_deq, F01_emp, F01_full; wire [`MERGW-1:0] F01_din, F01_dot; wire [1:0] F01_cnt;
  wire F02_enq, F02_deq, F02_emp, F02_full; wire [`MERGW-1:0] F02_din, F02_dot; wire [1:0] F02_cnt;
  wire F03_enq, F03_deq, F03_emp, F03_full; wire [`MERGW-1:0] F03_din, F03_dot; wire [1:0] F03_cnt;
  wire F04_enq, F04_deq, F04_emp, F04_full; wire [`MERGW-1:0] F04_din, F04_dot; wire [1:0] F04_cnt;
  wire F05_enq, F05_deq, F05_emp, F05_full; wire [`MERGW-1:0] F05_din, F05_dot; wire [1:0] F05_cnt;
  wire F06_enq, F06_deq, F06_emp, F06_full; wire [`MERGW-1:0] F06_din, F06_dot; wire [1:0] F06_cnt;
  wire F07_enq, F07_deq, F07_emp, F07_full; wire [`MERGW-1:0] F07_din, F07_dot; wire [1:0] F07_cnt;
  wire F08_enq, F08_deq, F08_emp, F08_full; wire [`MERGW-1:0] F08_din, F08_dot; wire [1:0] F08_cnt;
  wire F09_enq, F09_deq, F09_emp, F09_full; wire [`MERGW-1:0] F09_din, F09_dot; wire [1:0] F09_cnt;
  wire F10_enq, F10_deq, F10_emp, F10_full; wire [`MERGW-1:0] F10_din, F10_dot; wire [1:0] F10_cnt;
  wire F11_enq, F11_deq, F11_emp, F11_full; wire [`MERGW-1:0] F11_din, F11_dot; wire [1:0] F11_cnt;
  wire F12_enq, F12_deq, F12_emp, F12_full; wire [`MERGW-1:0] F12_din, F12_dot; wire [1:0] F12_cnt;
  wire F13_enq, F13_deq, F13_emp, F13_full; wire [`MERGW-1:0] F13_din, F13_dot; wire [1:0] F13_cnt;
  wire F14_enq, F14_deq, F14_emp, F14_full; wire [`MERGW-1:0] F14_din, F14_dot; wire [1:0] F14_cnt;
  wire F15_enq, F15_deq, F15_emp, F15_full; wire [`MERGW-1:0] F15_din, F15_dot; wire [1:0] F15_cnt;

  INBUF IN08(CLK, RST, full[0], F08_full, F08_enq, d00, F08_din, enq[0], phase, irst);
  INBUF IN09(CLK, RST, full[1], F09_full, F09_enq, d01, F09_din, enq[1], phase, irst);
  INBUF IN10(CLK, RST, full[2], F10_full, F10_enq, d02, F10_din, enq[2], phase, irst);
  INBUF IN11(CLK, RST, full[3], F11_full, F11_enq, d03, F11_din, enq[3], phase, irst);
  INBUF IN12(CLK, RST, full[4], F12_full, F12_enq, d04, F12_din, enq[4], phase, irst);
  INBUF IN13(CLK, RST, full[5], F13_full, F13_enq, d05, F13_din, enq[5], phase, irst);
  INBUF IN14(CLK, RST, full[6], F14_full, F14_enq, d06, F14_din, enq[6], phase, irst);
  INBUF IN15(CLK, RST, full[7], F15_full, F15_enq, d07, F15_din, enq[7], phase, irst);

  MRE2 #(1, `MERGW) F01(CLK, frst, F01_enq, F01_deq, F01_din, F01_dot, F01_emp, F01_full, F01_cnt);
  MRE2 #(1, `MERGW) F02(CLK, frst, F02_enq, F02_deq, F02_din, F02_dot, F02_emp, F02_full, F02_cnt);
  MRE2 #(1, `MERGW) F03(CLK, frst, F03_enq, F03_deq, F03_din, F03_dot, F03_emp, F03_full, F03_cnt);
  MRE2 #(1, `MERGW) F04(CLK, frst, F04_enq, F04_deq, F04_din, F04_dot, F04_emp, F04_full, F04_cnt);
  MRE2 #(1, `MERGW) F05(CLK, frst, F05_enq, F05_deq, F05_din, F05_dot, F05_emp, F05_full, F05_cnt);
  MRE2 #(1, `MERGW) F06(CLK, frst, F06_enq, F06_deq, F06_din, F06_dot, F06_emp, F06_full, F06_cnt);
  MRE2 #(1, `MERGW) F07(CLK, frst, F07_enq, F07_deq, F07_din, F07_dot, F07_emp, F07_full, F07_cnt);
  MRE2 #(1, `MERGW) F08(CLK, frst, F08_enq, F08_deq, F08_din, F08_dot, F08_emp, F08_full, F08_cnt);
  MRE2 #(1, `MERGW) F09(CLK, frst, F09_enq, F09_deq, F09_din, F09_dot, F09_emp, F09_full, F09_cnt);
  MRE2 #(1, `MERGW) F10(CLK, frst, F10_enq, F10_deq, F10_din, F10_dot, F10_emp, F10_full, F10_cnt);
  MRE2 #(1, `MERGW) F11(CLK, frst, F11_enq, F11_deq, F11_din, F11_dot, F11_emp, F11_full, F11_cnt);
  MRE2 #(1, `MERGW) F12(CLK, frst, F12_enq, F12_deq, F12_din, F12_dot, F12_emp, F12_full, F12_cnt);
  MRE2 #(1, `MERGW) F13(CLK, frst, F13_enq, F13_deq, F13_din, F13_dot, F13_emp, F13_full, F13_cnt);
  MRE2 #(1, `MERGW) F14(CLK, frst, F14_enq, F14_deq, F14_din, F14_dot, F14_emp, F14_full, F14_cnt);
  MRE2 #(1, `MERGW) F15(CLK, frst, F15_enq, F15_deq, F15_din, F15_dot, F15_emp, F15_full, F15_cnt);

  SCELL #(`SORTW, `M_LOG) S01(CLK, frst, !F02_emp, !F03_emp, F02_deq, F03_deq, F02_dot, F03_dot, F01_full, F01_din, F01_enq);
  SCELL #(`SORTW, `M_LOG) S02(CLK, frst, !F04_emp, !F05_emp, F04_deq, F05_deq, F04_dot, F05_dot, F02_full, F02_din, F02_enq);
  SCELL #(`SORTW, `M_LOG) S03(CLK, frst, !F06_emp, !F07_emp, F06_deq, F07_deq, F06_dot, F07_dot, F03_full, F03_din, F03_enq);
  SCELL #(`SORTW, `M_LOG) S04(CLK, frst, !F08_emp, !F09_emp, F08_deq, F09_deq, F08_dot, F09_dot, F04_full, F04_din, F04_enq);
  SCELL #(`SORTW, `M_LOG) S05(CLK, frst, !F10_emp, !F11_emp, F10_deq, F11_deq, F10_dot, F11_dot, F05_full, F05_din, F05_enq);
  SCELL #(`SORTW, `M_LOG) S06(CLK, frst, !F12_emp, !F13_emp, F12_deq, F13_deq, F12_dot, F13_dot, F06_full, F06_din, F06_enq);
  SCELL #(`SORTW, `M_LOG) S07(CLK, frst, !F14_emp, !F15_emp, F14_deq, F15_deq, F14_dot, F15_dot, F07_full, F07_din, F07_enq);
  
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
                  input  wire              P_Z,
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
  always @(posedge CLK) c_cflag <= (delta_a<=13'h1fff) && (delta_b<=13'h1fff) && (delta_c<=13'h1fff) && (delta_d<=13'h1fff) &&
                                   (delta_e<=13'h1fff) && (delta_f<=13'h1fff) && (delta_g<=13'h1fff) && (delta_h<=13'h1fff) &&
                                   (delta_i<=13'h1fff) && (delta_j<=13'h1fff) && (delta_k<=13'h1fff) && (delta_l<=13'h1fff) &&
                                   (delta_m<=13'h1fff) && (delta_n<=13'h1fff) && (delta_o<=13'h1fff) && !P_Z;
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
module DECOMPRESSOR #(parameter                          SIZE   = 7, 
                      parameter                          BLOCKS = 8)
                     (input  wire                        CLK,
                      input  wire                        RST,
                      input  wire [`SRTP_WAY+`DRAMW-1:0] DIN,
                      input  wire                        DIN_EN,
                      output reg  [`DRAMW-1:0]           DOUT,
                      output reg  [`SRTP_WAY:0]          COUT,
                      output reg                         DATA_REQ);

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
  wire                        dmft_full;
  
  wire                        dmf_emp;
  wire                        dmf_enq = DIN_EN;
  wire                        dmf_deq = !dmf_emp && !dmft_full;
  wire [`SRTP_WAY+`DRAMW-1:0] dmf_din = DIN;
  wire                        dmf_full;
  wire [`SRTP_WAY+`DRAMW-1:0] dmf_dout;
  wire [SIZE:0]               dmf_cnt;
  BFIFO #(SIZE, `SRTP_WAY+`DRAMW) dmf(CLK, RST, dmf_enq, dmf_deq, dmf_din, dmf_dout, dmf_emp, dmf_full, dmf_cnt);
  
  reg dmf_dataen;
  always @(posedge CLK) dmf_dataen <= dmf_deq;
  
  always @(posedge CLK) DATA_REQ <= (dmf_cnt <= (1<<SIZE)-BLOCKS);
  
  // FIFO (two entries) ///////////////////////////////////////////////////
  wire                        dmft_enq = dmf_dataen;
  wire [`SRTP_WAY+`DRAMW-1:0] dmft_din = dmf_dout;
  wire [`SRTP_WAY+`DRAMW-1:0] dmft_dout;
  wire                        dmft_emp;
  wire                        c_valid = (dmft_dout[`DRAMW-1:`DRAMW-33]=={32'b0,1'b1});  // check whether the data is compressed or not
  reg                         c_sel;
  wire                        dmft_deq = c_sel || (!c_valid && !dmft_emp);
  MRE2 #(1, `SRTP_WAY+`DRAMW) dmft(.CLK(CLK), .RST(RST), .enq(dmft_enq), .deq(dmft_deq), 
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
  reg [(`SRTP_WAY+1+1)-1:0] pcA;  // pipeline regester A for control
  always @(posedge CLK) pcA <= {dmft_dout[`SRTP_WAY+`DRAMW-1:`DRAMW], (!dmft_emp), c_valid};
  
  // Stage B
  //////////////////////////////////////////////////////////////////////////////
  wire [`SORTW-1:0] b15,b14,b13,b12,b11,b10,b09,b08,b07,b06,b05,b04,b03,b02,b01,b00; // input
  assign {b15,b14,b13,b12,b11,b10,b09,b08,b07,b06,b05,b04,b03,b02,b01,b00} = pdA;

  reg [511:0] pdB; // pipeline regester B for data
  always @(posedge CLK) pdB <= {b15,b14,b13,b12,b11,b10,b09,b08,b07,(b06+b05+b04+b03),(b05+b04+b03),(b04+b03),b03,b02,b01,b00};
  reg [`DRAMW-1:0] dmft_dout_B;
  always @(posedge CLK) dmft_dout_B <= dmft_dout_A;
  reg [(`SRTP_WAY+1+1)-1:0] pcB;  // pipeline regester B for control
  always @(posedge CLK) pcB <= pcA;

  // Stage C
  //////////////////////////////////////////////////////////////////////////////
  wire [`SORTW-1:0] c15,c14,c13,c12,c11,c10,c09,c08,c07,c06,c05,c04,c03,c02,c01,c00; // input
  assign {c15,c14,c13,c12,c11,c10,c09,c08,c07,c06,c05,c04,c03,c02,c01,c00} = pdB;

  reg [511:0] pdC; // pipeline regester C for data
  always @(posedge CLK) pdC <= {c15,c14,c13,c12,c11,c10,(c09+c08+c07+c06),(c08+c07+c06),(c07+c06),c06,c05,c04,c03,c02,c01,c00};
  reg [`DRAMW-1:0] dmft_dout_C;
  always @(posedge CLK) dmft_dout_C <= dmft_dout_B;
  reg [(`SRTP_WAY+1+1)-1:0] pcC;  // pipeline regester C for control
  always @(posedge CLK) pcC <= pcB;

  // Stage D
  //////////////////////////////////////////////////////////////////////////////
  wire [`SORTW-1:0] d15,d14,d13,d12,d11,d10,d09,d08,d07,d06,d05,d04,d03,d02,d01,d00; // input
  assign {d15,d14,d13,d12,d11,d10,d09,d08,d07,d06,d05,d04,d03,d02,d01,d00} = pdC;

  reg [511:0] pdD; // pipeline regester D for data
  always @(posedge CLK) pdD <= {d15,d14,d13,(d12+d11+d10+d09),(d11+d10+d09),(d10+d09),d09,d08,d07,d06,d05,d04,d03,d02,d01,d00};
  reg [`DRAMW-1:0] dmft_dout_D;
  always @(posedge CLK) dmft_dout_D <= dmft_dout_C;
  reg [(`SRTP_WAY+1+1)-1:0] pcD;  // pipeline regester D for control
  always @(posedge CLK) pcD <= pcC;

  // Stage E
  //////////////////////////////////////////////////////////////////////////////
  wire [`SORTW-1:0] e15,e14,e13,e12,e11,e10,e09,e08,e07,e06,e05,e04,e03,e02,e01,e00; // input
  assign {e15,e14,e13,e12,e11,e10,e09,e08,e07,e06,e05,e04,e03,e02,e01,e00} = pdD;

  reg [511:0] pdE; // pipeline regester E for data
  always @(posedge CLK) pdE <= {(e15+e14+e13+e12),(e14+e13+e12),(e13+e12),e12,e11,e10,e09,e08,e07,e06,e05,e04,e03,e02,e01,e00};
  reg [`DRAMW-1:0] dmft_dout_E;
  always @(posedge CLK) dmft_dout_E <= dmft_dout_D;
  reg [(`SRTP_WAY+1+1)-1:0] pcE;  // pipeline regester E for control
  always @(posedge CLK) pcE <= pcD;

  // Decompression Result
  //////////////////////////////////////////////////////////////////////////////
  wire [`DRAMW-1:0] dc_data  = pdE;
  wire [`DRAMW-1:0] dmft_dot = dmft_dout_E;
  wire              c_vld    = pcE[0];
  wire              dataen   = pcE[1];
  
  // Output ////////////////////////////////////////////////////////////////
  always @(posedge CLK) if (dataen) DOUT <= mux512(dmft_dot, dc_data, c_vld);
  
  always @(posedge CLK) COUT <= pcE[(`SRTP_WAY+1+1)-1:1];
  
endmodule

/***** Output Module                                                                          *****/
/**************************************************************************************************/
module OTMOD(input  wire              CLK, 
             input  wire              RST, 
             input  wire              d_busy, 
             input  wire [31:0]       w_block,
             input  wire              p_z,  // phase zero
             input  wire              F01_deq, 
             input  wire [`MERGW-1:0] F01_dot, 
             input  wire              OB_deq, 
             output wire [`DRAMW-1:0] OB_dot, 
             output wire              buf_t_ful, 
             output reg               OB_req);
  
  function [1-1:0] mux1;
    input [1-1:0] a;
    input [1-1:0] b;
    input         sel;
    begin
      case (sel)
        1'b0: mux1 = a;
        1'b1: mux1 = b;
      endcase
    end
  endfunction

  reg [1:0]         buf_t_cnt; // counter for temporary register
  reg               buf_t_en;
  reg [`DRAMW-1:0]  buf_t;
  wire              buf_t_emp;
                    
  wire [`DRAMW-1:0] c_din;
  wire              c_dinen;
  wire [`DRAMW-1:0] c_dout;
  wire              c_douten;
  
  wire [`DRAMW-1:0] OB_din = c_dout;
  wire              OB_enq = c_douten;
  wire              OB_full;
  wire [`OB_SIZE:0] OB_cnt;
  
  // 512-bit shift register ////////////////////////////////////////////////
  always @(posedge CLK) begin
    if (F01_deq) buf_t <= {F01_dot, buf_t[`DRAMW-1:`MERGW]};
  end
  always @(posedge CLK) begin
    if (RST) begin
      buf_t_cnt <= 0;
    end else begin
      if (F01_deq) buf_t_cnt <= buf_t_cnt + 1;
    end
  end
  always @(posedge CLK) buf_t_en <= (F01_deq && buf_t_cnt == 3);

  MRE2 #(1, `DRAMW) tmp(.CLK(CLK), .RST(RST), .enq(buf_t_en), .deq(c_dinen), 
                        .din(buf_t), .dot(c_din), .emp(buf_t_emp), .full(buf_t_ful));

  // Compressor ////////////////////////////////////////////////////////////
  assign c_dinen = (~|{buf_t_emp,OB_full});
  COMPRESSOR compressor(CLK, RST, c_din, c_dinen, p_z, OB_full, c_dout, c_douten);
  
  // Output Buffer /////////////////////////////////////////////////////////
  BFIFO #(`OB_SIZE, `DRAMW) OB(.CLK(CLK), .RST(RST), .enq(OB_enq), .deq(OB_deq), 
                               .din(OB_din), .dot(OB_dot), .full(OB_full), .cnt(OB_cnt));
  
  always @(posedge CLK) OB_req <= ((OB_cnt>=w_block) && !d_busy);
  
endmodule

/***** Sorting Network                                                                        *****/
/**************************************************************************************************/
module SORTINGNETWORK(input  wire         CLK,
                      input  wire         RST_IN,
                      input  wire         DATAEN_IN,
                      input  wire [511:0] DIN_T,
                      output reg  [511:0] DOUT,
                      output reg          DATAEN_OUT);

  reg         RST;
  reg [511:0] DIN;
  reg         DATAEN;
  always @(posedge CLK) RST  <= RST_IN;
  always @(posedge CLK) DIN    <= DIN_T;
  always @(posedge CLK) DATAEN <= (RST) ? 0 : DATAEN_IN;
  
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

  reg [511:0] pdA; // pipeline regester A for data
  reg         pcA; // pipeline regester A for control
  always @(posedge CLK) pdA <= {A15,A14,A13,A12,A11,A10,A09,A08,A07,A06,A05,A04,A03,A02,A01,A00};
  always @(posedge CLK) pcA <= (RST) ? 0 : DATAEN;

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
  
  reg [511:0] pdB; // pipeline regester B for data
  reg         pcB; // pipeline regester B for control
  always @(posedge CLK) pdB <= {B15,B14,B13,B12,B11,B10,B09,B08,B07,B06,B05,B04,B03,B02,B01,B00};
  always @(posedge CLK) pcB <= (RST) ? 0 : pcA;

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

  reg [511:0] pdC; // pipeline regester C for data
  reg         pcC; // pipeline regester C for control
  always @(posedge CLK) pdC <= {C15,C14,C13,C12,C11,C10,C09,C08,C07,C06,C05,C04,C03,C02,C01,C00};
  always @(posedge CLK) pcC <= (RST) ? 0 : pcB;
  
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
  
  reg [511:0] pdD; // pipeline regester D for data
  reg         pcD; // pipeline regester D for control
  always @(posedge CLK) pdD <= {D15,D14,D13,D12,D11,D10,D09,D08,D07,D06,D05,D04,D03,D02,D01,D00};
  always @(posedge CLK) pcD <= (RST) ? 0 : pcC;
  
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

  reg [511:0] pdE; // pipeline regester E for data
  reg         pcE; // pipeline regester E for control
  always @(posedge CLK) pdE <= {E15,E14,E13,E12,E11,E10,E09,E08,E07,E06,E05,E04,E03,E02,E01,E00};
  always @(posedge CLK) pcE <= (RST) ? 0 : pcD;
  
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
  
  reg [511:0] pdF; // pipeline regester F for data
  reg         pcF; // pipeline regester F for control
  always @(posedge CLK) pdF <= {F15,F14,F13,F12,F11,F10,F09,F08,F07,F06,F05,F04,F03,F02,F01,F00};
  always @(posedge CLK) pcF <= (RST) ? 0 : pcE;
  
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

  reg [511:0] pdG; // pipeline regester G for data
  reg         pcG; // pipeline regester G for control
  always @(posedge CLK) pdG <= {G15,G14,G13,G12,G11,G10,G09,G08,G07,G06,G05,G04,G03,G02,G01,G00};
  always @(posedge CLK) pcG <= (RST) ? 0 : pcF;
  
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

  reg [511:0] pdH; // pipeline regester H for data
  reg         pcH; // pipeline regester H for control
  always @(posedge CLK) pdH <= {H15,H14,H13,H12,H11,H10,H09,H08,H07,H06,H05,H04,H03,H02,H01,H00};
  always @(posedge CLK) pcH <= (RST) ? 0 : pcG;
  
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

  reg [511:0] pdI; // pipeline regester I for data
  reg         pcI; // pipeline regester I for control
  always @(posedge CLK) pdI <= {I15,I14,I13,I12,I11,I10,I09,I08,I07,I06,I05,I04,I03,I02,I01,I00};
  always @(posedge CLK) pcI <= (RST) ? 0 : pcH;
  
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
  always @(posedge CLK) DATAEN_OUT <= (RST) ? 0 : pcI;
endmodule
/**************************************************************************************************/


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
                 input  wire [31:0] elem_plast,
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
          if (elem_plast >= (SORTELM_WAY*2)-(`DRAM_WBLOCKS<<7)) reduce_flag <= 1;
          if (mgdrive && reduce_flag)                           w_block     <= mux32((w_block>>1), 1, (w_block==1));
          case (elem)
            SORTELM_WAY*2: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_b <= w_addr; end
            end
            SORTELM_WAY*4: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_d <= w_addr; end
            end
            SORTELM_WAY*6: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_f <= w_addr; end
            end
            SORTELM_WAY*8: begin 
              w_block <= `DRAM_WBLOCKS; reduce_flag <= 0; if (reduce_flag) begin adr_h <= w_addr; end
            end
          endcase
        end
      endcase
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


/***** Core User Logic                                                                        *****/
/**************************************************************************************************/
module CORE(input  wire              CLK,          // clock
            input  wire              RST_IN,       // reset
            input  wire              d_busy,       // DRAM busy
            output wire [`DRAMW-1:0] d_din,        // DRAM data in
            input  wire              d_w,          // DRAM write flag
            input  wire [`DRAMW-1:0] d_dout,       // DRAM data out
            input  wire              d_douten,     // DRAM data out enable
            output reg  [1:0]        d_req,        // DRAM REQ access request (read/write)
            output reg  [31:0]       d_initadr,    // DRAM REQ initial address for the access
            output reg  [31:0]       d_blocks,     // DRAM REQ the number of blocks per one access
            input  wire [`DRAMW-1:0] rx_data,
            input  wire              rx_data_valid,
            output wire              rx_wait,
            input  wire              chnl_tx_data_ren,
            input  wire              chnl_tx_data_valid,
            output wire [`MERGW-1:0] rslt,
            output wire              rslt_ready);
  
  function [1-1:0] mux1;
    input [1-1:0] a;
    input [1-1:0] b;
    input         sel;
    begin
      case (sel)
        1'b0: mux1 = a;
        1'b1: mux1 = b;
      endcase
    end
  endfunction
  
  function [`SORT_WAY-1:0] mux_sortway;
    input [`SORT_WAY-1:0] a;
    input [`SORT_WAY-1:0] b;
    input          sel;
    begin
      case (sel)
        1'b0: mux_sortway = a;
        1'b1: mux_sortway = b;
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
  
  /**********************************************************************************************/
  wire [`DRAMW-1:0]   OB_dot_a, OB_dot_b;
  wire                OB_req_a, OB_req_b;
  wire                OB_full_a, OB_full_b;
  
  reg  OB_granted_a, OB_granted_b;
  
  wire OB_deq_a = d_w && OB_granted_a;
  wire OB_deq_b = d_w && OB_granted_b;
  
  reg  OB_doten_a, OB_doten_b;
  
  assign d_din = mux512(OB_dot_b, OB_dot_a, OB_granted_a);

  reg [`DRAMW-1:0]    dout_ta;
  reg [`DRAMW-1:0]    dout_tb;
  reg [`DRAMW-1:0]    dout_tc;
  reg [`DRAMW-1:0]    dout_td;
  reg [`DRAMW-1:0]    dout_te;
  reg [`DRAMW-1:0]    dout_tf;
  
  reg                 doen_ta; 
  reg                 doen_tb; //
  reg                 doen_tc; //
  reg                 doen_td; //
  reg                 doen_te; //
  reg                 doen_tf; //
  
  reg [`SORT_WAY-1:0] req;     // use n-bit for n-way sorting, data read request from ways
  reg [`SORT_WAY-1:0] req_a, req_b;

  reg [`SORT_WAY-1:0] req_ta;     
  reg [`SORT_WAY-1:0] req_tb;     
  
  reg [`SORT_WAY-1:0] req_taa;  //
  reg [`SORT_WAY-1:0] req_tab;  //
  reg [`SORT_WAY-1:0] req_tba;  //
  reg [`SORT_WAY-1:0] req_tbb;  //
  
  reg [`SRTP_WAY-1:0] req_pzero;
     
  wire [`SORT_WAY-1:0] im_req_a;
  wire [`SORT_WAY-1:0] im_req_b;

  wire [`SRTP_WAY-1:0] rxw;
  
  reg [31:0]          elem_a, elem_b;       // sorted elements in a phase
  reg [31:0]          elem_way_a, elem_way_b;       // sorted elements in a phase
  reg [31:0]          elem_plast_a, elem_plast_b;       // sorted elements in a phase
  reg                 elem_en_a, elem_en_b;
  
  reg [`PHASE_W]      phase_a, phase_b;      //
  reg                 pchange_a, pchange_b;    // phase_change to reset some registers
  reg                 iter_done_a, iter_done_b;  //
  reg [31:0]          ecnt_a, ecnt_b;       // sorted elements in an iteration
  reg                 irst_a, irst_b;       // INBUF reset
  reg                 frst_a, frst_b;       // sort-tree FIFO reset
  
  reg                 plast_a, plast_b;
  
  reg                 phase_zero;
  reg                 last_phase;
     
  reg RSTa, RSTb;
  always @(posedge CLK) RSTa <= RST_IN;
  always @(posedge CLK) RSTb <= RST_IN;

  /**********************************************************************************************/
  wire [`MERGW-1:0] d00_a, d01_a, d02_a, d03_a, d04_a, d05_a, d06_a, d07_a;
  wire [1:0] ib00_req_a, ib01_req_a, ib02_req_a, ib03_req_a, ib04_req_a, ib05_req_a, ib06_req_a, ib07_req_a;
  wire [`MERGW-1:0] d00_b, d01_b, d02_b, d03_b, d04_b, d05_b, d06_b, d07_b;
  wire [1:0] ib00_req_b, ib01_req_b, ib02_req_b, ib03_req_b, ib04_req_b, ib05_req_b, ib06_req_b, ib07_req_b;

  (* mark_debug = "true" *) wire       rsltbuf_enq;
  (* mark_debug = "true" *) wire       rsltbuf_deq;
                            wire       rsltbuf_emp;
                            wire       rsltbuf_ful;
  (* mark_debug = "true" *) wire [4:0] rsltbuf_cnt;

  wire F01_emp_a, F01_emp_b;
  wire F01_deq_a = mux1((~|{F01_emp_a,OB_full_a}), (~|{F01_emp_a,rsltbuf_ful}), last_phase);
  wire F01_deq_b = (~|{F01_emp_b,OB_full_b});
  
  wire [`MERGW-1:0] F01_dot_a, F01_dot_b;
  
  wire [`MERGW*`SORT_WAY-1:0] s_din_a = {d00_a, d01_a, d02_a, d03_a, d04_a, d05_a, d06_a, d07_a};
  wire [`MERGW*`SORT_WAY-1:0] s_din_b = {d00_b, d01_b, d02_b, d03_b, d04_b, d05_b, d06_b, d07_b};

  wire [`SORT_WAY-1:0] enq_a, enq_b;
  wire [`SORT_WAY-1:0] s_ful_a, s_ful_b;
  

  wire [`SRTP_WAY+`DRAMW-1:0] dc_din   = {req_tb,req_ta,d_dout};
  wire                        dc_dinen = d_douten;
  wire [`DRAMW-1:0]           dc_dout;  // for data
  wire [`SRTP_WAY:0]          dc_cout;  // for control
  wire                        dc_req;
  DECOMPRESSOR #(`IB_SIZE, `DRAM_RBLOCKS)
  decompressor(CLK, RSTa, dc_din, dc_dinen, dc_dout, dc_cout, dc_req);
  
  wire [`DRAMW-1:0] stnet_dout;
  wire              stnet_douten;
  SORTINGNETWORK sortingnetwork(CLK, RSTa, rx_data_valid, rx_data, stnet_dout, stnet_douten);
  
  always @(posedge CLK) begin
    if      (RSTa)    req_pzero <= 1;
    else if (doen_tc) req_pzero <= {req_pzero[`SRTP_WAY-2:0],req_pzero[`SRTP_WAY-1]};
  end

  assign im_req_a = mux_sortway(req_tab, req_pzero[`SORT_WAY-1:0], phase_zero);
  assign im_req_b = mux_sortway(req_tbb, req_pzero[`SRTP_WAY-1:`SORT_WAY], phase_zero);

  wire im00_enq_a = doen_tc & im_req_a[0];
  wire im01_enq_a = doen_tc & im_req_a[1];
  wire im02_enq_a = doen_td & im_req_a[2];
  wire im03_enq_a = doen_td & im_req_a[3];
  wire im04_enq_a = doen_te & im_req_a[4];
  wire im05_enq_a = doen_te & im_req_a[5];
  wire im06_enq_a = doen_tf & im_req_a[6];
  wire im07_enq_a = doen_tf & im_req_a[7];

  wire im00_enq_b = doen_tc & im_req_b[0];
  wire im01_enq_b = doen_tc & im_req_b[1];
  wire im02_enq_b = doen_td & im_req_b[2];
  wire im03_enq_b = doen_td & im_req_b[3];
  wire im04_enq_b = doen_te & im_req_b[4];
  wire im05_enq_b = doen_te & im_req_b[5];
  wire im06_enq_b = doen_tf & im_req_b[6];
  wire im07_enq_b = doen_tf & im_req_b[7];

  INMOD2 im00_a(CLK, RSTa, dout_tc, im00_enq_a,  s_ful_a[0],  rxw[0],  d00_a, enq_a[0],  ib00_req_a);
  INMOD2 im01_a(CLK, RSTa, dout_tc, im01_enq_a,  s_ful_a[1],  rxw[1],  d01_a, enq_a[1],  ib01_req_a);
  INMOD2 im02_a(CLK, RSTa, dout_td, im02_enq_a,  s_ful_a[2],  rxw[2],  d02_a, enq_a[2],  ib02_req_a);
  INMOD2 im03_a(CLK, RSTa, dout_td, im03_enq_a,  s_ful_a[3],  rxw[3],  d03_a, enq_a[3],  ib03_req_a);
  INMOD2 im04_a(CLK, RSTa, dout_te, im04_enq_a,  s_ful_a[4],  rxw[4],  d04_a, enq_a[4],  ib04_req_a);
  INMOD2 im05_a(CLK, RSTa, dout_te, im05_enq_a,  s_ful_a[5],  rxw[5],  d05_a, enq_a[5],  ib05_req_a);
  INMOD2 im06_a(CLK, RSTa, dout_tf, im06_enq_a,  s_ful_a[6],  rxw[6],  d06_a, enq_a[6],  ib06_req_a);
  INMOD2 im07_a(CLK, RSTa, dout_tf, im07_enq_a,  s_ful_a[7],  rxw[7],  d07_a, enq_a[7],  ib07_req_a);

  INMOD2 im00_b(CLK, RSTb, dout_tc, im00_enq_b,  s_ful_b[0],  rxw[8],  d00_b, enq_b[0],  ib00_req_b);
  INMOD2 im01_b(CLK, RSTb, dout_tc, im01_enq_b,  s_ful_b[1],  rxw[9],  d01_b, enq_b[1],  ib01_req_b);
  INMOD2 im02_b(CLK, RSTb, dout_td, im02_enq_b,  s_ful_b[2],  rxw[10], d02_b, enq_b[2],  ib02_req_b);
  INMOD2 im03_b(CLK, RSTb, dout_td, im03_enq_b,  s_ful_b[3],  rxw[11], d03_b, enq_b[3],  ib03_req_b);
  INMOD2 im04_b(CLK, RSTb, dout_te, im04_enq_b,  s_ful_b[4],  rxw[12], d04_b, enq_b[4],  ib04_req_b);
  INMOD2 im05_b(CLK, RSTb, dout_te, im05_enq_b,  s_ful_b[5],  rxw[13], d05_b, enq_b[5],  ib05_req_b);
  INMOD2 im06_b(CLK, RSTb, dout_tf, im06_enq_b,  s_ful_b[6],  rxw[14], d06_b, enq_b[6],  ib06_req_b);
  INMOD2 im07_b(CLK, RSTb, dout_tf, im07_enq_b,  s_ful_b[7],  rxw[15], d07_b, enq_b[7],  ib07_req_b);

  assign rx_wait = |rxw;
                  
  STREE stree_a(CLK, RSTa, irst_a, frst_a, phase_a, s_din_a, enq_a, s_ful_a, F01_deq_a, F01_dot_a, F01_emp_a);
  STREE stree_b(CLK, RSTb, irst_b, frst_b, phase_b, s_din_b, enq_b, s_ful_b, F01_deq_b, F01_dot_b, F01_emp_b);

  // ----- for dram READ/WRITE controller -----
  reg [31:0] w_addr; // 
  reg [31:0] w_addr_pzero; // 
  reg [31:0] w_addr_a, w_addr_b; 
  reg [3:0]  state;  // state
  
  reg [31:0] radr_a, radr_b, radr_c, radr_d, radr_e, radr_f, radr_g, radr_h;
  reg [31:0] radr_a_a, radr_b_a, radr_c_a, radr_d_a, radr_e_a, radr_f_a, radr_g_a, radr_h_a;
  reg [31:0] radr_a_b, radr_b_b, radr_c_b, radr_d_b, radr_e_b, radr_f_b, radr_g_b, radr_h_b;
  
  reg [27:0] cnt_a, cnt_b, cnt_c, cnt_d, cnt_e, cnt_f, cnt_g, cnt_h;
  reg [27:0] cnt_a_a, cnt_b_a, cnt_c_a, cnt_d_a, cnt_e_a, cnt_f_a, cnt_g_a, cnt_h_a;
  reg [27:0] cnt_a_b, cnt_b_b, cnt_c_b, cnt_d_b, cnt_e_b, cnt_f_b, cnt_g_b, cnt_h_b;
  
  reg        c_a, c_b, c_c, c_d, c_e, c_f, c_g, c_h; // counter is full ?
  reg        c_a_a, c_b_a, c_c_a, c_d_a, c_e_a, c_f_a, c_g_a, c_h_a;
  reg        c_a_b, c_b_b, c_c_b, c_d_b, c_e_b, c_f_b, c_g_b, c_h_b;
  
  // ----- request counter manager -----
  // input
  wire mgdrive   = (state==3 && d_req!=0);
  wire mgdrive_a = (state==6 && d_req!=0 && (|req_ta));
  wire mgdrive_b = (state==6 && d_req!=0 && (|req_tb));
  
  wire [`SORT_WAY-1:0] im_enq_a = {im07_enq_a,im06_enq_a,im05_enq_a,im04_enq_a,im03_enq_a,im02_enq_a,im01_enq_a,im00_enq_a};
  wire [`SORT_WAY-1:0] im_enq_b = {im07_enq_b,im06_enq_b,im05_enq_b,im04_enq_b,im03_enq_b,im02_enq_b,im01_enq_b,im00_enq_b};
  
  wire [`SORT_WAY-1:0] im_emp_a = {ib07_req_a[1],ib06_req_a[1],ib05_req_a[1],ib04_req_a[1],ib03_req_a[1],ib02_req_a[1],ib01_req_a[1],ib00_req_a[1]};
  wire [`SORT_WAY-1:0] im_emp_b = {ib07_req_b[1],ib06_req_b[1],ib05_req_b[1],ib04_req_b[1],ib03_req_b[1],ib02_req_b[1],ib01_req_b[1],ib00_req_b[1]};
  
  // output
  wire reqcnt_a, reqcnt_b, reqcnt_c, reqcnt_d, reqcnt_e, reqcnt_f, reqcnt_g, reqcnt_h;
  wire reqcnt_a_a, reqcnt_b_a, reqcnt_c_a, reqcnt_d_a, reqcnt_e_a, reqcnt_f_a, reqcnt_g_a, reqcnt_h_a;
  wire reqcnt_a_b, reqcnt_b_b, reqcnt_c_b, reqcnt_d_b, reqcnt_e_b, reqcnt_f_b, reqcnt_g_b, reqcnt_h_b;

  REQCNTMG reqcntmg(CLK, RSTa, mgdrive, req_ta, im_enq_a, im_emp_a, reqcnt_a, reqcnt_b, reqcnt_c, reqcnt_d, reqcnt_e, reqcnt_f, reqcnt_g, reqcnt_h);
  REQCNTMG reqcntmg_a(CLK, RSTa, mgdrive_a, req_ta, im_enq_a, im_emp_a, reqcnt_a_a, reqcnt_b_a, reqcnt_c_a, reqcnt_d_a, reqcnt_e_a, reqcnt_f_a, reqcnt_g_a, reqcnt_h_a);
  REQCNTMG reqcntmg_b(CLK, RSTb, mgdrive_b, req_tb, im_enq_b, im_emp_b, reqcnt_a_b, reqcnt_b_b, reqcnt_c_b, reqcnt_d_b, reqcnt_e_b, reqcnt_f_b, reqcnt_g_b, reqcnt_h_b);
  
  
  // ----- write manager -----
  // output
  wire [31:0] w_block_a;
  wire [31:0] w_block_b;
  wire [31:0] r_endadr_a_a, r_endadr_b_a, r_endadr_c_a, r_endadr_d_a, r_endadr_e_a, r_endadr_f_a, r_endadr_g_a, r_endadr_h_a;
  wire [31:0] r_endadr_a_b, r_endadr_b_b, r_endadr_c_b, r_endadr_d_b, r_endadr_e_b, r_endadr_f_b, r_endadr_g_b, r_endadr_h_b;

  WRITEMG #((`SORT_ELM>>(`P_LOG+`WAY_LOG)))
  writemg_a(CLK, RSTa, pchange_a, plast_a, (state==7 && d_req!=0), elem_a, elem_way_a, elem_plast_a, w_addr_a, 
            w_block_a, r_endadr_a_a, r_endadr_b_a, r_endadr_c_a, r_endadr_d_a, r_endadr_e_a, r_endadr_f_a, r_endadr_g_a, r_endadr_h_a);
  WRITEMG #((`SORT_ELM>>(`P_LOG+`WAY_LOG)))
  writemg_b(CLK, RSTb, pchange_b, plast_b, (state==8 && d_req!=0), elem_b, elem_way_b, elem_plast_b, w_addr_b, 
            w_block_b, r_endadr_a_b, r_endadr_b_b, r_endadr_c_b, r_endadr_d_b, r_endadr_e_b, r_endadr_f_b, r_endadr_g_b, r_endadr_h_b);
  
  // ----- read manager -----
  // output
  wire [31:0] r_block_a, r_block_b, r_block_c, r_block_d, r_block_e, r_block_f, r_block_g, r_block_h;
  wire [31:0] r_block_a_a, r_block_b_a, r_block_c_a, r_block_d_a, r_block_e_a, r_block_f_a, r_block_g_a, r_block_h_a;
  wire [31:0] r_block_a_b, r_block_b_b, r_block_c_b, r_block_d_b, r_block_e_b, r_block_f_b, r_block_g_b, r_block_h_b;
  wire        readend_a, readend_b, readend_c, readend_d, readend_e, readend_f, readend_g, readend_h;
  wire        readend_a_a, readend_b_a, readend_c_a, readend_d_a, readend_e_a, readend_f_a, readend_g_a, readend_h_a;
  wire        readend_a_b, readend_b_b, readend_c_b, readend_d_b, readend_e_b, readend_f_b, readend_g_b, readend_h_b;

  READMG readmg(CLK, !last_phase, (state==3 && d_req!=0), req_ta, phase_a[0], 
                radr_a, radr_b, radr_c, radr_d, radr_e, radr_f, radr_g, radr_h, 
                r_endadr_b_a, r_endadr_d_a, r_endadr_f_a, r_endadr_h_a, r_endadr_b_b, r_endadr_d_b, r_endadr_f_b, r_endadr_h_b, 
                r_block_a, r_block_b, r_block_c, r_block_d, r_block_e, r_block_f, r_block_g, r_block_h, 
                readend_a, readend_b, readend_c, readend_d, readend_e, readend_f, readend_g, readend_h);
  READMG readmg_a(CLK, (RSTa || pchange_a), (state==6 && d_req!=0 && (|req_ta)), req_ta, phase_a[0], 
                  radr_a_a, radr_b_a, radr_c_a, radr_d_a, radr_e_a, radr_f_a, radr_g_a, radr_h_a, 
                  r_endadr_a_a, r_endadr_b_a, r_endadr_c_a, r_endadr_d_a, r_endadr_e_a, r_endadr_f_a, r_endadr_g_a, r_endadr_h_a, 
                  r_block_a_a, r_block_b_a, r_block_c_a, r_block_d_a, r_block_e_a, r_block_f_a, r_block_g_a, r_block_h_a, 
                  readend_a_a, readend_b_a, readend_c_a, readend_d_a, readend_e_a, readend_f_a, readend_g_a, readend_h_a);
  READMG readmg_b(CLK, (RSTb || pchange_b), (state==6 && d_req!=0 && (|req_tb)), req_tb, phase_b[0], 
                  radr_a_b, radr_b_b, radr_c_b, radr_d_b, radr_e_b, radr_f_b, radr_g_b, radr_h_b, 
                  r_endadr_a_b, r_endadr_b_b, r_endadr_c_b, r_endadr_d_b, r_endadr_e_b, r_endadr_f_b, r_endadr_g_b, r_endadr_h_b, 
                  r_block_a_b, r_block_b_b, r_block_c_b, r_block_d_b, r_block_e_b, r_block_f_b, r_block_g_b, r_block_h_b, 
                  readend_a_b, readend_b_b, readend_c_b, readend_d_b, readend_e_b, readend_f_b, readend_g_b, readend_h_b);

  // ----- output buffer -----
  reg OB_stopreq_a, OB_stopreq_b;
  always @(posedge CLK) OB_stopreq_a <= (d_busy || OB_doten_a || elem_en_a || (elem_way_a==(`SORT_ELM>>(`P_LOG+`WAY_LOG))));
  always @(posedge CLK) OB_stopreq_b <= (d_busy || OB_doten_b || elem_en_b || (elem_way_b==(`SORT_ELM>>(`P_LOG+`WAY_LOG))));
  
  always @(posedge CLK) OB_doten_a <= OB_deq_a;
  always @(posedge CLK) OB_doten_b <= OB_deq_b;
  OTMOD ob_a(CLK, RSTa, OB_stopreq_a, w_block_a, phase_zero, (!last_phase && F01_deq_a), F01_dot_a, OB_deq_a, OB_dot_a, OB_full_a, OB_req_a);
  OTMOD ob_b(CLK, RSTb, OB_stopreq_b, w_block_b, phase_zero, F01_deq_b, F01_dot_b, OB_deq_b, OB_dot_b, OB_full_b, OB_req_b);
  
  assign rsltbuf_enq = last_phase && F01_deq_a;
  assign rsltbuf_deq = chnl_tx_data_ren && chnl_tx_data_valid;
  SRL_FIFO #(4, `MERGW) rsltbuf(CLK, RSTa, rsltbuf_enq, rsltbuf_deq, F01_dot_a, 
                                rslt, rsltbuf_emp, rsltbuf_ful, rsltbuf_cnt);
  assign rslt_ready = !rsltbuf_emp;
  

  /***** dram READ/WRITE controller                                                         *****/
  /**********************************************************************************************/
  wire reqhalt_a_a = mux1((readend_a_a || (phase_a==`LAST_PHASE)), c_a_a, (phase_a==1));
  wire reqhalt_b_a = mux1((readend_b_a || (phase_a==`LAST_PHASE)), c_b_a, (phase_a==1));
  wire reqhalt_c_a = mux1((readend_c_a || (phase_a==`LAST_PHASE)), c_c_a, (phase_a==1));
  wire reqhalt_d_a = mux1((readend_d_a || (phase_a==`LAST_PHASE)), c_d_a, (phase_a==1));
  wire reqhalt_e_a = mux1((readend_e_a || (phase_a==`LAST_PHASE)), c_e_a, (phase_a==1));
  wire reqhalt_f_a = mux1((readend_f_a || (phase_a==`LAST_PHASE)), c_f_a, (phase_a==1));
  wire reqhalt_g_a = mux1((readend_g_a || (phase_a==`LAST_PHASE)), c_g_a, (phase_a==1));
  wire reqhalt_h_a = mux1((readend_h_a || (phase_a==`LAST_PHASE)), c_h_a, (phase_a==1));
  
  wire reqhalt_a_b = mux1((readend_a_b || (phase_b==`LAST_PHASE)), c_a_b, (phase_b==1));
  wire reqhalt_b_b = mux1((readend_b_b || (phase_b==`LAST_PHASE)), c_b_b, (phase_b==1));
  wire reqhalt_c_b = mux1((readend_c_b || (phase_b==`LAST_PHASE)), c_c_b, (phase_b==1));
  wire reqhalt_d_b = mux1((readend_d_b || (phase_b==`LAST_PHASE)), c_d_b, (phase_b==1));
  wire reqhalt_e_b = mux1((readend_e_b || (phase_b==`LAST_PHASE)), c_e_b, (phase_b==1));
  wire reqhalt_f_b = mux1((readend_f_b || (phase_b==`LAST_PHASE)), c_f_b, (phase_b==1));
  wire reqhalt_g_b = mux1((readend_g_b || (phase_b==`LAST_PHASE)), c_g_b, (phase_b==1));
  wire reqhalt_h_b = mux1((readend_h_b || (phase_b==`LAST_PHASE)), c_h_b, (phase_b==1));
  
  always @(posedge CLK) begin
    if (RSTa || pchange_a || pchange_b) begin 
      if (RSTa) state <= 0;
      if (RSTa) {d_req, d_initadr, d_blocks} <= 0;
      if (RSTa) w_addr_pzero <= (`SORT_ELM>>1);
      
      req <= 0;
      w_addr <= mux32((`SORT_ELM>>1), 0, phase_a[0]);
      radr_a <= ((`SELM_PER_WAY>>3)*0);
      radr_b <= ((`SELM_PER_WAY>>3)*1);
      radr_c <= ((`SELM_PER_WAY>>3)*2);
      radr_d <= ((`SELM_PER_WAY>>3)*3);
      radr_e <= ((`SELM_PER_WAY>>3)*4);
      radr_f <= ((`SELM_PER_WAY>>3)*5);
      radr_g <= ((`SELM_PER_WAY>>3)*6);
      radr_h <= ((`SELM_PER_WAY>>3)*7);
      {cnt_a, cnt_b, cnt_c, cnt_d, cnt_e, cnt_f, cnt_g, cnt_h} <= 0;
      {c_a, c_b, c_c, c_d, c_e, c_f, c_g, c_h} <= 0;

      if ((RSTa || pchange_a) && !plast_a) begin
        req_a <= 0;
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
        OB_granted_a <= 0;
      end
      
      if ((RSTa || pchange_b) && !plast_b) begin
        req_b <= 0;
        w_addr_b  <= mux32(((`SORT_ELM>>2) | (`SORT_ELM>>1)), (`SORT_ELM>>2), phase_b[0]);
        radr_a_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*0) | (`SORT_ELM>>2);
        radr_b_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>2);
        radr_c_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>2);
        radr_d_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>2);
        radr_e_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>2);
        radr_f_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>2);
        radr_g_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>2);
        radr_h_b  <= ((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>2);
        {cnt_a_b, cnt_b_b, cnt_c_b, cnt_d_b, cnt_e_b, cnt_f_b, cnt_g_b, cnt_h_b} <= 0;
        {c_a_b, c_b_b, c_c_b, c_d_b, c_e_b, c_f_b, c_g_b, c_h_b} <= 0;
        OB_granted_b <= 0;
      end
      
    end else begin
      case (state)
        ////////////////////////////////////////////////////////////////////////////////////////
        0: begin ///// Initialize memory, write data to DRAM
          if (!phase_zero) state <= 4;

          if (d_req != 0) d_req <= 0;
          else if (!d_busy) begin
            if (OB_req_a || OB_req_b) begin
              d_req     <= `DRAM_REQ_WRITE;   //
              d_blocks  <= `DRAM_WBLOCKS;     //
              d_initadr    <= w_addr_pzero;            //
              w_addr_pzero <= w_addr_pzero + (`D_WS);  // address for the next write
              if      (OB_req_a) begin OB_granted_a <= 1; OB_granted_b <= 0; end
              else if (OB_req_b) begin OB_granted_a <= 0; OB_granted_b <= 1; end
            end
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        1: begin ///// request arbitration 
          if (!d_busy && dc_req) begin
            if      (ib00_req_a[1] && !readend_a && ~reqcnt_a) begin req<=8'h01; state<=3; end // first priority
            else if (ib01_req_a[1] && !readend_b && ~reqcnt_b) begin req<=8'h02; state<=3; end // 
            else if (ib02_req_a[1] && !readend_c && ~reqcnt_c) begin req<=8'h04; state<=3; end // 
            else if (ib03_req_a[1] && !readend_d && ~reqcnt_d) begin req<=8'h08; state<=3; end // 
            else if (ib04_req_a[1] && !readend_e && ~reqcnt_e) begin req<=8'h10; state<=3; end // 
            else if (ib05_req_a[1] && !readend_f && ~reqcnt_f) begin req<=8'h20; state<=3; end // 
            else if (ib06_req_a[1] && !readend_g && ~reqcnt_g) begin req<=8'h40; state<=3; end // 
            else if (ib07_req_a[1] && !readend_h && ~reqcnt_h) begin req<=8'h80; state<=3; end //
            else                                                    state<=2;
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        2: begin ///// request arbitration 
          if (!d_busy && dc_req) begin  // can be removed
            if      (ib00_req_a[0] && !readend_a && ~reqcnt_a) begin req<=8'h01; state<=3; end // second priority
            else if (ib01_req_a[0] && !readend_b && ~reqcnt_b) begin req<=8'h02; state<=3; end // 
            else if (ib02_req_a[0] && !readend_c && ~reqcnt_c) begin req<=8'h04; state<=3; end // 
            else if (ib03_req_a[0] && !readend_d && ~reqcnt_d) begin req<=8'h08; state<=3; end // 
            else if (ib04_req_a[0] && !readend_e && ~reqcnt_e) begin req<=8'h10; state<=3; end // 
            else if (ib05_req_a[0] && !readend_f && ~reqcnt_f) begin req<=8'h20; state<=3; end // 
            else if (ib06_req_a[0] && !readend_g && ~reqcnt_g) begin req<=8'h40; state<=3; end // 
            else if (ib07_req_a[0] && !readend_h && ~reqcnt_h) begin req<=8'h80; state<=3; end // 
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        3: begin ///// READ data from DRAM
          if (d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            case (req)
              8'h01: begin
                d_initadr <= mux32(radr_a, (radr_a | (`SORT_ELM>>1)), phase_a[0]);
                radr_a    <= radr_a+(r_block_a<<3); 
                d_blocks  <= r_block_a;
              end
              8'h02: begin
                d_initadr <= mux32(radr_b, (radr_b | (`SORT_ELM>>1)), phase_a[0]);
                radr_b    <= radr_b+(r_block_b<<3); 
                d_blocks  <= r_block_b;
              end
              8'h04: begin
                d_initadr <= mux32(radr_c, (radr_c | (`SORT_ELM>>1)), phase_a[0]);
                radr_c    <= radr_c+(r_block_c<<3); 
                d_blocks  <= r_block_c;
              end
              8'h08: begin
                d_initadr <= mux32(radr_d, (radr_d | (`SORT_ELM>>1)), phase_a[0]);
                radr_d    <= radr_d+(r_block_d<<3); 
                d_blocks  <= r_block_d;
              end
              8'h10: begin
                d_initadr <= mux32(radr_e, (radr_e | (`SORT_ELM>>1)), phase_a[0]);
                radr_e     <= radr_e+(r_block_e<<3); 
                d_blocks  <= r_block_e;
              end
              8'h20: begin
                d_initadr <= mux32(radr_f, (radr_f | (`SORT_ELM>>1)), phase_a[0]);
                radr_f    <= radr_f+(r_block_f<<3); 
                d_blocks  <= r_block_f;
              end
              8'h40: begin
                d_initadr <= mux32(radr_g, (radr_g | (`SORT_ELM>>1)), phase_a[0]);
                radr_g    <= radr_g+(r_block_g<<3); 
                d_blocks  <= r_block_g;
              end
              8'h80: begin
                d_initadr <= mux32(radr_h, (radr_h | (`SORT_ELM>>1)), phase_a[0]);
                radr_h    <= radr_h+(r_block_h<<3); 
                d_blocks  <= r_block_h;
              end
            endcase
            d_req  <= `DRAM_REQ_READ;
            req_ta <= req;
            req_tb <= 0;
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        4: begin
          if (!d_busy && dc_req) begin
            ///////////////// can be parameterized
            if      (ib00_req_a[1] && !reqhalt_a_a && ~reqcnt_a_a) begin req_a<=8'h01; req_b<=0; state<=6; end // 1st priority
            else if (ib01_req_a[1] && !reqhalt_b_a && ~reqcnt_b_a) begin req_a<=8'h02; req_b<=0; state<=6; end // 
            else if (ib02_req_a[1] && !reqhalt_c_a && ~reqcnt_c_a) begin req_a<=8'h04; req_b<=0; state<=6; end // 
            else if (ib03_req_a[1] && !reqhalt_d_a && ~reqcnt_d_a) begin req_a<=8'h08; req_b<=0; state<=6; end //
            else if (ib04_req_a[1] && !reqhalt_e_a && ~reqcnt_e_a) begin req_a<=8'h10; req_b<=0; state<=6; end // 
            else if (ib05_req_a[1] && !reqhalt_f_a && ~reqcnt_f_a) begin req_a<=8'h20; req_b<=0; state<=6; end // 
            else if (ib06_req_a[1] && !reqhalt_g_a && ~reqcnt_g_a) begin req_a<=8'h40; req_b<=0; state<=6; end // 
            else if (ib07_req_a[1] && !reqhalt_h_a && ~reqcnt_h_a) begin req_a<=8'h80; req_b<=0; state<=6; end //
            else if (ib00_req_b[1] && !reqhalt_a_b && ~reqcnt_a_b) begin req_b<=8'h01; req_a<=0; state<=6; end // 
            else if (ib01_req_b[1] && !reqhalt_b_b && ~reqcnt_b_b) begin req_b<=8'h02; req_a<=0; state<=6; end // 
            else if (ib02_req_b[1] && !reqhalt_c_b && ~reqcnt_c_b) begin req_b<=8'h04; req_a<=0; state<=6; end // 
            else if (ib03_req_b[1] && !reqhalt_d_b && ~reqcnt_d_b) begin req_b<=8'h08; req_a<=0; state<=6; end //
            else if (ib04_req_b[1] && !reqhalt_e_b && ~reqcnt_e_b) begin req_b<=8'h10; req_a<=0; state<=6; end // 
            else if (ib05_req_b[1] && !reqhalt_f_b && ~reqcnt_f_b) begin req_b<=8'h20; req_a<=0; state<=6; end // 
            else if (ib06_req_b[1] && !reqhalt_g_b && ~reqcnt_g_b) begin req_b<=8'h40; req_a<=0; state<=6; end // 
            else if (ib07_req_b[1] && !reqhalt_h_b && ~reqcnt_h_b) begin req_b<=8'h80; req_a<=0; state<=6; end //
            else                                                                  state<=5;
          end
        end
        5: begin
          case (plast_a)
            0: begin
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
            1: begin
              case (elem_a)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_a <= mux32(((`SELM_PER_WAY>>3)*1) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>3)*1), phase_a[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_a <= mux32(((`SELM_PER_WAY>>3)*2) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>3)*2), phase_a[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_a <= mux32(((`SELM_PER_WAY>>3)*3) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>3)*3), phase_a[0]);
              endcase
            end
          endcase
          case (plast_b)
            0: begin
              case (elem_b)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*1: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*1) | (`SORT_ELM>>2)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*2) | (`SORT_ELM>>2)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*3: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*3) | (`SORT_ELM>>2)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*4) | (`SORT_ELM>>2)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*5: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*5) | (`SORT_ELM>>2)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*6) | (`SORT_ELM>>2)), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*7: w_addr_b <= mux32((((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>2)) | (`SORT_ELM>>1), (((`SELM_PER_WAY>>(`P_LOG+3))*7) | (`SORT_ELM>>2)), phase_b[0]);
              endcase
            end
            1: begin
              case (elem_b)
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*2: w_addr_b <= mux32(((`SELM_PER_WAY>>3)*5) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>3)*5), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*4: w_addr_b <= mux32(((`SELM_PER_WAY>>3)*6) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>3)*6), phase_b[0]);
                (`SORT_ELM>>(`P_LOG+`WAY_LOG))*6: w_addr_b <= mux32(((`SELM_PER_WAY>>3)*7) | (`SORT_ELM>>1), ((`SELM_PER_WAY>>3)*7), phase_b[0]);
              endcase
            end
          endcase
          ///////////////// can be parameterized
          if      (ib00_req_a[0] && !reqhalt_a_a && ~reqcnt_a_a) begin req_a<=8'h01; req_b<=0; state<=6; end // 2nd priority
          else if (ib01_req_a[0] && !reqhalt_b_a && ~reqcnt_b_a) begin req_a<=8'h02; req_b<=0; state<=6; end // 
          else if (ib02_req_a[0] && !reqhalt_c_a && ~reqcnt_c_a) begin req_a<=8'h04; req_b<=0; state<=6; end // 
          else if (ib03_req_a[0] && !reqhalt_d_a && ~reqcnt_d_a) begin req_a<=8'h08; req_b<=0; state<=6; end //
          else if (ib04_req_a[0] && !reqhalt_e_a && ~reqcnt_e_a) begin req_a<=8'h10; req_b<=0; state<=6; end // 
          else if (ib05_req_a[0] && !reqhalt_f_a && ~reqcnt_f_a) begin req_a<=8'h20; req_b<=0; state<=6; end // 
          else if (ib06_req_a[0] && !reqhalt_g_a && ~reqcnt_g_a) begin req_a<=8'h40; req_b<=0; state<=6; end // 
          else if (ib07_req_a[0] && !reqhalt_h_a && ~reqcnt_h_a) begin req_a<=8'h80; req_b<=0; state<=6; end //
          else if (ib00_req_b[0] && !reqhalt_a_b && ~reqcnt_a_b) begin req_b<=8'h01; req_a<=0; state<=6; end // 
          else if (ib01_req_b[0] && !reqhalt_b_b && ~reqcnt_b_b) begin req_b<=8'h02; req_a<=0; state<=6; end // 
          else if (ib02_req_b[0] && !reqhalt_c_b && ~reqcnt_c_b) begin req_b<=8'h04; req_a<=0; state<=6; end // 
          else if (ib03_req_b[0] && !reqhalt_d_b && ~reqcnt_d_b) begin req_b<=8'h08; req_a<=0; state<=6; end //
          else if (ib04_req_b[0] && !reqhalt_e_b && ~reqcnt_e_b) begin req_b<=8'h10; req_a<=0; state<=6; end // 
          else if (ib05_req_b[0] && !reqhalt_f_b && ~reqcnt_f_b) begin req_b<=8'h20; req_a<=0; state<=6; end // 
          else if (ib06_req_b[0] && !reqhalt_g_b && ~reqcnt_g_b) begin req_b<=8'h40; req_a<=0; state<=6; end // 
          else if (ib07_req_b[0] && !reqhalt_h_b && ~reqcnt_h_b) begin req_b<=8'h80; req_a<=0; state<=6; end //
          else if (OB_req_a) begin OB_granted_a <= 1; OB_granted_b <= 0;        state<=7; end
          else if (OB_req_b) begin OB_granted_a <= 0; OB_granted_b <= 1;        state<=8; end
          else if (last_phase)                                                  state<=1; 
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        6: begin
          if (d_req!=0) begin d_req<=0; state<=4; end
          else if (!d_busy) begin
            case ({req_b,req_a})
              16'h0001: begin
                d_initadr <= mux32(radr_a_a, (radr_a_a | (`SORT_ELM>>1)), phase_a[0]);
                radr_a_a  <= mux32((radr_a_a+(r_block_a_a<<3)), radr_a_a+(`D_RS), (phase_a==1)); 
                cnt_a_a   <= cnt_a_a+1; 
                c_a_a     <= (cnt_a_a>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_a_a, `DRAM_RBLOCKS, (phase_a==1));
              end
              16'h0002: begin
                d_initadr <= mux32(radr_b_a, (radr_b_a | (`SORT_ELM>>1)), phase_a[0]);
                radr_b_a  <= mux32((radr_b_a+(r_block_b_a<<3)), radr_b_a+(`D_RS), (phase_a==1)); 
                cnt_b_a   <= cnt_b_a+1; 
                c_b_a     <= (cnt_b_a>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_b_a, `DRAM_RBLOCKS, (phase_a==1));
              end
              16'h0004: begin
                d_initadr <= mux32(radr_c_a, (radr_c_a | (`SORT_ELM>>1)), phase_a[0]);
                radr_c_a  <= mux32((radr_c_a+(r_block_c_a<<3)), radr_c_a+(`D_RS), (phase_a==1)); 
                cnt_c_a   <= cnt_c_a+1; 
                c_c_a     <= (cnt_c_a>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_c_a, `DRAM_RBLOCKS, (phase_a==1));
              end
              16'h0008: begin
                d_initadr <= mux32(radr_d_a, (radr_d_a | (`SORT_ELM>>1)), phase_a[0]);
                radr_d_a  <= mux32((radr_d_a+(r_block_d_a<<3)), radr_d_a+(`D_RS), (phase_a==1)); 
                cnt_d_a   <= cnt_d_a+1; 
                c_d_a     <= (cnt_d_a>=`WAYP_CN_);                        
                d_blocks  <= mux32(r_block_d_a, `DRAM_RBLOCKS, (phase_a==1));
              end
              16'h0010: begin
                d_initadr <= mux32(radr_e_a, (radr_e_a | (`SORT_ELM>>1)), phase_a[0]);
                radr_e_a  <= mux32((radr_e_a+(r_block_e_a<<3)), radr_e_a+(`D_RS), (phase_a==1));  
                cnt_e_a   <= cnt_e_a+1; 
                c_e_a     <= (cnt_e_a>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_e_a, `DRAM_RBLOCKS, (phase_a==1));
              end
              16'h0020: begin
                d_initadr <= mux32(radr_f_a, (radr_f_a | (`SORT_ELM>>1)), phase_a[0]);
                radr_f_a  <= mux32((radr_f_a+(r_block_f_a<<3)), radr_f_a+(`D_RS), (phase_a==1));  
                cnt_f_a   <= cnt_f_a+1; 
                c_f_a     <= (cnt_f_a>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_f_a, `DRAM_RBLOCKS, (phase_a==1));
              end
              16'h0040: begin
                d_initadr <= mux32(radr_g_a, (radr_g_a | (`SORT_ELM>>1)), phase_a[0]);
                radr_g_a  <= mux32((radr_g_a+(r_block_g_a<<3)), radr_g_a+(`D_RS), (phase_a==1));  
                cnt_g_a   <= cnt_g_a+1; 
                c_g_a     <= (cnt_g_a>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_g_a, `DRAM_RBLOCKS, (phase_a==1));
              end
              16'h0080: begin
                d_initadr <= mux32(radr_h_a, (radr_h_a | (`SORT_ELM>>1)), phase_a[0]);
                radr_h_a  <= mux32((radr_h_a+(r_block_h_a<<3)), radr_h_a+(`D_RS), (phase_a==1));  
                cnt_h_a   <= cnt_h_a+1; 
                c_h_a     <= (cnt_h_a>=`WAYP_CN_);                        
                d_blocks  <= mux32(r_block_h_a, `DRAM_RBLOCKS, (phase_a==1));
              end
              16'h0100: begin
                d_initadr <= mux32(radr_a_b, (radr_a_b | (`SORT_ELM>>1)), phase_b[0]);
                radr_a_b  <= mux32((radr_a_b+(r_block_a_b<<3)), radr_a_b+(`D_RS), (phase_b==1)); 
                cnt_a_b   <= cnt_a_b+1; 
                c_a_b     <= (cnt_a_b>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_a_b, `DRAM_RBLOCKS, (phase_b==1));
              end
              16'h0200: begin
                d_initadr <= mux32(radr_b_b, (radr_b_b | (`SORT_ELM>>1)), phase_b[0]);
                radr_b_b  <= mux32((radr_b_b+(r_block_b_b<<3)), radr_b_b+(`D_RS), (phase_b==1)); 
                cnt_b_b   <= cnt_b_b+1; 
                c_b_b     <= (cnt_b_b>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_b_b, `DRAM_RBLOCKS, (phase_b==1));
              end
              16'h0400: begin
                d_initadr <= mux32(radr_c_b, (radr_c_b | (`SORT_ELM>>1)), phase_b[0]);
                radr_c_b  <= mux32((radr_c_b+(r_block_c_b<<3)), radr_c_b+(`D_RS), (phase_b==1)); 
                cnt_c_b   <= cnt_c_b+1; 
                c_c_b     <= (cnt_c_b>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_c_b, `DRAM_RBLOCKS, (phase_b==1));
              end
              16'h0800: begin
                d_initadr <= mux32(radr_d_b, (radr_d_b | (`SORT_ELM>>1)), phase_b[0]);
                radr_d_b  <= mux32((radr_d_b+(r_block_d_b<<3)), radr_d_b+(`D_RS), (phase_b==1)); 
                cnt_d_b   <= cnt_d_b+1; 
                c_d_b     <= (cnt_d_b>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_d_b, `DRAM_RBLOCKS, (phase_b==1));
              end
              16'h1000: begin
                d_initadr <= mux32(radr_e_b, (radr_e_b | (`SORT_ELM>>1)), phase_b[0]);
                radr_e_b  <= mux32((radr_e_b+(r_block_e_b<<3)), radr_e_b+(`D_RS), (phase_b==1)); 
                cnt_e_b   <= cnt_e_b+1; 
                c_e_b     <= (cnt_e_b>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_e_b, `DRAM_RBLOCKS, (phase_b==1));
              end
              16'h2000: begin
                d_initadr <= mux32(radr_f_b, (radr_f_b | (`SORT_ELM>>1)), phase_b[0]);
                radr_f_b  <= mux32((radr_f_b+(r_block_f_b<<3)), radr_f_b+(`D_RS), (phase_b==1)); 
                cnt_f_b   <= cnt_f_b+1; 
                c_f_b     <= (cnt_f_b>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_f_b, `DRAM_RBLOCKS, (phase_b==1));
              end
              16'h4000: begin
                d_initadr <= mux32(radr_g_b, (radr_g_b | (`SORT_ELM>>1)), phase_b[0]);
                radr_g_b  <= mux32((radr_g_b+(r_block_g_b<<3)), radr_g_b+(`D_RS), (phase_b==1)); 
                cnt_g_b   <= cnt_g_b+1; 
                c_g_b     <= (cnt_g_b>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_g_b, `DRAM_RBLOCKS, (phase_b==1));
              end
              16'h8000: begin
                d_initadr <= mux32(radr_h_b, (radr_h_b | (`SORT_ELM>>1)), phase_b[0]);
                radr_h_b  <= mux32((radr_h_b+(r_block_h_b<<3)), radr_h_b+(`D_RS), (phase_b==1)); 
                cnt_h_b   <= cnt_h_b+1; 
                c_h_b     <= (cnt_h_b>=`WAYP_CN_);
                d_blocks  <= mux32(r_block_h_b, `DRAM_RBLOCKS, (phase_b==1));
              end
            endcase
            d_req  <= `DRAM_REQ_READ;
            req_ta <= req_a;
            req_tb <= req_b;
          end
        end
        7: begin ///// WRITE data to DRAM
          if (d_req!=0) begin d_req<=0; state<=4; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;     //
            d_blocks  <= w_block_a;       //
            d_initadr <= w_addr_a;            //
            w_addr_a  <= w_addr_a + (w_block_a<<3);  // address for the next write
          end
        end
        8: begin ///// WRITE data to DRAM
          if (d_req!=0) begin d_req<=0; state<=4; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;     //
            d_blocks  <= w_block_b;       //
            d_initadr <= w_addr_b;            //
            w_addr_b  <= w_addr_b + (w_block_b<<3);  // address for the next write
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
      endcase
    end
  end

  
  /**********************************************************************************************/    
  always @(posedge CLK) begin
    
    // Stage 0
    ////////////////////////////////////
    dout_ta <= mux512(dc_dout, stnet_dout, phase_zero);
    dout_tb <= mux512(dc_dout, stnet_dout, phase_zero);
    
    doen_ta <= mux1(dc_cout[0], stnet_douten, phase_zero);
    doen_tb <= mux1(dc_cout[0], stnet_douten, phase_zero);
    
    req_taa <= dc_cout[`SORT_WAY:1];
    req_tba <= dc_cout[`SORT_WAY*2:`SORT_WAY+1];

    // Stage 1
    ////////////////////////////////////
    dout_tc <= dout_ta;
    dout_td <= dout_ta;
    dout_te <= dout_tb;
    dout_tf <= dout_tb;

    doen_tc <= doen_ta;
    doen_td <= doen_ta;
    doen_te <= doen_tb;
    doen_tf <= doen_tb;
    
    req_tab <= req_taa;
    req_tbb <= req_tba;
  end

  // for phase
  // ###########################################################################
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
  
  // for plast
  // ###########################################################################
  always @(posedge CLK) begin
    if (RSTa) begin
      plast_a <= 0;
    end else begin
      if (phase_a==`LAST_PHASE-1) plast_a <= 1;
    end
  end
  always @(posedge CLK) begin
    if (RSTb) begin
      plast_b <= 0;
    end else begin
      if (phase_b==`LAST_PHASE-1) plast_b <= 1;
    end
  end
  
  // for elem
  // ###########################################################################
  reg c_valid_a;  // check whether the data is compressed or not
  always @(posedge CLK) c_valid_a <= (OB_dot_a[`DRAMW-1:`DRAMW-33] == {32'b0,1'b1});
  always @(posedge CLK) elem_en_a <= OB_doten_a;
  
  always @(posedge CLK) begin
    if (RSTa) begin
      elem_a <= 0;
    end else begin
      case ({elem_en_a, (elem_a==`SRTP_ELM)})
        2'b01: elem_a <= 0;
        2'b10: elem_a <= mux32(elem_a+16, elem_a+32, c_valid_a);
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
  always @(posedge CLK) begin
    if (RSTa) begin
      elem_plast_a <= 0;
    end else begin
      case ({elem_en_a, (elem_plast_a==(`SORT_ELM>>(`P_LOG+`WAY_LOG))*2)})
        2'b01: elem_plast_a <= 0;
        2'b10: elem_plast_a <= mux32(elem_plast_a+16, elem_plast_a+32, c_valid_a);
      endcase
    end
  end

  reg c_valid_b;  // check whether the data is compressed or not
  always @(posedge CLK) c_valid_b <= (OB_dot_b[`DRAMW-1:`DRAMW-33] == {32'b0,1'b1});
  always @(posedge CLK) elem_en_b <= OB_doten_b;
  
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
  always @(posedge CLK) begin
    if (RSTb) begin
      elem_plast_b <= 0;
    end else begin
      case ({elem_en_b, (elem_plast_b==(`SORT_ELM>>(`P_LOG+`WAY_LOG))*2)})
        2'b01: elem_plast_b <= 0;
        2'b10: elem_plast_b <= mux32(elem_plast_b+16, elem_plast_b+32, c_valid_b);
      endcase
    end
  end
  
  // for iter_done
  // ###########################################################################
  always @(posedge CLK) iter_done_a <= (ecnt_a==8);
  always @(posedge CLK) iter_done_b <= (ecnt_b==8);
  
  // for pchange
  // ###########################################################################
  always @(posedge CLK) pchange_a <= (elem_a==`SRTP_ELM);
  always @(posedge CLK) pchange_b <= (elem_b==`SRTP_ELM);
  
  // for irst
  // ###########################################################################
  always @(posedge CLK) irst_a <= (ecnt_a==8) || pchange_a;
  always @(posedge CLK) irst_b <= (ecnt_b==8) || pchange_b;
  
  // for frst
  // ###########################################################################
  always @(posedge CLK) frst_a <= RSTa || (ecnt_a==8) || (elem_a==`SRTP_ELM);
  always @(posedge CLK) frst_b <= RSTb || (ecnt_b==8) || (elem_b==`SRTP_ELM);
  
  // for ecnt
  // ###########################################################################
  always @(posedge CLK) begin
    if (RSTa || iter_done_a || pchange_a) begin
      ecnt_a <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase_a * `WAY_LOG));
    end else begin
      if (ecnt_a!=0 && F01_deq_a) ecnt_a <= ecnt_a - 4;
    end
  end
  always @(posedge CLK) begin
    if (RSTb || iter_done_b || pchange_b) begin
      ecnt_b <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase_b * `WAY_LOG));
    end else begin
      if (ecnt_b!=0 && F01_deq_b) ecnt_b <= ecnt_b - 4;
    end
  end
  
  // for phase zero
  // ###########################################################################
  always @(posedge CLK) phase_zero <= ((phase_a == 0) || (phase_b == 0));
  
  // for last phase
  // ###########################################################################
  always @(posedge CLK) last_phase <= ((phase_a == `LAST_PHASE) && (phase_b == `LAST_PHASE));
  
  // for debug
  // ###########################################################################
  // (* mark_debug = "true" *) reg [31:0] dcnt;
  // always @(posedge CLK) begin
  //   if (RST) begin
  //     dcnt <= 0;
  //   end else begin
  //     case ({F01_deq, (dcnt==`SORT_ELM)})
  //       2'b01: dcnt <= 0;
  //       2'b10: dcnt <= dcnt + 4;
  //     endcase
  //   end
  // end
  
endmodule // CORE
/**************************************************************************************************/

`default_nettype wire
