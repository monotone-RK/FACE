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
  
  wire [`MERGW-1:0] d00, d01, d02, d03, d04, d05, d06, d07, d08, d09, d10, d11, d12, d13, d14, d15;
  assign {d00, d01, d02, d03, d04, d05, d06, d07, d08, d09, d10, d11, d12, d13, d14, d15} = s_din;
  
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
  wire F16_enq, F16_deq, F16_emp, F16_full; wire [`MERGW-1:0] F16_din, F16_dot; wire [1:0] F16_cnt;
  wire F17_enq, F17_deq, F17_emp, F17_full; wire [`MERGW-1:0] F17_din, F17_dot; wire [1:0] F17_cnt;
  wire F18_enq, F18_deq, F18_emp, F18_full; wire [`MERGW-1:0] F18_din, F18_dot; wire [1:0] F18_cnt;
  wire F19_enq, F19_deq, F19_emp, F19_full; wire [`MERGW-1:0] F19_din, F19_dot; wire [1:0] F19_cnt;
  wire F20_enq, F20_deq, F20_emp, F20_full; wire [`MERGW-1:0] F20_din, F20_dot; wire [1:0] F20_cnt;
  wire F21_enq, F21_deq, F21_emp, F21_full; wire [`MERGW-1:0] F21_din, F21_dot; wire [1:0] F21_cnt;
  wire F22_enq, F22_deq, F22_emp, F22_full; wire [`MERGW-1:0] F22_din, F22_dot; wire [1:0] F22_cnt;
  wire F23_enq, F23_deq, F23_emp, F23_full; wire [`MERGW-1:0] F23_din, F23_dot; wire [1:0] F23_cnt;
  wire F24_enq, F24_deq, F24_emp, F24_full; wire [`MERGW-1:0] F24_din, F24_dot; wire [1:0] F24_cnt;
  wire F25_enq, F25_deq, F25_emp, F25_full; wire [`MERGW-1:0] F25_din, F25_dot; wire [1:0] F25_cnt;
  wire F26_enq, F26_deq, F26_emp, F26_full; wire [`MERGW-1:0] F26_din, F26_dot; wire [1:0] F26_cnt;
  wire F27_enq, F27_deq, F27_emp, F27_full; wire [`MERGW-1:0] F27_din, F27_dot; wire [1:0] F27_cnt;
  wire F28_enq, F28_deq, F28_emp, F28_full; wire [`MERGW-1:0] F28_din, F28_dot; wire [1:0] F28_cnt;
  wire F29_enq, F29_deq, F29_emp, F29_full; wire [`MERGW-1:0] F29_din, F29_dot; wire [1:0] F29_cnt;
  wire F30_enq, F30_deq, F30_emp, F30_full; wire [`MERGW-1:0] F30_din, F30_dot; wire [1:0] F30_cnt;
  wire F31_enq, F31_deq, F31_emp, F31_full; wire [`MERGW-1:0] F31_din, F31_dot; wire [1:0] F31_cnt;

  INBUF IN16(CLK, RST, full[0],  F16_full, F16_enq, d00, F16_din, enq[0],  phase, irst);
  INBUF IN17(CLK, RST, full[1],  F17_full, F17_enq, d01, F17_din, enq[1],  phase, irst);
  INBUF IN18(CLK, RST, full[2],  F18_full, F18_enq, d02, F18_din, enq[2],  phase, irst);
  INBUF IN19(CLK, RST, full[3],  F19_full, F19_enq, d03, F19_din, enq[3],  phase, irst);
  INBUF IN20(CLK, RST, full[4],  F20_full, F20_enq, d04, F20_din, enq[4],  phase, irst);
  INBUF IN21(CLK, RST, full[5],  F21_full, F21_enq, d05, F21_din, enq[5],  phase, irst);
  INBUF IN22(CLK, RST, full[6],  F22_full, F22_enq, d06, F22_din, enq[6],  phase, irst);
  INBUF IN23(CLK, RST, full[7],  F23_full, F23_enq, d07, F23_din, enq[7],  phase, irst);
  INBUF IN24(CLK, RST, full[8],  F24_full, F24_enq, d08, F24_din, enq[8],  phase, irst);
  INBUF IN25(CLK, RST, full[9],  F25_full, F25_enq, d09, F25_din, enq[9],  phase, irst);
  INBUF IN26(CLK, RST, full[10], F26_full, F26_enq, d10, F26_din, enq[10], phase, irst);
  INBUF IN27(CLK, RST, full[11], F27_full, F27_enq, d11, F27_din, enq[11], phase, irst);
  INBUF IN28(CLK, RST, full[12], F28_full, F28_enq, d12, F28_din, enq[12], phase, irst);
  INBUF IN29(CLK, RST, full[13], F29_full, F29_enq, d13, F29_din, enq[13], phase, irst);
  INBUF IN30(CLK, RST, full[14], F30_full, F30_enq, d14, F30_din, enq[14], phase, irst);
  INBUF IN31(CLK, RST, full[15], F31_full, F31_enq, d15, F31_din, enq[15], phase, irst);

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
  MRE2 #(1, `MERGW) F16(CLK, frst, F16_enq, F16_deq, F16_din, F16_dot, F16_emp, F16_full, F16_cnt);
  MRE2 #(1, `MERGW) F17(CLK, frst, F17_enq, F17_deq, F17_din, F17_dot, F17_emp, F17_full, F17_cnt);
  MRE2 #(1, `MERGW) F18(CLK, frst, F18_enq, F18_deq, F18_din, F18_dot, F18_emp, F18_full, F18_cnt);
  MRE2 #(1, `MERGW) F19(CLK, frst, F19_enq, F19_deq, F19_din, F19_dot, F19_emp, F19_full, F19_cnt);
  MRE2 #(1, `MERGW) F20(CLK, frst, F20_enq, F20_deq, F20_din, F20_dot, F20_emp, F20_full, F20_cnt);
  MRE2 #(1, `MERGW) F21(CLK, frst, F21_enq, F21_deq, F21_din, F21_dot, F21_emp, F21_full, F21_cnt);
  MRE2 #(1, `MERGW) F22(CLK, frst, F22_enq, F22_deq, F22_din, F22_dot, F22_emp, F22_full, F22_cnt);
  MRE2 #(1, `MERGW) F23(CLK, frst, F23_enq, F23_deq, F23_din, F23_dot, F23_emp, F23_full, F23_cnt);
  MRE2 #(1, `MERGW) F24(CLK, frst, F24_enq, F24_deq, F24_din, F24_dot, F24_emp, F24_full, F24_cnt);
  MRE2 #(1, `MERGW) F25(CLK, frst, F25_enq, F25_deq, F25_din, F25_dot, F25_emp, F25_full, F25_cnt);
  MRE2 #(1, `MERGW) F26(CLK, frst, F26_enq, F26_deq, F26_din, F26_dot, F26_emp, F26_full, F26_cnt);
  MRE2 #(1, `MERGW) F27(CLK, frst, F27_enq, F27_deq, F27_din, F27_dot, F27_emp, F27_full, F27_cnt);
  MRE2 #(1, `MERGW) F28(CLK, frst, F28_enq, F28_deq, F28_din, F28_dot, F28_emp, F28_full, F28_cnt);
  MRE2 #(1, `MERGW) F29(CLK, frst, F29_enq, F29_deq, F29_din, F29_dot, F29_emp, F29_full, F29_cnt);
  MRE2 #(1, `MERGW) F30(CLK, frst, F30_enq, F30_deq, F30_din, F30_dot, F30_emp, F30_full, F30_cnt);
  MRE2 #(1, `MERGW) F31(CLK, frst, F31_enq, F31_deq, F31_din, F31_dot, F31_emp, F31_full, F31_cnt);

  SCELL #(`SORTW, `M_LOG) S01(CLK, frst, !F02_emp, !F03_emp, F02_deq, F03_deq, F02_dot, F03_dot, F01_full, F01_din, F01_enq);
  SCELL #(`SORTW, `M_LOG) S02(CLK, frst, !F04_emp, !F05_emp, F04_deq, F05_deq, F04_dot, F05_dot, F02_full, F02_din, F02_enq);
  SCELL #(`SORTW, `M_LOG) S03(CLK, frst, !F06_emp, !F07_emp, F06_deq, F07_deq, F06_dot, F07_dot, F03_full, F03_din, F03_enq);
  SCELL #(`SORTW, `M_LOG) S04(CLK, frst, !F08_emp, !F09_emp, F08_deq, F09_deq, F08_dot, F09_dot, F04_full, F04_din, F04_enq);
  SCELL #(`SORTW, `M_LOG) S05(CLK, frst, !F10_emp, !F11_emp, F10_deq, F11_deq, F10_dot, F11_dot, F05_full, F05_din, F05_enq);
  SCELL #(`SORTW, `M_LOG) S06(CLK, frst, !F12_emp, !F13_emp, F12_deq, F13_deq, F12_dot, F13_dot, F06_full, F06_din, F06_enq);
  SCELL #(`SORTW, `M_LOG) S07(CLK, frst, !F14_emp, !F15_emp, F14_deq, F15_deq, F14_dot, F15_dot, F07_full, F07_din, F07_enq);
  SCELL #(`SORTW, `M_LOG) S08(CLK, frst, !F16_emp, !F17_emp, F16_deq, F17_deq, F16_dot, F17_dot, F08_full, F08_din, F08_enq);
  SCELL #(`SORTW, `M_LOG) S09(CLK, frst, !F18_emp, !F19_emp, F18_deq, F19_deq, F18_dot, F19_dot, F09_full, F09_din, F09_enq);
  SCELL #(`SORTW, `M_LOG) S10(CLK, frst, !F20_emp, !F21_emp, F20_deq, F21_deq, F20_dot, F21_dot, F10_full, F10_din, F10_enq);
  SCELL #(`SORTW, `M_LOG) S11(CLK, frst, !F22_emp, !F23_emp, F22_deq, F23_deq, F22_dot, F23_dot, F11_full, F11_din, F11_enq);
  SCELL #(`SORTW, `M_LOG) S12(CLK, frst, !F24_emp, !F25_emp, F24_deq, F25_deq, F24_dot, F25_dot, F12_full, F12_din, F12_enq);
  SCELL #(`SORTW, `M_LOG) S13(CLK, frst, !F26_emp, !F27_emp, F26_deq, F27_deq, F26_dot, F27_dot, F13_full, F13_din, F13_enq);
  SCELL #(`SORTW, `M_LOG) S14(CLK, frst, !F28_emp, !F29_emp, F28_deq, F29_deq, F28_dot, F29_dot, F14_full, F14_din, F14_enq);
  SCELL #(`SORTW, `M_LOG) S15(CLK, frst, !F30_emp, !F31_emp, F30_deq, F31_deq, F30_dot, F31_dot, F15_full, F15_din, F15_enq);
  
  assign F01_deq = deq;
  assign dot = F01_dot;
  assign emp = F01_emp;

endmodule

/***** Output Module                                                                          *****/
/**************************************************************************************************/
module OTMOD(input  wire              CLK, 
             input  wire              RST, 
             input  wire              F01_deq, 
             input  wire [`MERGW-1:0] F01_dot, 
             input  wire              OB_deq, 
             output wire [`DRAMW-1:0] OB_dot, 
             output wire              OB_full, 
             output reg               OB_req);
  
  reg [1:0]        ob_buf_t_cnt; // counter for temporary register
  reg              ob_enque;
  reg [`DRAMW-1:0] ob_buf_t;
  wire [`DRAMW-1:0] OB_din = ob_buf_t;
  wire              OB_enq = ob_enque;
  wire [`OB_SIZE:0] OB_cnt;
  
  always @(posedge CLK) OB_req <= (OB_cnt>=`DRAM_WBLOCKS);
  
  always @(posedge CLK) begin
    if (F01_deq) ob_buf_t <= {F01_dot, ob_buf_t[`DRAMW-1:`MERGW]};
  end
  always @(posedge CLK) begin
    if (RST) begin
      ob_buf_t_cnt <= 0;
    end else begin
      if (F01_deq) ob_buf_t_cnt <= ob_buf_t_cnt + 1;
    end
  end
  always @(posedge CLK) ob_enque <= (F01_deq && ob_buf_t_cnt == 3);
  
  BFIFO #(`OB_SIZE, `DRAMW) OB(.CLK(CLK), .RST(RST), .enq(OB_enq), .deq(OB_deq), 
                               .din(OB_din), .dot(OB_dot), .full(OB_full), .cnt(OB_cnt));
endmodule

/***** Sorting Network                                                                        *****/
/**************************************************************************************************/
module SORTINGNETWORK(input  wire               CLK,
                      input  wire               RST_IN,
                      input  wire [`SORT_WAY:0] DATAEN_IN,
                      input  wire [511:0]       DIN_T,
                      output reg  [511:0]       DOUT,
                      output reg  [`SORT_WAY:0] DATAEN_OUT);

  reg               RST;
  reg [511:0]       DIN;
  reg [`SORT_WAY:0] DATAEN;
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

  reg [511:0]       pdA; // pipeline regester A for data
  reg [`SORT_WAY:0] pcA; // pipeline regester A for control
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
  
  reg [511:0]       pdB; // pipeline regester B for data
  reg [`SORT_WAY:0] pcB; // pipeline regester B for control
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

  reg [511:0]       pdC; // pipeline regester C for data
  reg [`SORT_WAY:0] pcC; // pipeline regester C for control
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
  
  reg [511:0]       pdD; // pipeline regester D for data
  reg [`SORT_WAY:0] pcD; // pipeline regester D for control
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

  reg [511:0]       pdE; // pipeline regester E for data
  reg [`SORT_WAY:0] pcE; // pipeline regester E for control
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
  
  reg [511:0]       pdF; // pipeline regester F for data
  reg [`SORT_WAY:0] pcF; // pipeline regester F for control
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

  reg [511:0]       pdG; // pipeline regester G for data
  reg [`SORT_WAY:0] pcG; // pipeline regester G for control
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

  reg [511:0]       pdH; // pipeline regester H for data
  reg [`SORT_WAY:0] pcH; // pipeline regester H for control
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

  reg [511:0]       pdI; // pipeline regester I for data
  reg [`SORT_WAY:0] pcI; // pipeline regester I for control
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
  
  function [16-1:0] mux16;
    input [16-1:0] a;
    input [16-1:0] b;
    input          sel;
    begin
      case (sel)
        1'b0: mux16 = a;
        1'b1: mux16 = b;
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
  wire [`DRAMW-1:0]   OB_dot;
  wire                OB_req;
  wire                OB_full;
  
  assign d_din = OB_dot;

  (* mark_debug = "true" *) reg [`DRAMW-1:0] dout_t;
  reg [`DRAMW-1:0]    dout_ta;
  reg [`DRAMW-1:0]    dout_tb;
  reg [`DRAMW-1:0]    dout_tc;
  reg [`DRAMW-1:0]    dout_td;
  reg [`DRAMW-1:0]    dout_te;
  reg [`DRAMW-1:0]    dout_tf;
  
  (* mark_debug = "true" *) reg doen_t;
  reg                 doen_ta; 
  reg                 doen_tb; //
  reg                 doen_tc; //
  reg                 doen_td; //
  reg                 doen_te; //
  reg                 doen_tf; //
  
  reg [`SORT_WAY-1:0] req;     // use n-bit for n-way sorting, data read request from ways
  (* mark_debug = "true" *) reg [`SORT_WAY-1:0] req_t;   //
  reg [`SORT_WAY-1:0] req_ta;  //
  reg [`SORT_WAY-1:0] req_tb;  //
  reg [`SORT_WAY-1:0] req_pzero;
     
  wire [`SORT_WAY-1:0] im_req;

  wire [`SORT_WAY-1:0] rxw;
  
  reg [31:0]          elem;       // sorted elements in a phase
  (* mark_debug = "true" *) reg [`PHASE_W]      phase;      //
  reg                 pchange;    // phase_change to reset some registers
  reg                 iter_done;  //
  reg [31:0]          ecnt;       // sorted elements in an iteration
  reg                 irst;       // INBUF reset
  reg                 frst;       // sort-tree FIFO reset
  
  reg                 phase_zero;
  reg                 last_phase;
     
  reg RST;
  always @(posedge CLK) RST <= RST_IN;

  /**********************************************************************************************/
  wire [`MERGW-1:0] d00, d01, d02, d03, d04, d05, d06, d07, d08, d09, d10, d11, d12, d13, d14, d15;
  wire [1:0] ib00_req, ib01_req, ib02_req, ib03_req, ib04_req, ib05_req, ib06_req, ib07_req, ib08_req, ib09_req, ib10_req, ib11_req, ib12_req, ib13_req, ib14_req, ib15_req;

  (* mark_debug = "true" *) wire       rsltbuf_enq;
  (* mark_debug = "true" *) wire       rsltbuf_deq;
                            wire       rsltbuf_emp;
                            wire       rsltbuf_ful;
  (* mark_debug = "true" *) wire [4:0] rsltbuf_cnt;

  wire F01_emp;
  wire F01_deq = mux1((~|{F01_emp,OB_full}), (~|{F01_emp,rsltbuf_ful}), last_phase);
  
  (* mark_debug = "true" *) wire [`MERGW-1:0] F01_dot;
  wire [`MERGW*`SORT_WAY-1:0] s_din = {d00, d01, d02, d03, d04, d05, d06, d07, d08, d09, d10, d11, d12, d13, d14, d15};
  wire [`SORT_WAY-1:0] enq;
  wire [`SORT_WAY-1:0] s_ful;

  
  wire [`DRAMW-1:0]  stnet_dout;
  wire [`SORT_WAY:0] stnet_douten;
  SORTINGNETWORK sortingnetwork(CLK, RST, {req_t, doen_t}, dout_t, stnet_dout, stnet_douten);
  
  always @(posedge CLK) begin
    if      (RST)     req_pzero <= 1;
    else if (doen_tc) req_pzero <= {req_pzero[`SORT_WAY-2:0],req_pzero[`SORT_WAY-1]};
  end

  assign im_req = mux16(req_tb, req_pzero, phase_zero);

  INMOD2 im00(CLK, RST, dout_tc, doen_tc & im_req[0],  s_ful[0],  rxw[0],  d00, enq[0],  ib00_req);
  INMOD2 im01(CLK, RST, dout_tc, doen_tc & im_req[1],  s_ful[1],  rxw[1],  d01, enq[1],  ib01_req);
  INMOD2 im02(CLK, RST, dout_tc, doen_tc & im_req[2],  s_ful[2],  rxw[2],  d02, enq[2],  ib02_req);
  INMOD2 im03(CLK, RST, dout_tc, doen_tc & im_req[3],  s_ful[3],  rxw[3],  d03, enq[3],  ib03_req);
  INMOD2 im04(CLK, RST, dout_td, doen_td & im_req[4],  s_ful[4],  rxw[4],  d04, enq[4],  ib04_req);
  INMOD2 im05(CLK, RST, dout_td, doen_td & im_req[5],  s_ful[5],  rxw[5],  d05, enq[5],  ib05_req);
  INMOD2 im06(CLK, RST, dout_td, doen_td & im_req[6],  s_ful[6],  rxw[6],  d06, enq[6],  ib06_req);
  INMOD2 im07(CLK, RST, dout_td, doen_td & im_req[7],  s_ful[7],  rxw[7],  d07, enq[7],  ib07_req);
  INMOD2 im08(CLK, RST, dout_te, doen_te & im_req[8],  s_ful[8],  rxw[8],  d08, enq[8],  ib08_req);
  INMOD2 im09(CLK, RST, dout_te, doen_te & im_req[9],  s_ful[9],  rxw[9],  d09, enq[9],  ib09_req);
  INMOD2 im10(CLK, RST, dout_te, doen_te & im_req[10], s_ful[10], rxw[10], d10, enq[10], ib10_req);
  INMOD2 im11(CLK, RST, dout_te, doen_te & im_req[11], s_ful[11], rxw[11], d11, enq[11], ib11_req);
  INMOD2 im12(CLK, RST, dout_tf, doen_tf & im_req[12], s_ful[12], rxw[12], d12, enq[12], ib12_req);
  INMOD2 im13(CLK, RST, dout_tf, doen_tf & im_req[13], s_ful[13], rxw[13], d13, enq[13], ib13_req);
  INMOD2 im14(CLK, RST, dout_tf, doen_tf & im_req[14], s_ful[14], rxw[14], d14, enq[14], ib14_req);
  INMOD2 im15(CLK, RST, dout_tf, doen_tf & im_req[15], s_ful[15], rxw[15], d15, enq[15], ib15_req);

  assign rx_wait = |rxw;
                  
  STREE stree(CLK, RST, irst, frst, phase, s_din, enq, s_ful, F01_deq, F01_dot, F01_emp);

  wire OB_deq = d_w;
  OTMOD ob(CLK, RST, (!last_phase && F01_deq), F01_dot, OB_deq, OB_dot, OB_full, OB_req);
  
  assign rsltbuf_enq = last_phase && F01_deq;
  assign rsltbuf_deq = chnl_tx_data_ren && chnl_tx_data_valid;
  SRL_FIFO #(4, `MERGW) rsltbuf(CLK, RST, rsltbuf_enq, rsltbuf_deq, F01_dot, 
                                rslt, rsltbuf_emp, rsltbuf_ful, rsltbuf_cnt);
  assign rslt_ready = !rsltbuf_emp;
  

  /***** dram READ/WRITE controller                                                         *****/
  /**********************************************************************************************/
  reg [31:0] w_addr; // 
  reg [2:0]  state;  // state

  reg [31:0] radr_a, radr_b, radr_c, radr_d, radr_e, radr_f, radr_g, radr_h, radr_i, radr_j, radr_k, radr_l, radr_m, radr_n, radr_o, radr_p;
  (* mark_debug = "true" *) reg [27:0] cnt_a, cnt_b, cnt_c, cnt_d, cnt_e, cnt_f, cnt_g, cnt_h, cnt_i, cnt_j, cnt_k, cnt_l, cnt_m, cnt_n, cnt_o, cnt_p;
  reg        c_a, c_b, c_c, c_d, c_e, c_f, c_g, c_h, c_i, c_j, c_k, c_l, c_m, c_n, c_o, c_p;   // counter is full ?

  always @(posedge CLK) begin
    if (RST || pchange) begin 
      if (RST) state <= 0;
      if (RST) {d_req, d_initadr, d_blocks} <= 0;
      
      req <= 0;
      w_addr <= mux32((`SORT_ELM>>1), 0, phase[0]);
      radr_a <= ((`SELM_PER_WAY>>3)*0);
      radr_b <= ((`SELM_PER_WAY>>3)*1);
      radr_c <= ((`SELM_PER_WAY>>3)*2);
      radr_d <= ((`SELM_PER_WAY>>3)*3);
      radr_e <= ((`SELM_PER_WAY>>3)*4);
      radr_f <= ((`SELM_PER_WAY>>3)*5);
      radr_g <= ((`SELM_PER_WAY>>3)*6);
      radr_h <= ((`SELM_PER_WAY>>3)*7);
      radr_i <= ((`SELM_PER_WAY>>3)*8);
      radr_j <= ((`SELM_PER_WAY>>3)*9);
      radr_k <= ((`SELM_PER_WAY>>3)*10);
      radr_l <= ((`SELM_PER_WAY>>3)*11);
      radr_m <= ((`SELM_PER_WAY>>3)*12);
      radr_n <= ((`SELM_PER_WAY>>3)*13);
      radr_o <= ((`SELM_PER_WAY>>3)*14);
      radr_p <= ((`SELM_PER_WAY>>3)*15);
      {cnt_a, cnt_b, cnt_c, cnt_d, cnt_e, cnt_f, cnt_g, cnt_h, cnt_i, cnt_j, cnt_k, cnt_l, cnt_m, cnt_n, cnt_o, cnt_p} <= 0;
      {c_a, c_b, c_c, c_d, c_e, c_f, c_g, c_h, c_i, c_j, c_k, c_l, c_m, c_n, c_o, c_p} <= 0;
    end else begin
      case (state)
        ////////////////////////////////////////////////////////////////////////////////////////
        0: begin ///// Initialize memory, write data to DRAM
          state <= !(phase_zero);
          
          if (d_req != 0) d_req <= 0; 
          else if (!d_busy) begin
            if (OB_req) begin
              d_req     <= `DRAM_REQ_WRITE;   //
              d_blocks  <= `DRAM_WBLOCKS;     //
              d_initadr <= w_addr;            //
              w_addr    <= w_addr + (`D_WS);  // address for the next write
            end
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        1: begin ///// request arbitration 
          if (!d_busy) begin
            if      (ib00_req[1] && !c_a) begin req<=16'h0001; state<=3; end // first priority
            else if (ib01_req[1] && !c_b) begin req<=16'h0002; state<=3; end // 
            else if (ib02_req[1] && !c_c) begin req<=16'h0004; state<=3; end // 
            else if (ib03_req[1] && !c_d) begin req<=16'h0008; state<=3; end // 
            else if (ib04_req[1] && !c_e) begin req<=16'h0010; state<=3; end // 
            else if (ib05_req[1] && !c_f) begin req<=16'h0020; state<=3; end // 
            else if (ib06_req[1] && !c_g) begin req<=16'h0040; state<=3; end // 
            else if (ib07_req[1] && !c_h) begin req<=16'h0080; state<=3; end //
            else if (ib08_req[1] && !c_i) begin req<=16'h0100; state<=3; end // 
            else if (ib09_req[1] && !c_j) begin req<=16'h0200; state<=3; end // 
            else if (ib10_req[1] && !c_k) begin req<=16'h0400; state<=3; end // 
            else if (ib11_req[1] && !c_l) begin req<=16'h0800; state<=3; end // 
            else if (ib12_req[1] && !c_m) begin req<=16'h1000; state<=3; end // 
            else if (ib13_req[1] && !c_n) begin req<=16'h2000; state<=3; end // 
            else if (ib14_req[1] && !c_o) begin req<=16'h4000; state<=3; end //
            else if (ib15_req[1] && !c_p) begin req<=16'h8000; state<=3; end //
            else                                               state<=2;
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        2: begin ///// request arbitration 
          if (!d_busy) begin
            if      (ib00_req[0] && !c_a) begin req<=16'h0001; state<=3; end // second priority
            else if (ib01_req[0] && !c_b) begin req<=16'h0002; state<=3; end // 
            else if (ib02_req[0] && !c_c) begin req<=16'h0004; state<=3; end // 
            else if (ib03_req[0] && !c_d) begin req<=16'h0008; state<=3; end // 
            else if (ib04_req[0] && !c_e) begin req<=16'h0010; state<=3; end // 
            else if (ib05_req[0] && !c_f) begin req<=16'h0020; state<=3; end // 
            else if (ib06_req[0] && !c_g) begin req<=16'h0040; state<=3; end // 
            else if (ib07_req[0] && !c_h) begin req<=16'h0080; state<=3; end // 
            else if (ib08_req[0] && !c_i) begin req<=16'h0100; state<=3; end // 
            else if (ib09_req[0] && !c_j) begin req<=16'h0200; state<=3; end // 
            else if (ib10_req[0] && !c_k) begin req<=16'h0400; state<=3; end // 
            else if (ib11_req[0] && !c_l) begin req<=16'h0800; state<=3; end // 
            else if (ib12_req[0] && !c_m) begin req<=16'h1000; state<=3; end // 
            else if (ib13_req[0] && !c_n) begin req<=16'h2000; state<=3; end // 
            else if (ib14_req[0] && !c_o) begin req<=16'h4000; state<=3; end //
            else if (ib15_req[0] && !c_p) begin req<=16'h8000; state<=3; end //
            else if (OB_req)              begin                state<=4; end // WRITE
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        3: begin ///// READ data from DRAM
          if (d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            case (req)
              16'h0001: begin
                d_initadr <= mux32(radr_a, (radr_a | (`SORT_ELM>>1)), phase[0]);
                radr_a    <= radr_a+(`D_RS);
                cnt_a     <= cnt_a+1;
                c_a       <= (cnt_a>=`WAY_CN_);
              end
              16'h0002: begin
                d_initadr <= mux32(radr_b, (radr_b | (`SORT_ELM>>1)), phase[0]);
                radr_b    <= radr_b+(`D_RS);
                cnt_b     <= cnt_b+1;
                c_b       <= (cnt_b>=`WAY_CN_);
              end
              16'h0004: begin
                d_initadr <= mux32(radr_c, (radr_c | (`SORT_ELM>>1)), phase[0]);
                radr_c    <= radr_c+(`D_RS);
                cnt_c     <= cnt_c+1;
                c_c       <= (cnt_c>=`WAY_CN_);
              end
              16'h0008: begin
                d_initadr <= mux32(radr_d, (radr_d | (`SORT_ELM>>1)), phase[0]);
                radr_d    <= radr_d+(`D_RS);
                cnt_d     <= cnt_d+1;
                c_d       <= (cnt_d>=`WAY_CN_);
              end
              16'h0010: begin
                d_initadr <= mux32(radr_e, (radr_e | (`SORT_ELM>>1)), phase[0]);
                radr_e    <= radr_e+(`D_RS);
                cnt_e     <= cnt_e+1;
                c_e       <= (cnt_e>=`WAY_CN_);
              end
              16'h0020: begin
                d_initadr <= mux32(radr_f, (radr_f | (`SORT_ELM>>1)), phase[0]);
                radr_f    <= radr_f+(`D_RS);
                cnt_f     <= cnt_f+1;
                c_f       <= (cnt_f>=`WAY_CN_);
              end
              16'h0040: begin
                d_initadr <= mux32(radr_g, (radr_g | (`SORT_ELM>>1)), phase[0]);
                radr_g    <= radr_g+(`D_RS);
                cnt_g     <= cnt_g+1;
                c_g       <= (cnt_g>=`WAY_CN_);                
              end
              16'h0080: begin
                d_initadr <= mux32(radr_h, (radr_h | (`SORT_ELM>>1)), phase[0]);
                radr_h    <= radr_h+(`D_RS);
                cnt_h     <= cnt_h+1;
                c_h       <= (cnt_h>=`WAY_CN_);
              end
              16'h0100: begin
                d_initadr <= mux32(radr_i, (radr_i | (`SORT_ELM>>1)), phase[0]);
                radr_i    <= radr_i+(`D_RS);
                cnt_i     <= cnt_i+1;
                c_i       <= (cnt_i>=`WAY_CN_);
              end
              16'h0200: begin
                d_initadr <= mux32(radr_j, (radr_j | (`SORT_ELM>>1)), phase[0]);
                radr_j    <= radr_j+(`D_RS);
                cnt_j     <= cnt_j+1;
                c_j       <= (cnt_j>=`WAY_CN_);
              end
              16'h0400: begin
                d_initadr <= mux32(radr_k, (radr_k | (`SORT_ELM>>1)), phase[0]);
                radr_k    <= radr_k+(`D_RS);
                cnt_k     <= cnt_k+1;
                c_k       <= (cnt_k>=`WAY_CN_);
              end
              16'h0800: begin
                d_initadr <= mux32(radr_l, (radr_l | (`SORT_ELM>>1)), phase[0]);
                radr_l    <= radr_l+(`D_RS);
                cnt_l     <= cnt_l+1;
                c_l       <= (cnt_l>=`WAY_CN_);
              end
              16'h1000: begin
                d_initadr <= mux32(radr_m, (radr_m | (`SORT_ELM>>1)), phase[0]);
                radr_m    <= radr_m+(`D_RS);
                cnt_m     <= cnt_m+1;
                c_m       <= (cnt_m>=`WAY_CN_);
              end
              16'h2000: begin
                d_initadr <= mux32(radr_n, (radr_n | (`SORT_ELM>>1)), phase[0]);
                radr_n    <= radr_n+(`D_RS);
                cnt_n     <= cnt_n+1;
                c_n       <= (cnt_n>=`WAY_CN_);
              end
              16'h4000: begin
                d_initadr <= mux32(radr_o, (radr_o | (`SORT_ELM>>1)), phase[0]);
                radr_o    <= radr_o+(`D_RS);
                cnt_o     <= cnt_o+1;
                c_o       <= (cnt_o>=`WAY_CN_);
              end
              16'h8000: begin
                d_initadr <= mux32(radr_p, (radr_p | (`SORT_ELM>>1)), phase[0]);
                radr_p    <= radr_p+(`D_RS);
                cnt_p     <= cnt_p+1;
                c_p       <= (cnt_p>=`WAY_CN_);
              end
            endcase
            d_req    <= `DRAM_REQ_READ;
            d_blocks <= `DRAM_RBLOCKS;
            req_t    <= req;
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        4: begin ///// WRITE data to DRAM
          if (d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;   //
            d_blocks  <= `DRAM_WBLOCKS;     //
            d_initadr <= w_addr;            //
            w_addr    <= w_addr + (`D_WS);  // address for the next write
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
      endcase
    end
  end

  
  /**********************************************************************************************/    
  always @(posedge CLK) begin
    dout_t <= mux512(d_dout, rx_data, phase_zero);
    doen_t <= mux1(d_douten, rx_data_valid, phase_zero);
    
    // Stage 0
    ////////////////////////////////////
    dout_ta <= stnet_dout;
    dout_tb <= stnet_dout;
    
    doen_ta <= stnet_douten[0];
    doen_tb <= stnet_douten[0];
    
    req_ta  <= stnet_douten[`SORT_WAY:1];

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
    
    req_tb  <= req_ta;
  end

  // for phase
  // ###########################################################################
  always @(posedge CLK) begin
    if (RST) begin
      phase <= 0;
    end else begin 
      if (elem==`SORT_ELM) phase <= phase + 1;
    end
  end
  
  // for elem
  // ###########################################################################
  always @(posedge CLK) begin
    if (RST) begin
      elem <= 0;
    end else begin
      case ({OB_deq, (elem==`SORT_ELM)})
        2'b01: elem <= 0;
        2'b10: elem <= elem + 16;
      endcase
    end
  end
  
  // for iter_done
  // ###########################################################################
  always @(posedge CLK) iter_done <= (ecnt==8);
  
  // for pchange
  // ###########################################################################
  always @(posedge CLK) pchange <= (elem==`SORT_ELM);
  
  // for irst
  // ###########################################################################
  always @(posedge CLK) irst <= (ecnt==8) || pchange;
  
  // for frst
  // ###########################################################################
  always @(posedge CLK) frst <= RST || (ecnt==8) || (elem==`SORT_ELM);
  
  // for ecnt
  // ###########################################################################
  always @(posedge CLK) begin
    if (RST || iter_done || pchange) begin
      ecnt <= ((`ELEMS_PER_UNIT<<`WAY_LOG) << (phase * `WAY_LOG));
    end else begin
      if (ecnt!=0 && F01_deq) ecnt <= ecnt - 4;
    end
  end
  
  // for phase zero
  // ###########################################################################
  always @(posedge CLK) phase_zero <= (phase == 0);
  
  // for last phase
  // ###########################################################################
  always @(posedge CLK) last_phase <= (phase == `LAST_PHASE);
  
  // for debug
  // ###########################################################################
  (* mark_debug = "true" *) reg [31:0] dcnt;
  always @(posedge CLK) begin
    if (RST) begin
      dcnt <= 0;
    end else begin
      case ({F01_deq, (dcnt==`SORT_ELM)})
        2'b01: dcnt <= 0;
        2'b10: dcnt <= dcnt + 4;
      endcase
    end
  end
  
endmodule // CORE
/**************************************************************************************************/

`default_nettype wire
