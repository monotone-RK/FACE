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
              input  wire [`DRAMW-1:0] din,      // input data 
              input  wire              den,      // input data enable
              input  wire              IB_full,  // the next module is full ? 
              output wire [`SORTW-1:0] dot,      // this module's data output
              output wire              IB_enq,   // the next module's enqueue signal
              output reg               im_req);  // DRAM data request
  
  wire req;
  reg  deq;
  wire [`DRAMW-1:0] im_dot;
  wire [`IB_SIZE:0] im_cnt;
  wire im_full, im_emp;
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
             input  wire                        deq, 
             output wire [`SORTW-1:0]           dot,       // output data 
             output wire                        emp);
  
  reg RST;
  always @(posedge CLK) RST <= RST_in;
  
  reg [`PHASE_W]   phase;
  always @(posedge CLK) phase <= phase_in;
  
  wire [`SORTW-1:0] d00, d01, d02, d03, d04, d05, d06, d07;
  wire [`SORTW-1:0] d08, d09, d10, d11, d12, d13, d14, d15;
  assign {d00, d01, d02, d03, d04, d05, d06, d07, d08, d09, d10, d11, d12, d13, d14, d15} = s_din;
  
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
  wire F16_enq, F16_deq, F16_emp, F16_full; wire [31:0] F16_din, F16_dot; wire [1:0] F16_cnt;
  wire F17_enq, F17_deq, F17_emp, F17_full; wire [31:0] F17_din, F17_dot; wire [1:0] F17_cnt;
  wire F18_enq, F18_deq, F18_emp, F18_full; wire [31:0] F18_din, F18_dot; wire [1:0] F18_cnt;
  wire F19_enq, F19_deq, F19_emp, F19_full; wire [31:0] F19_din, F19_dot; wire [1:0] F19_cnt;
  wire F20_enq, F20_deq, F20_emp, F20_full; wire [31:0] F20_din, F20_dot; wire [1:0] F20_cnt;
  wire F21_enq, F21_deq, F21_emp, F21_full; wire [31:0] F21_din, F21_dot; wire [1:0] F21_cnt;
  wire F22_enq, F22_deq, F22_emp, F22_full; wire [31:0] F22_din, F22_dot; wire [1:0] F22_cnt;
  wire F23_enq, F23_deq, F23_emp, F23_full; wire [31:0] F23_din, F23_dot; wire [1:0] F23_cnt;
  wire F24_enq, F24_deq, F24_emp, F24_full; wire [31:0] F24_din, F24_dot; wire [1:0] F24_cnt;
  wire F25_enq, F25_deq, F25_emp, F25_full; wire [31:0] F25_din, F25_dot; wire [1:0] F25_cnt;
  wire F26_enq, F26_deq, F26_emp, F26_full; wire [31:0] F26_din, F26_dot; wire [1:0] F26_cnt;
  wire F27_enq, F27_deq, F27_emp, F27_full; wire [31:0] F27_din, F27_dot; wire [1:0] F27_cnt;
  wire F28_enq, F28_deq, F28_emp, F28_full; wire [31:0] F28_din, F28_dot; wire [1:0] F28_cnt;
  wire F29_enq, F29_deq, F29_emp, F29_full; wire [31:0] F29_din, F29_dot; wire [1:0] F29_cnt;
  wire F30_enq, F30_deq, F30_emp, F30_full; wire [31:0] F30_din, F30_dot; wire [1:0] F30_cnt;
  wire F31_enq, F31_deq, F31_emp, F31_full; wire [31:0] F31_din, F31_dot; wire [1:0] F31_cnt;
  
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
  MRE2 #(1,32) F16(CLK, frst, F16_enq, F16_deq, F16_din, F16_dot, F16_emp, F16_full, F16_cnt);
  MRE2 #(1,32) F17(CLK, frst, F17_enq, F17_deq, F17_din, F17_dot, F17_emp, F17_full, F17_cnt);
  MRE2 #(1,32) F18(CLK, frst, F18_enq, F18_deq, F18_din, F18_dot, F18_emp, F18_full, F18_cnt);
  MRE2 #(1,32) F19(CLK, frst, F19_enq, F19_deq, F19_din, F19_dot, F19_emp, F19_full, F19_cnt);
  MRE2 #(1,32) F20(CLK, frst, F20_enq, F20_deq, F20_din, F20_dot, F20_emp, F20_full, F20_cnt);
  MRE2 #(1,32) F21(CLK, frst, F21_enq, F21_deq, F21_din, F21_dot, F21_emp, F21_full, F21_cnt);
  MRE2 #(1,32) F22(CLK, frst, F22_enq, F22_deq, F22_din, F22_dot, F22_emp, F22_full, F22_cnt);
  MRE2 #(1,32) F23(CLK, frst, F23_enq, F23_deq, F23_din, F23_dot, F23_emp, F23_full, F23_cnt);
  MRE2 #(1,32) F24(CLK, frst, F24_enq, F24_deq, F24_din, F24_dot, F24_emp, F24_full, F24_cnt);
  MRE2 #(1,32) F25(CLK, frst, F25_enq, F25_deq, F25_din, F25_dot, F25_emp, F25_full, F25_cnt);
  MRE2 #(1,32) F26(CLK, frst, F26_enq, F26_deq, F26_din, F26_dot, F26_emp, F26_full, F26_cnt);
  MRE2 #(1,32) F27(CLK, frst, F27_enq, F27_deq, F27_din, F27_dot, F27_emp, F27_full, F27_cnt);
  MRE2 #(1,32) F28(CLK, frst, F28_enq, F28_deq, F28_din, F28_dot, F28_emp, F28_full, F28_cnt);
  MRE2 #(1,32) F29(CLK, frst, F29_enq, F29_deq, F29_din, F29_dot, F29_emp, F29_full, F29_cnt);
  MRE2 #(1,32) F30(CLK, frst, F30_enq, F30_deq, F30_din, F30_dot, F30_emp, F30_full, F30_cnt);
  MRE2 #(1,32) F31(CLK, frst, F31_enq, F31_deq, F31_din, F31_dot, F31_emp, F31_full, F31_cnt);

  SCELL S01(!F02_emp, !F03_emp, F02_deq, F03_deq, F02_dot, F03_dot, F01_full, F01_din, F01_enq);
  SCELL S02(!F04_emp, !F05_emp, F04_deq, F05_deq, F04_dot, F05_dot, F02_full, F02_din, F02_enq);
  SCELL S03(!F06_emp, !F07_emp, F06_deq, F07_deq, F06_dot, F07_dot, F03_full, F03_din, F03_enq);
  SCELL S04(!F08_emp, !F09_emp, F08_deq, F09_deq, F08_dot, F09_dot, F04_full, F04_din, F04_enq);
  SCELL S05(!F10_emp, !F11_emp, F10_deq, F11_deq, F10_dot, F11_dot, F05_full, F05_din, F05_enq);
  SCELL S06(!F12_emp, !F13_emp, F12_deq, F13_deq, F12_dot, F13_dot, F06_full, F06_din, F06_enq);
  SCELL S07(!F14_emp, !F15_emp, F14_deq, F15_deq, F14_dot, F15_dot, F07_full, F07_din, F07_enq);
  SCELL S08(!F16_emp, !F17_emp, F16_deq, F17_deq, F16_dot, F17_dot, F08_full, F08_din, F08_enq);
  SCELL S09(!F18_emp, !F19_emp, F18_deq, F19_deq, F18_dot, F19_dot, F09_full, F09_din, F09_enq);
  SCELL S10(!F20_emp, !F21_emp, F20_deq, F21_deq, F20_dot, F21_dot, F10_full, F10_din, F10_enq);
  SCELL S11(!F22_emp, !F23_emp, F22_deq, F23_deq, F22_dot, F23_dot, F11_full, F11_din, F11_enq);
  SCELL S12(!F24_emp, !F25_emp, F24_deq, F25_deq, F24_dot, F25_dot, F12_full, F12_din, F12_enq);
  SCELL S13(!F26_emp, !F27_emp, F26_deq, F27_deq, F26_dot, F27_dot, F13_full, F13_din, F13_enq);
  SCELL S14(!F28_emp, !F29_emp, F28_deq, F29_deq, F28_dot, F29_dot, F14_full, F14_din, F14_enq);
  SCELL S15(!F30_emp, !F31_emp, F30_deq, F31_deq, F30_dot, F31_dot, F15_full, F15_din, F15_enq);

  assign F01_deq = deq;
  assign dot = F01_dot;
  assign emp = F01_emp;
endmodule

/***** Output Module                                                                          *****/
/**************************************************************************************************/
module OTMOD(input  wire              CLK, 
             input  wire              RST, 
             input  wire              F01_deq, 
             input  wire [`SORTW-1:0] F01_dot, 
             input  wire              OB_deq, 
             output wire [`DRAMW-1:0] OB_dot, 
             output wire              OB_full, 
             output reg               OB_req);
    
  reg [3:0]        ob_buf_t_cnt; // counter for temporary register
  reg              ob_enque;
  reg [`DRAMW-1:0] ob_buf_t;
  wire [`DRAMW-1:0] OB_din = ob_buf_t;
  wire              OB_enq = ob_enque;
  wire [`OB_SIZE:0] OB_cnt;
  
  always @(posedge CLK) OB_req <= (OB_cnt>=`DRAM_WBLOCKS);
    
  always @(posedge CLK) begin
    if (F01_deq) ob_buf_t <= {F01_dot, ob_buf_t[`DRAMW-1:32]};
  end
  always @(posedge CLK) begin
    if (RST) begin
      ob_buf_t_cnt <= 0;
    end else begin
      if (F01_deq) ob_buf_t_cnt <= ob_buf_t_cnt + 1;
    end
  end
  always @(posedge CLK) ob_enque <= (F01_deq && ob_buf_t_cnt == 15);
    
  BFIFO #(`OB_SIZE, `DRAMW) OB(.CLK(CLK), .RST(RST), .enq(OB_enq), .deq(OB_deq), 
                               .din(OB_din), .dot(OB_dot), .full(OB_full), .cnt(OB_cnt));
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
module SORTINGNETWORK(input  wire               CLK,
                      input  wire               RST_IN,
                      input  wire [`SRTP_WAY:0] DATAEN_IN,
                      input  wire [511:0]       DIN_T,
                      output reg  [511:0]       DOUT,
                      output reg  [`SRTP_WAY:0] DATAEN_OUT);

  reg               RST;
  reg [511:0]       DIN;
  reg [`SRTP_WAY:0] DATAEN;
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
  reg [`SRTP_WAY:0] pcA; // pipeline regester A for control
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
  
  reg [511:0]       pdB; // pipeline regester A for data
  reg [`SRTP_WAY:0] pcB; // pipeline regester A for control
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
  
  reg [511:0]       pdC; // pipeline regester A for data
  reg [`SRTP_WAY:0] pcC; // pipeline regester A for control
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
  
  reg [511:0]       pdD; // pipeline regester A for data
  reg [`SRTP_WAY:0] pcD; // pipeline regester A for control
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
  
  reg [511:0]       pdE; // pipeline regester A for data
  reg [`SRTP_WAY:0] pcE; // pipeline regester A for control
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
  
  reg [511:0]       pdF; // pipeline regester A for data
  reg [`SRTP_WAY:0] pcF; // pipeline regester A for control
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
  
  reg [511:0]       pdG; // pipeline regester A for data
  reg [`SRTP_WAY:0] pcG; // pipeline regester A for control
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
  
  reg [511:0]       pdH; // pipeline regester A for data
  reg [`SRTP_WAY:0] pcH; // pipeline regester A for control
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
  
  reg [511:0]       pdI; // pipeline regester A for data
  reg [`SRTP_WAY:0] pcI; // pipeline regester A for control
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
  
  function [256-1:0] mux256;
    input [256-1:0] a;
    input [256-1:0] b;
    input          sel;
    begin
      case (sel)
        1'b0: mux256 = a;
        1'b1: mux256 = b;
      endcase
    end
  endfunction
  
  function [255:0] mux2in256;
    input [255:0] a;
    input [255:0] b;
    input [1:0]   sel;
    begin
      case (sel)
        2'b01: mux2in256 = a;
        2'b10: mux2in256 = b;
      endcase
    end
  endfunction
  
  /**********************************************************************************************/
  reg idone_a;
  reg idone_b;
  
  wire [`DRAMW-1:0] OB_dot0;
  wire [`DRAMW-1:0] OB_dot1;

  wire OB_req_a;
  wire OB_req_b;
  
  wire OB_full0;
  wire OB_full1;
  
  wire [`PHASE_W] l_phase = `LAST_PHASE;
  
  reg [`DRAMW-1:0] dout_t;
  reg [`DRAMW-1:0] dout_tta, dout_ttb, dout_ttc, dout_ttd, dout_tte, dout_ttf;
  reg [`DRAMW-1:0] dout_t0_a;
  reg [`DRAMW-1:0] dout_t0_b; 
  reg [`DRAMW-1:0] dout_t0_c; 
  reg [`DRAMW-1:0] dout_t0_d; 
  reg [`DRAMW-1:0] dout_t0_e;
  reg [`DRAMW-1:0] dout_t0_f; 
  reg [`DRAMW-1:0] dout_t0_g; 
  reg [`DRAMW-1:0] dout_t0_h; 
  reg [`DRAMW-1:0] dout_t1_a, dout_t2_a, dout_t3_a, dout_t4_a, dout_t5_a, dout_t6_a, dout_t7_a, dout_t8_a;
  reg [`DRAMW-1:0] dout_t1_b, dout_t2_b, dout_t3_b, dout_t4_b, dout_t5_b, dout_t6_b, dout_t7_b, dout_t8_b;
  
  reg              doen_t;
  reg              doen_tta, doen_ttb, doen_ttc, doen_ttd, doen_tte, doen_ttf;  
  reg              doen_t0_a;
  reg              doen_t0_b;
  reg              doen_t0_c;
  reg              doen_t0_d;
  reg              doen_t0_e;
  reg              doen_t0_f;
  reg              doen_t0_g;
  reg              doen_t0_h;
  reg              doen_t1_a, doen_t2_a, doen_t3_a, doen_t4_a, doen_t5_a, doen_t6_a, doen_t7_a, doen_t8_a;
  reg              doen_t1_b, doen_t2_b, doen_t3_b, doen_t4_b, doen_t5_b, doen_t6_b, doen_t7_b, doen_t8_b;

  reg [`SORT_WAY-1:0] req_tt0_a, req_tt1_a, req_tt2_a, req_tt3_a;    
  reg [`SORT_WAY-1:0] req_tt0_b, req_tt1_b, req_tt2_b, req_tt3_b;    
  
  reg req_gga;
  reg req_ggb; 
  
  reg req_ga; 
  reg req_gb; 
  
  reg [`SORT_WAY-1:0] req_ta;     //
  reg [`SORT_WAY-1:0] req_tb;     //
  
  reg [`SORT_WAY-1:0] req;       // use n-bit for n-way sorting, data read request from ways
  reg [`SORT_WAY-1:0] req_a;       
  reg [`SORT_WAY-1:0] req_b;       
  
  reg [31:0]          elem;      // sorted elements in a phase
  reg [31:0]          elem_a;      
  reg [31:0]          elem_b;      
  
  reg [`PHASE_W]      phase;     //
  reg [`PHASE_W]      phase_a;     //
  reg [`PHASE_W]      phase_b;     //

  reg                 last_phase;
  
  reg                 pchange;   // phase_change to reset some registers
  reg                 pchange_a;   
  reg                 pchange_b;   
  
  reg                 iter_done; //
  reg                 iter_done_a; //
  reg                 iter_done_b; //
  
  reg [31:0]          ecnt;      // sorted elements in an iteration
  reg [31:0]          ecnt_a;
  reg [31:0]          ecnt_b;

  reg                 irst_a;      
  reg                 irst_b;      

  reg                 frst_a;    
  reg                 frst_b;    
  
  reg                 pexe_done_a;
  reg                 pexe_done_b;
  
  reg                 pexe_done_a_p;
  reg                 pexe_done_b_p;
  
  reg RSTa;
  always @(posedge CLK) RSTa <= RST_IN;
  reg RSTb;
  always @(posedge CLK) RSTb <= RST_IN;

  /**********************************************************************************************/
  wire [`SORTW-1:0] d00_0, d01_0, d02_0, d03_0, d04_0, d05_0, d06_0, d07_0, d08_0, d09_0, d10_0, d11_0, d12_0, d13_0, d14_0, d15_0;
  wire [`SORTW-1:0] d00_1, d01_1, d02_1, d03_1, d04_1, d05_1, d06_1, d07_1, d08_1, d09_1, d10_1, d11_1, d12_1, d13_1, d14_1, d15_1;
  
  wire ib00_req_a, ib01_req_a, ib02_req_a, ib03_req_a, ib04_req_a, ib05_req_a, ib06_req_a, ib07_req_a, ib08_req_a, ib09_req_a, ib10_req_a, ib11_req_a, ib12_req_a, ib13_req_a, ib14_req_a, ib15_req_a;
  wire ib00_req_b, ib01_req_b, ib02_req_b, ib03_req_b, ib04_req_b, ib05_req_b, ib06_req_b, ib07_req_b, ib08_req_b, ib09_req_b, ib10_req_b, ib11_req_b, ib12_req_b, ib13_req_b, ib14_req_b, ib15_req_b;

  wire F01_emp0;
  wire F01_emp1;
  
  wire F01_deq0 = !F01_emp0 && !OB_full0;
  wire F01_deq1 = !F01_emp1 && !OB_full1;
  
  wire [`SORTW-1:0] F01_dot0;
  wire [`SORTW-1:0] F01_dot1;
  
  wire [`SORTW*`SORT_WAY-1:0] s_din0 = {d00_0, d01_0, d02_0, d03_0, d04_0, d05_0, d06_0, d07_0, d08_0, d09_0, d10_0, d11_0, d12_0, d13_0, d14_0, d15_0};
  wire [`SORTW*`SORT_WAY-1:0] s_din1 = {d00_1, d01_1, d02_1, d03_1, d04_1, d05_1, d06_1, d07_1, d08_1, d09_1, d10_1, d11_1, d12_1, d13_1, d14_1, d15_1};
                              
  wire [`SORT_WAY-1:0] enq0; 
  wire [`SORT_WAY-1:0] enq1; 
  
  wire [`SORT_WAY-1:0] s_ful0;
  wire [`SORT_WAY-1:0] s_ful1;

  wire [`DRAMW-1:0]  stnet_dout;
  wire [`SRTP_WAY:0] stnet_douten;
  
  SORTINGNETWORK sortingnetwork(CLK, 
                                RSTa, 
                                {req_tb, req_ta, doen_t},
                                dout_t,
                                stnet_dout,
                                stnet_douten);
  
  INMOD2 im00_0(CLK, RSTa, dout_t1_a, doen_t1_a & req_tt3_a[0],  s_ful0[0],  d00_0, enq0[0],  ib00_req_a);
  INMOD2 im01_0(CLK, RSTa, dout_t1_a, doen_t1_a & req_tt3_a[1],  s_ful0[1],  d01_0, enq0[1],  ib01_req_a);
  INMOD2 im02_0(CLK, RSTa, dout_t2_a, doen_t2_a & req_tt3_a[2],  s_ful0[2],  d02_0, enq0[2],  ib02_req_a);
  INMOD2 im03_0(CLK, RSTa, dout_t2_a, doen_t2_a & req_tt3_a[3],  s_ful0[3],  d03_0, enq0[3],  ib03_req_a);
  INMOD2 im04_0(CLK, RSTa, dout_t3_a, doen_t3_a & req_tt3_a[4],  s_ful0[4],  d04_0, enq0[4],  ib04_req_a);
  INMOD2 im05_0(CLK, RSTa, dout_t3_a, doen_t3_a & req_tt3_a[5],  s_ful0[5],  d05_0, enq0[5],  ib05_req_a);
  INMOD2 im06_0(CLK, RSTa, dout_t4_a, doen_t4_a & req_tt3_a[6],  s_ful0[6],  d06_0, enq0[6],  ib06_req_a);
  INMOD2 im07_0(CLK, RSTa, dout_t4_a, doen_t4_a & req_tt3_a[7],  s_ful0[7],  d07_0, enq0[7],  ib07_req_a);
  INMOD2 im08_0(CLK, RSTa, dout_t5_a, doen_t5_a & req_tt3_a[8],  s_ful0[8],  d08_0, enq0[8],  ib08_req_a);
  INMOD2 im09_0(CLK, RSTa, dout_t5_a, doen_t5_a & req_tt3_a[9],  s_ful0[9],  d09_0, enq0[9],  ib09_req_a);
  INMOD2 im10_0(CLK, RSTa, dout_t6_a, doen_t6_a & req_tt3_a[10], s_ful0[10], d10_0, enq0[10], ib10_req_a);
  INMOD2 im11_0(CLK, RSTa, dout_t6_a, doen_t6_a & req_tt3_a[11], s_ful0[11], d11_0, enq0[11], ib11_req_a);
  INMOD2 im12_0(CLK, RSTa, dout_t7_a, doen_t7_a & req_tt3_a[12], s_ful0[12], d12_0, enq0[12], ib12_req_a);
  INMOD2 im13_0(CLK, RSTa, dout_t7_a, doen_t7_a & req_tt3_a[13], s_ful0[13], d13_0, enq0[13], ib13_req_a);
  INMOD2 im14_0(CLK, RSTa, dout_t8_a, doen_t8_a & req_tt3_a[14], s_ful0[14], d14_0, enq0[14], ib14_req_a);
  INMOD2 im15_0(CLK, RSTa, dout_t8_a, doen_t8_a & req_tt3_a[15], s_ful0[15], d15_0, enq0[15], ib15_req_a);

  INMOD2 im00_1(CLK, RSTb, dout_t1_b, doen_t1_b & req_tt3_b[0],  s_ful1[0],  d00_1, enq1[0],  ib00_req_b);
  INMOD2 im01_1(CLK, RSTb, dout_t1_b, doen_t1_b & req_tt3_b[1],  s_ful1[1],  d01_1, enq1[1],  ib01_req_b);
  INMOD2 im02_1(CLK, RSTb, dout_t2_b, doen_t2_b & req_tt3_b[2],  s_ful1[2],  d02_1, enq1[2],  ib02_req_b);
  INMOD2 im03_1(CLK, RSTb, dout_t2_b, doen_t2_b & req_tt3_b[3],  s_ful1[3],  d03_1, enq1[3],  ib03_req_b);
  INMOD2 im04_1(CLK, RSTb, dout_t3_b, doen_t3_b & req_tt3_b[4],  s_ful1[4],  d04_1, enq1[4],  ib04_req_b);
  INMOD2 im05_1(CLK, RSTb, dout_t3_b, doen_t3_b & req_tt3_b[5],  s_ful1[5],  d05_1, enq1[5],  ib05_req_b);
  INMOD2 im06_1(CLK, RSTb, dout_t4_b, doen_t4_b & req_tt3_b[6],  s_ful1[6],  d06_1, enq1[6],  ib06_req_b);
  INMOD2 im07_1(CLK, RSTb, dout_t4_b, doen_t4_b & req_tt3_b[7],  s_ful1[7],  d07_1, enq1[7],  ib07_req_b);
  INMOD2 im08_1(CLK, RSTb, dout_t5_b, doen_t5_b & req_tt3_b[8],  s_ful1[8],  d08_1, enq1[8],  ib08_req_b);
  INMOD2 im09_1(CLK, RSTb, dout_t5_b, doen_t5_b & req_tt3_b[9],  s_ful1[9],  d09_1, enq1[9],  ib09_req_b);
  INMOD2 im10_1(CLK, RSTb, dout_t6_b, doen_t6_b & req_tt3_b[10], s_ful1[10], d10_1, enq1[10], ib10_req_b);
  INMOD2 im11_1(CLK, RSTb, dout_t6_b, doen_t6_b & req_tt3_b[11], s_ful1[11], d11_1, enq1[11], ib11_req_b);
  INMOD2 im12_1(CLK, RSTb, dout_t7_b, doen_t7_b & req_tt3_b[12], s_ful1[12], d12_1, enq1[12], ib12_req_b);
  INMOD2 im13_1(CLK, RSTb, dout_t7_b, doen_t7_b & req_tt3_b[13], s_ful1[13], d13_1, enq1[13], ib13_req_b);
  INMOD2 im14_1(CLK, RSTb, dout_t8_b, doen_t8_b & req_tt3_b[14], s_ful1[14], d14_1, enq1[14], ib14_req_b);
  INMOD2 im15_1(CLK, RSTb, dout_t8_b, doen_t8_b & req_tt3_b[15], s_ful1[15], d15_1, enq1[15], ib15_req_b);

  STREE stree0(CLK, RSTa, irst_a, frst_a, phase_a, s_din0, enq0, s_ful0, F01_deq0, F01_dot0, F01_emp0);
  STREE stree1(CLK, RSTb, irst_b, frst_b, phase_b, s_din1, enq1, s_ful1, F01_deq1, F01_dot1, F01_emp1);

  reg  OB_deq_ta;
  reg  OB_deq_tb;
  reg  OB_deq_tc;
  reg  OB_deq_td;
  
  wire [3:0] OB_dot_sel ={OB_deq_td, OB_deq_tc, OB_deq_tb, OB_deq_ta};
  
  wire OB_deq0 = idone_a && d_w && OB_deq_ta;
  wire OB_deq1 = idone_b && d_w && OB_deq_tb;
  
  OTMOD ob0(CLK, RSTa, F01_deq0, F01_dot0, OB_deq0, OB_dot0, OB_full0, OB_req_a);
  OTMOD ob1(CLK, RSTb, F01_deq1, F01_dot1, OB_deq1, OB_dot1, OB_full1, OB_req_b);

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
  reg [31:0] w_addr; // 
  reg [31:0] w_addr_a; // 
  reg [31:0] w_addr_b; // 
  reg [2:0]  state;  // state

  reg [31:0] radr_a, radr_b, radr_c, radr_d, radr_e, radr_f, radr_g, radr_h, radr_i, radr_j, radr_k, radr_l, radr_m, radr_n, radr_o, radr_p;
  reg [31:0] radr_a_a, radr_b_a, radr_c_a, radr_d_a, radr_e_a, radr_f_a, radr_g_a, radr_h_a, radr_i_a, radr_j_a, radr_k_a, radr_l_a, radr_m_a, radr_n_a, radr_o_a, radr_p_a;
  reg [31:0] radr_a_b, radr_b_b, radr_c_b, radr_d_b, radr_e_b, radr_f_b, radr_g_b, radr_h_b, radr_i_b, radr_j_b, radr_k_b, radr_l_b, radr_m_b, radr_n_b, radr_o_b, radr_p_b;
  
  reg [27:0] cnt_a, cnt_b, cnt_c, cnt_d, cnt_e, cnt_f, cnt_g, cnt_h, cnt_i, cnt_j, cnt_k, cnt_l, cnt_m, cnt_n, cnt_o, cnt_p;
  reg [27:0] cnt_a_a, cnt_b_a, cnt_c_a, cnt_d_a, cnt_e_a, cnt_f_a, cnt_g_a, cnt_h_a, cnt_i_a, cnt_j_a, cnt_k_a, cnt_l_a, cnt_m_a, cnt_n_a, cnt_o_a, cnt_p_a;
  reg [27:0] cnt_a_b, cnt_b_b, cnt_c_b, cnt_d_b, cnt_e_b, cnt_f_b, cnt_g_b, cnt_h_b, cnt_i_b, cnt_j_b, cnt_k_b, cnt_l_b, cnt_m_b, cnt_n_b, cnt_o_b, cnt_p_b;
  
  reg        c_a, c_b, c_c, c_d, c_e, c_f, c_g, c_h, c_i, c_j, c_k, c_l, c_m, c_n, c_o, c_p;   // counter is full ?
  reg        c_a_a, c_b_a, c_c_a, c_d_a, c_e_a, c_f_a, c_g_a, c_h_a, c_i_a, c_j_a, c_k_a, c_l_a, c_m_a, c_n_a, c_o_a, c_p_a;
  reg        c_a_b, c_b_b, c_c_b, c_d_b, c_e_b, c_f_b, c_g_b, c_h_b, c_i_b, c_j_b, c_k_b, c_l_b, c_m_b, c_n_b, c_o_b, c_p_b;

  always @(posedge CLK) begin
    if (RSTa || pchange_a || pchange_b) begin // sortdone ??
      if (RSTa) {initdone, state} <= 0;
      if (RSTa) {d_req, d_initadr, d_blocks} <= 0;
      if (RSTa) {req_a, req_b} <= 0;
      if (RSTa) {req_ta, req_tb} <= 0;
      if (RSTa) {req_ga, req_gb} <= 0;
      if (RSTa) {req_gga, req_ggb} <= 0;

      req    <= 0;
      w_addr <= mux32((`SORT_ELM>>1), 0, l_phase[0]);
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

      if ((RSTa || pchange_a) && !pexe_done_a_p) begin
        w_addr_a <= mux32((`SORT_ELM>>1), 0, phase_a[0]);
        radr_a_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*0);
        radr_b_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*1);
        radr_c_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*2);
        radr_d_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*3);
        radr_e_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*4);
        radr_f_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*5);
        radr_g_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*6);
        radr_h_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*7);
        radr_i_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*8);
        radr_j_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*9);
        radr_k_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*10);
        radr_l_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*11);
        radr_m_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*12);
        radr_n_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*13);
        radr_o_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*14);
        radr_p_a <= ((`SELM_PER_WAY>>(`P_LOG+3))*15);
        {cnt_a_a, cnt_b_a, cnt_c_a, cnt_d_a, cnt_e_a, cnt_f_a, cnt_g_a, cnt_h_a, cnt_i_a, cnt_j_a, cnt_k_a, cnt_l_a, cnt_m_a, cnt_n_a, cnt_o_a, cnt_p_a} <= 0;
        {c_a_a, c_b_a, c_c_a, c_d_a, c_e_a, c_f_a, c_g_a, c_h_a, c_i_a, c_j_a, c_k_a, c_l_a, c_m_a, c_n_a, c_o_a, c_p_a} <= 0;
        OB_deq_ta <= 0;
      end
      
      if ((RSTa || pchange_b) && !pexe_done_b_p) begin
        w_addr_b <= mux32(((`SORT_ELM>>2) | (`SORT_ELM>>1)), (`SORT_ELM>>2), phase_b[0]);
        radr_a_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*0)  | (`SORT_ELM>>2);
        radr_b_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*1)  | (`SORT_ELM>>2);
        radr_c_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*2)  | (`SORT_ELM>>2);
        radr_d_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*3)  | (`SORT_ELM>>2);
        radr_e_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*4)  | (`SORT_ELM>>2);
        radr_f_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*5)  | (`SORT_ELM>>2);
        radr_g_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*6)  | (`SORT_ELM>>2);
        radr_h_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*7)  | (`SORT_ELM>>2);
        radr_i_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*8)  | (`SORT_ELM>>2);
        radr_j_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*9)  | (`SORT_ELM>>2);
        radr_k_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*10) | (`SORT_ELM>>2);
        radr_l_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*11) | (`SORT_ELM>>2);
        radr_m_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*12) | (`SORT_ELM>>2);
        radr_n_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*13) | (`SORT_ELM>>2);
        radr_o_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*14) | (`SORT_ELM>>2);
        radr_p_b <= ((`SELM_PER_WAY>>(`P_LOG+3))*15) | (`SORT_ELM>>2);
        {cnt_a_b, cnt_b_b, cnt_c_b, cnt_d_b, cnt_e_b, cnt_f_b, cnt_g_b, cnt_h_b, cnt_i_b, cnt_j_b, cnt_k_b, cnt_l_b, cnt_m_b, cnt_n_b, cnt_o_b, cnt_p_b} <= 0;
        {c_a_b, c_b_b, c_c_b, c_d_b, c_e_b, c_f_b, c_g_b, c_h_b, c_i_b, c_j_b, c_k_b, c_l_b, c_m_b, c_n_b, c_o_b, c_p_b} <= 0;
        OB_deq_tb <= 0;
      end

    end else begin
      case (state)
        ////////////////////////////////////////////////////////////////////////////////////////
        0: begin ///// Initialize memory, write data to DRAM
          if (d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy)begin
            d_req     <= `DRAM_REQ_WRITE; //
            d_blocks  <= (`SORT_ELM>>4);    // 16word/block for VC707, 2word/b for Tokuden
            d_initadr <= 0;               //
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        1: begin ///// request arbitration 
          if (!d_busy) begin
            initdone<=1;
            OB_deq_ta <= 0;
            OB_deq_tb <= 0;
            OB_deq_tc <= 0;
            OB_deq_td <= 0;
            case (last_phase)
              1'b0: begin
                if      (ib00_req_a && !c_a_a) begin req_a<=16'h0001; req_gga<=1; end //
                else if (ib01_req_a && !c_b_a) begin req_a<=16'h0002; req_gga<=1; end // 
                else if (ib02_req_a && !c_c_a) begin req_a<=16'h0004; req_gga<=1; end // 
                else if (ib03_req_a && !c_d_a) begin req_a<=16'h0008; req_gga<=1; end // 
                else if (ib04_req_a && !c_e_a) begin req_a<=16'h0010; req_gga<=1; end // 
                else if (ib05_req_a && !c_f_a) begin req_a<=16'h0020; req_gga<=1; end // 
                else if (ib06_req_a && !c_g_a) begin req_a<=16'h0040; req_gga<=1; end // 
                else if (ib07_req_a && !c_h_a) begin req_a<=16'h0080; req_gga<=1; end //
                else if (ib08_req_a && !c_i_a) begin req_a<=16'h0100; req_gga<=1; end // 
                else if (ib09_req_a && !c_j_a) begin req_a<=16'h0200; req_gga<=1; end // 
                else if (ib10_req_a && !c_k_a) begin req_a<=16'h0400; req_gga<=1; end // 
                else if (ib11_req_a && !c_l_a) begin req_a<=16'h0800; req_gga<=1; end // 
                else if (ib12_req_a && !c_m_a) begin req_a<=16'h1000; req_gga<=1; end // 
                else if (ib13_req_a && !c_n_a) begin req_a<=16'h2000; req_gga<=1; end // 
                else if (ib14_req_a && !c_o_a) begin req_a<=16'h4000; req_gga<=1; end //
                else if (ib15_req_a && !c_p_a) begin req_a<=16'h8000; req_gga<=1; end //
                
                if      (ib00_req_b && !c_a_b) begin req_b<=16'h0001; req_ggb<=1; end //
                else if (ib01_req_b && !c_b_b) begin req_b<=16'h0002; req_ggb<=1; end // 
                else if (ib02_req_b && !c_c_b) begin req_b<=16'h0004; req_ggb<=1; end // 
                else if (ib03_req_b && !c_d_b) begin req_b<=16'h0008; req_ggb<=1; end // 
                else if (ib04_req_b && !c_e_b) begin req_b<=16'h0010; req_ggb<=1; end // 
                else if (ib05_req_b && !c_f_b) begin req_b<=16'h0020; req_ggb<=1; end // 
                else if (ib06_req_b && !c_g_b) begin req_b<=16'h0040; req_ggb<=1; end // 
                else if (ib07_req_b && !c_h_b) begin req_b<=16'h0080; req_ggb<=1; end //
                else if (ib08_req_b && !c_i_b) begin req_b<=16'h0100; req_ggb<=1; end // 
                else if (ib09_req_b && !c_j_b) begin req_b<=16'h0200; req_ggb<=1; end // 
                else if (ib10_req_b && !c_k_b) begin req_b<=16'h0400; req_ggb<=1; end // 
                else if (ib11_req_b && !c_l_b) begin req_b<=16'h0800; req_ggb<=1; end // 
                else if (ib12_req_b && !c_m_b) begin req_b<=16'h1000; req_ggb<=1; end // 
                else if (ib13_req_b && !c_n_b) begin req_b<=16'h2000; req_ggb<=1; end // 
                else if (ib14_req_b && !c_o_b) begin req_b<=16'h4000; req_ggb<=1; end //
                else if (ib15_req_b && !c_p_b) begin req_b<=16'h8000; req_ggb<=1; end //
                
                state <= 2;
              end
              1'b1: begin
                if      (ib00_req_a && !c_a) begin req<=16'h0001;  state<=3; end //
                else if (ib01_req_a && !c_b) begin req<=16'h0002;  state<=3; end // 
                else if (ib02_req_a && !c_c) begin req<=16'h0004;  state<=3; end // 
                else if (ib03_req_a && !c_d) begin req<=16'h0008;  state<=3; end // 
                else if (ib04_req_a && !c_e) begin req<=16'h0010;  state<=3; end // 
                else if (ib05_req_a && !c_f) begin req<=16'h0020;  state<=3; end // 
                else if (ib06_req_a && !c_g) begin req<=16'h0040;  state<=3; end // 
                else if (ib07_req_a && !c_h) begin req<=16'h0080;  state<=3; end //
                else if (ib08_req_a && !c_i) begin req<=16'h0100;  state<=3; end // 
                else if (ib09_req_a && !c_j) begin req<=16'h0200;  state<=3; end // 
                else if (ib10_req_a && !c_k) begin req<=16'h0400;  state<=3; end // 
                else if (ib11_req_a && !c_l) begin req<=16'h0800;  state<=3; end // 
                else if (ib12_req_a && !c_m) begin req<=16'h1000;  state<=3; end // 
                else if (ib13_req_a && !c_n) begin req<=16'h2000;  state<=3; end // 
                else if (ib14_req_a && !c_o) begin req<=16'h4000;  state<=3; end //
                else if (ib15_req_a && !c_p) begin req<=16'h8000;  state<=3; end //
                else if (OB_req_a)           begin OB_deq_ta <= 1; state<=4; end
              end
            endcase
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        2: begin ///// request arbitration 
          if (!d_busy) begin
            if      (req_gga) begin req_ga <= 1; req_b <= 0; state <= 3; end
            else if (req_ggb) begin req_gb <= 1; req_a <= 0; state <= 3; end
            else if (OB_req_a) begin OB_deq_ta <= 1;         state <= 4; end // WRITE
            else if (OB_req_b) begin OB_deq_tb <= 1;         state <= 5; end // WRITE
            else                                             state <= 1;
            {req_gga, req_ggb} <= 0;
          end
        end
        /////////////////////////////////////////////////////////////////////////////////////
        3: begin ///// READ data from DRAM
          if (d_req!=0) begin 
            d_req <= 0; 
            state <= 1;
            {req_ga, req_gb} <= 0;
          end else if (!d_busy) begin
            case (last_phase)
              1'b0: begin
                req_ta <= req_a;
                case ({req_gb, req_ga})
                  2'b01: begin
                    case (req_a)
                      16'h0001: begin
                        d_initadr <= mux32(radr_a_a, (radr_a_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_a_a  <= radr_a_a+(`D_RS);
                        cnt_a_a   <= cnt_a_a+1;
                        c_a_a     <= (cnt_a_a>=`WAYP_CN_);
                      end
                      16'h0002: begin
                        d_initadr <= mux32(radr_b_a, (radr_b_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_b_a  <= radr_b_a+(`D_RS);
                        cnt_b_a   <= cnt_b_a+1;
                        c_b_a     <= (cnt_b_a>=`WAYP_CN_);
                      end
                      16'h0004: begin
                        d_initadr <= mux32(radr_c_a, (radr_c_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_c_a  <= radr_c_a+(`D_RS);
                        cnt_c_a   <= cnt_c_a+1;
                        c_c_a     <= (cnt_c_a>=`WAYP_CN_);
                      end
                      16'h0008: begin
                        d_initadr <= mux32(radr_d_a, (radr_d_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_d_a  <= radr_d_a+(`D_RS);
                        cnt_d_a   <= cnt_d_a+1;
                        c_d_a     <= (cnt_d_a>=`WAYP_CN_);
                      end
                      16'h0010: begin
                        d_initadr <= mux32(radr_e_a, (radr_e_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_e_a  <= radr_e_a+(`D_RS);
                        cnt_e_a   <= cnt_e_a+1;
                        c_e_a     <= (cnt_e_a>=`WAYP_CN_);
                      end
                      16'h0020: begin
                        d_initadr <= mux32(radr_f_a, (radr_f_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_f_a  <= radr_f_a+(`D_RS);
                        cnt_f_a   <= cnt_f_a+1;
                        c_f_a     <= (cnt_f_a>=`WAYP_CN_);
                      end
                      16'h0040: begin
                        d_initadr <= mux32(radr_g_a, (radr_g_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_g_a  <= radr_g_a+(`D_RS);
                        cnt_g_a   <= cnt_g_a+1;
                        c_g_a     <= (cnt_g_a>=`WAYP_CN_);                
                      end
                      16'h0080: begin
                        d_initadr <= mux32(radr_h_a, (radr_h_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_h_a  <= radr_h_a+(`D_RS);
                        cnt_h_a   <= cnt_h_a+1;
                        c_h_a     <= (cnt_h_a>=`WAYP_CN_);
                      end
                      16'h0100: begin
                        d_initadr <= mux32(radr_i_a, (radr_i_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_i_a  <= radr_i_a+(`D_RS);
                        cnt_i_a   <= cnt_i_a+1;
                        c_i_a     <= (cnt_i_a>=`WAYP_CN_);
                      end
                      16'h0200: begin
                        d_initadr <= mux32(radr_j_a, (radr_j_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_j_a  <= radr_j_a+(`D_RS);
                        cnt_j_a   <= cnt_j_a+1;
                        c_j_a     <= (cnt_j_a>=`WAYP_CN_);
                      end
                      16'h0400: begin
                        d_initadr <= mux32(radr_k_a, (radr_k_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_k_a  <= radr_k_a+(`D_RS);
                        cnt_k_a   <= cnt_k_a+1;
                        c_k_a     <= (cnt_k_a>=`WAYP_CN_);
                      end
                      16'h0800: begin
                        d_initadr <= mux32(radr_l_a, (radr_l_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_l_a  <= radr_l_a+(`D_RS);
                        cnt_l_a   <= cnt_l_a+1;
                        c_l_a     <= (cnt_l_a>=`WAYP_CN_);
                      end
                      16'h1000: begin
                        d_initadr <= mux32(radr_m_a, (radr_m_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_m_a  <= radr_m_a+(`D_RS);
                        cnt_m_a   <= cnt_m_a+1;
                        c_m_a     <= (cnt_m_a>=`WAYP_CN_);
                      end
                      16'h2000: begin
                        d_initadr <= mux32(radr_n_a, (radr_n_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_n_a  <= radr_n_a+(`D_RS);
                        cnt_n_a   <= cnt_n_a+1;
                        c_n_a     <= (cnt_n_a>=`WAYP_CN_);
                      end
                      16'h4000: begin
                        d_initadr <= mux32(radr_o_a, (radr_o_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_o_a  <= radr_o_a+(`D_RS);
                        cnt_o_a   <= cnt_o_a+1;
                        c_o_a     <= (cnt_o_a>=`WAYP_CN_);
                      end
                      16'h8000: begin
                        d_initadr <= mux32(radr_p_a, (radr_p_a | (`SORT_ELM>>1)), phase_a[0]);
                        radr_p_a  <= radr_p_a+(`D_RS);
                        cnt_p_a   <= cnt_p_a+1;
                        c_p_a     <= (cnt_p_a>=`WAYP_CN_);
                      end
                    endcase
                  end
                  2'b10: begin
                    case (req_b)
                      16'h0001: begin
                        d_initadr <= mux32(radr_a_b, (radr_a_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_a_b  <= radr_a_b+(`D_RS);
                        cnt_a_b   <= cnt_a_b+1;
                        c_a_b     <= (cnt_a_b>=`WAYP_CN_);
                      end
                      16'h0002: begin
                        d_initadr <= mux32(radr_b_b, (radr_b_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_b_b  <= radr_b_b+(`D_RS);
                        cnt_b_b   <= cnt_b_b+1;
                        c_b_b     <= (cnt_b_b>=`WAYP_CN_);
                      end
                      16'h0004: begin
                        d_initadr <= mux32(radr_c_b, (radr_c_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_c_b  <= radr_c_b+(`D_RS);
                        cnt_c_b   <= cnt_c_b+1;
                        c_c_b     <= (cnt_c_b>=`WAYP_CN_);
                      end
                      16'h0008: begin
                        d_initadr <= mux32(radr_d_b, (radr_d_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_d_b  <= radr_d_b+(`D_RS);
                        cnt_d_b   <= cnt_d_b+1;
                        c_d_b     <= (cnt_d_b>=`WAYP_CN_);
                      end
                      16'h0010: begin
                        d_initadr <= mux32(radr_e_b, (radr_e_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_e_b  <= radr_e_b+(`D_RS);
                        cnt_e_b   <= cnt_e_b+1;
                        c_e_b     <= (cnt_e_b>=`WAYP_CN_);
                      end
                      16'h0020: begin
                        d_initadr <= mux32(radr_f_b, (radr_f_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_f_b  <= radr_f_b+(`D_RS);
                        cnt_f_b   <= cnt_f_b+1;
                        c_f_b     <= (cnt_f_b>=`WAYP_CN_);
                      end
                      16'h0040: begin
                        d_initadr <= mux32(radr_g_b, (radr_g_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_g_b  <= radr_g_b+(`D_RS);
                        cnt_g_b   <= cnt_g_b+1;
                        c_g_b     <= (cnt_g_b>=`WAYP_CN_);                
                      end
                      16'h0080: begin
                        d_initadr <= mux32(radr_h_b, (radr_h_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_h_b  <= radr_h_b+(`D_RS);
                        cnt_h_b   <= cnt_h_b+1;
                        c_h_b     <= (cnt_h_b>=`WAYP_CN_);
                      end
                      16'h0100: begin
                        d_initadr <= mux32(radr_i_b, (radr_i_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_i_b  <= radr_i_b+(`D_RS);
                        cnt_i_b   <= cnt_i_b+1;
                        c_i_b     <= (cnt_i_b>=`WAYP_CN_);
                      end
                      16'h0200: begin
                        d_initadr <= mux32(radr_j_b, (radr_j_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_j_b  <= radr_j_b+(`D_RS);
                        cnt_j_b   <= cnt_j_b+1;
                        c_j_b     <= (cnt_j_b>=`WAYP_CN_);
                      end
                      16'h0400: begin
                        d_initadr <= mux32(radr_k_b, (radr_k_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_k_b  <= radr_k_b+(`D_RS);
                        cnt_k_b   <= cnt_k_b+1;
                        c_k_b     <= (cnt_k_b>=`WAYP_CN_);
                      end
                      16'h0800: begin
                        d_initadr <= mux32(radr_l_b, (radr_l_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_l_b  <= radr_l_b+(`D_RS);
                        cnt_l_b   <= cnt_l_b+1;
                        c_l_b     <= (cnt_l_b>=`WAYP_CN_);
                      end
                      16'h1000: begin
                        d_initadr <= mux32(radr_m_b, (radr_m_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_m_b  <= radr_m_b+(`D_RS);
                        cnt_m_b   <= cnt_m_b+1;
                        c_m_b     <= (cnt_m_b>=`WAYP_CN_);
                      end
                      16'h2000: begin
                        d_initadr <= mux32(radr_n_b, (radr_n_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_n_b  <= radr_n_b+(`D_RS);
                        cnt_n_b   <= cnt_n_b+1;
                        c_n_b     <= (cnt_n_b>=`WAYP_CN_);
                      end
                      16'h4000: begin
                        d_initadr <= mux32(radr_o_b, (radr_o_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_o_b  <= radr_o_b+(`D_RS);
                        cnt_o_b   <= cnt_o_b+1;
                        c_o_b     <= (cnt_o_b>=`WAYP_CN_);
                      end
                      16'h8000: begin
                        d_initadr <= mux32(radr_p_b, (radr_p_b | (`SORT_ELM>>1)), phase_b[0]);
                        radr_p_b  <= radr_p_b+(`D_RS);
                        cnt_p_b   <= cnt_p_b+1;
                        c_p_b     <= (cnt_p_b>=`WAYP_CN_);
                      end
                    endcase
                  end
                endcase
              end
              1'b1: begin
                req_ta <= req;
                case (req)
                  16'h0001: begin
                    d_initadr <= mux32(radr_a, (radr_a | (`SORT_ELM>>1)), l_phase[0]);
                    radr_a    <= radr_a+(`D_RS);
                    cnt_a     <= cnt_a+1;
                    c_a       <= (cnt_a>=`WAY_CN_);
                  end
                  16'h0002: begin
                    d_initadr <= mux32(radr_b, (radr_b | (`SORT_ELM>>1)), l_phase[0]);
                    radr_b    <= radr_b+(`D_RS);
                    cnt_b     <= cnt_b+1;
                    c_b       <= (cnt_b>=`WAY_CN_);
                  end
                  16'h0004: begin
                    d_initadr <= mux32(radr_c, (radr_c | (`SORT_ELM>>1)), l_phase[0]);
                    radr_c    <= radr_c+(`D_RS);
                    cnt_c     <= cnt_c+1;
                    c_c       <= (cnt_c>=`WAY_CN_);
                  end
                  16'h0008: begin
                    d_initadr <= mux32(radr_d, (radr_d | (`SORT_ELM>>1)), l_phase[0]);
                    radr_d    <= radr_d+(`D_RS);
                    cnt_d     <= cnt_d+1;
                    c_d       <= (cnt_d>=`WAY_CN_);
                  end
                  16'h0010: begin
                    d_initadr <= mux32(radr_e, (radr_e | (`SORT_ELM>>1)), l_phase[0]);
                    radr_e    <= radr_e+(`D_RS);
                    cnt_e     <= cnt_e+1;
                    c_e       <= (cnt_e>=`WAY_CN_);
                  end
                  16'h0020: begin
                    d_initadr <= mux32(radr_f, (radr_f | (`SORT_ELM>>1)), l_phase[0]);
                    radr_f    <= radr_f+(`D_RS);
                    cnt_f     <= cnt_f+1;
                    c_f       <= (cnt_f>=`WAY_CN_);
                  end
                  16'h0040: begin
                    d_initadr <= mux32(radr_g, (radr_g | (`SORT_ELM>>1)), l_phase[0]);
                    radr_g    <= radr_g+(`D_RS);
                    cnt_g     <= cnt_g+1;
                    c_g       <= (cnt_g>=`WAY_CN_);                
                  end
                  16'h0080: begin
                    d_initadr <= mux32(radr_h, (radr_h | (`SORT_ELM>>1)), l_phase[0]);
                    radr_h    <= radr_h+(`D_RS);
                    cnt_h     <= cnt_h+1;
                    c_h       <= (cnt_h>=`WAY_CN_);
                  end
                  16'h0100: begin
                    d_initadr <= mux32(radr_i, (radr_i | (`SORT_ELM>>1)), l_phase[0]);
                    radr_i    <= radr_i+(`D_RS);
                    cnt_i     <= cnt_i+1;
                    c_i       <= (cnt_i>=`WAY_CN_);
                  end
                  16'h0200: begin
                    d_initadr <= mux32(radr_j, (radr_j | (`SORT_ELM>>1)), l_phase[0]);
                    radr_j    <= radr_j+(`D_RS);
                    cnt_j     <= cnt_j+1;
                    c_j       <= (cnt_j>=`WAY_CN_);
                  end
                  16'h0400: begin
                    d_initadr <= mux32(radr_k, (radr_k | (`SORT_ELM>>1)), l_phase[0]);
                    radr_k    <= radr_k+(`D_RS);
                    cnt_k     <= cnt_k+1;
                    c_k       <= (cnt_k>=`WAY_CN_);
                  end
                  16'h0800: begin
                    d_initadr <= mux32(radr_l, (radr_l | (`SORT_ELM>>1)), l_phase[0]);
                    radr_l    <= radr_l+(`D_RS);
                    cnt_l     <= cnt_l+1;
                    c_l       <= (cnt_l>=`WAY_CN_);
                  end
                  16'h1000: begin
                    d_initadr <= mux32(radr_m, (radr_m | (`SORT_ELM>>1)), l_phase[0]);
                    radr_m    <= radr_m+(`D_RS);
                    cnt_m     <= cnt_m+1;
                    c_m       <= (cnt_m>=`WAY_CN_);
                  end
                  16'h2000: begin
                    d_initadr <= mux32(radr_n, (radr_n | (`SORT_ELM>>1)), l_phase[0]);
                    radr_n    <= radr_n+(`D_RS);
                    cnt_n     <= cnt_n+1;
                    c_n       <= (cnt_n>=`WAY_CN_);
                  end
                  16'h4000: begin
                    d_initadr <= mux32(radr_o, (radr_o | (`SORT_ELM>>1)), l_phase[0]);
                    radr_o    <= radr_o+(`D_RS);
                    cnt_o     <= cnt_o+1;
                    c_o       <= (cnt_o>=`WAY_CN_);
                  end
                  16'h8000: begin
                    d_initadr <= mux32(radr_p, (radr_p | (`SORT_ELM>>1)), l_phase[0]);
                    radr_p    <= radr_p+(`D_RS);
                    cnt_p     <= cnt_p+1;
                    c_p       <= (cnt_p>=`WAY_CN_);
                  end
                endcase
              end
            endcase
            d_req    <= `DRAM_REQ_READ;
            d_blocks <= `DRAM_RBLOCKS;
            req_tb   <= req_b;
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        4: begin ///// WRITE data to DRAM
          if (d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= `DRAM_WBLOCKS;
            case (last_phase)
              1'b0: begin
                d_initadr <= w_addr_a;
                w_addr_a   <= w_addr_a + (`D_WS);
              end
              1'b1: begin
                d_initadr <= w_addr;
                w_addr    <= w_addr + (`D_WS);
              end
            endcase
          end
        end
        ////////////////////////////////////////////////////////////////////////////////////////
        5: begin ///// WRITE data to DRAM
          if(d_req!=0) begin d_req<=0; state<=1; end
          else if (!d_busy) begin
            d_req     <= `DRAM_REQ_WRITE;
            d_blocks  <= `DRAM_WBLOCKS;
            d_initadr <= w_addr_b;
            w_addr_b  <= w_addr_b + (`D_WS);
          end
        end
      endcase
    end
  end

  /***** WRITE : feed the initial data to be stored to DRAM                                 *****/
  /**********************************************************************************************/
  reg RST_INI; // reset signal for value initialization module
  always @(posedge CLK) RST_INI <= RSTa;
  
  reg [`SORTW-1:0] i_p,i_o,i_n,i_m,i_l,i_k,i_j,i_i,i_h,i_g,i_f,i_e,i_d,i_c,i_b,i_a;

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
            i_a  <= mux32(i_a, i_a+16, ocen);
            i_b  <= mux32(i_b, i_b+16, ocen);
            i_c  <= mux32(i_c, i_c+16, ocen);
            i_d  <= mux32(i_d, i_d+16, ocen);
            i_e  <= mux32(i_e, i_e+16, ocen);
            i_f  <= mux32(i_f, i_f+16, ocen);
            i_g  <= mux32(i_g, i_g+16, ocen);
            i_h  <= mux32(i_h, i_h+16, ocen);
            i_i  <= mux32(i_i, i_i+16, ocen);
            i_j  <= mux32(i_j, i_j+16, ocen);
            i_k  <= mux32(i_k, i_k+16, ocen);
            i_l  <= mux32(i_l, i_l+16, ocen);
            i_m  <= mux32(i_m, i_m+16, ocen);
            i_n  <= mux32(i_n, i_n+16, ocen);
            i_o  <= mux32(i_o, i_o+16, ocen);
            i_p  <= mux32(i_p, i_p+16, ocen);
          end
        end
      end
    end
  endgenerate

  always @(posedge CLK) idone_a <= initdone;
  always @(posedge CLK) idone_b <= initdone;
  
  assign d_din[255:  0] = mux256({i_h,i_g,i_f,i_e,i_d,i_c,i_b,i_a},
                                 mux2in256(OB_dot0[255:0], OB_dot1[255:0], OB_dot_sel),
                                 idone_a);
  assign d_din[511:256] = mux256({i_p,i_o,i_n,i_m,i_l,i_k,i_j,i_i},
                                 mux2in256(OB_dot0[511:256], OB_dot1[511:256], OB_dot_sel),
                                 idone_b);

  /**********************************************************************************************/    
  always @(posedge CLK) begin
    dout_t <= d_dout;
    doen_t <= d_douten;
    
    // Stage 0
    ////////////////////////////////////
    dout_tta  <= stnet_dout;
    dout_ttb  <= stnet_dout;
   
    doen_tta  <= stnet_douten[0];
    doen_ttb  <= stnet_douten[0];
    
    req_tt0_a <= stnet_douten[`SORT_WAY:1];
    req_tt0_b <= stnet_douten[`SORT_WAY*2:`SORT_WAY+1];
    
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
    
    // Stage 3
    ////////////////////////////////////
    dout_t1_a <= dout_t0_a;
    dout_t2_a <= dout_t0_a;
    dout_t3_a <= dout_t0_b;
    dout_t4_a <= dout_t0_b;
    dout_t5_a <= dout_t0_c;
    dout_t6_a <= dout_t0_c;
    dout_t7_a <= dout_t0_d;
    dout_t8_a <= dout_t0_d;
    dout_t1_b <= dout_t0_e;
    dout_t2_b <= dout_t0_e;
    dout_t3_b <= dout_t0_f;
    dout_t4_b <= dout_t0_f;
    dout_t5_b <= dout_t0_g;
    dout_t6_b <= dout_t0_g;
    dout_t7_b <= dout_t0_h;
    dout_t8_b <= dout_t0_h;
    
    doen_t1_a <= doen_t0_a;
    doen_t2_a <= doen_t0_a;
    doen_t3_a <= doen_t0_b;
    doen_t4_a <= doen_t0_b;
    doen_t5_a <= doen_t0_c;
    doen_t6_a <= doen_t0_c;
    doen_t7_a <= doen_t0_d;
    doen_t8_a <= doen_t0_d;
    doen_t1_b <= doen_t0_e;
    doen_t2_b <= doen_t0_e;
    doen_t3_b <= doen_t0_f;
    doen_t4_b <= doen_t0_f;
    doen_t5_b <= doen_t0_g;
    doen_t6_b <= doen_t0_g;
    doen_t7_b <= doen_t0_h;
    doen_t8_b <= doen_t0_h;
    
    req_tt3_a <= req_tt2_a;
    req_tt3_b <= req_tt2_b;
  end

  // for last_phase
  // ###########################################################################
  always @(posedge CLK) begin
    if (RSTa) begin
      last_phase <= 0;
    end else begin
      if (pexe_done_a && pexe_done_b) last_phase <= 1;
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
  
  // for elem
  // ###########################################################################
  always @(posedge CLK) begin
    if (RSTa) begin
      elem  <= 0;
      elem_a <= 0;
    end else begin
      case (last_phase)
        1'b0: begin
          case ({OB_deq0, (elem_a==`SRTP_ELM)})
            2'b01: elem_a <= 0;
            2'b10: elem_a <= elem_a + 16;
          endcase
        end
        1'b1: begin
          case ({OB_deq0, (elem==`SORT_ELM)})
            2'b01: elem <= 0;
            2'b10: elem <= elem + 16;
          endcase
        end
      endcase
    end
  end
  always @(posedge CLK) begin
    if (RSTb) begin
      elem_b <= 0;
    end else begin
      case ({OB_deq1, (elem_b==`SRTP_ELM)})
        2'b01: elem_b <= 0;
        2'b10: elem_b <= elem_b + 16;
      endcase
    end
  end
  
  // for iter_done
  // ###########################################################################
  always @(posedge CLK) iter_done_a <= (ecnt_a==2);
  always @(posedge CLK) iter_done_b <= (ecnt_b==2);

  // for pchange
  // ###########################################################################
  always @(posedge CLK) pchange_a <= (elem_a==`SRTP_ELM);
  always @(posedge CLK) pchange_b <= (elem_b==`SRTP_ELM);

  // for irst
  // ###########################################################################
  always @(posedge CLK) irst_a <= mux1(((ecnt_a==2) || pchange_a), (ecnt==2), last_phase);
  always @(posedge CLK) irst_b <= (ecnt_b==2) || pchange_b;

  // for frst
  // ###########################################################################
  always @(posedge CLK) frst_a <= mux1((RSTa || (ecnt_a==2) || (elem_a==`SRTP_ELM)), (ecnt==2), last_phase);
  always @(posedge CLK) frst_b <= RSTb || (ecnt_b==2) || (elem_b==`SRTP_ELM);
  
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
