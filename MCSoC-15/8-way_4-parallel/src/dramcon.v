`default_nettype none

`include "define.v"  

module DRAMCON(input  wire                      CLK_P,
               input  wire                      CLK_N,
               input  wire                      RST_X_IN,
               ////////// User logic interface ports //////////
               input  wire [1:0]                D_REQ,      // dram request, load or store
               input  wire [31:0]               D_INITADR,  // dram request, initial address
               input  wire [31:0]               D_ELEM,     // dram request, the number of elements
               input  wire [`APPDATA_WIDTH-1:0] D_DIN,      //
               output wire                      D_W,        //
               output reg  [`APPDATA_WIDTH-1:0] D_DOUT,     //
               output reg                       D_DOUTEN,   //
               output wire                      D_BUSY,     //
               output wire                      USERCLK,    //
               output wire                      RST_O,      //
               ////////// Memory interface ports //////////               
               inout  wire [`DDR3_DATA]         DDR3DQ,
               inout  wire [7:0]                DDR3DQS_N,
               inout  wire [7:0]                DDR3DQS_P,
               output wire [`DDR3_ADDR]         DDR3ADDR,
               output wire [2:0]                DDR3BA,
               output wire                      DDR3RAS_N,
               output wire                      DDR3CAS_N,
               output wire                      DDR3WE_N,
               output wire                      DDR3RESET_N,
               output wire [0:0]                DDR3CK_P,
               output wire [0:0]                DDR3CK_N,
               output wire [0:0]                DDR3CKE,
               output wire [0:0]                DDR3CS_N,
               output wire [7:0]                DDR3DM,
               output wire [0:0]                DDR3ODT);
    
  // inputs of u_dram
  reg [`APPADDR_WIDTH-1:0]  app_addr;
  reg [`DDR3_CMD]           app_cmd;
  reg                       app_en;
  wire [`APPDATA_WIDTH-1:0] app_wdf_data = D_DIN;
  reg                       app_wdf_wren;  
  wire                      app_wdf_end = app_wdf_wren;
  wire                      app_sr_req  = 0;  // no used
  wire                      app_ref_req = 0;  // no used
  wire                      app_zq_req  = 0;  // no used

  // outputs of u_dram
  wire [`APPDATA_WIDTH-1:0] app_rd_data;
  wire                      app_rd_data_end;
  wire                      app_rd_data_valid;
  wire                      app_rdy;
  wire                      app_wdf_rdy;
  wire                      app_sr_active;    // no used
  wire                      app_ref_ack;      // no used
  wire                      app_zq_ack;       // no used
  wire                      ui_clk;           
  wire                      ui_clk_sync_rst; 
  wire                      init_calib_complete;

//----------- Begin Cut here for INSTANTIATION Template ---// INST_TAG

  dram u_dram (

    // Memory interface ports
    .ddr3_addr                      (DDR3ADDR),
    .ddr3_ba                        (DDR3BA),  
    .ddr3_cas_n                     (DDR3CAS_N),
    .ddr3_ck_n                      (DDR3CK_N), 
    .ddr3_ck_p                      (DDR3CK_P), 
    .ddr3_cke                       (DDR3CKE),
    .ddr3_ras_n                     (DDR3RAS_N),  
    .ddr3_reset_n                   (DDR3RESET_N),
    .ddr3_we_n                      (DDR3WE_N), 
    .ddr3_dq                        (DDR3DQ),  
    .ddr3_dqs_n                     (DDR3DQS_N),
    .ddr3_dqs_p                     (DDR3DQS_P),
    .ddr3_cs_n                      (DDR3CS_N), 
    .ddr3_dm                        (DDR3DM),  
    .ddr3_odt                       (DDR3ODT), 
    .sys_clk_p                      (CLK_P),
    .sys_clk_n                      (CLK_N),
    // Application interface ports
    .app_addr                       (app_addr),  
    .app_cmd                        (app_cmd),  
    .app_en                         (app_en),  
    .app_wdf_data                   (app_wdf_data),  
    .app_wdf_end                    (app_wdf_end),  
    .app_wdf_wren                   (app_wdf_wren),  
    .app_rd_data                    (app_rd_data),  
    .app_rd_data_end                (app_rd_data_end),  
    .app_rd_data_valid              (app_rd_data_valid), 
    .app_rdy                        (app_rdy),  
    .app_wdf_rdy                    (app_wdf_rdy),  
    .app_sr_req                     (app_sr_req),  
    .app_ref_req                    (app_ref_req),  
    .app_zq_req                     (app_zq_req),  
    .app_sr_active                  (app_sr_active),  
    .app_ref_ack                    (app_ref_ack),  
    .app_zq_ack                     (app_zq_ack),  
    .ui_clk                         (ui_clk),  
    .ui_clk_sync_rst                (ui_clk_sync_rst),  
    .init_calib_complete            (init_calib_complete),  
    .app_wdf_mask                   ({`APPMASK_WIDTH{1'b0}}),
    .sys_rst                        (RST_X_IN)
    );

// INST_TAG_END ------ End INSTANTIATION Template ---------

  ///// READ & WRITE PORT CONTROL (begin) ////////////////////////////////////////////
  localparam M_REQ   = 0;
  localparam M_WRITE = 1;
  localparam M_READ  = 2;
    
  reg [1:0]                mode;
  reg [31:0]               remain, remain2;
  
  reg rst_o;
  always @(posedge ui_clk) rst_o <= (ui_clk_sync_rst || ~init_calib_complete);  // High Active
  
  assign USERCLK = ui_clk;
  assign RST_O   = rst_o;
  assign D_BUSY  = (mode != M_REQ);                              // DRAM busy
  assign D_W     = (mode == M_WRITE && app_rdy && app_wdf_rdy);  // store one element

  always @(posedge ui_clk) begin
    if (RST_O) begin
      mode         <= M_REQ;
      app_addr     <= 0;
      app_cmd      <= 0;
      app_en       <= 0;
      app_wdf_wren <= 0;
      D_DOUT       <= 0;
      D_DOUTEN     <= 0;
      remain       <= 0;
      remain2      <= 0;
    end else begin
      case (mode)
        ///////////////////////////////////////////////////////////////// request
        M_REQ: begin
          D_DOUTEN <= 0;
          case (D_REQ)
            `DRAM_REQ_READ: begin  ///// READ or LOAD request
              app_cmd      <= `DRAM_CMD_READ;
              mode         <= M_READ;
              app_wdf_wren <= 0;
              app_en       <= 1;
              app_addr     <= D_INITADR;  // param, initial address
              remain       <= D_ELEM;     // param, the number of blocks to be read
              remain2      <= D_ELEM;     // param, the number of blocks to be read
            end
            `DRAM_REQ_WRITE: begin  ///// WRITE or STORE request
              app_cmd      <= `DRAM_CMD_WRITE;
              mode         <= M_WRITE;
              app_wdf_wren <= 0;
              app_en       <= 1;
              app_addr     <= D_INITADR;  // param, initial address
              remain       <= D_ELEM;     // the number of blocks to be written
            end
            default: begin
              app_wdf_wren <= 0;
              app_en       <= 0;
            end
          endcase
        end
        ///////////////////////////////////////////////////////////////// read
        M_READ: begin
          if (app_rdy) begin  // read request is accepted.
            app_addr <= (app_addr == `MEM_LAST_ADDR) ? 0 : app_addr + 8;
            remain2  <= remain2 - 1;
            if (remain2 == 1) app_en <= 0;
          end
          D_DOUTEN <= app_rd_data_valid;  // dram data_out enable
          if (app_rd_data_valid) begin
            D_DOUT <= app_rd_data;
            remain <= remain - 1;
            if (remain == 1) mode <= M_REQ;
          end
        end
        ///////////////////////////////////////////////////////////////// write
        M_WRITE: begin
          if (app_rdy && app_wdf_rdy) begin
            app_wdf_wren <= 1;
            app_addr     <= (app_addr == `MEM_LAST_ADDR) ? 0 : app_addr + 8;
            remain       <= remain - 1;
            if (remain == 1) begin
              mode   <= M_REQ;
              app_en <= 0;
            end
          end else begin
            app_wdf_wren <= 0;
          end
        end
      endcase
    end
  end
  ///// READ & WRITE PORT CONTROL (end)   ////////////////////////////////////////////
endmodule
`default_nettype wire
