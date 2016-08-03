/******************************************************************************/
/* FPGA Sort on VC707                                        Ryohei Kobayashi */
/*                                                                 2016-08-01 */
/******************************************************************************/

`include "trellis.vh"
`include "riffa.vh"
`include "tlp.vh"
`include "xilinx.vh"
`include "define.vh"

`timescale 1ps / 1ps
module top #(parameter C_NUM_CHNL          = 1,  // Number of RIFFA Channels
             parameter C_NUM_LANES         = 8,  // Number of PCIe Lanes
             /* Settings from Vivado IP Generator */
             parameter C_PCI_DATA_WIDTH    = 128,
             parameter C_MAX_PAYLOAD_BYTES = 256,
             parameter C_LOG_NUM_TAGS      = 6)
  
            (output [(C_NUM_LANES - 1) : 0] PCI_EXP_TXP,
             output [(C_NUM_LANES - 1) : 0] PCI_EXP_TXN,
             input  [(C_NUM_LANES - 1) : 0] PCI_EXP_RXP,
             input  [(C_NUM_LANES - 1) : 0] PCI_EXP_RXN,
             
             output [3:0]                   LED,
             input                          PCIE_REFCLK_P,
             input                          PCIE_REFCLK_N,
             input                          PCIE_RESET_N,
             
             // DRAM
             input  wire                    CLK_P,
             input  wire                    CLK_N,
             inout  wire [`DDR3_DATA]       DDR3DQ,
             inout  wire [7:0]              DDR3DQS_N,
             inout  wire [7:0]              DDR3DQS_P,
             output wire [`DDR3_ADDR]       DDR3ADDR,
             output wire [2:0]              DDR3BA,
             output wire                    DDR3RAS_N,
             output wire                    DDR3CAS_N,
             output wire                    DDR3WE_N,
             output wire                    DDR3RESET_N,
             output wire [0:0]              DDR3CK_P,
             output wire [0:0]              DDR3CK_N,
             output wire [0:0]              DDR3CKE,
             output wire [0:0]              DDR3CS_N,
             output wire [7:0]              DDR3DM,
             output wire [0:0]              DDR3ODT);

  wire                               pcie_refclk;
  wire                               pcie_reset_n;

  wire                               user_clk;
  wire                               user_reset;
  wire                               user_lnk_up;
  wire                               user_app_rdy;
    
  wire                               s_axis_tx_tready;
  wire [C_PCI_DATA_WIDTH-1:0]        s_axis_tx_tdata;
  wire [(C_PCI_DATA_WIDTH/8)-1:0]    s_axis_tx_tkeep;
  wire                               s_axis_tx_tlast;
  wire                               s_axis_tx_tvalid;
  wire [`SIG_XIL_TX_TUSER_W:0]       s_axis_tx_tuser;
  
  wire [C_PCI_DATA_WIDTH-1:0]        m_axis_rx_tdata;
  wire [(C_PCI_DATA_WIDTH/8)-1:0]    m_axis_rx_tkeep;
  wire                               m_axis_rx_tlast;
  wire                               m_axis_rx_tvalid;
  wire                               m_axis_rx_tready;
  wire [`SIG_XIL_RX_TUSER_W-1:0]     m_axis_rx_tuser;

  wire                               tx_cfg_gnt;
  wire                               rx_np_ok;
  wire                               rx_np_req;
  wire                               cfg_turnoff_ok;
  wire                               cfg_trn_pending;
  wire                               cfg_pm_halt_aspm_l0s;
  wire                               cfg_pm_halt_aspm_l1;
  wire                               cfg_pm_force_state_en;
  wire [ 1:0]                        cfg_pm_force_state;
  wire                               cfg_pm_wake;
  wire [63:0]                        cfg_dsn;

  wire [11:0]                        fc_cpld;
  wire [ 7:0]                        fc_cplh;
  wire [11:0]                        fc_npd;
  wire [ 7:0]                        fc_nph;
  wire [11:0]                        fc_pd;
  wire [ 7:0]                        fc_ph;
  wire [ 2:0]                        fc_sel;
    
  wire [15:0]                        cfg_status;
  wire [15:0]                        cfg_command;
  wire [15:0]                        cfg_dstatus;
  wire [15:0]                        cfg_dcommand;
  wire [15:0]                        cfg_lstatus;
  wire [15:0]                        cfg_lcommand;
  wire [15:0]                        cfg_dcommand2;
    
  wire [2:0]                         cfg_pcie_link_state;
  wire                               cfg_pmcsr_pme_en;
  wire [1:0]                         cfg_pmcsr_powerstate;
  wire                               cfg_pmcsr_pme_status;
  wire                               cfg_received_func_lvl_rst;
  wire [4:0]                         cfg_pciecap_interrupt_msgnum;
  wire                               cfg_to_turnoff;
  wire [7:0]                         cfg_bus_number;
  wire [4:0]                         cfg_device_number;
  wire [2:0]                         cfg_function_number;

  wire                               cfg_interrupt;
  wire                               cfg_interrupt_rdy;
  wire                               cfg_interrupt_assert;
  wire [7:0]                         cfg_interrupt_di;
  wire [7:0]                         cfg_interrupt_do;
  wire [2:0]                         cfg_interrupt_mmenable;
  wire                               cfg_interrupt_msien;
  wire                               cfg_interrupt_msixenable;
  wire                               cfg_interrupt_msixfm;
  wire                               cfg_interrupt_stat;
    


  wire                                       rst_out;
  wire [C_NUM_CHNL-1:0]                      chnl_rx_clk; 
  wire [C_NUM_CHNL-1:0]                      chnl_rx; 
  wire [C_NUM_CHNL-1:0]                      chnl_rx_ack; 
  wire [C_NUM_CHNL-1:0]                      chnl_rx_last; 
  wire [(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0] chnl_rx_len; 
  wire [(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0] chnl_rx_off; 
  wire [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]   chnl_rx_data; 
  wire [C_NUM_CHNL-1:0]                      chnl_rx_data_valid; 
  wire [C_NUM_CHNL-1:0]                      chnl_rx_data_ren;
  
  wire [C_NUM_CHNL-1:0]                      chnl_tx_clk; 
  wire [C_NUM_CHNL-1:0]                      chnl_tx; 
  wire [C_NUM_CHNL-1:0]                      chnl_tx_ack;
  wire [C_NUM_CHNL-1:0]                      chnl_tx_last; 
  wire [(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0] chnl_tx_len; 
  wire [(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0] chnl_tx_off; 
  wire [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]   chnl_tx_data; 
  wire [C_NUM_CHNL-1:0]                      chnl_tx_data_valid; 
  wire [C_NUM_CHNL-1:0]                      chnl_tx_data_ren;

  
  assign cfg_turnoff_ok               = 0;
  assign cfg_trn_pending              = 0;
  assign cfg_pm_halt_aspm_l0s         = 0;
  assign cfg_pm_halt_aspm_l1          = 0;
  assign cfg_pm_force_state_en        = 0;
  assign cfg_pm_force_state           = 0;
  assign cfg_dsn                      = 0;
  assign cfg_interrupt_assert         = 0;
  assign cfg_interrupt_di             = 0;
  assign cfg_interrupt_stat           = 0;
  assign cfg_pciecap_interrupt_msgnum = 0;
  assign cfg_turnoff_ok               = 0;
  assign cfg_pm_wake                  = 0;

  IBUF 
    #()  
  pci_reset_n_ibuf 
    (.O(pcie_reset_n), 
     .I(PCIE_RESET_N));

  IBUFDS_GTE2 
    #()
  refclk_ibuf 
    (.O(pcie_refclk), 
     .ODIV2(), 
     .I(PCIE_REFCLK_P), 
     .CEB(1'b0), 
     .IB(PCIE_REFCLK_N));

  // Core Top Level Wrapper
  PCIeGen2x8If128 PCIeGen2x8If128_i
    (//---------------------------------------------------------------------
     // PCI Express (pci_exp) Interface                                     
     //---------------------------------------------------------------------
     // Tx
     .pci_exp_txn                               ( PCI_EXP_TXN ),
     .pci_exp_txp                               ( PCI_EXP_TXP ),
     
     // Rx
     .pci_exp_rxn                               ( PCI_EXP_RXN ),
     .pci_exp_rxp                               ( PCI_EXP_RXP ),
    
     //---------------------------------------------------------------------
     // AXI-S Interface                                                     
     //---------------------------------------------------------------------
     // Common
     .user_clk_out                              ( user_clk ),
     .user_reset_out                            ( user_reset ),
     .user_lnk_up                               ( user_lnk_up ),
     .user_app_rdy                              ( user_app_rdy ),
    
     // TX
     .s_axis_tx_tready                          ( s_axis_tx_tready ),
     .s_axis_tx_tdata                           ( s_axis_tx_tdata ),
     .s_axis_tx_tkeep                           ( s_axis_tx_tkeep ),
     .s_axis_tx_tuser                           ( s_axis_tx_tuser ),
     .s_axis_tx_tlast                           ( s_axis_tx_tlast ),
     .s_axis_tx_tvalid                          ( s_axis_tx_tvalid ),
    
     // Rx
     .m_axis_rx_tdata                           ( m_axis_rx_tdata ),
     .m_axis_rx_tkeep                           ( m_axis_rx_tkeep ),
     .m_axis_rx_tlast                           ( m_axis_rx_tlast ),
     .m_axis_rx_tvalid                          ( m_axis_rx_tvalid ),
     .m_axis_rx_tready                          ( m_axis_rx_tready ),
     .m_axis_rx_tuser                           ( m_axis_rx_tuser ),

     .tx_cfg_gnt                                ( tx_cfg_gnt ),
     .rx_np_ok                                  ( rx_np_ok ),
     .rx_np_req                                 ( rx_np_req ),
     .cfg_trn_pending                           ( cfg_trn_pending ),
     .cfg_pm_halt_aspm_l0s                      ( cfg_pm_halt_aspm_l0s ),
     .cfg_pm_halt_aspm_l1                       ( cfg_pm_halt_aspm_l1 ),
     .cfg_pm_force_state_en                     ( cfg_pm_force_state_en ),
     .cfg_pm_force_state                        ( cfg_pm_force_state ),
     .cfg_dsn                                   ( cfg_dsn ),
     .cfg_turnoff_ok                            ( cfg_turnoff_ok ),
     .cfg_pm_wake                               ( cfg_pm_wake ),
     .cfg_pm_send_pme_to                        ( 1'b0 ),
     .cfg_ds_bus_number                         ( 8'b0 ),
     .cfg_ds_device_number                      ( 5'b0 ),
     .cfg_ds_function_number                    ( 3'b0 ),

     //---------------------------------------------------------------------
     // Flow Control Interface                                              
     //---------------------------------------------------------------------
     .fc_cpld                                   ( fc_cpld ),
     .fc_cplh                                   ( fc_cplh ),
     .fc_npd                                    ( fc_npd ),
     .fc_nph                                    ( fc_nph ),
     .fc_pd                                     ( fc_pd ),
     .fc_ph                                     ( fc_ph ),
     .fc_sel                                    ( fc_sel ),
     
     //---------------------------------------------------------------------
     // Configuration (CFG) Interface                                       
     //---------------------------------------------------------------------
     .cfg_device_number                         ( cfg_device_number ),
     .cfg_dcommand2                             ( cfg_dcommand2 ),
     .cfg_pmcsr_pme_status                      ( cfg_pmcsr_pme_status ),
     .cfg_status                                ( cfg_status ),
     .cfg_to_turnoff                            ( cfg_to_turnoff ),
     .cfg_received_func_lvl_rst                 ( cfg_received_func_lvl_rst ),
     .cfg_dcommand                              ( cfg_dcommand ),
     .cfg_bus_number                            ( cfg_bus_number ),
     .cfg_function_number                       ( cfg_function_number ),
     .cfg_command                               ( cfg_command ),
     .cfg_dstatus                               ( cfg_dstatus ),
     .cfg_lstatus                               ( cfg_lstatus ),
     .cfg_pcie_link_state                       ( cfg_pcie_link_state ),
     .cfg_lcommand                              ( cfg_lcommand ),
     .cfg_pmcsr_pme_en                          ( cfg_pmcsr_pme_en ),
     .cfg_pmcsr_powerstate                      ( cfg_pmcsr_powerstate ),
     
     //------------------------------------------------//
     // EP Only                                        //
     //------------------------------------------------//
     .cfg_interrupt                             ( cfg_interrupt ),
     .cfg_interrupt_rdy                         ( cfg_interrupt_rdy ),
     .cfg_interrupt_assert                      ( cfg_interrupt_assert ),
     .cfg_interrupt_di                          ( cfg_interrupt_di ),
     .cfg_interrupt_do                          ( cfg_interrupt_do ),
     .cfg_interrupt_mmenable                    ( cfg_interrupt_mmenable ),
     .cfg_interrupt_msienable                   ( cfg_interrupt_msien ),
     .cfg_interrupt_msixenable                  ( cfg_interrupt_msixenable ),
     .cfg_interrupt_msixfm                      ( cfg_interrupt_msixfm ),
     .cfg_interrupt_stat                        ( cfg_interrupt_stat ),
     .cfg_pciecap_interrupt_msgnum              ( cfg_pciecap_interrupt_msgnum ),
     //---------------------------------------------------------------------
     // System  (SYS) Interface                                             
     //---------------------------------------------------------------------
     .sys_clk                                    ( pcie_refclk ),
     .sys_rst_n                                  ( pcie_reset_n )
     
     );


  riffa_wrapper_vc707
    #(/*AUTOINSTPARAM*/
      // Parameters
      .C_LOG_NUM_TAGS               (C_LOG_NUM_TAGS),
      .C_NUM_CHNL                   (C_NUM_CHNL),
      .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
      .C_MAX_PAYLOAD_BYTES          (C_MAX_PAYLOAD_BYTES))
  riffa
    (
     // Outputs
     .CFG_INTERRUPT                 (cfg_interrupt),
     .M_AXIS_RX_TREADY              (m_axis_rx_tready),
     .S_AXIS_TX_TDATA               (s_axis_tx_tdata[C_PCI_DATA_WIDTH-1:0]),
     .S_AXIS_TX_TKEEP               (s_axis_tx_tkeep[(C_PCI_DATA_WIDTH/8)-1:0]),
     .S_AXIS_TX_TLAST               (s_axis_tx_tlast),
     .S_AXIS_TX_TVALID              (s_axis_tx_tvalid),
     .S_AXIS_TX_TUSER               (s_axis_tx_tuser[`SIG_XIL_TX_TUSER_W-1:0]),
     .FC_SEL                        (fc_sel[`SIG_FC_SEL_W-1:0]),
     .RST_OUT                       (rst_out),
     .CHNL_RX                       (chnl_rx[C_NUM_CHNL-1:0]),
     .CHNL_RX_LAST                  (chnl_rx_last[C_NUM_CHNL-1:0]),
     .CHNL_RX_LEN                   (chnl_rx_len[(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0]),
     .CHNL_RX_OFF                   (chnl_rx_off[(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0]),
     .CHNL_RX_DATA                  (chnl_rx_data[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
     .CHNL_RX_DATA_VALID            (chnl_rx_data_valid[C_NUM_CHNL-1:0]),
     .CHNL_TX_ACK                   (chnl_tx_ack[C_NUM_CHNL-1:0]),
     .CHNL_TX_DATA_REN              (chnl_tx_data_ren[C_NUM_CHNL-1:0]),
     // Inputs
     .M_AXIS_RX_TDATA               (m_axis_rx_tdata[C_PCI_DATA_WIDTH-1:0]),
     .M_AXIS_RX_TKEEP               (m_axis_rx_tkeep[(C_PCI_DATA_WIDTH/8)-1:0]),
     .M_AXIS_RX_TLAST               (m_axis_rx_tlast),
     .M_AXIS_RX_TVALID              (m_axis_rx_tvalid),
     .M_AXIS_RX_TUSER               (m_axis_rx_tuser[`SIG_XIL_RX_TUSER_W-1:0]),
     .S_AXIS_TX_TREADY              (s_axis_tx_tready),
     .CFG_BUS_NUMBER                (cfg_bus_number[`SIG_BUSID_W-1:0]),
     .CFG_DEVICE_NUMBER             (cfg_device_number[`SIG_DEVID_W-1:0]),
     .CFG_FUNCTION_NUMBER           (cfg_function_number[`SIG_FNID_W-1:0]),
     .CFG_COMMAND                   (cfg_command[`SIG_CFGREG_W-1:0]),
     .CFG_DCOMMAND                  (cfg_dcommand[`SIG_CFGREG_W-1:0]),
     .CFG_LSTATUS                   (cfg_lstatus[`SIG_CFGREG_W-1:0]),
     .CFG_LCOMMAND                  (cfg_lcommand[`SIG_CFGREG_W-1:0]),
     .FC_CPLD                       (fc_cpld[`SIG_FC_CPLD_W-1:0]),
     .FC_CPLH                       (fc_cplh[`SIG_FC_CPLH_W-1:0]),
     .CFG_INTERRUPT_MSIEN           (cfg_interrupt_msien),
     .CFG_INTERRUPT_RDY             (cfg_interrupt_rdy),
     .USER_CLK                      (user_clk),
     .USER_RESET                    (user_reset),
     .CHNL_RX_CLK                   (chnl_rx_clk[C_NUM_CHNL-1:0]),
     .CHNL_RX_ACK                   (chnl_rx_ack[C_NUM_CHNL-1:0]),
     .CHNL_RX_DATA_REN              (chnl_rx_data_ren[C_NUM_CHNL-1:0]),
     .CHNL_TX_CLK                   (chnl_tx_clk[C_NUM_CHNL-1:0]),
     .CHNL_TX                       (chnl_tx[C_NUM_CHNL-1:0]),
     .CHNL_TX_LAST                  (chnl_tx_last[C_NUM_CHNL-1:0]),
     .CHNL_TX_LEN                   (chnl_tx_len[(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0]),
     .CHNL_TX_OFF                   (chnl_tx_off[(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0]),
     .CHNL_TX_DATA                  (chnl_tx_data[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
     .CHNL_TX_DATA_VALID            (chnl_tx_data_valid[C_NUM_CHNL-1:0]),
     .RX_NP_OK                      (rx_np_ok),
     .TX_CFG_GNT                    (tx_cfg_gnt),
     .RX_NP_REQ                     (rx_np_req)
     /*AUTOINST*/);


  // DRAM //////////////////////////////////////////////////////////////////////

  // Wires and Registers of DRAM Controller
  // ###########################################################################
  wire                      CLK_DRAM_USER;
  wire                      RST_DRAM_USER;
  wire                      d_busy;
  wire [`APPDATA_WIDTH-1:0] d_din;
  wire                      d_w;
  wire [`APPDATA_WIDTH-1:0] d_dout;
  wire                      d_douten;
  wire [1:0]                d_req;    
  wire [31:0]               d_initadr;
  wire [31:0]               d_blocks;

  /* DRAM Controller Instantiation                                              */
  /******************************************************************************/
  DRAMCON dramcon(.CLK_P(CLK_P),
                  .CLK_N(CLK_N),
                  .RST_X_IN(rst_out),
                  ////////// User logic interface ports //////////
                  .D_REQ(d_req),
                  .D_INITADR(d_initadr),
                  .D_ELEM(d_blocks),
                  .D_BUSY(d_busy),
                  .D_DIN(d_din),
                  .D_W(d_w),
                  .D_DOUT(d_dout),
                  .D_DOUTEN(d_douten),
                  .USERCLK(CLK_DRAM_USER),
                  .RST_O(RST_DRAM_USER),
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
  // DRAM //////////////////////////////////////////////////////////////////////


  /* User Logic Instantiation                                                   */
  /******************************************************************************/
  USER_LOGIC #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
  user_logic 
    (.CLK(CLK_DRAM_USER),
     .RST(RST_DRAM_USER),
     // Rx interface
     .CHNL_RX_CLK(chnl_rx_clk), 
     .CHNL_RX(chnl_rx), 
     .CHNL_RX_ACK(chnl_rx_ack), 
     .CHNL_RX_LAST(chnl_rx_last), 
     .CHNL_RX_LEN(chnl_rx_len), 
     .CHNL_RX_OFF(chnl_rx_off), 
     .CHNL_RX_DATA(chnl_rx_data), 
     .CHNL_RX_DATA_VALID(chnl_rx_data_valid), 
     .CHNL_RX_DATA_REN(chnl_rx_data_ren),
     // Tx interface
     .CHNL_TX_CLK(chnl_tx_clk), 
     .CHNL_TX(chnl_tx), 
     .CHNL_TX_ACK(chnl_tx_ack), 
     .CHNL_TX_LAST(chnl_tx_last), 
     .CHNL_TX_LEN(chnl_tx_len), 
     .CHNL_TX_OFF(chnl_tx_off), 
     .CHNL_TX_DATA(chnl_tx_data), 
     .CHNL_TX_DATA_VALID(chnl_tx_data_valid), 
     .CHNL_TX_DATA_REN(chnl_tx_data_ren),
     // DRAM
     .d_busy(d_busy),
     .d_din(d_din),
     .d_w(d_w),
     .d_dout(d_dout),
     .d_douten(d_douten),
     .d_req(d_req),
     .d_initadr(d_initadr),
     .d_blocks(d_blocks)
     );
  
endmodule
