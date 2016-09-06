// Copyright 1986-2015 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2015.4 (lin64) Build 1412921 Wed Nov 18 09:44:32 MST 2015
// Date        : Thu Aug 25 17:35:25 2016
// Host        : fpgaserv running 64-bit Ubuntu 14.04.4 LTS
// Command     : write_verilog -force -mode synth_stub
//               /home/kobayashi/PCIe_test/branches/IEICE/4-way/src/ip_pcie/PCIeGen2x8If128_stub.v
// Design      : PCIeGen2x8If128
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7vx485tffg1761-2
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "PCIeGen2x8If128_pcie2_top,Vivado 2015.4" *)
module PCIeGen2x8If128(pci_exp_txp, pci_exp_txn, pci_exp_rxp, pci_exp_rxn, user_clk_out, user_reset_out, user_lnk_up, user_app_rdy, tx_buf_av, tx_cfg_req, tx_err_drop, s_axis_tx_tready, s_axis_tx_tdata, s_axis_tx_tkeep, s_axis_tx_tlast, s_axis_tx_tvalid, s_axis_tx_tuser, tx_cfg_gnt, m_axis_rx_tdata, m_axis_rx_tkeep, m_axis_rx_tlast, m_axis_rx_tvalid, m_axis_rx_tready, m_axis_rx_tuser, rx_np_ok, rx_np_req, fc_cpld, fc_cplh, fc_npd, fc_nph, fc_pd, fc_ph, fc_sel, cfg_status, cfg_command, cfg_dstatus, cfg_dcommand, cfg_lstatus, cfg_lcommand, cfg_dcommand2, cfg_pcie_link_state, cfg_pmcsr_pme_en, cfg_pmcsr_powerstate, cfg_pmcsr_pme_status, cfg_received_func_lvl_rst, cfg_trn_pending, cfg_pm_halt_aspm_l0s, cfg_pm_halt_aspm_l1, cfg_pm_force_state_en, cfg_pm_force_state, cfg_dsn, cfg_interrupt, cfg_interrupt_rdy, cfg_interrupt_assert, cfg_interrupt_di, cfg_interrupt_do, cfg_interrupt_mmenable, cfg_interrupt_msienable, cfg_interrupt_msixenable, cfg_interrupt_msixfm, cfg_interrupt_stat, cfg_pciecap_interrupt_msgnum, cfg_to_turnoff, cfg_turnoff_ok, cfg_bus_number, cfg_device_number, cfg_function_number, cfg_pm_wake, cfg_pm_send_pme_to, cfg_ds_bus_number, cfg_ds_device_number, cfg_ds_function_number, cfg_bridge_serr_en, cfg_slot_control_electromech_il_ctl_pulse, cfg_root_control_syserr_corr_err_en, cfg_root_control_syserr_non_fatal_err_en, cfg_root_control_syserr_fatal_err_en, cfg_root_control_pme_int_en, cfg_aer_rooterr_corr_err_reporting_en, cfg_aer_rooterr_non_fatal_err_reporting_en, cfg_aer_rooterr_fatal_err_reporting_en, cfg_aer_rooterr_corr_err_received, cfg_aer_rooterr_non_fatal_err_received, cfg_aer_rooterr_fatal_err_received, cfg_vc_tcvc_map, sys_clk, sys_rst_n)
/* synthesis syn_black_box black_box_pad_pin="pci_exp_txp[7:0],pci_exp_txn[7:0],pci_exp_rxp[7:0],pci_exp_rxn[7:0],user_clk_out,user_reset_out,user_lnk_up,user_app_rdy,tx_buf_av[5:0],tx_cfg_req,tx_err_drop,s_axis_tx_tready,s_axis_tx_tdata[127:0],s_axis_tx_tkeep[15:0],s_axis_tx_tlast,s_axis_tx_tvalid,s_axis_tx_tuser[3:0],tx_cfg_gnt,m_axis_rx_tdata[127:0],m_axis_rx_tkeep[15:0],m_axis_rx_tlast,m_axis_rx_tvalid,m_axis_rx_tready,m_axis_rx_tuser[21:0],rx_np_ok,rx_np_req,fc_cpld[11:0],fc_cplh[7:0],fc_npd[11:0],fc_nph[7:0],fc_pd[11:0],fc_ph[7:0],fc_sel[2:0],cfg_status[15:0],cfg_command[15:0],cfg_dstatus[15:0],cfg_dcommand[15:0],cfg_lstatus[15:0],cfg_lcommand[15:0],cfg_dcommand2[15:0],cfg_pcie_link_state[2:0],cfg_pmcsr_pme_en,cfg_pmcsr_powerstate[1:0],cfg_pmcsr_pme_status,cfg_received_func_lvl_rst,cfg_trn_pending,cfg_pm_halt_aspm_l0s,cfg_pm_halt_aspm_l1,cfg_pm_force_state_en,cfg_pm_force_state[1:0],cfg_dsn[63:0],cfg_interrupt,cfg_interrupt_rdy,cfg_interrupt_assert,cfg_interrupt_di[7:0],cfg_interrupt_do[7:0],cfg_interrupt_mmenable[2:0],cfg_interrupt_msienable,cfg_interrupt_msixenable,cfg_interrupt_msixfm,cfg_interrupt_stat,cfg_pciecap_interrupt_msgnum[4:0],cfg_to_turnoff,cfg_turnoff_ok,cfg_bus_number[7:0],cfg_device_number[4:0],cfg_function_number[2:0],cfg_pm_wake,cfg_pm_send_pme_to,cfg_ds_bus_number[7:0],cfg_ds_device_number[4:0],cfg_ds_function_number[2:0],cfg_bridge_serr_en,cfg_slot_control_electromech_il_ctl_pulse,cfg_root_control_syserr_corr_err_en,cfg_root_control_syserr_non_fatal_err_en,cfg_root_control_syserr_fatal_err_en,cfg_root_control_pme_int_en,cfg_aer_rooterr_corr_err_reporting_en,cfg_aer_rooterr_non_fatal_err_reporting_en,cfg_aer_rooterr_fatal_err_reporting_en,cfg_aer_rooterr_corr_err_received,cfg_aer_rooterr_non_fatal_err_received,cfg_aer_rooterr_fatal_err_received,cfg_vc_tcvc_map[6:0],sys_clk,sys_rst_n" */;
  output [7:0]pci_exp_txp;
  output [7:0]pci_exp_txn;
  input [7:0]pci_exp_rxp;
  input [7:0]pci_exp_rxn;
  output user_clk_out;
  output user_reset_out;
  output user_lnk_up;
  output user_app_rdy;
  output [5:0]tx_buf_av;
  output tx_cfg_req;
  output tx_err_drop;
  output s_axis_tx_tready;
  input [127:0]s_axis_tx_tdata;
  input [15:0]s_axis_tx_tkeep;
  input s_axis_tx_tlast;
  input s_axis_tx_tvalid;
  input [3:0]s_axis_tx_tuser;
  input tx_cfg_gnt;
  output [127:0]m_axis_rx_tdata;
  output [15:0]m_axis_rx_tkeep;
  output m_axis_rx_tlast;
  output m_axis_rx_tvalid;
  input m_axis_rx_tready;
  output [21:0]m_axis_rx_tuser;
  input rx_np_ok;
  input rx_np_req;
  output [11:0]fc_cpld;
  output [7:0]fc_cplh;
  output [11:0]fc_npd;
  output [7:0]fc_nph;
  output [11:0]fc_pd;
  output [7:0]fc_ph;
  input [2:0]fc_sel;
  output [15:0]cfg_status;
  output [15:0]cfg_command;
  output [15:0]cfg_dstatus;
  output [15:0]cfg_dcommand;
  output [15:0]cfg_lstatus;
  output [15:0]cfg_lcommand;
  output [15:0]cfg_dcommand2;
  output [2:0]cfg_pcie_link_state;
  output cfg_pmcsr_pme_en;
  output [1:0]cfg_pmcsr_powerstate;
  output cfg_pmcsr_pme_status;
  output cfg_received_func_lvl_rst;
  input cfg_trn_pending;
  input cfg_pm_halt_aspm_l0s;
  input cfg_pm_halt_aspm_l1;
  input cfg_pm_force_state_en;
  input [1:0]cfg_pm_force_state;
  input [63:0]cfg_dsn;
  input cfg_interrupt;
  output cfg_interrupt_rdy;
  input cfg_interrupt_assert;
  input [7:0]cfg_interrupt_di;
  output [7:0]cfg_interrupt_do;
  output [2:0]cfg_interrupt_mmenable;
  output cfg_interrupt_msienable;
  output cfg_interrupt_msixenable;
  output cfg_interrupt_msixfm;
  input cfg_interrupt_stat;
  input [4:0]cfg_pciecap_interrupt_msgnum;
  output cfg_to_turnoff;
  input cfg_turnoff_ok;
  output [7:0]cfg_bus_number;
  output [4:0]cfg_device_number;
  output [2:0]cfg_function_number;
  input cfg_pm_wake;
  input cfg_pm_send_pme_to;
  input [7:0]cfg_ds_bus_number;
  input [4:0]cfg_ds_device_number;
  input [2:0]cfg_ds_function_number;
  output cfg_bridge_serr_en;
  output cfg_slot_control_electromech_il_ctl_pulse;
  output cfg_root_control_syserr_corr_err_en;
  output cfg_root_control_syserr_non_fatal_err_en;
  output cfg_root_control_syserr_fatal_err_en;
  output cfg_root_control_pme_int_en;
  output cfg_aer_rooterr_corr_err_reporting_en;
  output cfg_aer_rooterr_non_fatal_err_reporting_en;
  output cfg_aer_rooterr_fatal_err_reporting_en;
  output cfg_aer_rooterr_corr_err_received;
  output cfg_aer_rooterr_non_fatal_err_received;
  output cfg_aer_rooterr_fatal_err_received;
  output [6:0]cfg_vc_tcvc_map;
  input sys_clk;
  input sys_rst_n;
endmodule
