# ----------------------------------------------------------------------
# Copyright (c) 2016, The Regents of the University of California All
# rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
# 
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
# 
#     * Neither the name of The Regents of the University of California
#       nor the names of its contributors may be used to endorse or
#       promote products derived from this software without specific
#       prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REGENTS OF THE
# UNIVERSITY OF CALIFORNIA BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# ----------------------------------------------------------------------
#----------------------------------------------------------------------------
# Filename:            VC707_Top.xdc
# Version:             1.00.a
# Verilog Standard:    Verilog-2001
# Description:         Xilinx Design Constraints for the VC707 board.
# These constrain the PCIE_REFCLK, its DSBUF, LED Pins, and PCIE_RESET_N pin
#
# Author:              Dustin Richmond (@darichmond)
#-----------------------------------------------------------------------------
#
#########################################################################################################################
# User Constraints
#########################################################################################################################

###############################################################################
# User Time Names / User Time Groups / Time Specs
###############################################################################

###############################################################################
# User Physical Constraints
###############################################################################

#
# LED Status Indicators for Example Design.
# LED 0-2 should be all ON if link is up and functioning correctly
# LED 3 should be blinking if user application is receiving valid clock
#

#System Reset, User Reset, User Link Up, User Clk Heartbeat


#########################################################################################################################
# End User Constraints
#########################################################################################################################
#
#
#
#########################################################################################################################
# PCIE Core Constraints
#########################################################################################################################

#
# SYS reset (input) signal.  The sys_reset_n signal should be
# obtained from the PCI Express interface if possible.  For
# slot based form factors, a system reset signal is usually
# present on the connector.  For cable based form factors, a
# system reset signal may not be available.  In this case, the
# system reset signal must be generated locally by some form of
# supervisory circuit.  You may change the IOSTANDARD and LOC
# to suit your requirements and VCCO voltage banking rules.
# Some 7 series devices do not have 3.3 V I/Os available.
# Therefore the appropriate level shift is required to operate
# with these devices that contain only 1.8 V banks.
#

set_property PACKAGE_PIN AV35 [get_ports PCIE_RESET_N]
set_property IOSTANDARD LVCMOS18 [get_ports PCIE_RESET_N]
set_property PULLUP true [get_ports PCIE_RESET_N]

set_property IOSTANDARD LVCMOS18 [get_ports {LED[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[3]}]

set_property PACKAGE_PIN AM39 [get_ports {LED[0]}]
set_property PACKAGE_PIN AN39 [get_ports {LED[1]}]
set_property PACKAGE_PIN AR37 [get_ports {LED[2]}]
set_property PACKAGE_PIN AT37 [get_ports {LED[3]}]

set_false_path -to [get_ports -filter NAME=~LED*]
#
#
# SYS clock 100 MHz (input) signal. The sys_clk_p and sys_clk_n
# signals are the PCI Express reference clock. Virtex-7 GT
# Transceiver architecture requires the use of a dedicated clock
# resources (FPGA input pins) associated with each GT Transceiver.
# To use these pins an IBUFDS primitive (refclk_ibuf) is
# instantiated in user's design.
# Please refer to the Virtex-7 GT Transceiver User Guide
# (UG) for guidelines regarding clock resource selection.
#
set_property LOC IBUFDS_GTE2_X1Y5 [get_cells refclk_ibuf]

###############################################################################
# Timing Constraints
###############################################################################
create_clock -period 10.000 -name pcie_refclk [get_pins refclk_ibuf/O]

###############################################################################
# Physical Constraints
###############################################################################

set_false_path -from [get_ports PCIE_RESET_N]
###############################################################################
# End
###############################################################################

###### DRAM ######


####################################################################################
# Generated by PlanAhead 14.7 built on 'Fri Sep 27 19:29:51 MDT 2013' by 'xbuild'
####################################################################################


####################################################################################
# Constraints from file : 'main.ucf'
####################################################################################

################################################################################
# UCF for VC707                                            ArchLab. TOKYO TECH #
################################################################################


##################################################################################################
## 
##  Xilinx, Inc. 2010            www.xilinx.com 
##  金 1 16 19:55:13 2015
##  Generated by MIG Version 2.3
##  
##################################################################################################
##  File name :       dram.xdc
##  Details :     Constraints file
##                    FPGA Family:       VIRTEX7
##                    FPGA Part:         XC7VX485T-FFG1761
##                    Speedgrade:        -2
##                    Design Entry:      VERILOG
##                    Frequency:         800 MHz
##                    Time Period:       1250 ps
##################################################################################################

##################################################################################################
## Controller 0
## Memory Device: DDR3_SDRAM->SODIMMs->MT8KTF51264HZ-1G6
## Data Width: 64
## Time Period: 1250
## Data Mask: 1
##################################################################################################

create_clock -period 5.000 [get_ports CLK_P]
#set_propagated_clock CLK_P
          
# Note: CLK_REF FALSE Constraint

set_property CLOCK_DEDICATED_ROUTE FALSE [get_pins -hierarchical *clk_ref_mmcm_gen.mmcm_i*CLKIN1]
          
create_generated_clock -name dram_user_clk [get_pins dramcon/u_dram/u_dram_mig/u_ddr3_infrastructure/gen_mmcm.mmcm_i/CLKFBOUT]
set_clock_groups -asynchronous -group dram_user_clk

#set_false_path -through [get_pins dramcon/u_dram/u_dram_mig/u_ddr3_infrastructure/plle2_i/RST]
set_false_path -through [get_nets rst_out]

############## NET - IOSTANDARD ##################


# PadFunction: IO_L23N_T3_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[0]}]
set_property SLEW FAST [get_ports {DDR3DQ[0]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[0]}]
set_property PACKAGE_PIN N14 [get_ports {DDR3DQ[0]}]

# PadFunction: IO_L22P_T3_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[1]}]
set_property SLEW FAST [get_ports {DDR3DQ[1]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[1]}]
set_property PACKAGE_PIN N13 [get_ports {DDR3DQ[1]}]

# PadFunction: IO_L20N_T3_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[2]}]
set_property SLEW FAST [get_ports {DDR3DQ[2]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[2]}]
set_property PACKAGE_PIN L14 [get_ports {DDR3DQ[2]}]

# PadFunction: IO_L20P_T3_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[3]}]
set_property SLEW FAST [get_ports {DDR3DQ[3]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[3]}]
set_property PACKAGE_PIN M14 [get_ports {DDR3DQ[3]}]

# PadFunction: IO_L24P_T3_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[4]}]
set_property SLEW FAST [get_ports {DDR3DQ[4]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[4]}]
set_property PACKAGE_PIN M12 [get_ports {DDR3DQ[4]}]

# PadFunction: IO_L23P_T3_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[5]}]
set_property SLEW FAST [get_ports {DDR3DQ[5]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[5]}]
set_property PACKAGE_PIN N15 [get_ports {DDR3DQ[5]}]

# PadFunction: IO_L24N_T3_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[6]}]
set_property SLEW FAST [get_ports {DDR3DQ[6]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[6]}]
set_property PACKAGE_PIN M11 [get_ports {DDR3DQ[6]}]

# PadFunction: IO_L19P_T3_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[7]}]
set_property SLEW FAST [get_ports {DDR3DQ[7]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[7]}]
set_property PACKAGE_PIN L12 [get_ports {DDR3DQ[7]}]

# PadFunction: IO_L17P_T2_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[8]}]
set_property SLEW FAST [get_ports {DDR3DQ[8]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[8]}]
set_property PACKAGE_PIN K14 [get_ports {DDR3DQ[8]}]

# PadFunction: IO_L17N_T2_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[9]}]
set_property SLEW FAST [get_ports {DDR3DQ[9]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[9]}]
set_property PACKAGE_PIN K13 [get_ports {DDR3DQ[9]}]

# PadFunction: IO_L14N_T2_SRCC_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[10]}]
set_property SLEW FAST [get_ports {DDR3DQ[10]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[10]}]
set_property PACKAGE_PIN H13 [get_ports {DDR3DQ[10]}]

# PadFunction: IO_L14P_T2_SRCC_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[11]}]
set_property SLEW FAST [get_ports {DDR3DQ[11]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[11]}]
set_property PACKAGE_PIN J13 [get_ports {DDR3DQ[11]}]

# PadFunction: IO_L18P_T2_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[12]}]
set_property SLEW FAST [get_ports {DDR3DQ[12]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[12]}]
set_property PACKAGE_PIN L16 [get_ports {DDR3DQ[12]}]

# PadFunction: IO_L18N_T2_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[13]}]
set_property SLEW FAST [get_ports {DDR3DQ[13]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[13]}]
set_property PACKAGE_PIN L15 [get_ports {DDR3DQ[13]}]

# PadFunction: IO_L13N_T2_MRCC_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[14]}]
set_property SLEW FAST [get_ports {DDR3DQ[14]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[14]}]
set_property PACKAGE_PIN H14 [get_ports {DDR3DQ[14]}]

# PadFunction: IO_L16N_T2_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[15]}]
set_property SLEW FAST [get_ports {DDR3DQ[15]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[15]}]
set_property PACKAGE_PIN J15 [get_ports {DDR3DQ[15]}]

# PadFunction: IO_L7N_T1_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[16]}]
set_property SLEW FAST [get_ports {DDR3DQ[16]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[16]}]
set_property PACKAGE_PIN E15 [get_ports {DDR3DQ[16]}]

# PadFunction: IO_L8N_T1_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[17]}]
set_property SLEW FAST [get_ports {DDR3DQ[17]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[17]}]
set_property PACKAGE_PIN E13 [get_ports {DDR3DQ[17]}]

# PadFunction: IO_L11P_T1_SRCC_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[18]}]
set_property SLEW FAST [get_ports {DDR3DQ[18]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[18]}]
set_property PACKAGE_PIN F15 [get_ports {DDR3DQ[18]}]

# PadFunction: IO_L8P_T1_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[19]}]
set_property SLEW FAST [get_ports {DDR3DQ[19]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[19]}]
set_property PACKAGE_PIN E14 [get_ports {DDR3DQ[19]}]

# PadFunction: IO_L12N_T1_MRCC_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[20]}]
set_property SLEW FAST [get_ports {DDR3DQ[20]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[20]}]
set_property PACKAGE_PIN G13 [get_ports {DDR3DQ[20]}]

# PadFunction: IO_L10P_T1_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[21]}]
set_property SLEW FAST [get_ports {DDR3DQ[21]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[21]}]
set_property PACKAGE_PIN G12 [get_ports {DDR3DQ[21]}]

# PadFunction: IO_L11N_T1_SRCC_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[22]}]
set_property SLEW FAST [get_ports {DDR3DQ[22]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[22]}]
set_property PACKAGE_PIN F14 [get_ports {DDR3DQ[22]}]

# PadFunction: IO_L12P_T1_MRCC_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[23]}]
set_property SLEW FAST [get_ports {DDR3DQ[23]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[23]}]
set_property PACKAGE_PIN G14 [get_ports {DDR3DQ[23]}]

# PadFunction: IO_L2P_T0_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[24]}]
set_property SLEW FAST [get_ports {DDR3DQ[24]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[24]}]
set_property PACKAGE_PIN B14 [get_ports {DDR3DQ[24]}]

# PadFunction: IO_L4N_T0_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[25]}]
set_property SLEW FAST [get_ports {DDR3DQ[25]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[25]}]
set_property PACKAGE_PIN C13 [get_ports {DDR3DQ[25]}]

# PadFunction: IO_L1N_T0_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[26]}]
set_property SLEW FAST [get_ports {DDR3DQ[26]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[26]}]
set_property PACKAGE_PIN B16 [get_ports {DDR3DQ[26]}]

# PadFunction: IO_L5N_T0_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[27]}]
set_property SLEW FAST [get_ports {DDR3DQ[27]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[27]}]
set_property PACKAGE_PIN D15 [get_ports {DDR3DQ[27]}]

# PadFunction: IO_L4P_T0_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[28]}]
set_property SLEW FAST [get_ports {DDR3DQ[28]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[28]}]
set_property PACKAGE_PIN D13 [get_ports {DDR3DQ[28]}]

# PadFunction: IO_L6P_T0_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[29]}]
set_property SLEW FAST [get_ports {DDR3DQ[29]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[29]}]
set_property PACKAGE_PIN E12 [get_ports {DDR3DQ[29]}]

# PadFunction: IO_L1P_T0_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[30]}]
set_property SLEW FAST [get_ports {DDR3DQ[30]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[30]}]
set_property PACKAGE_PIN C16 [get_ports {DDR3DQ[30]}]

# PadFunction: IO_L5P_T0_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[31]}]
set_property SLEW FAST [get_ports {DDR3DQ[31]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[31]}]
set_property PACKAGE_PIN D16 [get_ports {DDR3DQ[31]}]

# PadFunction: IO_L1P_T0_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[32]}]
set_property SLEW FAST [get_ports {DDR3DQ[32]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[32]}]
set_property PACKAGE_PIN A24 [get_ports {DDR3DQ[32]}]

# PadFunction: IO_L4N_T0_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[33]}]
set_property SLEW FAST [get_ports {DDR3DQ[33]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[33]}]
set_property PACKAGE_PIN B23 [get_ports {DDR3DQ[33]}]

# PadFunction: IO_L5N_T0_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[34]}]
set_property SLEW FAST [get_ports {DDR3DQ[34]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[34]}]
set_property PACKAGE_PIN B27 [get_ports {DDR3DQ[34]}]

# PadFunction: IO_L5P_T0_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[35]}]
set_property SLEW FAST [get_ports {DDR3DQ[35]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[35]}]
set_property PACKAGE_PIN B26 [get_ports {DDR3DQ[35]}]

# PadFunction: IO_L2N_T0_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[36]}]
set_property SLEW FAST [get_ports {DDR3DQ[36]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[36]}]
set_property PACKAGE_PIN A22 [get_ports {DDR3DQ[36]}]

# PadFunction: IO_L2P_T0_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[37]}]
set_property SLEW FAST [get_ports {DDR3DQ[37]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[37]}]
set_property PACKAGE_PIN B22 [get_ports {DDR3DQ[37]}]

# PadFunction: IO_L1N_T0_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[38]}]
set_property SLEW FAST [get_ports {DDR3DQ[38]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[38]}]
set_property PACKAGE_PIN A25 [get_ports {DDR3DQ[38]}]

# PadFunction: IO_L6P_T0_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[39]}]
set_property SLEW FAST [get_ports {DDR3DQ[39]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[39]}]
set_property PACKAGE_PIN C24 [get_ports {DDR3DQ[39]}]

# PadFunction: IO_L7N_T1_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[40]}]
set_property SLEW FAST [get_ports {DDR3DQ[40]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[40]}]
set_property PACKAGE_PIN E24 [get_ports {DDR3DQ[40]}]

# PadFunction: IO_L10N_T1_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[41]}]
set_property SLEW FAST [get_ports {DDR3DQ[41]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[41]}]
set_property PACKAGE_PIN D23 [get_ports {DDR3DQ[41]}]

# PadFunction: IO_L11N_T1_SRCC_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[42]}]
set_property SLEW FAST [get_ports {DDR3DQ[42]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[42]}]
set_property PACKAGE_PIN D26 [get_ports {DDR3DQ[42]}]

# PadFunction: IO_L12P_T1_MRCC_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[43]}]
set_property SLEW FAST [get_ports {DDR3DQ[43]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[43]}]
set_property PACKAGE_PIN C25 [get_ports {DDR3DQ[43]}]

# PadFunction: IO_L7P_T1_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[44]}]
set_property SLEW FAST [get_ports {DDR3DQ[44]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[44]}]
set_property PACKAGE_PIN E23 [get_ports {DDR3DQ[44]}]

# PadFunction: IO_L10P_T1_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[45]}]
set_property SLEW FAST [get_ports {DDR3DQ[45]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[45]}]
set_property PACKAGE_PIN D22 [get_ports {DDR3DQ[45]}]

# PadFunction: IO_L8P_T1_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[46]}]
set_property SLEW FAST [get_ports {DDR3DQ[46]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[46]}]
set_property PACKAGE_PIN F22 [get_ports {DDR3DQ[46]}]

# PadFunction: IO_L8N_T1_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[47]}]
set_property SLEW FAST [get_ports {DDR3DQ[47]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[47]}]
set_property PACKAGE_PIN E22 [get_ports {DDR3DQ[47]}]

# PadFunction: IO_L17N_T2_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[48]}]
set_property SLEW FAST [get_ports {DDR3DQ[48]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[48]}]
set_property PACKAGE_PIN A30 [get_ports {DDR3DQ[48]}]

# PadFunction: IO_L13P_T2_MRCC_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[49]}]
set_property SLEW FAST [get_ports {DDR3DQ[49]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[49]}]
set_property PACKAGE_PIN D27 [get_ports {DDR3DQ[49]}]

# PadFunction: IO_L17P_T2_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[50]}]
set_property SLEW FAST [get_ports {DDR3DQ[50]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[50]}]
set_property PACKAGE_PIN A29 [get_ports {DDR3DQ[50]}]

# PadFunction: IO_L14P_T2_SRCC_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[51]}]
set_property SLEW FAST [get_ports {DDR3DQ[51]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[51]}]
set_property PACKAGE_PIN C28 [get_ports {DDR3DQ[51]}]

# PadFunction: IO_L13N_T2_MRCC_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[52]}]
set_property SLEW FAST [get_ports {DDR3DQ[52]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[52]}]
set_property PACKAGE_PIN D28 [get_ports {DDR3DQ[52]}]

# PadFunction: IO_L18N_T2_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[53]}]
set_property SLEW FAST [get_ports {DDR3DQ[53]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[53]}]
set_property PACKAGE_PIN B31 [get_ports {DDR3DQ[53]}]

# PadFunction: IO_L16P_T2_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[54]}]
set_property SLEW FAST [get_ports {DDR3DQ[54]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[54]}]
set_property PACKAGE_PIN A31 [get_ports {DDR3DQ[54]}]

# PadFunction: IO_L16N_T2_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[55]}]
set_property SLEW FAST [get_ports {DDR3DQ[55]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[55]}]
set_property PACKAGE_PIN A32 [get_ports {DDR3DQ[55]}]

# PadFunction: IO_L19P_T3_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[56]}]
set_property SLEW FAST [get_ports {DDR3DQ[56]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[56]}]
set_property PACKAGE_PIN E30 [get_ports {DDR3DQ[56]}]

# PadFunction: IO_L22P_T3_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[57]}]
set_property SLEW FAST [get_ports {DDR3DQ[57]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[57]}]
set_property PACKAGE_PIN F29 [get_ports {DDR3DQ[57]}]

# PadFunction: IO_L24P_T3_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[58]}]
set_property SLEW FAST [get_ports {DDR3DQ[58]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[58]}]
set_property PACKAGE_PIN F30 [get_ports {DDR3DQ[58]}]

# PadFunction: IO_L23N_T3_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[59]}]
set_property SLEW FAST [get_ports {DDR3DQ[59]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[59]}]
set_property PACKAGE_PIN F27 [get_ports {DDR3DQ[59]}]

# PadFunction: IO_L20N_T3_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[60]}]
set_property SLEW FAST [get_ports {DDR3DQ[60]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[60]}]
set_property PACKAGE_PIN C30 [get_ports {DDR3DQ[60]}]

# PadFunction: IO_L22N_T3_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[61]}]
set_property SLEW FAST [get_ports {DDR3DQ[61]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[61]}]
set_property PACKAGE_PIN E29 [get_ports {DDR3DQ[61]}]

# PadFunction: IO_L23P_T3_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[62]}]
set_property SLEW FAST [get_ports {DDR3DQ[62]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[62]}]
set_property PACKAGE_PIN F26 [get_ports {DDR3DQ[62]}]

# PadFunction: IO_L20P_T3_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQ[63]}]
set_property SLEW FAST [get_ports {DDR3DQ[63]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {DDR3DQ[63]}]
set_property PACKAGE_PIN D30 [get_ports {DDR3DQ[63]}]

# PadFunction: IO_L8N_T1_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[15]}]
set_property SLEW FAST [get_ports {DDR3ADDR[15]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[15]}]
set_property PACKAGE_PIN E17 [get_ports {DDR3ADDR[15]}]

# PadFunction: IO_L8P_T1_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[14]}]
set_property SLEW FAST [get_ports {DDR3ADDR[14]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[14]}]
set_property PACKAGE_PIN F17 [get_ports {DDR3ADDR[14]}]

# PadFunction: IO_L5N_T0_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[13]}]
set_property SLEW FAST [get_ports {DDR3ADDR[13]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[13]}]
set_property PACKAGE_PIN A21 [get_ports {DDR3ADDR[13]}]

# PadFunction: IO_L2N_T0_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[12]}]
set_property SLEW FAST [get_ports {DDR3ADDR[12]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[12]}]
set_property PACKAGE_PIN A15 [get_ports {DDR3ADDR[12]}]

# PadFunction: IO_L4P_T0_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[11]}]
set_property SLEW FAST [get_ports {DDR3ADDR[11]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[11]}]
set_property PACKAGE_PIN B17 [get_ports {DDR3ADDR[11]}]

# PadFunction: IO_L5P_T0_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[10]}]
set_property SLEW FAST [get_ports {DDR3ADDR[10]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[10]}]
set_property PACKAGE_PIN B21 [get_ports {DDR3ADDR[10]}]

# PadFunction: IO_L1P_T0_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[9]}]
set_property SLEW FAST [get_ports {DDR3ADDR[9]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[9]}]
set_property PACKAGE_PIN C19 [get_ports {DDR3ADDR[9]}]

# PadFunction: IO_L10N_T1_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[8]}]
set_property SLEW FAST [get_ports {DDR3ADDR[8]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[8]}]
set_property PACKAGE_PIN D17 [get_ports {DDR3ADDR[8]}]

# PadFunction: IO_L6P_T0_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[7]}]
set_property SLEW FAST [get_ports {DDR3ADDR[7]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[7]}]
set_property PACKAGE_PIN C18 [get_ports {DDR3ADDR[7]}]

# PadFunction: IO_L7P_T1_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[6]}]
set_property SLEW FAST [get_ports {DDR3ADDR[6]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[6]}]
set_property PACKAGE_PIN D20 [get_ports {DDR3ADDR[6]}]

# PadFunction: IO_L2P_T0_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[5]}]
set_property SLEW FAST [get_ports {DDR3ADDR[5]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[5]}]
set_property PACKAGE_PIN A16 [get_ports {DDR3ADDR[5]}]

# PadFunction: IO_L4N_T0_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[4]}]
set_property SLEW FAST [get_ports {DDR3ADDR[4]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[4]}]
set_property PACKAGE_PIN A17 [get_ports {DDR3ADDR[4]}]

# PadFunction: IO_L3N_T0_DQS_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[3]}]
set_property SLEW FAST [get_ports {DDR3ADDR[3]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[3]}]
set_property PACKAGE_PIN A19 [get_ports {DDR3ADDR[3]}]

# PadFunction: IO_L7N_T1_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[2]}]
set_property SLEW FAST [get_ports {DDR3ADDR[2]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[2]}]
set_property PACKAGE_PIN C20 [get_ports {DDR3ADDR[2]}]

# PadFunction: IO_L1N_T0_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[1]}]
set_property SLEW FAST [get_ports {DDR3ADDR[1]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[1]}]
set_property PACKAGE_PIN B19 [get_ports {DDR3ADDR[1]}]

# PadFunction: IO_L3P_T0_DQS_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ADDR[0]}]
set_property SLEW FAST [get_ports {DDR3ADDR[0]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ADDR[0]}]
set_property PACKAGE_PIN A20 [get_ports {DDR3ADDR[0]}]

# PadFunction: IO_L10P_T1_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3BA[2]}]
set_property SLEW FAST [get_ports {DDR3BA[2]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3BA[2]}]
set_property PACKAGE_PIN D18 [get_ports {DDR3BA[2]}]

# PadFunction: IO_L9N_T1_DQS_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3BA[1]}]
set_property SLEW FAST [get_ports {DDR3BA[1]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3BA[1]}]
set_property PACKAGE_PIN C21 [get_ports {DDR3BA[1]}]

# PadFunction: IO_L9P_T1_DQS_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3BA[0]}]
set_property SLEW FAST [get_ports {DDR3BA[0]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3BA[0]}]
set_property PACKAGE_PIN D21 [get_ports {DDR3BA[0]}]

# PadFunction: IO_L15N_T2_DQS_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3RAS_N}]
set_property SLEW FAST [get_ports {DDR3RAS_N}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3RAS_N}]
set_property PACKAGE_PIN E20 [get_ports {DDR3RAS_N}]

# PadFunction: IO_L16P_T2_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3CAS_N}]
set_property SLEW FAST [get_ports {DDR3CAS_N}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3CAS_N}]
set_property PACKAGE_PIN K17 [get_ports {DDR3CAS_N}]

# PadFunction: IO_L15P_T2_DQS_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3WE_N}]
set_property SLEW FAST [get_ports {DDR3WE_N}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3WE_N}]
set_property PACKAGE_PIN F20 [get_ports {DDR3WE_N}]

# PadFunction: IO_L14N_T2_SRCC_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3RESET_N}]
set_property SLEW FAST [get_ports {DDR3RESET_N}]
set_property IOSTANDARD LVCMOS15 [get_ports {DDR3RESET_N}]
set_property PACKAGE_PIN C29 [get_ports {DDR3RESET_N}]

# PadFunction: IO_L14P_T2_SRCC_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3CKE[0]}]
set_property SLEW FAST [get_ports {DDR3CKE[0]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3CKE[0]}]
set_property PACKAGE_PIN K19 [get_ports {DDR3CKE[0]}]

# PadFunction: IO_L17N_T2_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3ODT[0]}]
set_property SLEW FAST [get_ports {DDR3ODT[0]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3ODT[0]}]
set_property PACKAGE_PIN H20 [get_ports {DDR3ODT[0]}]

# PadFunction: IO_L16N_T2_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3CS_N[0]}]
set_property SLEW FAST [get_ports {DDR3CS_N[0]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3CS_N[0]}]
set_property PACKAGE_PIN J17 [get_ports {DDR3CS_N[0]}]

# PadFunction: IO_L22N_T3_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DM[0]}]
set_property SLEW FAST [get_ports {DDR3DM[0]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3DM[0]}]
set_property PACKAGE_PIN M13 [get_ports {DDR3DM[0]}]

# PadFunction: IO_L16P_T2_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DM[1]}]
set_property SLEW FAST [get_ports {DDR3DM[1]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3DM[1]}]
set_property PACKAGE_PIN K15 [get_ports {DDR3DM[1]}]

# PadFunction: IO_L10N_T1_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DM[2]}]
set_property SLEW FAST [get_ports {DDR3DM[2]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3DM[2]}]
set_property PACKAGE_PIN F12 [get_ports {DDR3DM[2]}]

# PadFunction: IO_L2N_T0_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DM[3]}]
set_property SLEW FAST [get_ports {DDR3DM[3]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3DM[3]}]
set_property PACKAGE_PIN A14 [get_ports {DDR3DM[3]}]

# PadFunction: IO_L4P_T0_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DM[4]}]
set_property SLEW FAST [get_ports {DDR3DM[4]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3DM[4]}]
set_property PACKAGE_PIN C23 [get_ports {DDR3DM[4]}]

# PadFunction: IO_L11P_T1_SRCC_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DM[5]}]
set_property SLEW FAST [get_ports {DDR3DM[5]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3DM[5]}]
set_property PACKAGE_PIN D25 [get_ports {DDR3DM[5]}]

# PadFunction: IO_L18P_T2_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DM[6]}]
set_property SLEW FAST [get_ports {DDR3DM[6]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3DM[6]}]
set_property PACKAGE_PIN C31 [get_ports {DDR3DM[6]}]

# PadFunction: IO_L24N_T3_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DM[7]}]
set_property SLEW FAST [get_ports {DDR3DM[7]}]
set_property IOSTANDARD SSTL15 [get_ports {DDR3DM[7]}]
set_property PACKAGE_PIN F31 [get_ports {DDR3DM[7]}]

# PadFunction: IO_L12P_T1_MRCC_38 
set_property VCCAUX_IO DONTCARE [get_ports {CLK_P}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {CLK_P}]
set_property PACKAGE_PIN E19 [get_ports {CLK_P}]

# PadFunction: IO_L12N_T1_MRCC_38 
set_property VCCAUX_IO DONTCARE [get_ports {CLK_N}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {CLK_N}]
set_property PACKAGE_PIN E18 [get_ports {CLK_N}]

# PadFunction: IO_L13N_T2_MRCC_15 
#set_property VCCAUX_IO DONTCARE [get_ports {RST_X_IN}]
#set_property IOSTANDARD LVCMOS18 [get_ports {RST_X_IN}]
#set_property PACKAGE_PIN AW40 [get_ports {RST_X_IN}]

# PadFunction: IO_L21P_T3_DQS_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_P[0]}]
set_property SLEW FAST [get_ports {DDR3DQS_P[0]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_P[0]}]
set_property PACKAGE_PIN N16 [get_ports {DDR3DQS_P[0]}]

# PadFunction: IO_L21N_T3_DQS_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_N[0]}]
set_property SLEW FAST [get_ports {DDR3DQS_N[0]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_N[0]}]
set_property PACKAGE_PIN M16 [get_ports {DDR3DQS_N[0]}]

# PadFunction: IO_L15P_T2_DQS_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_P[1]}]
set_property SLEW FAST [get_ports {DDR3DQS_P[1]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_P[1]}]
set_property PACKAGE_PIN K12 [get_ports {DDR3DQS_P[1]}]

# PadFunction: IO_L15N_T2_DQS_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_N[1]}]
set_property SLEW FAST [get_ports {DDR3DQS_N[1]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_N[1]}]
set_property PACKAGE_PIN J12 [get_ports {DDR3DQS_N[1]}]

# PadFunction: IO_L9P_T1_DQS_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_P[2]}]
set_property SLEW FAST [get_ports {DDR3DQS_P[2]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_P[2]}]
set_property PACKAGE_PIN H16 [get_ports {DDR3DQS_P[2]}]

# PadFunction: IO_L9N_T1_DQS_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_N[2]}]
set_property SLEW FAST [get_ports {DDR3DQS_N[2]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_N[2]}]
set_property PACKAGE_PIN G16 [get_ports {DDR3DQS_N[2]}]

# PadFunction: IO_L3P_T0_DQS_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_P[3]}]
set_property SLEW FAST [get_ports {DDR3DQS_P[3]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_P[3]}]
set_property PACKAGE_PIN C15 [get_ports {DDR3DQS_P[3]}]

# PadFunction: IO_L3N_T0_DQS_39 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_N[3]}]
set_property SLEW FAST [get_ports {DDR3DQS_N[3]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_N[3]}]
set_property PACKAGE_PIN C14 [get_ports {DDR3DQS_N[3]}]

# PadFunction: IO_L3P_T0_DQS_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_P[4]}]
set_property SLEW FAST [get_ports {DDR3DQS_P[4]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_P[4]}]
set_property PACKAGE_PIN A26 [get_ports {DDR3DQS_P[4]}]

# PadFunction: IO_L3N_T0_DQS_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_N[4]}]
set_property SLEW FAST [get_ports {DDR3DQS_N[4]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_N[4]}]
set_property PACKAGE_PIN A27 [get_ports {DDR3DQS_N[4]}]

# PadFunction: IO_L9P_T1_DQS_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_P[5]}]
set_property SLEW FAST [get_ports {DDR3DQS_P[5]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_P[5]}]
set_property PACKAGE_PIN F25 [get_ports {DDR3DQS_P[5]}]

# PadFunction: IO_L9N_T1_DQS_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_N[5]}]
set_property SLEW FAST [get_ports {DDR3DQS_N[5]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_N[5]}]
set_property PACKAGE_PIN E25 [get_ports {DDR3DQS_N[5]}]

# PadFunction: IO_L15P_T2_DQS_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_P[6]}]
set_property SLEW FAST [get_ports {DDR3DQS_P[6]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_P[6]}]
set_property PACKAGE_PIN B28 [get_ports {DDR3DQS_P[6]}]

# PadFunction: IO_L15N_T2_DQS_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_N[6]}]
set_property SLEW FAST [get_ports {DDR3DQS_N[6]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_N[6]}]
set_property PACKAGE_PIN B29 [get_ports {DDR3DQS_N[6]}]

# PadFunction: IO_L21P_T3_DQS_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_P[7]}]
set_property SLEW FAST [get_ports {DDR3DQS_P[7]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_P[7]}]
set_property PACKAGE_PIN E27 [get_ports {DDR3DQS_P[7]}]

# PadFunction: IO_L21N_T3_DQS_37 
set_property VCCAUX_IO HIGH [get_ports {DDR3DQS_N[7]}]
set_property SLEW FAST [get_ports {DDR3DQS_N[7]}]
set_property IOSTANDARD DIFF_SSTL15_T_DCI [get_ports {DDR3DQS_N[7]}]
set_property PACKAGE_PIN E28 [get_ports {DDR3DQS_N[7]}]

# PadFunction: IO_L13P_T2_MRCC_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3CK_P[0]}]
set_property SLEW FAST [get_ports {DDR3CK_P[0]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {DDR3CK_P[0]}]
set_property PACKAGE_PIN H19 [get_ports {DDR3CK_P[0]}]

# PadFunction: IO_L13N_T2_MRCC_38 
set_property VCCAUX_IO HIGH [get_ports {DDR3CK_N[0]}]
set_property SLEW FAST [get_ports {DDR3CK_N[0]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {DDR3CK_N[0]}]
set_property PACKAGE_PIN G18 [get_ports {DDR3CK_N[0]}]



set_property LOC PHASER_OUT_PHY_X1Y19 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/phaser_out}]
set_property LOC PHASER_OUT_PHY_X1Y18 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/phaser_out}]
set_property LOC PHASER_OUT_PHY_X1Y17 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/phaser_out}]
set_property LOC PHASER_OUT_PHY_X1Y16 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_A.ddr_byte_lane_A/phaser_out}]
set_property LOC PHASER_OUT_PHY_X1Y23 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/phaser_out}]
set_property LOC PHASER_OUT_PHY_X1Y22 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/phaser_out}]
set_property LOC PHASER_OUT_PHY_X1Y21 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/phaser_out}]
set_property LOC PHASER_OUT_PHY_X1Y27 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/phaser_out}]
set_property LOC PHASER_OUT_PHY_X1Y26 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/phaser_out}]
set_property LOC PHASER_OUT_PHY_X1Y25 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/phaser_out}]
set_property LOC PHASER_OUT_PHY_X1Y24 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_A.ddr_byte_lane_A/phaser_out}]

set_property LOC PHASER_IN_PHY_X1Y19 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/phaser_in_gen.phaser_in}]
set_property LOC PHASER_IN_PHY_X1Y18 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/phaser_in_gen.phaser_in}]
set_property LOC PHASER_IN_PHY_X1Y17 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/phaser_in_gen.phaser_in}]
set_property LOC PHASER_IN_PHY_X1Y16 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_A.ddr_byte_lane_A/phaser_in_gen.phaser_in}]
## set_property LOC PHASER_IN_PHY_X1Y23 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/phaser_in_gen.phaser_in}]
## set_property LOC PHASER_IN_PHY_X1Y22 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/phaser_in_gen.phaser_in}]
## set_property LOC PHASER_IN_PHY_X1Y21 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/phaser_in_gen.phaser_in}]
set_property LOC PHASER_IN_PHY_X1Y27 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/phaser_in_gen.phaser_in}]
set_property LOC PHASER_IN_PHY_X1Y26 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/phaser_in_gen.phaser_in}]
set_property LOC PHASER_IN_PHY_X1Y25 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/phaser_in_gen.phaser_in}]
set_property LOC PHASER_IN_PHY_X1Y24 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_A.ddr_byte_lane_A/phaser_in_gen.phaser_in}]



set_property LOC OUT_FIFO_X1Y19 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/out_fifo}]
set_property LOC OUT_FIFO_X1Y18 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/out_fifo}]
set_property LOC OUT_FIFO_X1Y17 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/out_fifo}]
set_property LOC OUT_FIFO_X1Y16 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_A.ddr_byte_lane_A/out_fifo}]
set_property LOC OUT_FIFO_X1Y23 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/out_fifo}]
set_property LOC OUT_FIFO_X1Y22 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/out_fifo}]
set_property LOC OUT_FIFO_X1Y21 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/out_fifo}]
set_property LOC OUT_FIFO_X1Y27 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/out_fifo}]
set_property LOC OUT_FIFO_X1Y26 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/out_fifo}]
set_property LOC OUT_FIFO_X1Y25 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/out_fifo}]
set_property LOC OUT_FIFO_X1Y24 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_A.ddr_byte_lane_A/out_fifo}]

set_property LOC IN_FIFO_X1Y19 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/in_fifo_gen.in_fifo}]
set_property LOC IN_FIFO_X1Y18 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/in_fifo_gen.in_fifo}]
set_property LOC IN_FIFO_X1Y17 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/in_fifo_gen.in_fifo}]
set_property LOC IN_FIFO_X1Y16 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_A.ddr_byte_lane_A/in_fifo_gen.in_fifo}]
set_property LOC IN_FIFO_X1Y27 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/in_fifo_gen.in_fifo}]
set_property LOC IN_FIFO_X1Y26 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/in_fifo_gen.in_fifo}]
set_property LOC IN_FIFO_X1Y25 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/in_fifo_gen.in_fifo}]
set_property LOC IN_FIFO_X1Y24 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_A.ddr_byte_lane_A/in_fifo_gen.in_fifo}]

set_property LOC PHY_CONTROL_X1Y4 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/phy_control_i}]
set_property LOC PHY_CONTROL_X1Y5 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/phy_control_i}]
set_property LOC PHY_CONTROL_X1Y6 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/phy_control_i}]

set_property LOC PHASER_REF_X1Y4 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/phaser_ref_i}]
set_property LOC PHASER_REF_X1Y5 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_1.u_ddr_phy_4lanes/phaser_ref_i}]
set_property LOC PHASER_REF_X1Y6 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/phaser_ref_i}]

set_property LOC OLOGIC_X1Y243 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/ddr_byte_group_io/*slave_ts}]
set_property LOC OLOGIC_X1Y231 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/ddr_byte_group_io/*slave_ts}]
set_property LOC OLOGIC_X1Y219 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/ddr_byte_group_io/*slave_ts}]
set_property LOC OLOGIC_X1Y207 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_2.u_ddr_phy_4lanes/ddr_byte_lane_A.ddr_byte_lane_A/ddr_byte_group_io/*slave_ts}]
set_property LOC OLOGIC_X1Y343 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_D.ddr_byte_lane_D/ddr_byte_group_io/*slave_ts}]
set_property LOC OLOGIC_X1Y331 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_C.ddr_byte_lane_C/ddr_byte_group_io/*slave_ts}]
set_property LOC OLOGIC_X1Y319 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_B.ddr_byte_lane_B/ddr_byte_group_io/*slave_ts}]
set_property LOC OLOGIC_X1Y307 [get_cells  -hier -filter {NAME =~ */ddr_phy_4lanes_0.u_ddr_phy_4lanes/ddr_byte_lane_A.ddr_byte_lane_A/ddr_byte_group_io/*slave_ts}]

set_property LOC PLLE2_ADV_X1Y5 [get_cells -hier -filter {NAME =~ */u_ddr3_infrastructure/plle2_i}]
set_property LOC MMCME2_ADV_X1Y5 [get_cells -hier -filter {NAME =~ */u_ddr3_infrastructure/gen_mmcm.mmcm_i}]


set_multicycle_path -from [get_cells -hier -filter {NAME =~ */mc0/mc_read_idle_r_reg}] \
                    -to   [get_cells -hier -filter {NAME =~ */input_[?].iserdes_dq_.iserdesdq}] \
                    -setup 6

set_multicycle_path -from [get_cells -hier -filter {NAME =~ */mc0/mc_read_idle_r_reg}] \
                    -to   [get_cells -hier -filter {NAME =~ */input_[?].iserdes_dq_.iserdesdq}] \
                    -hold 5

#set_multicycle_path -from [get_cells -hier -filter {NAME =~ */mc0/mc_read_idle_r*}] \
#                    -to   [get_cells -hier -filter {NAME =~ */input_[?].iserdes_dq_.iserdesdq}] \
#                    -setup 6

#set_multicycle_path -from [get_cells -hier -filter {NAME =~ */mc0/mc_read_idle_r*}] \
#                    -to   [get_cells -hier -filter {NAME =~ */input_[?].iserdes_dq_.iserdesdq}] \
#                    -hold 5

#set_max_delay -from [get_cells -hier -filter {NAME =~ */u_phase_detector && IS_SEQUENTIAL}] -to [get_cells -hier -filter {NAME =~ *pos_edge_samp*}] 1.250000
#set_max_delay -from [get_cells -hier -filter {NAME =~ */u_phase_detector && IS_SEQUENTIAL}] -to [get_cells -hier -filter {NAME =~ *neg_edge_samp*}] 1.250000
          
set_false_path -through [get_pins -filter {NAME =~ */DQSFOUND} -of [get_cells -hier -filter {REF_NAME == PHASER_IN_PHY}]]

set_multicycle_path -through [get_pins -filter {NAME =~ */OSERDESRST} -of [get_cells -hier -filter {REF_NAME == PHASER_OUT_PHY}]] -setup 2 -start
set_multicycle_path -through [get_pins -filter {NAME =~ */OSERDESRST} -of [get_cells -hier -filter {REF_NAME == PHASER_OUT_PHY}]] -hold 1 -start

set_max_delay -datapath_only -from [get_cells -hier -filter {NAME =~ *temp_mon_enabled.u_tempmon/* && IS_SEQUENTIAL}] -to [get_cells -hier -filter {NAME =~ *temp_mon_enabled.u_tempmon/device_temp_sync_r1*}] 20
set_max_delay -from [get_cells -hier *rstdiv0_sync_r1_reg*] -to [get_pins -filter {NAME =~ */RESET} -of [get_cells -hier -filter {REF_NAME == PHY_CONTROL}]] -datapath_only 5
#set_max_delay -datapath_only -from [get_cells -hier -filter {NAME =~ *temp_mon_enabled.u_tempmon/*}] -to [get_cells -hier -filter {NAME =~ *temp_mon_enabled.u_tempmon/device_temp_sync_r1*}] 20
#set_max_delay -from [get_cells -hier rstdiv0_sync_r1*] -to [get_pins -filter {NAME =~ */RESET} -of [get_cells -hier -filter {REF_NAME == PHY_CONTROL}]] -datapath_only 5
          
set_max_delay -datapath_only -from [get_cells -hier -filter {NAME =~ *ddr3_infrastructure/rstdiv0_sync_r1_reg*}] -to [get_cells -hier -filter {NAME =~ *temp_mon_enabled.u_tempmon/xadc_supplied_temperature.rst_r1*}] 20
#set_max_delay -datapath_only -from [get_cells -hier -filter {NAME =~ *ddr3_infrastructure/rstdiv0_sync_r1*}] -to [get_cells -hier -filter {NAME =~ *temp_mon_enabled.u_tempmon/*rst_r1*}] 20
          




          

          
