
N
Command: %s
53*	vivadotcl2 
route_design2default:defaultZ4-113h px
�
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2"
Implementation2default:default2
	xc7vx485t2default:defaultZ17-347h px
�
0Got license for feature '%s' and/or device '%s'
310*common2"
Implementation2default:default2
	xc7vx485t2default:defaultZ17-349h px
m
,Running DRC as a precondition to command %s
22*	vivadotcl2 
route_design2default:defaultZ4-22h px
M
Running DRC with %s threads
24*drc2
22default:defaultZ23-27h px
�	
Rule violation (%s) %s - %s
20*drc2
PLCK-232default:default2'
Clock Placer Checks2default:default2�
�Sub-optimal placement for a clock-capable IO pin and MMCM pair. 
Resolution: A dedicated routing path between the two can be used if: (a) The clock-capable IO (CCIO) is placed on a CCIO capable site (b) The MMCM is placed in the same clock region as the CCIO pin. If the IOB is driving multiple MMCMs, all MMCMs must be placed in the same clock region, one clock region above or one clock region below the IOB. Both the above conditions must be met at the same time, else it may lead to longer and less predictable clock insertion delays.
This is normally an ERROR but the CLOCK_DEDICATED_ROUTE constraint is set to FALSE allowing your design to continue. The use of this override is highly discouraged as it may lead to very poor timing results. It is recommended that this error condition be corrected in the design.

	dramcon/u_dram/u_dram_mig/u_ddr3_clk_ibuf/diff_input_clk.u_ibufg_sys_clk (IBUFDS.O) is locked to E19
	dramcon/u_dram/u_dram_mig/u_iodelay_ctrl/clk_ref_mmcm_gen.mmcm_i (MMCME2_ADV.CLKIN1) is provisionally placed by clockplacer on MMCME2_ADV_X1Y0
2default:defaultZ23-20h px
_
DRC finished with %s
79*	vivadotcl2(
0 Errors, 1 Warnings2default:defaultZ4-198h px
b
BPlease refer to the DRC report (report_drc) for more information.
80*	vivadotclZ4-199h px
S

Starting %s Task
103*constraints2
Routing2default:defaultZ18-103h px
v
BMultithreading enabled for route_design using a maximum of %s CPUs97*route2
22default:defaultZ35-254h px
m

Phase %s%s
101*constraints2
1 2default:default2#
Build RT Design2default:defaultZ18-101h px
?
-Phase 1 Build RT Design | Checksum: ea7ca64e
*commonh px
�

%s
*constraints2p
\Time (s): cpu = 00:03:11 ; elapsed = 00:02:38 . Memory (MB): peak = 2700.910 ; gain = 48.4382default:defaulth px
s

Phase %s%s
101*constraints2
2 2default:default2)
Router Initialization2default:defaultZ18-101h px
l

Phase %s%s
101*constraints2
2.1 2default:default2 
Create Timer2default:defaultZ18-101h px
>
,Phase 2.1 Create Timer | Checksum: ea7ca64e
*commonh px
�

%s
*constraints2p
\Time (s): cpu = 00:03:14 ; elapsed = 00:02:41 . Memory (MB): peak = 2700.910 ; gain = 48.4382default:defaulth px
q

Phase %s%s
101*constraints2
2.2 2default:default2%
Pre Route Cleanup2default:defaultZ18-101h px
C
1Phase 2.2 Pre Route Cleanup | Checksum: ea7ca64e
*commonh px
�

%s
*constraints2p
\Time (s): cpu = 00:03:15 ; elapsed = 00:02:41 . Memory (MB): peak = 2713.266 ; gain = 60.7932default:defaulth px
m

Phase %s%s
101*constraints2
2.3 2default:default2!
Update Timing2default:defaultZ18-101h px
@
.Phase 2.3 Update Timing | Checksum: 12907e517
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:04:22 ; elapsed = 00:03:22 . Memory (MB): peak = 2898.219 ; gain = 245.7462default:defaulth px
�
Estimated Timing Summary %s
57*route2K
7| WNS=0.142  | TNS=0      | WHS=-0.473 | THS=-3.4e+03|
2default:defaultZ35-57h px
F
4Phase 2 Router Initialization | Checksum: 1260fedd5
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:04:45 ; elapsed = 00:03:36 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
m

Phase %s%s
101*constraints2
3 2default:default2#
Initial Routing2default:defaultZ18-101h px
@
.Phase 3 Initial Routing | Checksum: 1b60aab9c
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:39 ; elapsed = 00:04:05 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
p

Phase %s%s
101*constraints2
4 2default:default2&
Rip-up And Reroute2default:defaultZ18-101h px
r

Phase %s%s
101*constraints2
4.1 2default:default2&
Global Iteration 02default:defaultZ18-101h px
o

Phase %s%s
101*constraints2
4.1.1 2default:default2!
Update Timing2default:defaultZ18-101h px
B
0Phase 4.1.1 Update Timing | Checksum: 21391eaa0
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:09 ; elapsed = 00:04:56 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
�
Estimated Timing Summary %s
57*route2J
6| WNS=0.0166 | TNS=0      | WHS=N/A    | THS=N/A    |
2default:defaultZ35-57h px
E
3Phase 4.1 Global Iteration 0 | Checksum: 1b247691a
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:11 ; elapsed = 00:04:57 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
C
1Phase 4 Rip-up And Reroute | Checksum: 1b247691a
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:11 ; elapsed = 00:04:57 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
k

Phase %s%s
101*constraints2
5 2default:default2!
Delay CleanUp2default:defaultZ18-101h px
m

Phase %s%s
101*constraints2
5.1 2default:default2!
Update Timing2default:defaultZ18-101h px
@
.Phase 5.1 Update Timing | Checksum: 2107cb425
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:20 ; elapsed = 00:05:03 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
�
Estimated Timing Summary %s
57*route2J
6| WNS=0.0296 | TNS=0      | WHS=N/A    | THS=N/A    |
2default:defaultZ35-57h px
>
,Phase 5 Delay CleanUp | Checksum: 2107cb425
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:21 ; elapsed = 00:05:03 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
u

Phase %s%s
101*constraints2
6 2default:default2+
Clock Skew Optimization2default:defaultZ18-101h px
H
6Phase 6 Clock Skew Optimization | Checksum: 2107cb425
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:21 ; elapsed = 00:05:03 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
k

Phase %s%s
101*constraints2
7 2default:default2!
Post Hold Fix2default:defaultZ18-101h px
m

Phase %s%s
101*constraints2
7.1 2default:default2!
Update Timing2default:defaultZ18-101h px
@
.Phase 7.1 Update Timing | Checksum: 16c6ccb6a
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:35 ; elapsed = 00:05:11 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
�
Estimated Timing Summary %s
57*route2J
6| WNS=0.0296 | TNS=0      | WHS=0.006  | THS=0      |
2default:defaultZ35-57h px
>
,Phase 7 Post Hold Fix | Checksum: 221548886
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:35 ; elapsed = 00:05:12 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
l

Phase %s%s
101*constraints2
8 2default:default2"
Route finalize2default:defaultZ18-101h px
?
-Phase 8 Route finalize | Checksum: 1be53c9b4
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:36 ; elapsed = 00:05:12 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
s

Phase %s%s
101*constraints2
9 2default:default2)
Verifying routed nets2default:defaultZ18-101h px
F
4Phase 9 Verifying routed nets | Checksum: 1be53c9b4
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:37 ; elapsed = 00:05:13 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
p

Phase %s%s
101*constraints2
10 2default:default2%
Depositing Routes2default:defaultZ18-101h px
C
1Phase 10 Depositing Routes | Checksum: 24d6a04aa
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:45 ; elapsed = 00:05:21 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
q

Phase %s%s
101*constraints2
11 2default:default2&
Post Router Timing2default:defaultZ18-101h px
�
Estimated Timing Summary %s
57*route2J
6| WNS=0.0296 | TNS=0      | WHS=0.006  | THS=0      |
2default:defaultZ35-57h px
�
�The final timing numbers are based on the router estimated timing analysis. For a complete and accurate timing signoff, please run report_timing_summary.
127*routeZ35-327h px
D
2Phase 11 Post Router Timing | Checksum: 24d6a04aa
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:07:45 ; elapsed = 00:05:21 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
:
Router Completed Successfully
16*routeZ35-16h px
�

%s
*constraints2q
]Time (s): cpu = 00:00:00 ; elapsed = 00:05:21 . Memory (MB): peak = 2912.828 ; gain = 260.3552default:defaulth px
W
Releasing license: %s
83*common2"
Implementation2default:defaultZ17-83h px
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
572default:default2
22default:default2
02default:default2
02default:defaultZ4-41h px
[
%s completed successfully
29*	vivadotcl2 
route_design2default:defaultZ4-42h px
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
route_design: 2default:default2
00:07:592default:default2
00:05:292default:default2
2912.8282default:default2
439.4412default:defaultZ17-268h px
A
Writing placer database...
1603*designutilsZ20-1893h px
:
Writing XDEF routing.
211*designutilsZ20-211h px
G
#Writing XDEF routing logical nets.
209*designutilsZ20-209h px
G
#Writing XDEF routing special nets.
210*designutilsZ20-210h px
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2)
Write XDEF Complete: 2default:default2
00:00:282default:default2
00:00:202default:default2
2912.8282default:default2
0.0002default:defaultZ17-268h px
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2&
write_checkpoint: 2default:default2
00:00:412default:default2
00:00:332default:default2
2912.8282default:default2
0.0002default:defaultZ17-268h px
M
Running DRC with %s threads
24*drc2
22default:defaultZ23-27h px
�
#The results of DRC are in file %s.
168*coretcl2�
WC:/Users/kobayashi/Desktop/FPGASort/trunk/vivado/vivado.runs/impl_1/main_drc_routed.rptWC:/Users/kobayashi/Desktop/FPGASort/trunk/vivado/vivado.runs/impl_1/main_drc_routed.rpt2default:default8Z2-168h px
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2 
report_drc: 2default:default2
00:00:372default:default2
00:00:242default:default2
3164.1052default:default2
251.2772default:defaultZ17-268h px
o
UpdateTimingParams:%s.
91*timing29
% Speed grade: -2, Delay Type: min_max2default:defaultZ38-91h px
y
CMultithreading enabled for timing update using a maximum of %s CPUs155*timing2
22default:defaultZ38-191h px
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2+
report_timing_summary: 2default:default2
00:01:242default:default2
00:00:512default:default2
3370.9222default:default2
206.8162default:defaultZ17-268h px
H
,Running Vector-less Activity Propagation...
51*powerZ33-51h px
M
3
Finished Running Vector-less Activity Propagation
1*powerZ33-1h px
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
report_power: 2default:default2
00:00:282default:default2
00:00:212default:default2
3403.5742default:default2
32.6522default:defaultZ17-268h px


End Record