
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
@
.Phase 1 Build RT Design | Checksum: 11d1bd760
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:03:14 ; elapsed = 00:02:46 . Memory (MB): peak = 2222.383 ; gain = 124.9922default:defaulth px
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
?
-Phase 2.1 Create Timer | Checksum: 11d1bd760
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:03:15 ; elapsed = 00:02:48 . Memory (MB): peak = 2222.383 ; gain = 124.9922default:defaulth px
q

Phase %s%s
101*constraints2
2.2 2default:default2%
Pre Route Cleanup2default:defaultZ18-101h px
D
2Phase 2.2 Pre Route Cleanup | Checksum: 11d1bd760
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:03:16 ; elapsed = 00:02:49 . Memory (MB): peak = 2236.098 ; gain = 138.7072default:defaulth px
m

Phase %s%s
101*constraints2
2.3 2default:default2!
Update Timing2default:defaultZ18-101h px
@
.Phase 2.3 Update Timing | Checksum: 163d79aa3
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:03:57 ; elapsed = 00:03:14 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
�
Estimated Timing Summary %s
57*route2L
8| WNS=0.142  | TNS=0      | WHS=-0.473 | THS=-3.29e+03|
2default:defaultZ35-57h px
F
4Phase 2 Router Initialization | Checksum: 1310bd49f
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:04:11 ; elapsed = 00:03:22 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
m

Phase %s%s
101*constraints2
3 2default:default2#
Initial Routing2default:defaultZ18-101h px
@
.Phase 3 Initial Routing | Checksum: 10ad21800
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:04:38 ; elapsed = 00:03:37 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
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
0Phase 4.1.1 Update Timing | Checksum: 134d5d8b3
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:15 ; elapsed = 00:03:57 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
�
Estimated Timing Summary %s
57*route2J
6| WNS=0.0776 | TNS=0      | WHS=N/A    | THS=N/A    |
2default:defaultZ35-57h px
D
2Phase 4.1 Global Iteration 0 | Checksum: eb2b3eab
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:15 ; elapsed = 00:03:58 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
B
0Phase 4 Rip-up And Reroute | Checksum: eb2b3eab
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:16 ; elapsed = 00:03:58 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
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
?
-Phase 5.1 Update Timing | Checksum: df198591
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:21 ; elapsed = 00:04:01 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
�
Estimated Timing Summary %s
57*route2J
6| WNS=0.0776 | TNS=0      | WHS=N/A    | THS=N/A    |
2default:defaultZ35-57h px
=
+Phase 5 Delay CleanUp | Checksum: df198591
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:21 ; elapsed = 00:04:01 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
u

Phase %s%s
101*constraints2
6 2default:default2+
Clock Skew Optimization2default:defaultZ18-101h px
G
5Phase 6 Clock Skew Optimization | Checksum: df198591
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:22 ; elapsed = 00:04:01 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
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
.Phase 7.1 Update Timing | Checksum: 129a76d22
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:30 ; elapsed = 00:04:06 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
�
Estimated Timing Summary %s
57*route2J
6| WNS=0.0776 | TNS=0      | WHS=0.042  | THS=0      |
2default:defaultZ35-57h px
>
,Phase 7 Post Hold Fix | Checksum: 140990480
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:31 ; elapsed = 00:04:06 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
l

Phase %s%s
101*constraints2
8 2default:default2"
Route finalize2default:defaultZ18-101h px
?
-Phase 8 Route finalize | Checksum: 149ccecbd
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:31 ; elapsed = 00:04:07 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
s

Phase %s%s
101*constraints2
9 2default:default2)
Verifying routed nets2default:defaultZ18-101h px
F
4Phase 9 Verifying routed nets | Checksum: 149ccecbd
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:31 ; elapsed = 00:04:07 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
p

Phase %s%s
101*constraints2
10 2default:default2%
Depositing Routes2default:defaultZ18-101h px
C
1Phase 10 Depositing Routes | Checksum: 1ecb82442
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:35 ; elapsed = 00:04:11 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
q

Phase %s%s
101*constraints2
11 2default:default2&
Post Router Timing2default:defaultZ18-101h px
�
Estimated Timing Summary %s
57*route2J
6| WNS=0.0776 | TNS=0      | WHS=0.042  | THS=0      |
2default:defaultZ35-57h px
�
�The final timing numbers are based on the router estimated timing analysis. For a complete and accurate timing signoff, please run report_timing_summary.
127*routeZ35-327h px
D
2Phase 11 Post Router Timing | Checksum: 1ecb82442
*commonh px
�

%s
*constraints2q
]Time (s): cpu = 00:05:36 ; elapsed = 00:04:11 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
:
Router Completed Successfully
16*routeZ35-16h px
�

%s
*constraints2q
]Time (s): cpu = 00:00:00 ; elapsed = 00:04:11 . Memory (MB): peak = 2363.059 ; gain = 265.6682default:defaulth px
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
00:05:452default:default2
00:04:172default:default2
2363.0592default:default2
428.8552default:defaultZ17-268h px
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
00:00:152default:default2
00:00:102default:default2
2363.0592default:default2
0.0002default:defaultZ17-268h px
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2&
write_checkpoint: 2default:default2
00:00:232default:default2
00:00:192default:default2
2363.0592default:default2
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
00:00:232default:default2
00:00:152default:default2
2484.9062default:default2
121.8482default:defaultZ17-268h px
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
00:00:452default:default2
00:00:272default:default2
2518.6802default:default2
33.7732default:defaultZ17-268h px
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
00:00:172default:default2
00:00:122default:default2
2544.2072default:default2
25.5272default:defaultZ17-268h px


End Record