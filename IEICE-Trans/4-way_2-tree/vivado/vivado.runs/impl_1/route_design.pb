
Q
Command: %s
53*	vivadotcl2 
route_design2default:defaultZ4-113h px� 
�
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2"
Implementation2default:default2
	xc7vx485t2default:defaultZ17-347h px� 
�
0Got license for feature '%s' and/or device '%s'
310*common2"
Implementation2default:default2
	xc7vx485t2default:defaultZ17-349h px� 
p
,Running DRC as a precondition to command %s
22*	vivadotcl2 
route_design2default:defaultZ4-22h px� 
P
Running DRC with %s threads
24*drc2
82default:defaultZ23-27h px� 
�	
Rule violation (%s) %s - %s
20*drc2
PLCK-232default:default2'
Clock Placer Checks2default:default2�
�Sub-optimal placement for a clock-capable IO pin and MMCM pair. 
Resolution: A dedicated routing path between the two can be used if: (a) The clock-capable IO (CCIO) is placed on a CCIO capable site (b) The MMCM is placed in the same clock region as the CCIO pin. If the IOB is driving multiple MMCMs, all MMCMs must be placed in the same clock region, one clock region above or one clock region below the IOB. Both the above conditions must be met at the same time, else it may lead to longer and less predictable clock insertion delays.
This is normally an ERROR but the CLOCK_DEDICATED_ROUTE constraint is set to FALSE allowing your design to continue. The use of this override is highly discouraged as it may lead to very poor timing results. It is recommended that this error condition be corrected in the design.

	dramcon/u_dram/u_dram_mig/u_ddr3_clk_ibuf/diff_input_clk.u_ibufg_sys_clk (IBUFDS.O) is locked to E19
	dramcon/u_dram/u_dram_mig/u_iodelay_ctrl/clk_ref_mmcm_gen.mmcm_i (MMCME2_ADV.CLKIN1) is provisionally placed by clockplacer on MMCME2_ADV_X0Y6
2default:defaultZ23-20h px� 
b
DRC finished with %s
79*	vivadotcl2(
0 Errors, 1 Warnings2default:defaultZ4-198h px� 
e
BPlease refer to the DRC report (report_drc) for more information.
80*	vivadotclZ4-199h px� 
V

Starting %s Task
103*constraints2
Routing2default:defaultZ18-103h px� 
y
BMultithreading enabled for route_design using a maximum of %s CPUs97*route2
82default:defaultZ35-254h px� 
p

Phase %s%s
101*constraints2
1 2default:default2#
Build RT Design2default:defaultZ18-101h px� 
C
.Phase 1 Build RT Design | Checksum: 10c3261df
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:01:16 ; elapsed = 00:00:57 . Memory (MB): peak = 2966.836 ; gain = 115.535 ; free physical = 2691 ; free virtual = 162522default:defaulth px� 
v

Phase %s%s
101*constraints2
2 2default:default2)
Router Initialization2default:defaultZ18-101h px� 
o

Phase %s%s
101*constraints2
2.1 2default:default2 
Create Timer2default:defaultZ18-101h px� 
B
-Phase 2.1 Create Timer | Checksum: 10c3261df
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:01:17 ; elapsed = 00:01:03 . Memory (MB): peak = 2966.836 ; gain = 115.535 ; free physical = 2668 ; free virtual = 162402default:defaulth px� 
t

Phase %s%s
101*constraints2
2.2 2default:default2%
Pre Route Cleanup2default:defaultZ18-101h px� 
G
2Phase 2.2 Pre Route Cleanup | Checksum: 10c3261df
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:01:18 ; elapsed = 00:01:17 . Memory (MB): peak = 3002.605 ; gain = 151.305 ; free physical = 2513 ; free virtual = 161342default:defaulth px� 
p

Phase %s%s
101*constraints2
2.3 2default:default2!
Update Timing2default:defaultZ18-101h px� 
C
.Phase 2.3 Update Timing | Checksum: 13155b239
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:02:22 ; elapsed = 00:02:07 . Memory (MB): peak = 3061.316 ; gain = 210.016 ; free physical = 2087 ; free virtual = 158432default:defaulth px� 
�
Intermediate Timing Summary %s164*route2L
8| WNS=0.141  | TNS=0.000  | WHS=-0.559 | THS=-4525.419|
2default:defaultZ35-416h px� 
I
4Phase 2 Router Initialization | Checksum: 12277f3a0
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:02:51 ; elapsed = 00:02:17 . Memory (MB): peak = 3061.316 ; gain = 210.016 ; free physical = 1998 ; free virtual = 157922default:defaulth px� 
p

Phase %s%s
101*constraints2
3 2default:default2#
Initial Routing2default:defaultZ18-101h px� 
C
.Phase 3 Initial Routing | Checksum: 13d996ea0
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:03:58 ; elapsed = 00:02:51 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 3438 ; free virtual = 183252default:defaulth px� 
s

Phase %s%s
101*constraints2
4 2default:default2&
Rip-up And Reroute2default:defaultZ18-101h px� 
u

Phase %s%s
101*constraints2
4.1 2default:default2&
Global Iteration 02default:defaultZ18-101h px� 
r

Phase %s%s
101*constraints2
4.1.1 2default:default2!
Update Timing2default:defaultZ18-101h px� 
E
0Phase 4.1.1 Update Timing | Checksum: 112f72c74
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:29:11 ; elapsed = 00:16:55 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 291 ; free virtual = 149372default:defaulth px� 
�
Intermediate Timing Summary %s164*route2J
6| WNS=-0.047 | TNS=-0.305 | WHS=N/A    | THS=N/A    |
2default:defaultZ35-416h px� 
v

Phase %s%s
101*constraints2
4.1.2 2default:default2%
GlobIterForTiming2default:defaultZ18-101h px� 
t

Phase %s%s
101*constraints2
4.1.2.1 2default:default2!
Update Timing2default:defaultZ18-101h px� 
G
2Phase 4.1.2.1 Update Timing | Checksum: 115fdb99e
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:29:18 ; elapsed = 00:17:03 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 263 ; free virtual = 149102default:defaulth px� 
u

Phase %s%s
101*constraints2
4.1.2.2 2default:default2"
Fast Budgeting2default:defaultZ18-101h px� 
G
2Phase 4.1.2.2 Fast Budgeting | Checksum: c725f796
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:29:24 ; elapsed = 00:17:09 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 315 ; free virtual = 149622default:defaulth px� 
I
4Phase 4.1.2 GlobIterForTiming | Checksum: 1738c6a09
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:29:25 ; elapsed = 00:17:10 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 315 ; free virtual = 149622default:defaulth px� 
H
3Phase 4.1 Global Iteration 0 | Checksum: 1738c6a09
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:29:25 ; elapsed = 00:17:11 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 315 ; free virtual = 149622default:defaulth px� 
u

Phase %s%s
101*constraints2
4.2 2default:default2&
Global Iteration 12default:defaultZ18-101h px� 
r

Phase %s%s
101*constraints2
4.2.1 2default:default2!
Update Timing2default:defaultZ18-101h px� 
E
0Phase 4.2.1 Update Timing | Checksum: 137b7dbc4
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:44:15 ; elapsed = 00:27:10 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 179 ; free virtual = 146582default:defaulth px� 
�
Intermediate Timing Summary %s164*route2J
6| WNS=0.015  | TNS=0.000  | WHS=N/A    | THS=N/A    |
2default:defaultZ35-416h px� 
v

Phase %s%s
101*constraints2
4.2.2 2default:default2%
GlobIterForTiming2default:defaultZ18-101h px� 
t

Phase %s%s
101*constraints2
4.2.2.1 2default:default2!
Update Timing2default:defaultZ18-101h px� 
F
1Phase 4.2.2.1 Update Timing | Checksum: 869f2c9a
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:44:17 ; elapsed = 00:27:13 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 166 ; free virtual = 146452default:defaulth px� 
u

Phase %s%s
101*constraints2
4.2.2.2 2default:default2"
Fast Budgeting2default:defaultZ18-101h px� 
H
3Phase 4.2.2.2 Fast Budgeting | Checksum: 1255d540f
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:44:22 ; elapsed = 00:27:19 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 184 ; free virtual = 146092default:defaulth px� 
H
3Phase 4.2.2 GlobIterForTiming | Checksum: fc213533
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:44:24 ; elapsed = 00:27:22 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 187 ; free virtual = 146122default:defaulth px� 
G
2Phase 4.2 Global Iteration 1 | Checksum: fc213533
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:44:24 ; elapsed = 00:27:22 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 187 ; free virtual = 146122default:defaulth px� 
u

Phase %s%s
101*constraints2
4.3 2default:default2&
Global Iteration 22default:defaultZ18-101h px� 
r

Phase %s%s
101*constraints2
4.3.1 2default:default2!
Update Timing2default:defaultZ18-101h px� 
E
0Phase 4.3.1 Update Timing | Checksum: 1b2d66e4c
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:58:56 ; elapsed = 00:37:46 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 256 ; free virtual = 140412default:defaulth px� 
�
Intermediate Timing Summary %s164*route2J
6| WNS=-0.007 | TNS=-0.009 | WHS=N/A    | THS=N/A    |
2default:defaultZ35-416h px� 
G
2Phase 4.3 Global Iteration 2 | Checksum: ba95c3b3
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:58:57 ; elapsed = 00:37:47 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 256 ; free virtual = 140412default:defaulth px� 
E
0Phase 4 Rip-up And Reroute | Checksum: ba95c3b3
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:58:57 ; elapsed = 00:37:47 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 256 ; free virtual = 140412default:defaulth px� 
|

Phase %s%s
101*constraints2
5 2default:default2/
Delay and Skew Optimization2default:defaultZ18-101h px� 
p

Phase %s%s
101*constraints2
5.1 2default:default2!
Delay CleanUp2default:defaultZ18-101h px� 
r

Phase %s%s
101*constraints2
5.1.1 2default:default2!
Update Timing2default:defaultZ18-101h px� 
D
/Phase 5.1.1 Update Timing | Checksum: e6765bea
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:06 ; elapsed = 00:38:03 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 244 ; free virtual = 140322default:defaulth px� 
�
Intermediate Timing Summary %s164*route2J
6| WNS=0.102  | TNS=0.000  | WHS=N/A    | THS=N/A    |
2default:defaultZ35-416h px� 
B
-Phase 5.1 Delay CleanUp | Checksum: e6765bea
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:06 ; elapsed = 00:38:03 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 244 ; free virtual = 140322default:defaulth px� 
z

Phase %s%s
101*constraints2
5.2 2default:default2+
Clock Skew Optimization2default:defaultZ18-101h px� 
L
7Phase 5.2 Clock Skew Optimization | Checksum: e6765bea
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:06 ; elapsed = 00:38:03 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 244 ; free virtual = 140322default:defaulth px� 
N
9Phase 5 Delay and Skew Optimization | Checksum: e6765bea
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:06 ; elapsed = 00:38:03 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 244 ; free virtual = 140322default:defaulth px� 
n

Phase %s%s
101*constraints2
6 2default:default2!
Post Hold Fix2default:defaultZ18-101h px� 
p

Phase %s%s
101*constraints2
6.1 2default:default2!
Update Timing2default:defaultZ18-101h px� 
C
.Phase 6.1 Update Timing | Checksum: 1159f07cb
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:17 ; elapsed = 00:38:13 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 230 ; free virtual = 140202default:defaulth px� 
�
Intermediate Timing Summary %s164*route2J
6| WNS=0.102  | TNS=0.000  | WHS=0.016  | THS=0.000  |
2default:defaultZ35-416h px� 
A
,Phase 6 Post Hold Fix | Checksum: 16c77008b
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:18 ; elapsed = 00:38:13 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 229 ; free virtual = 140192default:defaulth px� 
�
�The router encountered %s pins that are both setup-critical and hold-critical and tried to fix hold violations at the expense of setup slack.
Resolution: Run report_timing on the design before routing to identify timing paths with higher hold violations and low or negative setup margin.179*route2
22default:defaultZ35-446h px� 
o

Phase %s%s
101*constraints2
7 2default:default2"
Route finalize2default:defaultZ18-101h px� 
B
-Phase 7 Route finalize | Checksum: 17454e63f
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:19 ; elapsed = 00:38:15 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 227 ; free virtual = 140172default:defaulth px� 
v

Phase %s%s
101*constraints2
8 2default:default2)
Verifying routed nets2default:defaultZ18-101h px� 
I
4Phase 8 Verifying routed nets | Checksum: 17454e63f
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:19 ; elapsed = 00:38:15 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 227 ; free virtual = 140182default:defaulth px� 
r

Phase %s%s
101*constraints2
9 2default:default2%
Depositing Routes2default:defaultZ18-101h px� 
E
0Phase 9 Depositing Routes | Checksum: 1de13cc6c
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:26 ; elapsed = 00:39:06 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 166 ; free virtual = 139772default:defaulth px� 
t

Phase %s%s
101*constraints2
10 2default:default2&
Post Router Timing2default:defaultZ18-101h px� 
�
Estimated Timing Summary %s
57*route2J
6| WNS=0.102  | TNS=0.000  | WHS=0.016  | THS=0.000  |
2default:defaultZ35-57h px� 
�
�The final timing numbers are based on the router estimated timing analysis. For a complete and accurate timing signoff, please run report_timing_summary.
127*routeZ35-327h px� 
G
2Phase 10 Post Router Timing | Checksum: 1de13cc6c
*commonh px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:26 ; elapsed = 00:39:06 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 166 ; free virtual = 139772default:defaulth px� 
=
Router Completed Successfully
16*routeZ35-16h px� 
�

%s
*constraints2�
�Time (s): cpu = 00:59:26 ; elapsed = 00:39:07 . Memory (MB): peak = 3082.301 ; gain = 231.000 ; free physical = 164 ; free virtual = 139772default:defaulth px� 
Z
Releasing license: %s
83*common2"
Implementation2default:defaultZ17-83h px� 
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
612default:default2
972default:default2
02default:default2
02default:defaultZ4-41h px� 
^
%s completed successfully
29*	vivadotcl2 
route_design2default:defaultZ4-42h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2"
route_design: 2default:default2
00:59:352default:default2
00:39:202default:default2
3082.3012default:default2
231.0002default:default2
1572default:default2
139752default:defaultZ17-722h px� 
D
Writing placer database...
1603*designutilsZ20-1893h px� 
=
Writing XDEF routing.
211*designutilsZ20-211h px� 
J
#Writing XDEF routing logical nets.
209*designutilsZ20-209h px� 
J
#Writing XDEF routing special nets.
210*designutilsZ20-210h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2)
Write XDEF Complete: 2default:default2
00:00:262default:default2
00:00:342default:default2
3114.3162default:default2
0.0002default:default2
1792default:default2
138902default:defaultZ17-722h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2&
write_checkpoint: 2default:default2
00:00:372default:default2
00:01:142default:default2
3115.3162default:default2
33.0162default:default2
3172default:default2
139112default:defaultZ17-722h px� 
P
Running DRC with %s threads
24*drc2
82default:defaultZ23-27h px� 
�
#The results of DRC are in file %s.
168*coretcl2�
b/home/kobayashi/PCIe_test/branches/IEICE/4-way_2-tree/vivado/vivado.runs/impl_1/top_drc_routed.rptb/home/kobayashi/PCIe_test/branches/IEICE/4-way_2-tree/vivado/vivado.runs/impl_1/top_drc_routed.rpt2default:default8Z2-168h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2 
report_drc: 2default:default2
00:00:262default:default2
00:00:482default:default2
3146.3322default:default2
31.0162default:default2
1602default:default2
138602default:defaultZ17-722h px� 
r
UpdateTimingParams:%s.
91*timing29
% Speed grade: -2, Delay Type: min_max2default:defaultZ38-91h px� 
|
CMultithreading enabled for timing update using a maximum of %s CPUs155*timing2
82default:defaultZ38-191h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2+
report_timing_summary: 2default:default2
00:00:552default:default2
00:00:522default:default2
3146.3322default:default2
0.0002default:default2
1642default:default2
138062default:defaultZ17-722h px� 
K
,Running Vector-less Activity Propagation...
51*powerZ33-51h px� 
P
3
Finished Running Vector-less Activity Propagation
1*powerZ33-1h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2"
report_power: 2default:default2
00:00:292default:default2
00:00:412default:default2
3162.3482default:default2
16.0162default:default2
3322default:default2
138122default:defaultZ17-722h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
report_clock_utilization: 2default:default2
00:00:032default:default2
00:00:072default:default2
3162.3482default:default2
0.0002default:default2
3152default:default2
138002default:defaultZ17-722h px� 


End Record