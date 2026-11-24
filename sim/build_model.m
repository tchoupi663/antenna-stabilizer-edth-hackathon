function build_model(mdl)
% BUILD_MODEL  Generate final_gimbal.slx (Simscape Multibody) from the CAD STLs in ../cad/stl.
% The model is generated: change this script and rebuild, do not edit the .slx by hand.
%
%   World -> airframe attitude (prescribed revolutes: yaw Z, pitch Y, roll X = ZYX Euler)
%         -> posts + roll servo                         (airframe-fixed, CAD)
%         -> ROLL joint  (X, outer, servo torque)  -> roll frame + pitch servo
%         -> PITCH joint (Y, inner, servo torque)  -> collar + 370 mm antenna (axes through its CoG)
%
%   Input (10 channels, model axes X forward, Y left, Z up):
%     1-3  true airframe roll, pitch, yaw [rad]       4-5 roll / pitch rate seen by the flight controller
%     6-7  roll / pitch seen by the flight controller  8-10 apparent gravity in the world frame [m/s^2]
%   Source: LIVE == 0 -> From Workspace 'air_in' (recorded attitude, see README)
%           LIVE == 1 -> UDP packets (flight controller on a serial bridge)
%
%   Control chain per axis (ArduPilot servo mount, MNT1_TYPE = 1):
%     cmd = -(attitude + lead_s * rate) -> dead time t_pure -> mount task (mount_rate)
%         -> travel limit -> servo PWM frame (pwm_rate) -> servo_dynamics.m -> joint torque
%   Joint internal mechanics: cable stiffness cable_k and bearing friction bearing_b.
if nargin < 1, mdl = 'final_gimbal'; end
here = fileparts(mfilename('fullpath'));
stlDir = fullfile(fileparts(here), 'cad', 'stl');
gimbal_params();                                      % workspace variables for the block dialogs
assignin('base', 'LIVE', 0);
air0 = zeros(2, 11); air0(:, 1) = [0; 1]; air0(:, 11) = -9.81;   % time + 10 channels, level flight
assignin('base', 'air_in', air0);
assignin('base', 'log_dt', 0.005);

if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);

Rz2x = [0 0 1; 1 0 0; 0 1 0];   % frame whose Z axis is world X
Rz2y = [0 1 0; 0 0 1; 1 0 0];   % frame whose Z axis is world Y
% colours of the CAD renders (build_gimbal.py COLORS)
gray = [0.42 0.45 0.50]; blue = [0.15 0.39 0.92]; dark = [0.12 0.16 0.22];
green = [0.02 0.59 0.41]; white = [0.90 0.91 0.92];

% ------------------------------------------------------------------ world, configuration
w  = add_block('sm_lib/Frames and Transforms/World Frame', [mdl '/World'], 'Position', [20 60 60 100]);
mc = add_block('sm_lib/Utilities/Mechanism Configuration', [mdl '/Mechanism Configuration'], 'Position', [20 160 100 200]);
sc = add_block('nesl_utility/Solver Configuration', [mdl '/Solver Configuration'], 'Position', [20 240 100 280]);
conn(mdl, w, 'R1', mc, 'R1'); conn(mdl, w, 'R1', sc, 'R1');
set_param(mc, 'UniformGravity', 'TimeVarying');

% ------------------------------------------------------------------ airframe attitude (ZYX Euler)
[yB, yF, jYaw] = joint(mdl, 'air_yaw',   eye(3), 60);
[pB, pF, jPit] = joint(mdl, 'air_pitch', Rz2y,  160);
[rB, rF, jRol] = joint(mdl, 'air_roll',  Rz2x,  260);
conn(mdl, w, 'R1', yB, 'L1'); conn(mdl, yF, 'R1', pB, 'L1'); conn(mdl, pF, 'R1', rB, 'L1');
air = rF;

% airframe deck (visual reference, motion is imposed so its mass does not matter)
deck = add_block('sm_lib/Body Elements/Brick Solid', [mdl '/airframe_deck'], 'Position', [520 -40 580 0]);
set_param(deck, 'BrickDimensions', '[0.7 0.5 0.004]', 'GraphicDiffuseColor', mat2str([0.80 0.80 0.78]));
dT = rigid(mdl, 'airframe_deck_T', eye(3), [0 0 -192], [440 -40 490 0]);
conn(mdl, air, 'R1', dT, 'L1'); conn(mdl, dT, 'R1', deck, 'R1');

% ------------------------------------------------------------------ airframe-fixed group (CAD)
s = solid(mdl, 'post_servo_side',   'post_servo_side.stl',   gray, [520 40 580 80]);   conn(mdl, air, 'R1', s, 'R1');
s = solid(mdl, 'post_bearing_side', 'post_bearing_side.stl', gray, [520 100 580 140]); conn(mdl, air, 'R1', s, 'R1');
% roll servo: output +X, long axis -Z (hangs below the post), origin (188, 0, -127) mm  [build_gimbal.py ROLL_SV]
placed(mdl, air, 'roll_servo', [0 0 1; 0 1 0; -1 0 0], [188 0 -127], [520 160 580 200], dark, stlDir);

% ------------------------------------------------------------------ ROLL joint (outer, X)
[roB, roF, jGr] = joint(mdl, 'gimbal_roll', Rz2x, 360, 'torque');
conn(mdl, air, 'R1', roB, 'L1');
s = solid(mdl, 'roll_frame', 'roll_frame.stl', blue, [900 40 960 80]); conn(mdl, roF, 'R1', s, 'R1');
% pitch servo in the +X/+Y corner: output +Y, long axis +X, origin (140, 189, 0) mm  [PITCH_SV]
placed(mdl, roF, 'pitch_servo', [1 0 0; 0 0 1; 0 -1 0], [140 189 0], [900 100 960 140], dark, stlDir);

% ------------------------------------------------------------------ PITCH joint (inner, Y)
[piB, piF, jGp] = joint(mdl, 'gimbal_pitch', Rz2y, 460, 'torque');
conn(mdl, roF, 'R1', piB, 'L1');
s = solid(mdl, 'antenna_brackets', 'antenna_brackets.stl', green, [900 200 960 240]); conn(mdl, piF, 'R1', s, 'R1');
% antenna: CAD shape, total mass ant_mass, shifted by the residual CoG offset
aT = add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/antenna_cog_T'], 'Position', [820 260 870 300]);
set_param(aT, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', 'cog_offset_mm', ...
    'TranslationCartesianOffsetUnits', 'mm');
s = solid(mdl, 'antenna', 'antenna_crpa_d370.stl', white, [900 260 960 300], 'ant_mass');
set_param(s, 'GraphicOpacity', '0.55');        % see the brackets and pivots through the antenna
conn(mdl, piF, 'R1', aT, 'L1'); conn(mdl, aT, 'R1', s, 'R1');

% joint internal mechanics: cable stiffness + bearing friction; start at the leveling angle q0
for jj = [jGr jGp]
    set_param(jj, 'SpringStiffness', 'cable_k', 'SpringStiffnessUnits', 'N*m/rad', ...
        'DampingCoefficient', 'bearing_b', 'DampingCoefficientUnits', 'N*m*s/rad');
end
set_param(jGr, 'PositionTargetSpecify', 'on', 'PositionTargetPriority', 'High', ...
    'PositionTargetValue', 'q0(1)', 'PositionTargetValueUnits', 'rad');
set_param(jGp, 'PositionTargetSpecify', 'on', 'PositionTargetPriority', 'High', ...
    'PositionTargetValue', 'q0(2)', 'PositionTargetValueUnits', 'rad');

% ------------------------------------------------------------------ antenna attitude sensor
ts = add_block('sm_lib/Frames and Transforms/Transform Sensor', [mdl '/antenna_attitude'], 'Position', [700 380 760 420]);
set_param(ts, 'SenseR', 'on');
conn(mdl, w, 'R1', ts, 'L1'); conn(mdl, piF, 'R1', ts, 'R1');
tp = get_param(ts, 'PortHandles');
rc = add_block('nesl_utility/PS-Simulink Converter', [mdl '/R_ps'], 'Position', [800 380 840 410]);
add_line(mdl, tp.RConn(end), get_param(rc, 'PortHandles').LConn(1), 'autorouting', 'on');
fcn(mdl, 'antenna_angles', [880 370 1000 420], ...
    ['function ant = f(R)' newline ...
     '% antenna roll, pitch (flight-controller convention: nose up +) and total tilt [deg]' newline ...
     'R = reshape(R, 3, 3);' newline ...
     'ant = [atan2(R(3,2), R(3,3)); asin(max(-1, min(1, R(3,1)))); acos(max(-1, min(1, R(3,3))))] * 180/pi;']);
add_line(mdl, 'R_ps/1', 'antenna_angles/1', 'autorouting', 'on');

% ------------------------------------------------------------------ input source (logs or live)
fw = add_block('simulink/Sources/From Workspace', [mdl '/air_in'], 'Position', [1000 60 1070 100]);
set_param(fw, 'VariableName', 'air_in', 'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value');
clk = add_block('simulink/Sources/Clock', [mdl '/clock'], 'Position', [960 140 990 160]);
us = add_block('simulink/User-Defined Functions/MATLAB System', [mdl '/udp_live_in'], 'Position', [1000 125 1100 175]);
set_param(us, 'System', 'UdpAttitudeSource', 'SimulateUsing', 'Interpreted execution');
add_line(mdl, 'clock/1', 'udp_live_in/1', 'autorouting', 'on');
vs = add_block('simulink/Signal Routing/Variant Source', [mdl '/source'], 'Position', [1130 70 1160 170]);
set_param(vs, 'VariantControls', {'LIVE==0', 'LIVE==1'}, 'VariantActivationTime', 'update diagram');
add_line(mdl, 'air_in/1', 'source/1', 'autorouting', 'on');
add_line(mdl, 'udp_live_in/1', 'source/2', 'autorouting', 'on');
dm = add_block('simulink/Signal Routing/Demux', [mdl '/air_demux'], 'Position', [1200 40 1205 220]);
set_param(dm, 'Outputs', '[1 1 1 2 2 3]');
add_line(mdl, 'source/1', 'air_demux/1', 'autorouting', 'on');
% channels: 1 roll 2 pitch 3 yaw (true) | 4 FC rates [p q] | 5 FC attitude [phi theta] | 6 apparent gravity
drive(mdl, jRol, 'air_demux/1', 'air_roll_in',  40);
drive(mdl, jPit, 'air_demux/2', 'air_pitch_in', 100);
drive(mdl, jYaw, 'air_demux/3', 'air_yaw_in',   160);
gc = add_block('nesl_utility/Simulink-PS Converter', [mdl '/gravity_in'], 'Position', [1300 220 1340 250]);
set_param(gc, 'Unit', 'm/s^2');
add_line(mdl, 'air_demux/6', 'gravity_in/1', 'autorouting', 'on');
add_line(mdl, get_param(gc, 'PortHandles').RConn(1), get_param(mc, 'PortHandles').LConn(1), 'autorouting', 'on');

% ------------------------------------------------------------------ ArduPilot servo mount (both axes)
b = @(lib, n, pos, varargin) add_block(lib, [mdl '/' n], 'Position', pos, varargin{:});
b('simulink/Math Operations/Gain', 'lead', [1300 300 1340 330], 'Gain', 'lead_s');
b('simulink/Math Operations/Add', 'cmd_sum', [1380 290 1400 340], 'Inputs', '--');
b('simulink/Continuous/Transport Delay', 'dead_time', [1430 300 1470 330], 'DelayTime', 't_pure', 'InitialOutput', 'q0');
b('simulink/Discrete/Zero-Order Hold', 'mount_task', [1500 300 1540 330], 'SampleTime', '1/mount_rate');
b('simulink/Discontinuities/Saturation', 'travel_limit', [1570 300 1610 330], 'UpperLimit', 'travel', 'LowerLimit', '-travel');
b('simulink/Discrete/Zero-Order Hold', 'pwm_frame', [1640 300 1680 330], 'SampleTime', '1/pwm_rate');
add_line(mdl, 'air_demux/4', 'lead/1', 'autorouting', 'on');
add_line(mdl, 'air_demux/5', 'cmd_sum/1', 'autorouting', 'on');
L = @(a, c) add_line(mdl, a, c, 'autorouting', 'on');
L('lead/1', 'cmd_sum/2'); L('cmd_sum/1', 'dead_time/1'); L('dead_time/1', 'mount_task/1');
L('mount_task/1', 'travel_limit/1'); L('travel_limit/1', 'pwm_frame/1');

% ------------------------------------------------------------------ servos (both axes) -> joint torques
b('simulink/Sources/Constant', 'SV', [1640 420 1680 450], 'Value', 'SV');
fcn(mdl, 'servo', [1720 290 1840 470], ...
    ['function [tgear, qmdd, sat] = f(cmd, q, qd, qm, qmd, SV)' newline ...
     '[tgear, qmdd, sat] = servo_dynamics(cmd, q, qd, qm, qmd, SV);']);
b('simulink/Continuous/Integrator', 'motor_rate',  [1880 380 1910 410], 'InitialCondition', '[0;0]');
b('simulink/Continuous/Integrator', 'motor_angle', [1940 380 1970 410], 'InitialCondition', 'q0');
L('pwm_frame/1', 'servo/1'); L('SV/1', 'servo/6');
L('servo/2', 'motor_rate/1'); L('motor_rate/1', 'motor_angle/1');
L('motor_angle/1', 'servo/4'); L('motor_rate/1', 'servo/5');
% joint angle / rate out of Simscape, torque in
qa = sense(mdl, jGr, jGp, 2, 'q',  'rad',   [1500 480]);
qw = sense(mdl, jGr, jGp, 3, 'qd', 'rad/s', [1500 560]); %#ok<NASGU>
L('q_mux/1', 'servo/2'); L('qd_mux/1', 'servo/3');
b('simulink/Signal Routing/Demux', 'tau_demux', [1880 290 1885 340], 'Outputs', '2');
L('servo/1', 'tau_demux/1');
for k = 1:2
    jh = [jGr jGp]; jh = jh(k);
    c = b('nesl_utility/Simulink-PS Converter', sprintf('torque_in%d', k), [1920 270+40*k 1960 300+40*k]);
    set_param(c, 'Unit', 'N*m');
    L(sprintf('tau_demux/%d', k), sprintf('torque_in%d/1', k));
    add_line(mdl, get_param(c, 'PortHandles').RConn(1), get_param(jh, 'PortHandles').LConn(2), 'autorouting', 'on');
end

% ------------------------------------------------------------------ visual-only parts (CAD look)
visuals(mdl, air, roF, piF, Rz2x, Rz2y);

% ------------------------------------------------------------------ logging + live view
% glog columns: 1-3 airframe roll, pitch, yaw | 4-6 antenna roll, pitch, tilt [deg] | 7-8 joints [deg]
%                 9-10 servo torque [N.m] | 11-12 servo saturated | 13-14 command [deg]
b('simulink/Math Operations/Gain', 'air_deg', [1300 120 1340 150], 'Gain', '[1 -1 -1]''*180/pi');   % back to FC convention
b('simulink/Signal Routing/Mux', 'air_mux', [1260 30 1265 160], 'Inputs', '3');
L('air_demux/1', 'air_mux/1'); L('air_demux/2', 'air_mux/2'); L('air_demux/3', 'air_mux/3');
L('air_mux/1', 'air_deg/1');
b('simulink/Math Operations/Gain', 'q_deg',   [1600 480 1640 510], 'Gain', '180/pi');
b('simulink/Math Operations/Gain', 'cmd_deg', [1720 220 1760 250], 'Gain', '180/pi');
L('q_mux/1', 'q_deg/1'); L('pwm_frame/1', 'cmd_deg/1');
b('simulink/Signal Routing/Mux', 'log_mux', [2050 100 2055 400], 'Inputs', '6');
L('air_deg/1', 'log_mux/1'); L('antenna_angles/1', 'log_mux/2'); L('q_deg/1', 'log_mux/3');
L('servo/1', 'log_mux/4'); L('servo/3', 'log_mux/5'); L('cmd_deg/1', 'log_mux/6');
b('simulink/Sinks/To Workspace', 'log', [2100 230 2180 260], 'VariableName', 'glog', ...
  'SaveFormat', 'Array', 'SampleTime', 'log_dt', 'MaxDataPoints', 'inf');
lv = b('simulink/User-Defined Functions/MATLAB System', 'live_view', [2100 300 2200 340]);
set_param(lv, 'System', 'LiveView', 'SimulateUsing', 'Interpreted execution');
L('log_mux/1', 'log/1'); L('log_mux/1', 'live_view/1');

% ------------------------------------------------------------------ solver
set_param(mdl, 'Solver', 'ode23t', 'StopTime', '10', 'MaxStep', 'max_step', 'RelTol', '1e-4', ...
    'ReturnWorkspaceOutputs', 'on', 'SimMechanicsOpenEditorOnUpdate', 'off');
save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('Model %s built and saved.\n', mdl);
end

% =============================================================== local helpers
function conn(mdl, a, ai, b, bi)
add_line(mdl, port(a, ai), port(b, bi), 'autorouting', 'on');
end

function p = port(h, id)
ph = get_param(h, 'PortHandles'); n = str2double(id(2:end));
if id(1) == 'L', p = ph.LConn(n); else, p = ph.RConn(n); end
end

function t = rigid(mdl, name, R, origin_mm, pos)
t = add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/' name], 'Position', pos);
set_param(t, 'RotationMethod', 'RotationMatrix', 'RotationMatrix', mat2str(R));
set_param(t, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', mat2str(origin_mm), ...
    'TranslationCartesianOffsetUnits', 'mm');
end

function [jBase, jFoll, j] = joint(mdl, name, axisMat, y, mode, x)
% Revolute joints turn about Z; rigid transforms rotate Z onto the wanted axis.
if nargin < 5, mode = 'motion'; end
if nargin < 6, x = 200; end
rtIn  = rigid(mdl, [name '_axisIn'],  axisMat,  [0 0 0], [x y x+50 y+40]);
j     = add_block('sm_lib/Joints/Revolute Joint', [mdl '/' name], 'Position', [x+100 y x+190 y+40]);
rtOut = rigid(mdl, [name '_axisOut'], axisMat', [0 0 0], [x+240 y x+290 y+40]);
if strcmp(mode, 'motion')
    set_param(j, 'MotionActuationMode', 'InputMotion', 'TorqueActuationMode', 'ComputedTorque');
else
    set_param(j, 'TorqueActuationMode', 'InputTorque', 'MotionActuationMode', 'ComputedMotion', ...
        'SensePosition', 'on', 'SenseVelocity', 'on');    % RConn: 1 frame F, 2 angle, 3 rate
end
conn(mdl, rtIn, 'R1', j, 'L1'); conn(mdl, j, 'R1', rtOut, 'L1');
jBase = rtIn; jFoll = rtOut;
end

function s = solid(mdl, name, file, color, pos, mass)
s = add_block('sm_lib/Body Elements/File Solid', [mdl '/' name], 'Position', pos);
stlDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'cad', 'stl');
set_param(s, 'ExtGeomFileName', fullfile(stlDir, file), 'UnitType', 'Custom', 'ExtGeomFileUnits', 'mm', ...
    'GraphicDiffuseColor', mat2str(color));
if nargin >= 6
    set_param(s, 'BasedOnType', 'Mass', 'Mass', mass, 'MassUnits', 'kg');
else
    set_param(s, 'BasedOnType', 'Density', 'Density', 'rho_struct', 'DensityUnits', 'kg/m^3');
end
end

function visuals(mdl, air, roF, piF, Rz2x, Rz2y)
% Linkages, shafts, pins and cable exactly as in the CAD (STLs exported in their home pose by
% build_gimbal.py). Visual only: 1 mg each, so they do not change the dynamics. The servo horns
% turn with the joint angle (1:1 parallelogram), the rods translate without turning.
link = [0.85 0.47 0.02]; shaft = [0.61 0.64 0.69]; cable = [0.49 0.23 0.93];
x0 = 2300; y = 600;
st = @(name, file, col, parent) conn(mdl, parent, 'R1', ...
    solid(mdl, name, file, col, [x0 y x0+60 y+40], '1e-6'), 'R1');
st('vis_roll_shafts',  'vis_roll_shafts.stl',  shaft, roF); y = y + 60;
st('vis_roll_pin',     'vis_roll_pin.stl',     shaft, roF); y = y + 60;
st('vis_pitch_shafts', 'vis_pitch_shafts.stl', shaft, piF); y = y + 60;
st('vis_pitch_pin',    'vis_pitch_pin.stl',    shaft, piF); y = y + 60;
st('vis_cable',        'cable_dummy.stl',      cable, air); y = y + 60;
% joint angles for the linkage motion
% (sampled at 200 Hz through a unit delay: feeding the sensed joint angle straight back into the same
%  Simscape network would make an algebraic loop and slow the whole simulation down)
add_block('simulink/Discrete/Unit Delay', [mdl '/q_vis_delay'], 'Position', [x0-300 y x0-260 y+40], ...
    'SampleTime', '0.005', 'InitialCondition', 'q0');
add_line(mdl, 'q_mux/1', 'q_vis_delay/1', 'autorouting', 'on');
add_block('simulink/Signal Routing/Demux', [mdl '/q_vis'], 'Position', [x0-200 y x0-195 y+60], 'Outputs', '2');
add_line(mdl, 'q_vis_delay/1', 'q_vis/1', 'autorouting', 'on');
add_block('simulink/Math Operations/Gain', [mdl '/q_vis_neg'], 'Position', [x0-150 y+80 x0-110 y+110], 'Gain', '-1');
add_line(mdl, 'q_vis_delay/1', 'q_vis_neg/1', 'autorouting', 'on');
add_block('simulink/Signal Routing/Demux', [mdl '/q_vis_n'], 'Position', [x0-90 y+80 x0-85 y+140], 'Outputs', '2');
add_line(mdl, 'q_vis_neg/1', 'q_vis_n/1', 'autorouting', 'on');
% roll drive: servo axis along X through (0, 0, -127) mm, horn 25 mm towards +Y
horn_and_rod(mdl, air, 'roll',  Rz2x, [0 0 -127], [0 25 0], 'q_vis/1', 'q_vis_n/1', link, x0 + 200, 600);
% pitch drive: servo axis along Y through (140, 0, 0) mm, horn 35 mm towards +Z (on the roll frame)
horn_and_rod(mdl, roF, 'pitch', Rz2y, [140 0 0],  [0 0 35], 'q_vis/2', 'q_vis_n/2', link, x0 + 200, 900);
end

function horn_and_rod(mdl, parent, tag, axisMat, pivot, h, qSig, qnSig, col, x, y)
% horn: parent -> T(pivot) -> joint(q) -> T(-pivot) -> horn STL
t1 = rigid(mdl, ['vis_' tag '_hornT1'], eye(3), pivot, [x y x+50 y+40]);
[jb, jf, j] = joint(mdl, ['vis_' tag '_horn_q'], axisMat, y, 'motion', x + 70);
t2 = rigid(mdl, ['vis_' tag '_hornT2'], eye(3), -pivot, [x+400 y x+450 y+40]);
hs = solid(mdl, ['vis_' tag '_servo_horn'], ['vis_' tag '_servo_horn.stl'], col, [x+480 y x+540 y+40], '1e-6');
conn(mdl, parent, 'R1', t1, 'L1'); conn(mdl, t1, 'R1', jb, 'L1'); conn(mdl, jf, 'R1', t2, 'L1'); conn(mdl, t2, 'R1', hs, 'R1');
drive(mdl, j, qSig, ['vis_' tag '_horn_in'], y, x + 600);
% rod: parent -> T(pivot) -> joint(q) -> T(h) -> joint(-q) -> T(-pivot-h) -> rod STL (pure translation)
y2 = y + 120;
r1 = rigid(mdl, ['vis_' tag '_rodT1'], eye(3), pivot, [x y2 x+50 y2+40]);
[ab, af, ja] = joint(mdl, ['vis_' tag '_rod_q'], axisMat, y2, 'motion', x + 70);
r2 = rigid(mdl, ['vis_' tag '_rodT2'], eye(3), h, [x+400 y2 x+450 y2+40]);
[bb, bf, jb2] = joint(mdl, ['vis_' tag '_rod_qn'], axisMat, y2 + 60, 'motion', x + 470);
r3 = rigid(mdl, ['vis_' tag '_rodT3'], eye(3), -pivot - h, [x+800 y2 x+850 y2+40]);
rs = solid(mdl, ['vis_' tag '_rod'], ['vis_' tag '_rod.stl'], col, [x+880 y2 x+940 y2+40], '1e-6');
conn(mdl, parent, 'R1', r1, 'L1'); conn(mdl, r1, 'R1', ab, 'L1'); conn(mdl, af, 'R1', r2, 'L1');
conn(mdl, r2, 'R1', bb, 'L1'); conn(mdl, bf, 'R1', r3, 'L1'); conn(mdl, r3, 'R1', rs, 'R1');
drive(mdl, ja,  qSig,  ['vis_' tag '_rod_in'],  y2, x + 600);
drive(mdl, jb2, qnSig, ['vis_' tag '_rod_inn'], y2 + 60, x + 1000);
end

function placed(mdl, parent, name, R, origin_mm, pos, color, stlDir) %#ok<INUSD>
t = rigid(mdl, [name '_T'], R, origin_mm, pos);
s = solid(mdl, name, 'servo_dummy.stl', color, pos + [90 0 90 0], 'servo_mass');
conn(mdl, parent, 'R1', t, 'L1'); conn(mdl, t, 'R1', s, 'R1');
end

function drive(mdl, jh, srcSig, name, y, x)
% prescribed motion of a joint from a Simulink signal [rad], smoothed (air_filter_s)
if nargin < 6, x = 1300; end
c = add_block('nesl_utility/Simulink-PS Converter', [mdl '/' name], 'Position', [x y x+40 y+30]);
set_param(c, 'Unit', 'rad', 'FilteringAndDerivatives', 'filter', 'SimscapeFilterOrder', '2', ...
    'InputFilterTimeConstant', 'air_filter_s');
add_line(mdl, get_param(c, 'PortHandles').RConn(1), get_param(jh, 'PortHandles').LConn(2), 'autorouting', 'on');
add_line(mdl, srcSig, [name '/1'], 'autorouting', 'on');
end

function m = sense(mdl, j1, j2, idx, name, unit, xy)
% joint sensor outputs of both gimbal joints -> one 2-vector Simulink signal
for k = 1:2
    jh = [j1 j2]; jh = jh(k);
    c = add_block('nesl_utility/PS-Simulink Converter', sprintf('%s/%s_%d', mdl, name, k), ...
        'Position', [xy(1) xy(2)+35*(k-1) xy(1)+40 xy(2)+25+35*(k-1)]);
    set_param(c, 'Unit', unit);
    add_line(mdl, get_param(jh, 'PortHandles').RConn(idx), get_param(c, 'PortHandles').LConn(1), 'autorouting', 'on');
end
m = add_block('simulink/Signal Routing/Mux', sprintf('%s/%s_mux', mdl, name), ...
    'Position', [xy(1)+70 xy(2) xy(1)+75 xy(2)+60], 'Inputs', '2');
add_line(mdl, sprintf('%s_1/1', name), sprintf('%s_mux/1', name), 'autorouting', 'on');
add_line(mdl, sprintf('%s_2/1', name), sprintf('%s_mux/2', name), 'autorouting', 'on');
end

function fcn(mdl, name, pos, code)
% MATLAB Function block with the given code
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/' name], 'Position', pos);
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [mdl '/' name]);
ch.Script = code;
end
