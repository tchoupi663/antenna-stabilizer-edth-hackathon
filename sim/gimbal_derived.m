function P = gimbal_derived(P)
% GIMBAL_DERIVED  Dependent values (timing preset, servo loop gains, initial state) and push every
% field of P to the base workspace, where the Simulink blocks read them.

% ---- timing preset (ArduPilot servo mount)
switch P.timing
    case 'stock'   % default ArduPlane: 50 Hz mount + 50 Hz servo output, ~70 ms effective lag
        P.mount_rate = 50; P.pwm_rate = 50; P.t_pure = 0.031; lead = 0.070;
    case 'fast'    % raised loop rate, 400 Hz mount, 333 Hz digital-servo frame, ~45 ms lag
        P.mount_rate = 400; P.pwm_rate = 333; P.t_pure = 0.010; lead = 0.045;
    otherwise
        error('timing must be ''stock'' or ''fast''');
end
if isempty(P.lead_s), P.lead_s = lead; end

% ---- output inertia per axis, from the CAD mass properties (../cad/gimbal_results.json, written by
%      build_gimbal.py); linear in the antenna mass between the 2 kg and 4 kg cases
cadf = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'cad', 'gimbal_results.json');
B = jsondecode(fileread(cadf)); B = B.budget;
lin = @(a, b) a + (P.ant_mass - 2) * (b - a) / 2;
J_pitch = lin(B.x2kg.J_pitch_kgm2, B.x4kg.J_pitch_kgm2);
J_roll  = lin(B.x2kg.J_roll_kgm2,  B.x4kg.J_roll_kgm2);
Jout = [J_roll; J_pitch];

% ---- servo loop (per axis, all at the output shaft): see servo_dynamics.m
Ts   = P.servo_Tstall * [1; 1];
w0   = deg2rad(P.servo_speed);
pb   = deg2rad(P.servo_pb_deg);
Jtot = Jout + P.servo_Jm;
Kp   = Ts / pb;                                     % N.m/rad equivalent stiffness
cTot = 2 * P.servo_zeta * sqrt(Kp .* Jtot);         % damping that gives servo_zeta at this load
Kdv  = max(0, (cTot - Ts / w0) ./ Ts);              % extra derivative action (s/rad, drive units)
Jeff = P.servo_Jm * Jout ./ (P.servo_Jm + Jout);
cg   = 2 * P.gear_zeta * sqrt(P.gear_k * Jeff);
% one row per axis: [Tstall w0 pb Kdv deadband half_backlash gear_k gear_c Jm coulomb]
P.SV = [Ts, w0*[1;1], pb*[1;1], Kdv, deg2rad(P.servo_db_deg)*[1;1], deg2rad(P.backlash_deg(:))/2, ...
        P.gear_k*[1;1], cg, P.servo_Jm*[1;1], P.gear_coulomb*[1;1]];
P.travel = deg2rad(P.travel_deg(:));
P.Kp_servo = Kp; P.Jout = Jout;

% ---- initial joint angles (set by simulate.m from the first attitude sample)
if ~isfield(P, 'q0'), P.q0 = [0; 0]; end

f = fieldnames(P);
for k = 1:numel(f), assignin('base', f{k}, P.(f{k})); end
end
