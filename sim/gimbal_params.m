function P = gimbal_params()
% GIMBAL_PARAMS  Every parameter of the final gimbal model, in one struct.
%   Tags: [measured]  from the team (antenna photos), [CAD]  from ../cad/build_gimbal.py,
%         [datasheet] Savox SV-1270TG+, [brief]  Fire Point CHALLENGE.md,
%         [ArduPilot] stock ArduPlane servo-mount behaviour, [assumed] not measured yet -> bench test.
% Override any field for one run:  simulate(prof, struct('ant_mass', 2, 'timing', 'fast'))
% gimbal_derived.m computes the dependent values and pushes everything to the base workspace.

% ---- antenna -----------------------------------------------------------------------------
P.ant_mass       = 4.0;          % kg     2 (minimum) or 4 (optional)                 [brief]
P.ant_diameter   = 0.370;        % m      round CRPA, 50 mm thick, 6 rim tabs          [measured]
P.cog_offset_mm  = [0 0 5];      % mm     antenna CoG minus axis crossing (x fwd, y left, z up),
                                 %        AFTER the +-8 mm clamp trim. 5 mm up = residual    [assumed]

% ---- structure ---------------------------------------------------------------------------
P.rho_struct = 2700;             % kg/m^3 aluminium 6061 frame, posts and collar          [CAD]
P.servo_mass = 0.066;            % kg     DSServo DS3240 (standard size, 40 x 20 x 40.5 mm) [seller spec]

% ---- servo: DSServo DS3240 (180 deg) at 6.0 V -- baseline bill of materials (branch feat/component),
%      the list's choice once the CoG is on the axes. Seller spec 36 / 45 kg.cm and 0.20 / 0.17 s/60deg
%      at 5.0 / 6.8 V; buyer reviews report less torque than rated -> derated to 70 % at 6 V.
%      Servo model from youssef-exploration (compare/sim_servo_gimbal.m).
P.servo_model    = 'DSServo DS3240';
P.servo_Tstall   = 28 * 0.0980665;   % N.m  ~40 kg.cm at 6 V (interpolated) x 0.7          [seller spec, derated]
P.servo_speed    = 60 / 0.185;       % deg/s no-load speed at 6 V (interpolated)          [seller spec]
P.servo_pb_deg   = 2.0;              % deg  error at which the servo drives at full power   [assumed]
P.servo_zeta     = 0.7;              % -    damping of the servo position loop              [assumed]
P.servo_db_deg   = 0.1;              % deg  dead band (~1-2 us of pulse)                      [assumed]
P.backlash_deg   = [0.5 0.5];        % deg  total play: servo gears + 1:1 linkage (roll, pitch) [assumed]
P.servo_Jm       = 0.01;             % kg.m^2 rotor + gears reflected to the output          [assumed]
P.gear_k         = 500;              % N.m/rad gear-train stiffness beyond the play          [assumed]
P.gear_zeta      = 0.2;              % -    gear-train damping                               [assumed]
P.gear_coulomb   = 0.15;             % N.m  gear dry friction at the output                   [assumed]
P.travel_deg     = [50 45];          % deg  joint travel (roll, pitch): CAD clearance sweep   [CAD]

% ---- joints (bearings and cable) ---------------------------------------------------------
P.cable_k   = 0.30;              % N.m/rad antenna cable stiffness per joint                   [assumed]
P.bearing_b = 0.05;              % N.m.s/rad viscous friction per joint                         [assumed]

% ---- flight controller -> ArduPilot servo mount -> PWM (MNT1_TYPE = 1) ------------------
% 'stock': default ArduPlane, mount task and servo output at 50 Hz, ~70 ms total lag.
% 'fast' : SCHED_LOOP_RATE raised, mount at 400 Hz, 333 Hz digital-servo PWM, 10 ms dead time.
P.timing         = 'stock';      %                                                          [ArduPilot]
P.lead_s         = [];           % s      MNT1_LEAD_RLL / MNT1_LEAD_PTCH (rate feed-forward);
                                 %        [] = the timing preset's value (0.07 stock, 0.045 fast)
P.att_noise_deg  = 0.05;         % deg rms attitude-estimate noise (synthetic profiles only)    [assumed]
P.gyro_noise_dps = 0.3;          % deg/s rms gyro noise (synthetic profiles only)                [assumed]
P.cal_offset_deg = 0;            % deg   level-calibration error of the flight controller        [assumed]

% ---- flight loads -----------------------------------------------------------------------
P.accel_model = 'fixedwing';     % 'fixedwing' coordinated turns + pull-ups | 'none'             [brief]
P.airspeed    = 40;              % m/s   Fire Point fixed-wing cruise (pull-up load V*dgamma/dt) [assumed]
P.lever_arm   = 0.30;            % m     airframe CG below the gimbal axes                       [assumed]

% ---- solver / pass criterion --------------------------------------------------------------
P.max_step     = 1e-3;           % s
P.air_filter_s = 0.002;          % s     smoothing of the imposed airframe motion (0.02 in live mode)
P.limit_deg    = 2;              % deg   leveling requirement                                    [brief]
P.t_settle     = 2;              % s     start-up excluded from the metrics

P = gimbal_derived(P);
end
