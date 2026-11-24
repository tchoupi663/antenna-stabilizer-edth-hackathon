classdef LiveView < matlab.System
% LIVEVIEW  Live window of final_gimbal.slx (active only when LIVE == 1): the CAD gimbal in 3D,
% airframe attitude, antenna attitude and tilt vs the +-2 deg requirement, servo torque.
% Input: the 14-column log vector of the model (see build_model.m). Close the window to stop.

    properties (Nontunable)
        Window = 20;          % s shown in the time plots
        Show = true;          % false: no window (timing tests)
    end
    properties (Access = private)
        Active = false; Fig = []; Tf = []; Lines = []; Ttl = []; Ax = [];
        TiltMax = 0; NOut = 0; N = 0; Limit = 2; Tmax = 3.9;
        Clock = []; Buf = zeros(0, 8);   % data collected between two redraws (t + 7 plotted values)
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Active = obj.Show && evalin('base', 'exist(''LIVE'', ''var'') && LIVE == 1');
            if ~obj.Active, return; end
            obj.Limit = evalin('base', 'limit_deg'); obj.Tmax = evalin('base', 'servo_Tstall');
            obj.build();
        end

        function stepImpl(obj, u)
            if ~obj.Active, return; end
            if isempty(obj.Fig) || ~isvalid(obj.Fig)
                set_param(bdroot, 'SimulationCommand', 'stop'); obj.Active = false; return
            end
            t = obj.N * 0.05; obj.N = obj.N + 1;
            air = u(1:3); ant = u(4:6); q = u(7:8); tau = u(9:10);
            if t > 2
                obj.TiltMax = max(obj.TiltMax, ant(3)); obj.NOut = obj.NOut + (ant(3) > obj.Limit);
            end
            obj.Buf(end+1, :) = [t air(1) air(2) ant(1) ant(2) ant(3) tau(1) tau(2)];
            % redraw at most 10 times per wall-clock second: drawing every 50 ms step made the
            % simulation slower than real time; every sample still reaches the plots
            if ~isempty(obj.Clock) && toc(obj.Clock) < 0.1, return; end
            obj.Clock = tic;
            for k = 1:numel(obj.Lines), addpoints(obj.Lines(k), obj.Buf(:, 1), obj.Buf(:, k+1)); end
            obj.Buf = zeros(0, 8);
            tw = [max(0, t - obj.Window), max(obj.Window, t)];
            for a = obj.Ax, a.XLim = tw; end
            % 3D pose (display: yaw ignored so the view does not spin)
            Ra = rz(0) * ry(-air(2)) * rx(air(1));            % model axes: pitch about Y-left = -pitch
            Rf = Ra * rx(q(1));
            Rp = Rf * ry(q(2));
            obj.Tf(1).Matrix = h4(Ra); obj.Tf(2).Matrix = h4(Rf); obj.Tf(3).Matrix = h4(Rp);
            % 1:1 parallelograms: servo horns turn with the joint, rods translate
            pr = [0; 0; -127]; hr = [0; 25; 0]; pp = [140; 0; 0]; hp = [0; 0; 35];
            obj.Tf(4).Matrix = about(rx(q(1)), pr); obj.Tf(5).Matrix = shift(rx(q(1)) * hr - hr);
            obj.Tf(6).Matrix = about(ry(q(2)), pp); obj.Tf(7).Matrix = shift(ry(q(2)) * hp - hp);
            obj.Ttl.String = sprintf(['Airframe  roll %+6.1f   pitch %+6.1f deg\n' ...
                'Servos  roll %+6.1f   pitch %+6.1f deg   torque %4.2f / %4.2f N.m\n' ...
                'Antenna tilt %.2f deg   (max %.2f, samples > %g deg: %d)'], ...
                air(1), air(2), q(1), q(2), tau(1), tau(2), ant(3), obj.TiltMax, obj.Limit, obj.NOut);
            if ant(3) > obj.Limit, obj.Ttl.Color = [0.8 0.1 0.1]; else, obj.Ttl.Color = [0 0 0]; end
            drawnow limitrate
        end

        function build(obj)
            stl = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'cad', 'stl');
            obj.Fig = figure('Name', 'Final gimbal - live', 'Color', 'w', 'Position', [40 60 1450 780]);
            try, theme(obj.Fig, 'light'); catch, end
            ax3 = axes(obj.Fig, 'Position', [0.01 0.06 0.47 0.86]); hold(ax3, 'on');
            axis(ax3, 'equal'); grid(ax3, 'on'); view(ax3, -55, 20);
            xlim(ax3, [-320 320]); ylim(ax3, [-320 320]); zlim(ax3, [-260 260]);
            xlabel(ax3, 'x (roll axis) [mm]'); ylabel(ax3, 'y (pitch axis) [mm]'); zlabel(ax3, 'z [mm]');
            obj.Tf = [hgtransform('Parent', ax3), hgtransform('Parent', ax3), hgtransform('Parent', ax3)];
            obj.Tf(4) = hgtransform('Parent', obj.Tf(1)); obj.Tf(5) = hgtransform('Parent', obj.Tf(1));   % roll horn, rod
            obj.Tf(6) = hgtransform('Parent', obj.Tf(2)); obj.Tf(7) = hgtransform('Parent', obj.Tf(2));   % pitch horn, rod
            sv = @(R, o) struct('R', R, 'o', o);
            gr = [0.42 0.45 0.50]; dk = [0.12 0.16 0.22]; bl = [0.15 0.39 0.92]; gn = [0.02 0.59 0.41];
            lk = [0.85 0.47 0.02]; sh = [0.61 0.64 0.69]; cb = [0.49 0.23 0.93];      % CAD render colours
            parts = { ...
              'post_servo_side',      1, gr, 1,    []
              'post_bearing_side',    1, gr, 1,    []
              'servo_dummy',          1, dk, 1,    sv([0 0 1; 0 1 0; -1 0 0], [188 0 -127])
              'cable_dummy',          1, cb, 1,    []
              'roll_frame',           2, bl, 1,    []
              'servo_dummy',          2, dk, 1,    sv([1 0 0; 0 0 1; 0 -1 0], [140 189 0])
              'vis_roll_shafts',      2, sh, 1,    []
              'vis_roll_pin',         2, sh, 1,    []
              'antenna_brackets',     3, gn, 1,    []
              'vis_pitch_shafts',     3, sh, 1,    []
              'vis_pitch_pin',        3, sh, 1,    []
              'antenna_crpa_d370',    3, [0.90 0.91 0.92], 0.55, []
              'vis_roll_servo_horn',  4, lk, 1,    []
              'vis_roll_rod',         5, lk, 1,    []
              'vis_pitch_servo_horn', 6, lk, 1,    []
              'vis_pitch_rod',        7, lk, 1,    []};
            for k = 1:size(parts, 1)
                f = fullfile(stl, [parts{k,1} '.stl']);
                if ~isfile(f), continue; end
                TR = stlread(f); V = TR.Points; F = TR.ConnectivityList;
                if ~isempty(parts{k,5}), V = V * parts{k,5}.R.' + parts{k,5}.o; end
                if size(F, 1) > 1500, [F, V] = reducepatch(F, V, 1500); end   % lighter meshes for live drawing
                patch('Parent', obj.Tf(parts{k,2}), 'Faces', F, 'Vertices', V, ...
                      'FaceColor', parts{k,3}, 'FaceAlpha', parts{k,4}, 'EdgeColor', 'none');
            end
            [x, y] = meshgrid([-350 350], [-250 250]);                % airframe deck
            surf(x, y, -192 + 0*x, 'Parent', obj.Tf(1), 'FaceColor', [0.8 0.8 0.78], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
            plot3(ax3, [0 0], [0 0], [0 250], 'k--');                 % true vertical
            th = linspace(0, 2*pi, 90);
            plot3(ax3, 200*cos(th), 200*sin(th), 0*th, 'Color', [0.1 0.7 0.3]);   % true horizon
            camlight(ax3, 'headlight'); lighting(ax3, 'gouraud'); material(ax3, 'dull');
            obj.Ttl = title(ax3, 'Waiting for data from live\_bridge.py ...');

            axd = axes(obj.Fig, 'Position', [0.55 0.70 0.42 0.24]); hold(axd, 'on'); grid(axd, 'on');
            ylabel(axd, 'Airframe [deg]'); ylim(axd, [-60 60]); title(axd, 'Airframe attitude (flight controller)');
            axa = axes(obj.Fig, 'Position', [0.55 0.38 0.42 0.24]); hold(axa, 'on'); grid(axa, 'on');
            ylabel(axa, 'Antenna [deg]'); ylim(axa, [-3.5 3.5]); title(axa, 'Antenna platform (simulated)');
            yline(axa, obj.Limit, 'k--'); yline(axa, -obj.Limit, 'k--');
            axt = axes(obj.Fig, 'Position', [0.55 0.07 0.42 0.22]); hold(axt, 'on'); grid(axt, 'on');
            ylabel(axt, 'Servo torque [N.m]'); xlabel(axt, 'Time [s]'); ylim(axt, [-1.2 1.2] * obj.Tmax);
            yline(axt, [-obj.Tmax obj.Tmax], 'r--');
            c = {[0 0.35 0.9], [0.1 0.65 0.3]};
            obj.Lines = [animatedline(axd, 'Color', c{1}, 'LineWidth', 1.2), animatedline(axd, 'Color', c{2}, 'LineWidth', 1.2), ...
                         animatedline(axa, 'Color', c{1}, 'LineWidth', 1.2), animatedline(axa, 'Color', c{2}, 'LineWidth', 1.2), ...
                         animatedline(axa, 'Color', [0.85 0.2 0.1], 'LineWidth', 1.8), ...
                         animatedline(axt, 'Color', c{1}), animatedline(axt, 'Color', c{2})];
            for h = obj.Lines, h.MaximumNumPoints = obj.Window * 25; end
            legend(axd, obj.Lines(1:2), {'roll', 'pitch'}, 'Location', 'northwest');
            legend(axa, obj.Lines(3:5), {'roll', 'pitch', 'total tilt'}, 'Location', 'northwest');
            legend(axt, obj.Lines(6:7), {'roll servo', 'pitch servo'}, 'Location', 'northwest');
            obj.Ax = [axd axa axt];
        end

        function n = getNumInputsImpl(~),  n = 1; end
        function n = getNumOutputsImpl(~), n = 0; end
        function st = getSampleTimeImpl(obj)
            st = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', 0.05);
        end
    end
end

function M = h4(R), M = [R zeros(3, 1); 0 0 0 1]; end
function M = about(R, p), M = [R, p - R * p; 0 0 0 1]; end          % rotation R about the point p
function M = shift(d), M = [eye(3), d; 0 0 0 1]; end
function M = rx(d), M = [1 0 0; 0 cosd(d) -sind(d); 0 sind(d) cosd(d)]; end
function M = ry(d), M = [cosd(d) 0 sind(d); 0 1 0; -sind(d) 0 cosd(d)]; end
function M = rz(d), M = [cosd(d) -sind(d) 0; sind(d) cosd(d) 0; 0 0 1]; end
