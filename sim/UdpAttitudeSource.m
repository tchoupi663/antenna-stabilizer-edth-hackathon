classdef UdpAttitudeSource < matlab.System
% UDPATTITUDESOURCE  Live airframe attitude for final_gimbal.slx (LIVE == 1).
% Reads UDP packets from a flight-controller bridge (7 little-endian doubles:
% time_s, roll, pitch, yaw [deg], roll/pitch/yaw rate [deg/s], flight-controller convention:
% roll + right wing down, pitch + nose up) from a Cube Orange (MAVLink) or a SpeedyBee F405 V3
% (Betaflight MSP, or ArduPilot MAVLink once flashed). Output: the 10 input channels of the model
% (see build_model.m). The stream is replayed Lag seconds behind real time and linearly
% interpolated between packets, so the 20-50 Hz stream becomes a smooth airframe motion.
% In live mode the bridge's attitude is both the "true" airframe attitude and what the flight
% controller sees; the model still adds the ArduPilot mount timing and the servo.

    properties (Nontunable)
        Port = 9870;          % UDP port of the bridge
        Lag  = 0.06;          % s, replay delay (covers the packet interval + jitter)
    end
    properties (Access = private)
        U = [];  Buf = zeros(0, 7);  T0 = NaN;  Active = false;  LastFc = -Inf;  YawOff = 0;  PrevYaw = NaN;  Resyncs = 0;
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Active = evalin('base', 'exist(''LIVE'', ''var'') && LIVE == 1');
            if obj.Active
                obj.U = udpport("byte", "IPV4", "LocalPort", obj.Port);
                flush(obj.U);
                fprintf('Live input: listening on UDP %d (no bridge connected)\n', obj.Port);
            end
        end

        function y = stepImpl(obj, t)
            g = 9.81;
            y = [zeros(7, 1); 0; 0; -g];
            if ~obj.Active, return; end
            obj.readPackets();
            if isempty(obj.Buf), return; end
            if isnan(obj.T0), obj.T0 = obj.Buf(end, 1) - t; end
            tf = t + obj.T0 - obj.Lag;                       % flight-controller time to replay
            B = obj.Buf;
            if tf < B(1, 1) || tf < B(end, 1) - 1
                % the simulation fell behind the stream (slow PC): jump to the newest data
                obj.T0 = B(end, 1) - t; tf = B(end, 1) - obj.Lag;
                obj.Resyncs = obj.Resyncs + 1;
                if obj.Resyncs == 1 || mod(obj.Resyncs, 20) == 0
                    fprintf('Live input: simulation slower than real time, skipped ahead (%d times)\n', obj.Resyncs);
                end
            end
            if tf <= B(1, 1)
                s = B(1, 2:7);
            elseif tf >= B(end, 1)
                s = B(end, 2:7);
                if tf > B(end, 1) + 1, obj.T0 = B(end, 1) - t + obj.Lag; end   % stream paused: resync
            else
                k = find(B(:, 1) <= tf, 1, 'last');
                a = (tf - B(k, 1)) / max(B(k+1, 1) - B(k, 1), 1e-6);
                s = (1 - a) * B(k, 2:7) + a * B(k+1, 2:7);
            end
            d = pi / 180;
            phi = s(1)*d; th = -s(2)*d; psi = -s(3)*d;      % model axes: X forward, Y left, Z up
            p = s(4)*d;  q = -s(5)*d;
            zb = [cos(psi)*sin(th)*cos(phi) + sin(psi)*sin(phi), ...
                  sin(psi)*sin(th)*cos(phi) - cos(psi)*sin(phi), cos(th)*cos(phi)];
            % coordinated flight: apparent gravity along the airframe "down" axis, g*cos(pitch)/cos(bank)
            geff = -g * cos(th) / max(cos(phi), 0.3) * zb;
            y = [phi; th; psi; p; q; phi; th; geff(:)];
        end

        function readPackets(obj)
            nb = obj.U.NumBytesAvailable;
            if nb < 56, return; end
            raw = read(obj.U, floor(nb / 56) * 7, "double");
            S = reshape(raw, 7, []).';
            for i = 1:size(S, 1)
                r = S(i, :);
                if r(1) < obj.LastFc - 0.5                  % bridge restarted: start over
                    obj.Buf = zeros(0, 7); obj.T0 = NaN; obj.YawOff = 0; obj.PrevYaw = NaN;
                end
                if ~isnan(obj.PrevYaw)                      % unwrap the heading
                    dy = r(4) + obj.YawOff - obj.PrevYaw;
                    obj.YawOff = obj.YawOff - 360 * round(dy / 360);
                end
                r(4) = r(4) + obj.YawOff; obj.PrevYaw = r(4);
                obj.LastFc = r(1);
                obj.Buf(end+1, :) = r;
            end
            if size(obj.Buf, 1) > 400, obj.Buf = obj.Buf(end-399:end, :); end
        end

        function releaseImpl(obj)
            if ~isempty(obj.U), clear obj.U; obj.U = []; end
        end

        function n = getNumInputsImpl(~),  n = 1; end
        function n = getNumOutputsImpl(~), n = 1; end
        function s = getOutputSizeImpl(~), s = [10 1]; end
        function f = isOutputFixedSizeImpl(~), f = true; end
        function d = getOutputDataTypeImpl(~), d = 'double'; end
        function c = isOutputComplexImpl(~), c = false; end
        function st = getSampleTimeImpl(obj)
            st = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', 0.01);
        end
    end
end
