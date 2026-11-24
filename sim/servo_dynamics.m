function [tgear, qmdd, sat] = servo_dynamics(cmd, q, qd, qm, qmd, SV)
%#codegen
% SERVO_DYNAMICS  Hobby digital servo, both axes at once (model from youssef-exploration,
% compare/sim_servo_gimbal.m, design C). All quantities at the servo output shaft.
%   cmd  [2x1] commanded joint angle (PWM pulse decoded by the servo)        rad
%   q,qd [2x1] gimbal joint angle / rate (the servo potentiometer reads q)  rad, rad/s
%   qm,qmd [2x1] motor-side angle / rate (rotor + gears, reflected)          rad, rad/s
%   SV   [2x10] per axis: Tstall w0 pb Kdv deadband half_backlash gear_k gear_c Jm coulomb
% Chain:  error - dead band -> drive V = sat(e/pb - Kdv*qmd, +-1)
%         motor torque = Tstall*(V - qmd/w0), limited to +-Tstall (linear torque-speed curve)
%         gear train with play (half_backlash), stiffness gear_k and damping gear_c -> tgear
%         motor side: Jm*qmdd = motor torque - tgear - coulomb*tanh(qmd/0.05)
tgear = zeros(2, 1); qmdd = zeros(2, 1); sat = zeros(2, 1);
for i = 1:2
    Ts = SV(i,1); w0 = SV(i,2); pb = SV(i,3); Kdv = SV(i,4); db = SV(i,5);
    bl2 = SV(i,6); kg = SV(i,7); cg = SV(i,8); Jm = SV(i,9); cm = SV(i,10);
    e = cmd(i) - q(i);
    eEff = sign(e) * max(0, abs(e) - db);
    Vraw = eEff / pb - Kdv * qmd(i);
    V = max(-1, min(1, Vraw));
    tm = Ts * (V - qmd(i) / w0);
    sat(i) = double(abs(Vraw) >= 1 || abs(tm) > Ts);
    tm = max(-Ts, min(Ts, tm));
    del = qm(i) - q(i);
    if abs(del) > bl2
        tgear(i) = kg * (del - sign(del) * bl2) + cg * (qmd(i) - qd(i));
    end
    qmdd(i) = (tm - tgear(i) - cm * tanh(qmd(i) / 0.05)) / Jm;
end
end
