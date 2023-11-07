function out1 = multirotorStateFcn(in1,in2)
%MULTIROTORSTATEFCN
%    OUT1 = MULTIROTORSTATEFCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    27-Sep-2022 11:52:49

Ta1 = in2(4,:);
Ta2 = in2(5,:);
Ta3 = in2(6,:);
fax = in2(1,:);
fay = in2(2,:);
faz = in2(3,:);
phi = in1(8,:);
phidot = in1(11,:);
psidot = in1(13,:);
psi = in1(10,:);
theta = in1(9,:);
thetadot = in1(12,:);
xdot = in1(4,:);
ydot = in1(5,:);
zdot = in1(6,:);
t2 = sin(theta);
t3 = Ta3.*5.0;
t4 = phidot.^2;
t5 = psidot.^2;
t6 = t2.^2;
t7 = Ta1.*t2.*5.0;
t8 = psidot.*t2.*thetadot.*6.0;
t9 = -t8;
t10 = t6-1.0;
t11 = 1.0./t10;
out1 = [xdot;ydot;zdot;fax./2.0-t5./2.0-thetadot.^2./2.0;t11.*(fay.*6.0+t3+t7+t9-fay.*t6.*6.0+phidot.*thetadot.*6.0).*(-1.0./1.2e+1);Ta2.*(-5.0./1.2e+1)+faz./2.0+(phidot.*psidot)./2.0+(t2.*t4)./2.0-(t2.*t5)./2.0+9.81e+2./1.0e+2;0.0;(phi.*phidot)./2.0;(theta.*thetadot)./2.0;(psidot.*psi)./2.0;t11.*(Ta1.*5.0+t2.*t3+phidot.*t2.*thetadot.*6.0-psidot.*t6.*thetadot.*6.0).*(-1.0./6.0);Ta2.*(5.0./6.0)-t2.*t4+t2.*t5;t11.*(t3+t7+t9+phidot.*t6.*thetadot.*6.0).*(-1.0./6.0)];