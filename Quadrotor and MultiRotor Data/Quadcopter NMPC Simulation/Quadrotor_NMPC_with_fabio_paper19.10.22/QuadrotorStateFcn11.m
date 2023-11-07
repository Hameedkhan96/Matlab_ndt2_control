function out1 = QuadrotorStateFcn11(in1,in2)
%QUADROTORSTATEFCN11
%    OUT1 = QUADROTORSTATEFCN11(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    18-Oct-2022 09:36:51

phi = in1(4,:);
phidot = in1(10,:);
psidot = in1(12,:);
psi = in1(6,:);
theta = in1(5,:);
thetadot = in1(11,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
xdot = in1(7,:);
ydot = in1(8,:);
zdot = in1(9,:);
t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t8 = phi.*2.0;
t9 = theta.*2.0;
t10 = thetadot.^2;
t18 = u1+u2+u3+u4;
t11 = t2.^2;
t12 = t2.^3;
t14 = t4.^2;
t15 = t4.^3;
t16 = sin(t8);
t17 = sin(t9);
t13 = t11.^2;
t19 = 1.0./t14;
et1 = u2.*-1.15e+2+u4.*1.15e+2-t10.*t16.*(2.53e+2./2.0)-t7.*u1.*9.2e+1+t7.*u2.*9.2e+1-t7.*u3.*9.2e+1+t7.*u4.*9.2e+1+t11.*u2.*5.5e+1-t11.*u4.*5.5e+1+phidot.*t17.*thetadot.*(2.3e+1./2.0)+psidot.*t4.*thetadot.*2.76e+2+t5.*t10.*t12.*1.21e+2+t7.*t11.*u1.*4.4e+1-t7.*t11.*u2.*4.4e+1+t7.*t11.*u3.*4.4e+1-t7.*t11.*u4.*4.4e+1-t11.*t14.*u2.*5.5e+1+t11.*t14.*u4.*5.5e+1+psidot.*t4.*t11.*thetadot.*2.53e+2-psidot.*t4.*t13.*thetadot.*1.21e+2-psidot.*t11.*t15.*thetadot.*2.53e+2+psidot.*t13.*t15.*thetadot.*1.21e+2+t2.*t5.*t10.*t14.*2.53e+2-t5.*t10.*t12.*t14.*1.21e+2+phidot.*t4.*t7.*t11.*thetadot.*2.53e+2-t2.*t4.*t5.*t7.*u1.*5.5e+1+t2.*t4.*t5.*t7.*u3.*5.5e+1;
et2 = phidot.*psidot.*t2.*t5.*t7.*t14.*3.85e+2;
mt1 = [xdot,ydot,zdot,phidot,thetadot,psidot,(t18.*(t5.*t6+t2.*t3.*t7))./2.0,t18.*(t3.*t5-t2.*t6.*t7).*(-1.0./2.0),(t2.*t4.*t18)./2.0-9.81e+2./1.0e+2,(t19.*(et1+et2))./5.52e+2,((t4.*u1.*6.0e+1-t4.*u3.*6.0e+1+t16.*u1.*2.2e+1-t16.*u2.*2.2e+1+t16.*u3.*2.2e+1-t16.*u4.*2.2e+1+phidot.*psidot.*t14.*1.32e+2+t7.*t10.*t11.*1.21e+2-t7.*t10.*t13.*1.21e+2+t4.*t11.*u1.*5.5e+1-t4.*t11.*u3.*5.5e+1-phidot.*psidot.*t11.*t14.*3.85e+2+t2.*t5.*t7.*u2.*5.5e+1-t2.*t5.*t7.*u4.*5.5e+1+phidot.*t2.*t4.*t5.*thetadot.*2.53e+2-psidot.*t4.*t5.*t7.*t12.*thetadot.*1.21e+2).*(-1.0./5.52e+2))./t4];
mt2 = [(t19.*(u1.*-9.2e+1+u2.*9.2e+1-u3.*9.2e+1+u4.*9.2e+1-t7.*u2.*1.15e+2+t7.*u4.*1.15e+2+t11.*u1.*4.4e+1-t11.*u2.*4.4e+1+t11.*u3.*4.4e+1-t11.*u4.*4.4e+1+phidot.*t4.*thetadot.*2.3e+1+psidot.*t17.*thetadot.*1.38e+2+t7.*t11.*u2.*5.5e+1-t7.*t11.*u4.*5.5e+1+phidot.*t4.*t11.*thetadot.*2.53e+2-t2.*t5.*t7.*t10.*2.53e+2+t5.*t7.*t10.*t12.*1.21e+2-t2.*t4.*t5.*u1.*5.5e+1+t2.*t4.*t5.*u3.*5.5e+1+phidot.*psidot.*t2.*t5.*t14.*3.85e+2+psidot.*t4.*t7.*t11.*thetadot.*2.53e+2-psidot.*t4.*t7.*t13.*thetadot.*1.21e+2))./5.52e+2];
out1 = reshape([mt1,mt2],12,1);
