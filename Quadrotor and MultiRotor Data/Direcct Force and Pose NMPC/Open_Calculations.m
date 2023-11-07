
syms Px Py Pz Vx Vy Vz qx qy qz Wx Wy Wz Fz tax tay taz Pdx Pdy Pdz fcx fcy fcz dFx dFy dFz m g Ixx Iyy Izz
P=[Px;Py;Pz];
V=[Vx;Vy;Vz];
Q=[0;qx;qy;qz];
W=[Wx;Wy;Wz];
ta=[tax;tay;taz];
Pd=[Pdx;Pdy;Pdz];
fc=[fcx;fcy;fcz];
dF=[dFx;dFy;dFz];
clc
% m=1.25;
Rz=[cos(Q(4)) sin(Q(4)) 0;-sin(Q(4)) cos(Q(4)) 0;0 0 1];
Rx=[1 0 0;0 cos(Q(2)) sin(Q(2));0 -sin(Q(2)) cos(Q(2))];
Ry=[cos(Q(3)) 0 sin(Q(3));0 1 0;-sin(Q(3)) 0 cos(Q(3))];
Rs=Rz*Rx*Ry;
%lambda=L;
L=0;
Rd=[1 0 0;0 1 0;0 0 1-L];
SL=Rs*Rd*transpose(Rs);
%V=[Vx;Vy;Vz];
Rb=Rz;
fa=[0;0;Fz];

% g=9.8;
ab=(m^-1)*((Rb*fa)+(L*fc)+dF)+[0;0;g];

P_dot=SL*V;
%rT=[rx;ry;rz];
rT=P-Pd;
%W=[Wx;Wy;Wz];


Q_dot=(1/2)*Q.*[0;W];

% Ixx=1.7e-2;     Iyy=1.7e-3;     Izz=1e-4;
J=[Ixx 0 0;0 Iyy 0;0 0 Izz];
%ta=[tx;ty;tz];
h=J*W;
rcom=[0;0;0];
h1=transpose(Rb)*[0;0;m*g];
dT=[0;0;0];
h2=L*transpose(Rb)*fc;
W_dot=(inv(J))*(ta-cross(W,h)+cross(rcom,h1)+cross(rT,h2)+dT);
V_dot=SL*((ab)+Rb*(cross(W_dot,rT)+(cross(W,cross(W,rT)))));


P_dot,V_dot,Q_dot,W_dot