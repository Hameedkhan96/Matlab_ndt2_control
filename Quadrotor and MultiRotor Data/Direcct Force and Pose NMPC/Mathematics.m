clc
syms a b c L Vx Vy Vz Fz Wx Wy Wz rx ry rz
m=5;
Rz=[cos(c) sin(c) 0;-sin(c) cos(c) 0;0 0 1];
Rx=[1 0 0;0 cos(a) sin(a);0 -sin(a) cos(a)];
Ry=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
Rs=Rz*Rx*Ry
%lambda=L;
L=0;
Rd=[1 0 0;0 1 0;0 0 1-L];
SL=Rs*Rd*transpose(Rs)
V=[Vx;Vy;Vz];
Rb=Rz;
fa=[0;0;Fz];
fc=[0;0;0];
dF=[0;0;0];
g=9.8;
ab=(m^-1)*((Rb*fa)+(L*fc)+dF)+[0;0;g]
P_dot=SL*V;
rT=[rx;ry;rz];
W=[Wx;Wy;Wz];
V_dot=SL*((ab)+Rb(corss(W_dot,rT)+(cross(W,cross(W,rT))));
Q_dot=(1/2)*[0;qx;qy;qz]*[0;wx;wy;wz];
Ixx=1;
Iyy=1;
Izz=1;
J=[Ixx 0 0;0 Iyy 0;0 0 Izz];
ta=[tx;ty;tz];
h=J*W;
rcom=[0;0;0];
h1=transpose(Rb)*[0;0;m*g];
dT=[0;0;0];
h2=L*tanspose(Rb)*fc
W_dot=(J^-1)*(ta-cross(W,h)+Cross(rcom,h1)+Cross(rT,h2)+dT);

