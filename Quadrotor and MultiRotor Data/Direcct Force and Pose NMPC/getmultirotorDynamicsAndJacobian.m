% This script defines a continuous-time nonlinear multirotor model and
% generates a state function and its Jacobian function used by the
% nonlinear MPC controller in the multirotor path following example.
 
% Copyright 2019 The MathWorks, Inc.
 
% Create symbolix variables for states, MVs and parameters
syms xt(t) yt(t) zt(t) xdott(t) ydott(t) zdott(t) zerot(t) phit(t) thetat(t) psit(t) ...
    phidott(t) thetadott(t) psidott(t) 

syms fax fay faz Ta1 Ta2 Ta3 
% syms fc1 fc2 fc3 dF1 dF2 dF3 Pd1 Pd2 Pd3

syms Ixx Iyy Izz k l m b g L dT1 dT2 dT3 rcom1 rcom2 rcom3
syms x y z xdot ydot zdot zero phi theta psi phidot thetadot psidot
 
% phi: roll angle 
% theta: pitch angle 
% psi: yaw angle 
% ui: squared angular velocity of rotor i
% g: gravity
% Iii: diagonal elements of inertia matrix
 
% Set values for dynamics parameters
IxxVal = 1.2; 
IyyVal = 1.2;
IzzVal = 1.2;

% IxxVal = 1.2; 
% IyyVal = 1.2;
% IzzVal = 2.3;
kVal = 1;
lVal = 0.25;
mVal = 2;
bVal = 0.2;
gVal = 9.81;
LVal = 0;
dTVal_1=0;dTVal_2=0;dTVal_3=0;
rcomVal_1=0;rcomVal_2=0;rcomVal_3=0;

paramValues = [IxxVal IyyVal IzzVal kVal lVal mVal bVal gVal LVal dTVal_1 dTVal_2 dTVal_3 rcomVal_1 rcomVal_2 rcomVal_3];
 
% Group symbolic variables
statet = {xt(t) yt(t) zt(t) xdott(t) ydott(t) zdott(t) zerot(t)...
    phit(t) thetat(t) psit(t) phidott(t) thetadott(t) psidott(t)};

state = {x y z xdot ydot zdot zero phi theta psi phidot thetadot psidot};

state_diff = {diff(xt(t),t), diff(yt(t),t), diff(zt(t),t), ...
    diff(zerot(t),t),diff(phit(t),t), diff(thetat(t),t), diff(psit(t),t)};

state_dot = {xdot ydot zdot phidot thetadot psidot};
 
% Transformation matrix for angular velocities from inertial frame to body frame
W = [1, 0, -sin(thetat);
    0, cos(phit), cos(thetat)*sin(phit);
    0, -sin(phit), cos(thetat)*cos(phit)];
 
%R-ZXY Euler
Rz =[cos(psit), sin(psit), 0;
    -sin(psit), cos(psit), 0;
    0, 0, 1];
Rx = [1, 0, 0;
    0, cos(thetat), -sin(thetat);
    0, -sin(thetat), cos(thetat)];
Ry = [cos(phit), 0, -sin(phit);
    0, 1, 0;
    sin(phit), 0, cos(phit)];
 
% Rotation matrix from body frame to inertial frame
% Rs = Rz*Rx*Ry;
Rs = [1 0 0;0 -1 0;0 0 -1];
Rb=[1 0 0;0 1 0;0 0 1];
%Rb=Rz;

 
% % Jacobian (relates body frame to inertial frame velocities)
%J = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
J = W.'*I*W;
 
% % Coriolis forces
% dJ_dt = diff(J);
% dJ_dt = subs(dJ_dt,[state_diff statet],[state_dot state]);
% h_dot_J = [phidott(t), thetadott(t), psidott(t)]*J;
% h_dot_J = subs(h_dot_J,[state_diff statet],[state_dot state]);
% grad_temp_h = jacobian(h_dot_J,[phi theta psi]);
% C = dJ_dt - 1/2*grad_temp_h;
%  
% Torques in the direction of phi, theta, psi

ta =[Ta1; Ta2; Ta3];
 
% Total thrust
%T = [fax; fay; faz];
% Dynamics
%lambda=L;
L=0;
%L=1;

Rd=[1 0 0;0 1 0;0 0 1-L];
SL=Rs*Rd*transpose(Rs);
%V=[Vx;Vy;Vz];

fa=[fax; fay; faz];
fc=[0;0;0.5];
dF=[0;0;0];
% fc=[fc1;fc2;fc3];
% dF=[dF1;dF2;dF3];
 
 
ab=(m^-1)*((Rb*fa)+(L*fc)+dF)+[0;0;g];
V=[xdott(t);ydott(t);zdott(t)];
P_dot=SL*V;
%rT=[rx;ry;rz];
Pd=[0; 0; 0];
%Pd=[Pd1; Pd2; Pd3];
P=[xt(t);yt(t);zt(t)];
%P=[xt(t);yt(t);zt(t)];
rT=[0.5;0;0];
%rT=P-Pd;
Q=[zerot(t); phit(t);thetat(t);psit(t)];
W=[phidott(t);thetadott(t);psidott(t)];
Q_dot=(1/2)*Q.*[0;W];
 
%J=[Ixx 0 0;0 Iyy 0;0 0 Izz];
%J=[Ixx 0 0;0 Iyy 0;0 0 Izz];
%ta=[tx;ty;tz];
h=J*W;
%%rcom=[rcom1;rcom2;rcom3];
rcom=[0;0;0];
h1=transpose(Rb)*[0;0;m*g];
%%dT=[dT1;dT2;dT3];
dT=[0;0;0];
h2=L*transpose(Rb)*fc;
W_dot=(inv(J))*(ta-cross(W,h)+cross(rcom,h1)+cross(rT,h2)+dT);
V_dot=SL*((ab)+Rb*(cross(W_dot,rT)+(cross(W,cross(W,rT)))));
 
f(1:13)=[P_dot;V_dot;Q_dot;W_dot];
% Replace parameters and drop time dependence
f = subs(f, [Ixx Iyy Izz k l m b g L dT1 dT2 dT3 rcom1 rcom2 rcom3], paramValues);
f = subs(f,statet,state);
f = simplify(f);
 
% Calculate linearization
A = jacobian(f,[state{:}]);
control = [fax, fay, faz, Ta1, Ta2, Ta3];
B = jacobian(f,control);
 
% Create multirotorStateFcn.m
matlabFunction(transpose(f),'File','multirotorStateFcn',...
    'Vars',{transpose([state{:}]),transpose(control)})

% Create multirotorStateJacobianFcn.m 
matlabFunction(A, B,'File','multirotorStateJacobianFcn',...
    'Vars',{transpose([state{:}]),transpose(control)})
 
%Clear symbolic variables
clear
 
% Confirm the functions are generated successfully
while isempty(which('multirotorStateJacobianFcn'))
    pause(0.1);
end
