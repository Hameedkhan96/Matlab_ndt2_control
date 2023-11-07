function dX = QuadrotorStatesHnew(X)
global dX
% This script defines a continuous-time nonlinear quadrotor model and
% generates a state function and its Jacobian function used by the
% nonlinear MPC controller in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

% Create symbolix variables for states, MVs and parameters
% syms xt(t) yt(t) zt(t) phit(t) thetat(t) psit(t) xdott(t) ydott(t)...
%     zdott(t) phidott(t) thetadott(t) psidott(t) 
syms u1 u2 u3 u4 Ixx Iyy Izz k l m b g
%syms x y z phi theta psi xdot ydot zdot phidot thetadot psidot

x=X(1);  y=X(2); z=X(3); phi=X(4);  theta=X(5);  psi=X(6);  xdot=X(7);  ydot=X(8);  zdot=X(9);  phidot=X(10); thetadot=X(11);  psidot=X(12);

% phi: roll angle 
% theta: pitch angle 
% psi: yaw angle 
% ui:squared angular velocity of rotor i
% g: gravity
% b: drag constant
% k: lift constant
% l: distance between rotor and com
% Iii: diagonal elements of inertia matrix

% Set values for dynamics parameters
Ixx = 1.2; 
Iyy = 1.2;
Izz= 2.3;
k = 1;
l = 0.25;
m = 2;
b= 0.2;
g = 9.81;

% IxxVal = 1.2; 
% IyyVal = 1.2;
% IzzVal = 2.3;
% kVal = 1;
% lVal = 0.25;
% mVal = 2;
% bVal = 0.2;
% gVal = 9.81;

%paramValues = [IxxVal IyyVal IzzVal kVal lVal mVal bVal gVal];

% Group symbolic variables
% statet = {xt(t) yt(t) zt(t) phit(t) thetat(t) psit(t) xdott(t) ...
%     ydott(t) zdott(t) phidott(t) thetadott(t) psidott(t)};
% state = {x y z phi theta psi xdot ydot zdot phidot thetadot psidot};
% state_diff = {diff(xt(t),t), diff(yt(t),t), diff(zt(t),t), ...
%     diff(phit(t),t), diff(thetat(t),t), diff(psit(t),t)};
% state_dot = {xdot ydot zdot phidot thetadot psidot};

% Transformation matrix for angular velocities from inertial frame to body frame
W = [1, 0, -sin(theta);
    0, cos(phi), cos(theta)*sin(phi);
    0, -sin(phi), cos(theta)*cos(phi)];

% W = [1, 0, -sin(thetat);
%     0, cos(phit), cos(thetat)*sin(phit);
%     0, -sin(phit), cos(thetat)*cos(phit)];

%R-ZYX Euler
Rz = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi), 0;
    0, 0, 1];
Ry = [cos(theta), 0, sin(theta);
    0, 1, 0;
    -sin(theta), 0, cos(theta)];
Rx = [1, 0, 0;
    0, cos(phi), -sin(phi);
    0, sin(phi), cos(phi)];

% Rz = [cos(psit), -sin(psit), 0;
%     sin(psit), cos(psit), 0;
%     0, 0, 1];
% Ry = [cos(thetat), 0, sin(thetat);
%     0, 1, 0;
%     -sin(thetat), 0, cos(thetat)];
% Rx = [1, 0, 0;
%     0, cos(phit), -sin(phit);
%     0, sin(phit), cos(phit)];

% Rotation matrix from body frame to inertial frame
R = Rz*Ry*Rx;

% Jacobian (relates body frame to inertial frame velocities)
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
J = W.'*I*W;

% Coriolis forces
% dJ_dt = diff(J);
% dJ_dt = subs(dJ_dt,[state_diff statet],[state_dot state]);
% h_dot_J = [phidott(t), thetadott(t), psidott(t)]*J;
% h_dot_J = subs(h_dot_J,[state_diff statet],[state_dot state]);
% grad_temp_h = jacobian(h_dot_J,[phi theta psi]);
% C = dJ_dt - 1/2*grad_temp_h;

C11=0;
C12=(Iyy-Izz)*(x11*cos(x4)*sin(x4)+x12*sin(x4)^2*cos(x5))+(Izz-Iyy)*x12*cos(x4)^2*cos(x5)-Ixx*x12*cos(x5);
C13=(Izz-Iyy)*(x12*cos(x4)*sin(x4)*cos(x5)^2);
C21=(Izz-Iyy)*(x11*cos(x4)*sin(x4)+x12*sin(x4)*cos(x5))+(Iyy-Izz)*x12*cos(x4)^2*cos(x5)+Ixx*x12*cos(x5);
C22=(Izz-Iyy)*x10*cos(x4)*sin(x4);
C23= -Ixx*x12*sin(x5)*cos(x5)+Iyy*x12*sin(x4)^2*sin(x5)*cos(x5)+ Izz*x12*cos(x4)^2*sin(x5)*cos(x5);
C31=(Iyy-Izz)*(x12*cos(x5)^2*sin(x4)*cos(x4))-Ixx*x11*cos(x5);
C32= (Izz-Iyy)*(x11*cos(x4)*sin(x4)*sin(x5)+x10*sin(x4)^2*cos(x5))+(Iyy-Izz)*x10*cos(x4)^2*cos(x5)+Ixx*x12*sin(x5)*cos(x5)-Iyy*x12*sin(x10)^2*sin(x5)*cos(x5)-Izz*x12*cos(x10)^2*sin(x5)*cos(x5);
C33=(Iyy-Izz)*x10*cos(x4)*sin(x4)*cos(x5)^2-Iyy*x11*sin(x4)^2*cos(x5)*sin(x5)-Izz*x11*cos(x4)^2*cos(x5)*sin(x5)+Ixx*x11*cos(x5)*sin(x5);
%Cnndot=[C11 C12 C13;C21 C22 C23;C31 C32 C33];
C=[C11 C12 C13;C21 C22 C23;C31 C32 C33];

% Torques in the direction of phi, theta, psi
tau_beta = [l*k*(-u2 + u4);l*k*(-u1 + u3);b*(-u1+u2-u3+u4)];

% Total thrust
T = k*(u1+u2+u3+u4);

% Dynamics
% f(1) = xdott;
% f(2) = ydott;
% f(3) = zdott;
% f(4) = phidott;
% f(5) = thetadott;
% f(6) = psidott;
dX(1) = xdot;
dX(2) = ydot;
dX(3) = zdot;
dX(4) = phidot;
dX(5) = thetadot;
dX(6) = psidot;

% Equations for COM configuration
%f(7:9) = -g*[0;0;1] + R*[0;0;T]/m;
dX(7:9) = -g*[0;0;1] + R*[0;0;T]/m;

% Euler Lagrange equations for angular dynamics
%f(10:12) = inv(J)*(tau_beta - C*[phidott(t); thetadott(t); psidott(t)]);
dX(10:12) = inv(J)*(tau_beta - C*[phidot; thetadot; psidot]);
% Replace parameters and drop time dependence
% DX = subs(f, [Ixx Iyy Izz k l m b g], paramValues);
% DX= subs(f,statet,state);
% DX = simplify(f);
% 
%  dX = subs(dX, [Ixx Iyy Izz k l m b g], paramValues);
%  dX= subs(dX,statet,state);
%  dX = simplify(dX);
% 
% % Calculate linearization
% A = jacobian(f,[state{:}]);
% control = [u1, u2, u3, u4];
% B = jacobian(f,control);
% 
% % Create QuadrotorStateFcn.m
% matlabFunction(transpose(f),'File','QuadrotorStateFcn11',...
%     'Vars',{transpose([state{:}]),transpose(control)})
% % Create QuadrotorStateJacobianFcn.m 
% matlabFunction(A, B,'File','QuadrotorStateJacobianFcn11',...
%     'Vars',{transpose([state{:}]),transpose(control)})
% 
% %Clear symbolic variables
% clear
% 
% % Confirm the functions are generated successfully
% while isempty(which('QuadrotorStateJacobianFcn11'))
%     pause(0.1);
% end
