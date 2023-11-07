function dX = QuadrotorStatesHnewF(X,u)
%global dX
% ui:squared angular velocity of rotor i
% b: drag constant
% k: lift constant
% l: distance between rotor and com
% Iii: diagonal elements of inertia matrix

% Set values for dynamics parameters
Ixx = 1.2; 
Iyy = 1.2;
Izz= 2.3;
%Izz= 0.3;
k = 1;
L = 0.25;
m = 2;
b= 0.2;
g = 9.81;

%% FR pap Prm
% Ixx = 3.4; 
% Iyy = 3.4;
% Izz= 4.7;
% m = 1.2;
% L=0.21;
% %pu=1.8e-5
% k=1.8e-5;
% %c=8e-7
% b=8e-7;
% g = 9.81;
% % kf = 1; km = 0.2; gamma = km/kf; 

%% Transformation matrix for angular velocities from inertial frame to body frame
Q = [1, 0, -sin(X(5));
     0, cos(X(4)), cos(X(5))*sin(X(4));
     0, -sin(X(4)), cos(X(5))*cos(X(4))];
 
dQ = [0 0 -cos(X(5));
    0 -sin(X(4)) -sin(X(5))*sin(X(4))+cos(X(5))*cos(X(4));
    0 -cos(X(4)) -sin(X(5))*cos(X(4))-cos(X(5))*sin(X(4))];

%%
R = [cos(X(5))*cos(X(6)),-sin(X(6))*cos(X(4))+cos(X(6))*sin(X(5))*sin(X(4)), sin(X(6))*sin(X(4))+cos(X(6))*sin(X(5))*cos(X(4));
  sin(X(6))*cos(X(5)), cos(X(6))*cos(X(4))+sin(X(6))*sin(X(5))*sin(X(4)), -cos(X(6))*sin(X(4))+sin(X(6))*sin(X(5))*cos(X(4));
   -sin(X(5)), cos(X(5))*sin(X(4)), cos(X(5))*cos(X(4))];

%% Jacobian (relates body frame to inertial frame velocities)
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
M = Q.'*I*Q;

%% Coriolis forces
s = Q*[X(10);X(11);X(12)];
S = [0 -s(3) s(2);s(3) 0 -s(1);-s(2) s(1) 0];
C = Q'*S*I*Q + Q'*I*dQ;

%% Controller
u = [u(1);u(2);u(3);u(4)];

% Total thrust
T = k*(u(1)+u(2)+u(3)+u(4));

% Torques in the direction of phi, theta, psi
tau_beta = [L*k*(u(2)-u(4));L*k*(-u(1)+u(3));b*(u(1)-u(2)+u(3)-u(4))];

%% Dynamics

%dX = [0;0;0;0;0;0;0;0;0;0;0];
%dX = [1;-1;0;0;0;0;0;0;0;0;0;0];
% dX= [4;-7;0;0;0;0;0;0;0;0;0;0];
  dX=  [2;-4;0;0;0;0;0;0;0;0;0;0];
  
dX(1) = X(7);
dX(2) = X(8);
dX(3) = X(9);
dX(4) = X(10);
dX(5) = X(11);
dX(6) = X(12);

% Equations for COM configuration
dX(7:9) = g*[0;0;1]-R*[0;0;T]/m;

% Euler Lagrange equations for angular dynamics
%dX(10:12) = inv(M)*(tau_beta - C*[X(10); X(11); X(12)]);
dX(10:12) = pinv(M)*(Q'*tau_beta-C*[X(10); X(11); X(12)]);

dX=[dX(1);dX(2);dX(3);dX(4);dX(5);dX(6);dX(7);dX(8);dX(9);dX(10);dX(11);dX(12)];


%% ------------------------------------------------------------------------









%R-ZYX Euler
% Rz = [cos(X(6)), -sin(X(6)), 0; sin(X(6)), cos(X(6)), 0; 0, 0, 1];
% Ry = [cos(X(5)), 0, sin(X(5)); 0, 1, 0; -sin(X(5)), 0, cos(X(5))];
% Rx = [1, 0, 0; 0, cos(X(4)), -sin(X(4)); 0, sin(X(4)), cos(X(4))];
% Rotation matrix from body frame to inertial frame
%R = Rz*Ry*Rx;


% C11=0;
% C12=(Iyy-Izz)*(X(11)*cos(X(4))*sin(X(4))+X(12)*sin(X(4))^2*cos(X(5)))+(Izz-Iyy)*X(12)*cos(X(4))^2*cos(X(5))-Ixx*X(12)*cos(X(5));
% C13=(Izz-Iyy)*(X(12)*cos(X(4))*sin(X(4))*cos(X(5))^2);
% C21=(Izz-Iyy)*(X(11)*cos(X(4))*sin(X(4))+X(12)*sin(X(4))*cos(X(4)))+(Iyy-Izz)*X(12)*cos(X(4))^2*cos(X(5))+Ixx*X(12)*cos(X(5));
% C22=(Izz-Iyy)*X(10)*cos(X(4))*sin(X(4));
% C23= -Ixx*X(12)*sin(X(5))*cos(X(5))+Iyy*X(12)*sin(X(4))^2*sin(X(5))*cos(X(5))+ Izz*X(12)*cos(X(4))^2*sin(X(5))*cos(X(5));
% C31=(Iyy-Izz)*(X(12)*cos(X(5))^2*sin(X(4))*cos(X(4)))-Ixx*X(11)*cos(X(5));
% C32= (Izz-Iyy)*(X(11)*cos(X(4))*sin(X(4))*sin(X(5))+X(10)*sin(X(4))^2*cos(X(5)))+(Iyy-Izz)*X(10)*cos(X(4))^2*cos(X(4))+Ixx*X(12)*sin(X(5))*cos(X(5))-Iyy*X(12)*sin(X(10))^2*sin(X(5))*cos(X(5))-Izz*X(12)*cos(X(10))^2*sin(X(5))*cos(X(5));
% C33=(Iyy-Izz)*X(10)*cos(X(4))*sin(X(4))*cos(X(5))^2-Iyy*X(11)*sin(X(4))^2*cos(X(5))*sin(X(5))-Izz*X(11)*cos(X(4))^2*cos(X(5))*sin(X(5))+Ixx*X(11)*cos(X(5))*sin(X(5));
% C=[C11 C12 C13;C21 C22 C23;C31 C32 C33];
