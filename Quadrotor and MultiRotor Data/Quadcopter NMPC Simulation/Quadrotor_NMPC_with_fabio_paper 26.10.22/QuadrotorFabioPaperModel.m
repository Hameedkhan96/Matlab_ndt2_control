function dX = quadrotor(X,u)
%global dX

x = X(1);  y = X(2); z = X(3);
phi = X(4);  theta = X(5);  psi = X(6);
dx = X(7); dy = X(8); dz = X(9);
dphi = X(10); dtheta = X(11); dpsi = X(12);

%parameters

Ixx = 1.2; 
Iyy = 1.2;
Izz= 2.3;
 k = 1;
% k = 1e-3;
l = 0.25;
 m = 2;
 b= 0.2;
 %b = 0.5e-4;
 g = 9.81;
 %L = 0.25;

% Ixx = (1/12)*m*(0.2^2+0.2^2);
% Iyy = (1/12)*m*(0.2^2+0.2^2);
% Izz = (1/12)*m*(0.2^2+0.2^2);
JB = [Ixx 0 0;0 Iyy 0;0 0 Izz];

%kf = 1e-3; km = 0.5e-4; gamma = km/kf;

% Rz = [cos(psi), -sin(psi), 0;
%       sin(psi), cos(psi), 0;
%       0, 0, 1];
% Ry = [cos(theta), 0, sin(theta);
%       0, 1, 0;
%       -sin(theta), 0, cos(theta)];
% Rx = [1, 0, 0;
%       0, cos(phi), -sin(phi);
%       0, sin(phi), cos(phi)];
%   
%   R = Rz*Ry*Rx;

%rotation matrix
R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
     cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

%transformation of rotations
Q = [1 0 -sin(theta);
    0 cos(phi) cos(theta)*sin(phi);
    0 -sin(phi) cos(theta)*cos(phi)];

dQ = [0 0 -cos(theta);
    0 -sin(phi) -sin(theta)*sin(phi)+cos(theta)*cos(phi);
    0 -cos(phi) -sin(theta)*cos(phi)-cos(theta)*sin(phi)];


M = Q.'*JB*Q;
s = Q*[dphi;dtheta;dpsi];
S = [0 -s(3) s(2);s(3) 0 -s(1);-s(2) s(1) 0];
C = Q'*S*JB*Q + Q'*JB*dQ;

% 
% C11=0;
% C12=(Iyy-Izz)*(dtheta*cos(phi)*sin(phi)+dpsi*sin(phi)^2*cos(theta))+(Izz-Iyy)*dpsi*cos(phi)^2*cos(theta)-Ixx*dpsi*cos(theta);
% C13=(Izz-Iyy)*(dpsi*cos(phi)*sin(phi)*cos(theta)^2);
% C21=(Izz-Iyy)*(dtheta*cos(phi)*sin(phi)+dpsi *sin(phi)*cos(theta))+(Iyy-Izz)*dpsi*cos(phi)^2*cos(theta)+Ixx*dpsi*cos(theta);
% C22=(Izz-Iyy)*dphi*cos(phi)*sin(phi);
% C23= -Ixx*dpsi*sin(theta)*cos(theta)+Iyy*dpsi*sin(phi)^2*sin(theta)*cos(theta)+ Izz*dpsi*cos(phi)^2*sin(theta)*cos(theta);
% C31=(Iyy-Izz)*(dpsi*cos(theta)^2*sin(phi)*cos(phi))-Ixx*dtheta *cos(theta);
% C32= (Izz-Iyy)*(dtheta*cos(phi)*sin(phi)*sin(theta)+dphi*sin(phi)^2*cos(theta))+(Iyy-Izz)*dphi*cos(phi)^2*cos(theta)+Ixx*dpsi*sin(theta)*cos(theta)-Iyy*dpsi*sin(dphi)^2*sin(theta)*cos(theta)-Izz*dpsi*cos(dphi)^2*sin(theta)*cos(theta);
% C33=(Iyy-Izz)*dphi*cos(phi)*sin(phi)*cos(theta)^2-Iyy*dtheta*sin(phi)^2*cos(theta)*sin(theta)-Izz*dtheta*cos(phi)^2*cos(theta)*sin(theta)+Ixx*dtheta*cos(theta)*sin(theta);
% C=[C11 C12 C13;C21 C22 C23;C31 C32 C33];

% 
% A = [ 1 1 1 1;
%       0  L 0 -L;
%      -L 0  L  0;
%      gamma -gamma gamma -gamma];


%controller

u = [u(1);u(2);u(3);u(4)];

% T = k*(u(1)+u(2)+u(3)+u(4));
% tau_beta = [l*k*(u(2)-u(4));l*k*(-u(1)+u(3));b*(u(1)-u(2)+u(3)-u(4))];
% 
% 
T = k*(u(1)+u(2)+u(3)+u(4));
tau_beta = [l*k*(u(2)-u(4));l*k*(-u(1)+u(3));b*(u(1)-u(2)+u(3)-u(4))];

%  T= (u(1)
% tau_beta =[u(2);u(3);u(4)]
% th = pinv(A)*u;
% th = pinv(A)*u;
% w = sqrt(th/kf);
%equations of motion

dX= [7;-10;0;0;0;0;0;0;0;0;0;0];
%dX = zeros(12,1);
dX(1:3,1) = [dx;dy;dz];
dX(4:6,1) = [dphi;dtheta;dpsi];
% dX(7:9,1) = (1/m)*(kf*R*A(1,:)*w.^2 - m*g*[0;0;1] );

dX(7:9,1) = g*[0;0;1] - R*[0;0;T]/m;
dX(10:12,1) = pinv(M)*(Q'*tau_beta-C*[dphi;dtheta;dpsi]);

dX=[dX(1);dX(2);dX(3);dX(4);dX(5);dX(6);dX(7);dX(8);dX(9);dX(10);dX(11);dX(12)];

%dX(10:12,1) = (1/M)*(kf*A(4:6,:)*w.^2- C*[dphi;dtheta;dpsi] );
%dX=[dX(1);dX(2);dX(3);dX(4);dX(5);dX(6);dX(7);dX(8);dX(9);dX(10);dX(11);dX(12)];
% %dX(10:12,1) = inv(M)*(u(4:6,:)- C*[dphi;dtheta;dpsi] );