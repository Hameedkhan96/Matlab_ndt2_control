function dX = quadrotor(X,u)
%global dX
%syms x y z phi theta psi dx dy dz dphi dtheta dpsi
%syms X(1) X(2) X(3) X(4) X(5) X61) X(7) X(8) X(9) X(10) X(11) X(12) 
x = X(1);  y = X(2); z = X(3);
phi = X(4);  theta = X(5);  psi = X(6);
dx = X(7); dy = X(8); dz = X(9);
dphi = X(10); dtheta = X(11); dpsi = X(12);
%iez = X(13);

%parameters

Jx = 1.2; 
Jy = 1.2;
Jz= 2.3;
% k = 1;
% l = 0.25;
% m = 2;
% b= 0.2;
% g = 9.81;

m = 2; g = 9.81; L = 0.25;

% Jx = (1/12)*m*(0.2^2+0.2^2);
% Jy = (1/12)*m*(0.2^2+0.2^2);
% Jz = (1/12)*m*(0.2^2+0.2^2);
JB = [Jx 0 0;0 Jy 0;0 0 Jz];

%kf = 1e-3; km = 0.5e-4; gamma = km/kf;
kf = 1; km = 0.2; gamma = km/kf;

%rotation matrix
R = [cos(psi)*cos(theta) -sin(psi)*cos(theta)+cos(psi)*sin(theta)*sin(phi) sin(psi)*sin(theta)+cos(psi)*sin(theta)*cos(phi);
     sin(psi)*sin(theta) cos(psi)*cos(theta)+sin(psi)*sin(theta)*sin(phi) -cos(psi)*sin(theta)+sin(psi)*sin(theta)*cos(phi);
    -sin(theta) cos(theta)*cos(phi) cos(theta)*cos(phi)];

%transformation of rotations
Q = [1 0 -sin(theta);
    0 cos(phi) cos(theta)*sin(phi);
    0 -sin(phi) cos(theta)*cos(phi)];

dQ = [0 0 -cos(theta);
    0 -sin(phi) -sin(theta)*sin(phi)+cos(theta)*cos(phi);
    0 -cos(phi) -sin(theta)*cos(phi)-cos(theta)*sin(phi)];

% dQ = [0 0 -cos(theta);0 -sin(phi)...
%     -sin(theta)*sin(phi)+cos(theta)*cos(phi);
%     0 -cos(phi)...
%     sin(theta)*cos(phi)+cos(theta)*sin(phi)];

M = Q'*JB*Q;
%s = Q*JB;
s = Q*[dphi;dtheta;dpsi];
%s = Q*JB;
%S = [0 -s(3) s(2);s(3) 0 -s(1);-s(2) s(1) 0];
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
% C33=(Iyy-Izz)*dphi*cos(phi)*sin(phi)*cos(theta)^2-Iyy*dtheta *sin(phi)^2*cos(theta)*sin(theta)-Izz*x11*cos(phi)^2*cos(theta)*sin(theta)+Ixx*x11*cos(theta)*sin(theta);
% C=[C11 C12 C13;C21 C22 C23;C31 C32 C33];


A = [ 1 1 1 1;
      0  L 0 -L;
     -L 0  L  0;
     gamma -gamma gamma -gamma];

%references
% zd = 2; phid = pi/4.; thetad = pi/4.; psid = pi/4.;

%error signals
% ez = zd-z;
%  ephi = phid-phi;
%  etheta = thetad-theta;
% epsi = psid-psi;

%controller
%u(1)  = 50*ez-60*dz+30*iez;
% u(1)  = 50*ez-60*dz;
% u(2)  = 1*ephi-2*dphi;
% u(3)  = 1*etheta-2*dtheta;
% u(4) = 1*epsi-2*dpsi;

% uz = 50*ez-60*dz;
% tx = 1*ephi-2*dphi;
% ty = 1*etheta-2*dtheta;
% tz = 1*epsi-2*dpsi;
%X= [7;-10;0;0;0;0;0;0;0;0;0;0];
%u = [0;0;uz;tx;ty;tz];
u = [u(1);u(2);u(3);u(4)];
th = pinv(A)*u;
%th = pinv(A)*u;
w = sqrt(th/kf);
%equations of motion

dX= [7;-10;0;0;0;0;0;0;0;0;0;0];
%dX = zeros(12,1);
dX(1:3,1) = [dx;dy;dz];
dX(4:6,1) = [dphi;dtheta;dpsi];
dX(7:9,1) = (1/m)*(kf*R*A(1,:)*w.^2 - m*g*[0;0;1] );
%dX(7:9,1) = (1/m)*( R*u(1:3,:)- m*g*[0;0;1] );
%dX(7:9,1) = (1/m)*( R*A(1:3,:)*w - m*g*[0;0;1] );
%dX(10:12,1) = inv(M)*(u(4:6,:)- C*[dphi;dtheta;dpsi] );
dX(10:12,1) = inv(M)*(kf*A(2:4,:)*w.^2- C*[dphi;dtheta;dpsi] );
%dX(10:12,1) = (1/M)*(kf*A(4:6,:)*w.^2- C*[dphi;dtheta;dpsi] );
%dX=[dX(1);dX(2);dX(3);dX(4);dX(5);dX(6);dX(7);dX(8);dX(9);dX(10);dX(11);dX(12)];
