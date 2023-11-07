function dX = quadrotor(t,X)
global dX
x = X(1);  y = X(2); z = X(3);
phi = X(4);  theta = X(5);  psi = X(6);
dx = X(7); dy = X(8); dz = X(9);
dphi = X(10); dtheta = X(11); dpsi = X(12);

%parameters

% Jx = 1.2; 
% Jy = 1.2;
% Jz= 2.3;
k = 1e-3;
%k = 1;
l = 0.25;
L = 0.25;
 m = 2;
% b= 0.2;
  b=  0.5e-4;
 g = 9.81;
gamma = b/k;

%kf = 1; km = 0.2; gamma = km/kf;

Jx = (1/12)*m*(0.2^2+0.2^2);
Jy = (1/12)*m*(0.2^2+0.2^2);
Jz = (1/12)*m*(0.2^2+0.2^2);
JB = [Jx 0 0;0 Jy 0;0 0 Jz];

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



M = Q'*JB*Q;
s = Q*[dphi;dtheta;dpsi];
S = [0 -s(3) s(2);s(3) 0 -s(1);-s(2) s(1) 0];
C = Q'*S*JB*Q + Q'*JB*dQ;

A = [ 0 0 0 0;
      0 0 0 0;
      1 1 1 1;
      0  L 0 -L;
     -L 0  L  0;
     gamma -gamma gamma -gamma];
% A = [ 1 1 1 1;
%       0  L 0 -L;
%      -L 0  L  0;
%      gamma -gamma gamma -gamma];

%references
xd = 0; yd = 0; zd = 2;
phid = pi/4.; thetad = pi/4.; psid = pi/4.;

%error signals
ez = zd-z;
ephi = phid-phi;
etheta = thetad-theta;
epsi = psid-psi;

%controller
%u(1)  = 50*ez-60*dz+30*iez;
% u(1)  = 50*ez-60*dz;
% u(2)  = 1*ephi-2*dphi;
% u(3)  = 1*etheta-2*dtheta;
% u(4) = 1*epsi-2*dpsi;

uz = 50*ez-60*dz;
tx = 1*ephi-2*dphi;
ty = 1*etheta-2*dtheta;
tz = 1*epsi-2*dpsi;
%X= [7;-10;0;0;0;0;0;0;0;0;0;0];
u = [0;0;uz;tx;ty;tz];
%u = [uz;tx;ty;tz];
th = pinv(A)*u;
%w = (th/kf);
w = sqrt(th/k);

%equations of motion
%dX = zeros(12,1);
% 
 T = k*(w(1)^2+w(2)^2+w(3)^2+w(4)^2);
% 
 tB= [l*k*(-w(2)^2+w(4)^2);l*k*(w(1)^2-w(3)^2);b*(-w(1)^2+w(2)^2-w(3)^2+w(4)^2)];



dX(1:3,1) = [dx;dy;dz];
dX(4:6,1) = [dphi;dtheta;dpsi];
%dX(7:9,1) = (1/m)*(kf*R*A(1:3,:)*w.^2 + m*g*[0;0;1] );

%dX(7:9,1) = g*[0;0;1]-R*[0;0;T]/m;
dX(7:9,1) = (1/m)*(k*R*A(1:3,:)*w.^2 + m*g*[0;0;1] );

%dx(10:12,1)= inv(M)*(tB-C*[dphi;dtheta;dpsi]);
dX(10:12,1) = inv(M)*(k*A(4:6,:)*w.^2- C*[dphi;dtheta;dpsi] );




%%----------------------------------------------------------

%dX(10:12,1) = inv(M)*(u(4:6,:)- C*[dphi;dtheta;dpsi] );
%dX(7:9,1) = (1/m)*( R*A(1:3,:)*w - m*g*[0;0;1] );