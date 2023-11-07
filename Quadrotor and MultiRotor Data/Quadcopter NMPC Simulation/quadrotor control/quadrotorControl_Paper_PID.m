function dX = Quadrotor(t,X)
global dX
x = X(1);  y = X(2); z = X(3);
phi = X(4);  theta = X(5);  psi = X(6);
dx = X(7); dy = X(8); dz = X(9);
dphi = X(10); dtheta = X(11); dpsi = X(12);

iez = X(13);iep = X(14);iet = X(15);ieps = X(16);

%parameters

% Jx = 1.2; 
% Jy = 1.2;
% Jz= 2.3;
% k = 1;
kf = 1e-3;
L = 0.25;
 m = 2;
 g = 9.81;
% b= 0.2;
 km = 0.5e-4; 
 gamma = km/kf;


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

A = [0 0 0 0;
     0 0 0 0;
     1 1 1 1;
     0  L 0 -L;
     -L 0  L  0;
     gamma -gamma gamma -gamma];

%references
xd = 0; yd = 0; zd = -2;
phid = pi/4.; thetad = pi/4.; psid = pi/4.;

%error signals
ez = zd-z;
ephi = phid-phi;
etheta = thetad-theta;
epsi = psid-psi;



%controller
uz = -(50*ez-60*dz+30*iez);
%u(1)  = 50*ez-60*dz;
tx = 1*ephi-2*dphi+0.1*iep;
ty  = 1*etheta-2*dtheta+0.1*iet;
tz  = 1*epsi-2*dpsi+0.1*ieps;

% uz = -(50*ez-60*dz);
% tx = (1*ephi-2*dphi);
% ty = (1*etheta-2*dtheta);
% tz = (1*epsi-2*dpsi);

% %X= [7;-10;0;0;0;0;0;0;0;0;0;0];
% u = [0;0;uz;tx;ty;tz];
% %th = pinv(A)*u1;
% th = pinv(A)*u;
% w = sqrt(th/kf);
% %w =(th/kf);

T=uz;
tB=[tx;ty;tz];
% T = kf*(w(1)^2+w(2)^2+w(3)^2+w(4)^2);
% tB= [L*kf*(w(2)^2-w(4)^2);L*kf*(-w(1)^2+w(3)^2);km*(w(1)^2-w(2)^2+w(3)^2-w(4)^2)];
% 

% T = kf*(w(1)+w(2)+w(3)+w(4));
% tB= [L*kf*(w(2)-w(4));L*kf*(-w(1)+w(3));km*(w(1)-w(2)+w(3)-w(4))];
% 

%equations of motion

dX(1:3,1) = [dx;dy;dz];
dX(4:6,1) = [dphi;dtheta;dpsi];

%dX(7:9,1) = (1/m)*(-kf*R*A(1:3,:)*w.^2 + m*g*[0;0;1] );
dX(7:9,1) = (1/m)*(-R*[0;0;T]+ m*g*[0;0;1] );

%dX(10:12,1) = inv(M)*(kf*A(4:6,:)*w.^2- C*[dphi;dtheta;dpsi] );
dX(10:12,1) = inv(M)*(Q'*tB- C*[dphi;dtheta;dpsi]);


dX(13) = ez;
dX(14) = ephi;
dX(15) = etheta;
dX(16) = epsi;




%dX(10:12,1) = inv(M)*(u(4:6,:)- C*[dphi;dtheta;dpsi] );
%dX(7:9,1) = (1/m)*( R*A(1:3,:)*w - m*g*[0;0;1] );