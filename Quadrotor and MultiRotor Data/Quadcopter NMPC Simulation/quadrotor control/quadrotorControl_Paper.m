function dX = Quadrotor(t,X)
global dX
x = X(1);  y = X(2); z = X(3);
phi = X(4);  theta = X(5);  psi = X(6);
dx = X(7); dy = X(8); dz = X(9);
dphi = X(10); dtheta = X(11); dpsi = X(12);
%iez = X(13);

%parameters

Ixx = 1.2; 
Iyy = 1.2;
Izz= 0.05;
 %Izz= 2.3;
k = 1;
 %k = 1e-3;
 L = 0.25;
 m = 2;
 g = 9.81;
 km= 0.2; 
 %km = 0.5e-4; 
 gamma = km/k;


% Ixx = (1/12)*m*(0.2^2+0.2^2);
% Iyy= (1/12)*m*(0.2^2+0.2^2);
%Izz = (1/12)*m*(0.2^2+0.2^2);

I = [Ixx 0 0;0 Iyy 0;0 0 Izz];

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


M = Q'*I*Q;

s = Q*[dphi;dtheta;dpsi];
S = [0 -s(3) s(2);s(3) 0 -s(1);-s(2) s(1) 0];
C = Q'*S*I*Q + Q'*I*dQ;

%%-----------------------------------------------------------------
%  C11=0;
%  C12=(Iyy-Izz)*(dtheta*cos(phi)*sin(phi)+dpsi*sin(phi)^2*cos(theta))+(Izz-Iyy)*dpsi*cos(phi)^2*cos(theta)-Ixx*dpsi*cos(theta);
%  C13=(Izz-Iyy)*(dpsi*cos(phi)*sin(phi)*cos(theta)^2);
%  C21=(Izz-Iyy)*(dtheta*cos(phi)*sin(phi)+dpsi*sin(phi)*cos(theta))+(Iyy-Izz)*dpsi*cos(phi)^2*cos(theta)+Ixx*dpsi*cos(theta);
%  C22=(Izz-Iyy)*dphi*cos(phi)*sin(phi);
%  C23= -Ixx*dpsi*sin(theta)*cos(theta)+Iyy*dpsi*sin(phi)^2*sin(theta)*cos(theta)+ Izz*dpsi*cos(phi)^2*sin(theta)*cos(theta);
%  C31=(Iyy-Izz)*(dpsi*cos(theta)^2*sin(phi)*cos(phi))-Ixx*dtheta*cos(theta);
%  C32= (Izz-Iyy)*(dtheta*cos(phi)*sin(phi)*sin(theta)+dphi*sin(phi)^2*cos(theta))+(Iyy-Izz)*dphi*cos(phi)^2*cos(theta)+Ixx*dpsi*sin(theta)*cos(theta)-Iyy*dpsi*sin(dphi)^2*sin(theta)*cos(theta)-Izz*dpsi*cos(dphi)^2*sin(theta)*cos(theta);
%  C33=(Iyy-Izz)*dphi*cos(phi)*sin(phi)*cos(theta)^2-Iyy*dtheta*sin(phi)^2*cos(theta)*sin(theta)-Izz*dtheta*cos(phi)^2*cos(theta)*sin(theta)+Ixx*dtheta*cos(theta)*sin(theta);
%  C=[C11 C12 C13;C21 C22 C23;C31 C32 C33];

%%-----------------------------------------------------------------
A = [1 1 1 1;
     0  L 0 -L;
     -L 0  L  0;
     gamma -gamma gamma -gamma];

%  A = [0 0 0 0;
%      0 0 0 0;
%      1 1 1 1;
%      0  L 0 -L;
%      -L 0  L  0;
%      gamma -gamma gamma -gamma];

%references
xd = 0; yd = 0; zd = -2;
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

uz = -(50*ez-60*dz);
tx = (1*ephi-2*dphi);
ty = (1*etheta-2*dtheta);
tz = (1*epsi-2*dpsi);

u = [uz;tx;ty;tz];
%u = [0;0;uz;tx;ty;tz];
%th = pinv(A)*u1;
th = pinv(A)*u;
w = sqrt(th/k);
%w =(th/k);

T = k*(w(1)^2+w(2)^2+w(3)^2+w(4)^2);
tB= [L*k*(w(2)^2-w(4)^2);L*k*(-w(1)^2+w(3)^2);km*(w(1)^2-w(2)^2+w(3)^2-w(4)^2)];


%equations of motion

dX(1:3,1) = [dx;dy;dz];
dX(4:6,1) = [dphi;dtheta;dpsi];

%dX(7:9,1) = (1/m)*(-kf*R*A(1:3,:)*w.^2 + m*g*[0;0;1] );
dX(7:9,1) = (1/m)*(-R*[0;0;T]+ m*g*[0;0;1]);

%dX(10:12,1) = inv(M)*(kf*A(4:6,:)*w.^2- C*[dphi;dtheta;dpsi] );
dX(10:12,1) = inv(M)*(Q'*tB- C*[dphi;dtheta;dpsi]);


%dX(10:12,1) = inv(M)*(u(4:6,:)- C*[dphi;dtheta;dpsi] );
%dX(7:9,1) = (1/m)*( R*A(1:3,:)*w - m*g*[0;0;1] );