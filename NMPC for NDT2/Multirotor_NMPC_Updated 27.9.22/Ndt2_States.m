function dX = MultirotorStatesHnew(X,u)
%global dX


%% system dimensions
nx = 24;
nu = 6;
np = 18;

%% system parameters
m = 6.3833;
g = 9.81;
%%J = [0.29, 0, 0; 0, 0.29, 0; 0, 0, 0.3527];
f_ext = zeros(3,1);
tau_ext = zeros(3,1);

Ixx = 0.29;
Iyy = 0.29;
Izz = 0.3527;
J = [Ixx 0 0;0 Iyy 0;0 0 Izz];

%% system parameters

% input (input is the derivative fo f and tau)
fxd = u(1);        
fyd = u(2);         
fzd = u(3);

tauxd = u(4);
tauyd = u(5);
tauzd = u(6); 

Fd = [fxd;fyd;fzd];
taud = [tauxd;tauyd;tauzd];


%fd=[u(1);u(2);u(3)];
%taud=[u(4);u(5);u(6)];


%% dynamics

p_lin = [x,y,z]';    % Linear Position   
v = [vx,vy,vz]';     % Linear Velocity  
w = [wx,wy,wz]';     % Angular Velocity  
F = [fx,fy,fz]';
tau = [tx,ty,tz]';


skew_w =  [ 0, -wz,  wy;
           wz,   0, -wx; 
          -wy,  wx,   0];

skew_Jw =  [         0,   -J(3,3)*wz,  J(2,2)*wy;
             J(3,3)*wz,            0, -J(1,1)*wx; 
            -J(2,2)*wy,    J(1,1)*wx,         0];


R_mat = [ R11, R12, R13; R21, R22, R23; R31, R32, R33];

%%linear positon dynamics
pos_dyn = R_mat*v;     %% linear Velocity

% attitude dynamics
% att_dyn = [cos(pitch), -sin(pitch)*cos(roll), -cos(pitch)*sin(roll); 0, cos(pitch), -sin(pitch); sin(pitch)/cos(roll), cos(pitch)/cos(roll), 0]*w;
att_dyn = R_mat*skew_w;   %% angular Velocity
% uav dynamics


I = [1 0 0;0 1 0;0 0 1];

% [xd_dot(1);xd_dot(2);xd_dot(3)]
v_dot =(1/m)*I*(F - m*skew_w*v + m*g*[0;0;1]); % dynamics (linear part) according to order of the classic drone without external disturbances (contact force/friction etc.)

% [wxd_dot(1);wxd_dot(2);wxd_dot(3)]
w_dot = (inv(J))*(tau + skew_Jw*w);          % dynamic (angular part) according to order of the classic drone without external disturbances (contact force/friction etc.)

% [x_dot(1);x_dot(2);x_dot(3); q_dot(1);q_dot(2);q_dot(3);q_dot(4)] 
expr_f_expl3 = vertcat( expr_f_expl2, att_dyn(1,1), att_dyn(1,2), att_dyn(1,3), att_dyn(2,1), att_dyn(2,2), att_dyn(2,3), att_dyn(3,1), att_dyn(3,2), att_dyn(3,3), pos_dyn );                 % evoluzione posizione lineare e angolare
% [F_dot(1);F_dot(2);F_dot(3); tau_d(1);tau_d(2);tau_d(3)] 
u  =  [Fd,taud];                         % dynamics of control inputs







% states
x = X(1);    %% linear position
y = X(2); 
z = X(3);

vx = X(4);   %% linear Velocity
vy = X(5); 
vz = X(6);

phi = X(7);         %% angular Position 
theeta = X(8); 
psi = X(9);

wx = X(10);    %% angular Velocity
wy = X(11); 
wz = X(12);

fx = X(13);
fy = X(14)
fz = X(15);

tx = X(16);
ty = X(17)
tz = X(18);


R11 = X(19);  %% Rotation Matrix
R12 = X(20);
R13 = X(21);
R21 = X(22); 
R22 = X(23);
R23 = X(24);
R31 = X(25); 
R32 = X(26);
R33 = X(27);

% x = vertcat(vx, vy, vz, wx, wy, wz, R11, R12, R13, R21, R22, R23, R31, R32, R33, x, y, z, fx, fy, fz, tx, ty, tz);
% u = vertcat(fxd, fyd, fzd, tauxd, tauyd, tauzd);















































Rz=[cos(X(7)) sin(X(7)) 0; -sin(X(7)) cos(X(7)) 0; 0 0 1];
Rx=[1 0 0; 0 cos(X(5)) sin(X(5)); 0 -sin(X(5)) cos(X(5))];
Ry=[cos(X(6)) 0 -sin(X(6)); 0 1 0; sin(X(6)) 0 cos(X(6))];

Rb=Rz*Ry*Rx;
Rs=[1 0 0;
    0 0 -1;
    0 1 0 ];

%Rs=[1 0 0;0 -1 0;0 0 -1];
%lambda=L;
L=0;
%L=1;
Rd=[1 0 0;
    0 1 0;
    0 0 1-L];

SL=Rs*Rd*transpose(Rs);

rT=[0.5;0;0];

Ixx=1;
Iyy=1;
Izz=1;
J=[Ixx 0 0;0 Iyy 0;0 0 Izz];

g=9.8;
dF=[0;0;0];
fc=[0;0;0];
rcom=[0;0;0];
dT=[0;0;0];

% fa=[fax;fay;faz];
% ta=[Tax;Tay;Taz];

fa=[u(1);u(2);u(3)];
ta=[u(4);u(5);u(6)];

ab=(m^-1)*((Rb*fa)+(L*fc)+dF)+[0;0;g];

% T=[0 -sin(Q(2)) cos(Q(2))*sin(Q(3));
% 0 cos(Q(2)) sin(Q(2))*sin(Q(3));
% 1 0 cos(Q(3))];

% Sq=[0 -Q(4) Q(3);
%     Q(4) 0 -Q(2);
%     -Q(3) Q(2) 0];
Sq=[0 -X(7) X(6);
    X(7) 0 -X(5);
    -X(6) X(5) 0];

%eata_dot=[Q(2);Q(3);Q(4)]
%eata_dot=[X(5);X(6);X(7)]
%w=T*eata_dot;

W=[X(11);X(12);X(13)];
h=J*W;
%h=J*[X(11);X(12);X(13)];
h1=transpose(Rb)*[0;0;m*g];
h2=L*transpose(Rb)*fc;

%P_dot=SL*V;

% dX(1:3)=SL*V;

dX=[7;-10;0;0;0;0;0;0;0;0;0;0;0];

dX(1:3)=SL*[X(8);X(9);X(10)];

dX(4)=-1/2*[X(5) X(6) X(7)]*W;
dX(5:7)=1/2*(X(4)*J-Sq)*W;

dX(11:13)=(inv(J))*(ta-cross(W,h)+cross(rcom,h1)+cross(rT,h2)+dT);
dX(8:10)=SL*((ab)+Rb*(cross(dX(11:13),rT)+(cross(W,cross(W,rT)))));

dX=[dX(1);dX(2);dX(3);dX(4);dX(5);dX(6);dX(7);dX(8);dX(9);dX(10);dX(11);dX(12);dX(13)];
%%------------------------------------------------------------------
%%-----------------------------------------------------------------



% 
% % Dynamics
% 
% %dX = zeros(12,1);
% 
% dX = [7;-10;0;0;0;0;0;0;0;0;0;0];
% 
% dX(1) = X(7);
% dX(2) = X(8);
% dX(3) = X(9);
% dX(4) = X(10);
% dX(5) = X(11);
% dX(6) = X(12);
% 
% % Equations for COM configuration
% dX(7:9) = -g*[0;0;1] + R*[0;0;T]/m;
% 
% % Euler Lagrange equations for angular dynamics
% dX(10:12) = inv(J)*(tau_beta - C*[X(10); X(11); X(12)]);
% %dX=[dX(1);dX(2);dX(3);dX(4);dX(5);dX(6);dX(7);dX(8);dX(9);dX(10);dX(11);dX(12)];
% 
% 
% %%%------------------------------------------------------------------------
% 
