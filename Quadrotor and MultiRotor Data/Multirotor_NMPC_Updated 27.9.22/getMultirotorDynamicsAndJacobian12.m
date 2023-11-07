% This script defines a continuous-time nonlinear multirotor model and
% generates a state function and its Jacobian function used by the
% nonlinear MPC controller in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

% Create symbolix variables for states, MVs and parameters
% syms xt(t) yt(t) zt(t) phit(t) thetat(t) psit(t) xdott(t) ydott(t)...
%     zdott(t) phidott(t) thetadott(t) psidott(t) 
syms xt(t) yt(t) zt(t) qwt(t) phit(t) thetat(t) psit(t)xdott(t) ydott(t) zdott(t)  ...
    phidott(t) thetadott(t) psidott(t) 
syms fax fay faz Tax Tay Taz
syms Ixx Iyy Izz m g L dT rcom fc dF
syms x y z qw phi theta psi xdot ydot zdot phidot thetadot psidot

% Set values for dynamics parameters
IxxVal = 1.2; 
IyyVal = 1.2;
IzzVal = 2.3;
mVal = 5;
gVal = 9.81;
LVal = 0;
dTVal=0;
rcomVal=0;
fcVal=0;
dFVal=0;

paramValues = [IxxVal IyyVal IzzVal mVal gVal LVal dTVal rcomVal fcVal dFVal];

% Group symbolic variables
statet = {xt(t) yt(t) zt(t) qwt(t) phit(t) thetat(t) psit(t) xdott(t) ydott(t) zdott(t) ...
     phidott(t) thetadott(t) psidott(t)};
state = {x y z qw phi theta psi xdot ydot zdot phidot thetadot psidot};
state_diff = {diff(xt(t),t), diff(yt(t),t), diff(zt(t),t), ...
    diff(qwt(t),t),diff(phit(t),t), diff(thetat(t),t), diff(psit(t),t)};
state_dot = {xdot ydot zdot qwdot phidot thetadot psidot};


%R-ZXY Euler
Rz = [cos(psit), sin(psit), 0;
    -sin(psit), cos(psit), 0;
    0, 0, 1];
Rx = [1, 0, 0;
    0, cos(phit), sin(phit);
    0, -sin(phit), cos(phit)];
Ry = [cos(thetat), 0, -sin(thetat);
    0, 1, 0;
    sin(thetat), 0, cos(thetat)];

Rs=[1 0 0;0 0 -1;0 1 0 ];
% Rotation matrix from body frame to inertial frame
Rb = Rz*Ry*Rx;

% % Jacobian (relates body frame to inertial frame velocities)
J = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];

% Torques in the direction of phi, theta, psi
%lambda=L;
L=0;
Rd=[1 0 0;0 1 0;0 0 1-L];
SL=Rs*Rd*transpose(Rs);

fc=0;
dF=0;
m=5;
g=9.8;
rcom=0;
dT=0;
rT=[0.5;0;0];

fa=[fax;fay;faz];
ta=[Tax;Tay;Taz];


ab=(m^-1)*((Rb*fa)+(L*fc)+dF)+[0;0;g];

T =[0 -sin(phit) cos(phit)*sin(thetat);
    0 cos(phit) sin(phit)*sin(thetat);
    1 0 cos(thetat)];

Sq=[0 -psit thetat;
    psit 0 -phit;
    -thetat phit 0];

eata_dot=[phidott(t);thetadott(t);psidott(t)];
%eata_dot=[phit;thetat;psit]
%eata_dot=[Q(2);Q(3);Q(4)]
w=T*eata_dot;

h=J*w;
h1=transpose(Rb)*[0;0;m*g];
h2=L*transpose(Rb)*fc;

V=[xdott(t);ydott(t);zdott(t)];
%W=[phidott(t);thetadott(t);psidott(t)];

P_dot=SL*V;

Qw_dot=-1/2*[phidott(t);thetadott(t);psidott(t)]*w;
Qv_dot=1/2*(qwt(t)*J-Sq)*w;

% Qw_dot=-1/2*[Q(2) Q(3) Q(4)]*W
% Qv_dot=1/2*(Q(1)*J-Sq)*W;
Q_dot=[Qw_dot;Qv_dot];

W_dot=(inv(J))*(ta-cross(w,h)+cross(rcom,h1)+cross(rT,h2)+dT);

V_dot=SL*((ab)+Rb*(cross(W_dot,rT)+(cross(w,cross(w,rT)))));

f=[P_dot,Q_dot,V_dot,W_dot];
% Replace parameters and drop time dependence
f = subs(f, [Ixx Iyy Izz m g L dT rcom fc dF], paramValues);
%f = subs(f, [Ixx Iyy Izz k l m b g L dT rcom], paramValues);
f = subs(f,statet,state);
f = simplify(f);

% Calculate linearization
A = jacobian(f,[state{:}]);
control = [fax, fay, faz, Tax, Tay, Taz];
B = jacobian(f,control);

% Create MultirotorStateFcn.m
matlabFunction(transpose(f),'File','MultirotorStateFcn12',...
    'Vars',{transpose([state{:}]),transpose(control)})
% Create MultirotorStateJacobianFcn.m 
matlabFunction(A, B,'File','MultirotorStateJacobianFcn12',...
    'Vars',{transpose([state{:}]),transpose(control)})

%Clear symbolic variables
clear

% Confirm the functions are generated successfully
while isempty(which('MultirotorStateJacobianFcn12'))
    pause(0.1);
end

%%------------------------__________________--------

%V=[xdott(t);ydott(t);zdott(t)];

%P_dot=SL*V;
%Q=[phit(t);thetat(t);psit(t)];
%W=[phidott(t);thetadott(t);psidott(t)];
%Q_dot=(1/2)*Q.*[0;W];
% W_dot=(inv(J))*(ta-cross(W,h)+cross(rcom,h1)+cross(rT,h2)+dT);
% V_dot=SL*((ab)+Rb*(cross(W_dot,rT)+(cross(W,cross(W,rT)))));
%%-----------------------------------------------------------------------------------

%m=5;
% Rz=[cos(Q(4)) sin(Q(4)) 0; -sin(Q(4)) cos(Q(4)) 0; 0 0 1];
% Rx=[1 0 0; 0 cos(Q(2)) sin(Q(2)); 0 -sin(Q(2)) cos(Q(2))];
% Ry=[cos(Q(3)) 0 -sin(Q(3)); 0 1 0; sin(Q(3)) 0 cos(Q(3))];
% 
% Rb=Rz*Ry*Rx;
% Rs=[1 0 0;
%     0 0 -1;
%     0 1 0 ];

%Rs=[1 0 0;0 -1 0;0 0 -1];
%lambda=L;
% L=0;
% %L=1;
% Rd=[1 0 0;
%     0 1 0;
%     0 0 1-L];
% 
% SL=Rs*Rd*transpose(Rs);

% rT=[0.5;0;0];

% Ixx=1;
% Iyy=1;
% Izz=1;
% J=[Ixx 0 0;0 Iyy 0;0 0 Izz];

% g=9.8;
% dF=[0;0;0];
% fc=[0;0;0];
% rcom=[0;0;0];
% dT=[0;0;0];

% fa=[fax;fay;faz];
% ta=[Tax;Tay;Taz];

% ab=(m^-1)*((Rb*fa)+(L*fc)+dF)+[0;0;g];
% 
% T =[0 -sin(phit) cos(phit)*sin(thetat);
%     0 cos(phit) sin(phit)*sin(thetat);
%     1 0 cos(thetat)];
% 
% % T=[0 -sin(Q(2)) cos(Q(2))*sin(Q(3));
% % 0 cos(Q(2)) sin(Q(2))*sin(Q(3));
% % 1 0 cos(Q(3))];
% 
% Sq=[0 -psit thetat;
%     psit 0 -phit;
%     -thetat phit 0];

% Sq=[0 -Q(4) Q(3);
%     Q(4) 0 -Q(2);
%     -Q(3) Q(2) 0];

% eata_dot=[phidott(t);thetadott(t);psidott(t)];
% %eata_dot=[phit;thetat;psit]
% %eata_dot=[Q(2);Q(3);Q(4)]
% w=T*eata_dot;
% 
% h=J*W;
% h1=transpose(Rb)*[0;0;m*g];
% h2=L*transpose(Rb)*fc;

% P_dot=SL*V;
% 
% Qw_dot=-1/2*[phidott(t);thetadott(t);psidott(t)]*w;
% Qv_dot=1/2*(qwt*J-Sq)*w;
% 
% % Qw_dot=-1/2*[Q(2) Q(3) Q(4)]*W
% % Qv_dot=1/2*(Q(1)*J-Sq)*W;
% Q_dot=[Qw_dot;Qv_dot];
% 
% W_dot=(inv(J))*(ta-cross(W,h)+cross(rcom,h1)+cross(rT,h2)+dT);
% 
% V_dot=SL*((ab)+Rb*(cross(W_dot,rT)+(cross(W,cross(W,rT)))));


%%---------------------------------------------------------------------------------