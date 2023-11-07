function dX = MultirotorStatesHnew(X,u)
%global dX

m=5;

%%% phi=Q(2); % theta=Q(3); % psi=Q(4);

% Rz=[cos(psi) sin(psi) 0;-sin(psi)) cos(psi) 0;0 0 1];
% Rx=[1 0 0;0 cos(phi) sin(phi);0 -sin(phi) cos(phi)];
% Ry=[cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)];

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

% Qw_dot=-1/2*[Q(2) Q(3) Q(4)]*W;
% Qv_dot=1/2*(Q(1)*J-Sq)*W;

% Qw_dot=-1/2*[Q(2) Q(3) Q(4)]*W
% Qv_dot=1/2*(Q(1)*J-Sq)*W;
%$Q_dot=[Qw_dot;Qv_dot];

% W_dot=(inv(J))*(ta-cross(W,h)+cross(rcom,h1)+cross(rT,h2)+dT);
% 
% V_dot=SL*((ab)+Rb*(cross(W_dot,rT)+(cross(W,cross(W,rT)))));

% dX(8:10)=SL*((ab)+Rb*(cross(W_dot,rT)+(cross(W,cross(W,rT)))));
% dX(11:13)=(inv(J))*(ta-cross(W,h)+cross(rcom,h1)+cross(rT,h2)+dT);

dX(11:13)=(inv(J))*(ta-cross(W,h)+cross(rcom,h1)+cross(rT,h2)+dT);
dX(8:10)=SL*((ab)+Rb*(cross(dX(11:13),rT)+(cross(W,cross(W,rT)))));

dX=[dX(1);dX(2);dX(3);dX(4);dX(5);dX(6);dX(7);dX(8);dX(9);dX(10);dX(11);dX(12);dX(13)];
%%------------------------------------------------------------------
%%-----------------------------------------------------------------
% 
% % Set values for dynamics parameters
% Ixx = 1.2; 
% Iyy = 1.2;
% Izz= 2.3;
% k = 1;
% l = 0.25;
% m = 2;
% b= 0.2;
% g = 9.81;
% 
% %kf = 1; km = 0.2; gamma = km/kf; L= 0.25;
% 
% % Transformation matrix for angular velocities from inertial frame to body frame
% W = [1, 0, -sin(X(5));
%      0, cos(X(4)), cos(X(5))*sin(X(4));
%      0, -sin(X(4)), cos(X(5))*cos(X(4))];
%  
% dW = [0 0 -cos(X(5));
%     0 -sin(X(4)) -sin(X(5))*sin(X(4))+cos(X(5))*cos(X(4));
%     0 -cos(X(4)) -sin(X(5))*cos(X(4))-cos(X(5))*sin(X(4))];
% 
% % W = [1, 0, -sin(thetat);
% %     0, cos(phit), cos(thetat)*sin(phit);
% %     0, -sin(phit), cos(thetat)*cos(phit)];
% 
% %R-ZYX Euler
% Rz = [cos(X(6)), -sin(X(6)), 0;
%       sin(X(6)), cos(X(6)), 0;
%       0, 0, 1];
% Ry = [cos(X(5)), 0, sin(X(5));
%       0, 1, 0;
%       -sin(X(5)), 0, cos(X(5))];
% Rx = [1, 0, 0;
%       0, cos(X(4)), -sin(X(4));
%       0, sin(X(4)), cos(X(4))];
% 
% % Rz = [cos(psit), -sin(psit), 0;
% %     sin(psit), cos(psit), 0;
% %     0, 0, 1];
% % Ry = [cos(thetat), 0, sin(thetat);
% %     0, 1, 0;
% %     -sin(thetat), 0, cos(thetat)];
% % Rx = [1, 0, 0;
% %     0, cos(phit), -sin(phit);
% %     0, sin(phit), cos(phit)];
% 
% % Rotation matrix from body frame to inertial frame
% R = Rz*Ry*Rx;
% 
% 
% 
% % Jacobian (relates body frame to inertial frame velocities)
% I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
% J = W.'*I*W;
% 
% % Coriolis forces
% % dJ_dt = diff(J);
% % dJ_dt = subs(dJ_dt,[state_diff statet],[state_dot state]);
% % h_dot_J = [phidott(t), thetadott(t), psidott(t)]*J;
% % h_dot_J = subs(h_dot_J,[state_diff statet],[state_dot state]);
% % grad_temp_h = jacobian(h_dot_J,[phi theta psi]);
% % C = dJ_dt - 1/2*grad_temp_h;
% 
% 
% %M = W'*I*W;
% % s = W*[X(10);X(11);X(12)];
% % S = [0 -s(3) s(2);s(3) 0 -s(1);-s(2) s(1) 0];
% % C = W'*S*I*W + W'*I*dW;
% % 
% C11=0;
% C12=(Iyy-Izz)*(X(11)*cos(X(4))*sin(X(4))+X(12)*sin(X(4))^2*cos(X(5)))+(Izz-Iyy)*X(12)*cos(X(4))^2*cos(X(5))-Ixx*X(12)*cos(X(5));
% C13=(Izz-Iyy)*(X(12)*cos(X(4))*sin(X(4))*cos(X(5))^2);
% C21=(Izz-Iyy)*(X(11)*cos(X(4))*sin(X(4))+X(12)*sin(X(4))*cos(X(4)))+(Iyy-Izz)*X(12)*cos(X(4))^2*cos(X(5))+Ixx*X(12)*cos(X(5));
% C22=(Izz-Iyy)*X(10)*cos(X(4))*sin(X(4));
% C23= -Ixx*X(12)*sin(X(5))*cos(X(5))+Iyy*X(12)*sin(X(4))^2*sin(X(5))*cos(X(5))+ Izz*X(12)*cos(X(4))^2*sin(X(5))*cos(X(5));
% C31=(Iyy-Izz)*(X(12)*cos(X(5))^2*sin(X(4))*cos(X(4)))-Ixx*X(11)*cos(X(5));
% C32= (Izz-Iyy)*(X(11)*cos(X(4))*sin(X(4))*sin(X(5))+X(10)*sin(X(4))^2*cos(X(5)))+(Iyy-Izz)*X(10)*cos(X(4))^2*cos(X(4))+Ixx*X(12)*sin(X(5))*cos(X(5))-Iyy*X(12)*sin(X(10))^2*sin(X(5))*cos(X(5))-Izz*X(12)*cos(X(10))^2*sin(X(5))*cos(X(5));
% C33=(Iyy-Izz)*X(10)*cos(X(4))*sin(X(4))*cos(X(5))^2-Iyy*X(11)*sin(X(4))^2*cos(X(5))*sin(X(5))-Izz*X(11)*cos(X(4))^2*cos(X(5))*sin(X(5))+Ixx*X(11)*cos(X(5))*sin(X(5));
% %Cnndot=[C11 C12 C13;C21 C22 C23;C31 C32 C33];
% C=[C11 C12 C13;C21 C22 C23;C31 C32 C33];
% 
% 
% % A = [1 1 1 1;
% %       0  -L 0 L;
% %      -L 0  L  0;
% %      -gamma gamma -gamma gamma];
% 
% u = [u(1);u(2);u(3);u(4)];
% % th = pinv(A)*u;
% %  w=  (th/kf);         % actually it's omeega square
% %w= sqrt(th/kf);
% 
% % Total thrust
% T = k*(u(1)+u(2)+u(3)+u(4));
% %T = k*(w(1)+w(2)+w(3)+w(4));
% %T = k*(u1+u2+u3+u4);
% % u(1)= k*(u1+u2+u3+u4);
% 
% % Torques in the direction of phi, theta, psi
% tau_beta = [l*k*(-u(2) + u(4));l*k*(-u(1) + u(3));b*(-u(1)+u(2)-u(3)+u(4))];
% %tau_beta = [l*k*(-u2 + u4);l*k*(-u1 + u3);b*(-u1+u2-u3+u4)];
% %tau_beta= [l*k*(-w(2)+w(4));l*k*(-w(1)+w(3));b*(-w(1)+w(2)-w(3)+w(4))];
% 
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
% % Replace parameters and drop time dependence
% % DX = subs(f, [Ixx Iyy Izz k l m b g], paramValues);
% % DX= subs(f,statet,state);
% % DX = simplify(f);
% % 
% %  dX = subs(dX, [Ixx Iyy Izz k l m b g], paramValues);
% %  dX= subs(dX,statet,state);
% %  dX = simplify(dX);
% % 
% % % Calculate linearization
% % A = jacobian(f,[state{:}]);
% % control = [u1, u2, u3, u4];
% % B = jacobian(f,control);
% % 
% % % Create QuadrotorStateFcn.m
% % matlabFunction(transpose(f),'File','QuadrotorStateFcn11',...
% %     'Vars',{transpose([state{:}]),transpose(control)})
% % % Create QuadrotorStateJacobianFcn.m 
% % matlabFunction(A, B,'File','QuadrotorStateJacobianFcn11',...
% %     'Vars',{transpose([state{:}]),transpose(control)})
% % 
% % %Clear symbolic variables
% % clear
% % 
% % % Confirm the functions are generated successfully
% % while isempty(which('QuadrotorStateJacobianFcn11'))
% %     pause(0.1);
% % end
