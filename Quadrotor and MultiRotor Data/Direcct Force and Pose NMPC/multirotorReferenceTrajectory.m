function [ xdesired ] = multirotorReferenceTrajectory( t )
% This function generates reference signal for nonlinear MPC controller
% used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

%#codegen
% x =6*sin(t/3);
% y = -6*sin(t/3).*cos(t/3);
% z = 6*cos(t/3);
x =6*sin(t/3);
y = -6*sin(t/3).*cos(t/3);
z = 6*cos(t/3);
%z = 10+zeros(1,length(t));
zero=zeros(1,length(t));

phi = zeros(1,length(t));
theta = zeros(1,length(t));
psi = zeros(1,length(t));
% phi = pi/6*sin(t/3);
% theta = -pi/3*sin(t/3).*cos(t/3);
% psi = pi*ones(1,length(t));

xdot = zeros(1,length(t));
ydot = zeros(1,length(t));
zdot = zeros(1,length(t));

phidot = zeros(1,length(t));
thetadot = zeros(1,length(t));
psidot = zeros(1,length(t));

xdesired = [x;y;z; zero; phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot]';
end



% 
% %#codegen
% x =6*sin(t/3);
% y = -6*sin(t/3).*cos(t/3);
% z = 6*cos(t/3);
% zero=zeros(1,length(t));
% phi = zeros(1,length(t));
% theta = zeros(1,length(t));
% psi = zeros(1,length(t));
% xdot = zeros(1,length(t));
% ydot = zeros(1,length(t));
% zdot = zeros(1,length(t));
% phidot = zeros(1,length(t));
% thetadot = zeros(1,length(t));
% psidot = zeros(1,length(t));
% 
% xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
% end

