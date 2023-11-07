clear all
close all
clc

global dX
%dX = zeros(13,1);
dX = zeros(12,1);
%Initial conditions
x0 = [0 0 0 0 0 0 0 0 0 0 0 0];
%x0 = [0 0 0 0 0 0 0 0 0 0 0 0 0];
%Simulation
[t,X] = ode45('quadrotor',[0 20],x0);
%State variables
x = X(:,1); y = X(:,2); z = X(:,3);
phi = X(:,4); theta = X(:,5); psi = X(:,6);
%Plots
figure(1)
subplot(3,2,1)
plot(t,x,'k','LineWidth',2)
subplot(3,2,3)
plot(t,y,'k','LineWidth',2)
subplot(3,2,5)
plot(t,z,'k','LineWidth',2)
subplot(3,2,2)
plot(t,phi,'k','LineWidth',2)
subplot(3,2,4)
plot(t,theta,'k','LineWidth',2)
subplot(3,2,6)
plot(t,psi,'k','LineWidth',2)
