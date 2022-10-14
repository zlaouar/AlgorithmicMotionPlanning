%% Housekeeping
clear all
close all
clc

set(groot,'defaultTextInterpreter','tex')
%% Case (a)
% Plot 2-link robot
a1 = 1;
a2 = 1;
figure
theta1 = 0;
theta2 = 0;
xs = [0 a1*cos(theta1) (a1*cos(theta1)+ a2*cos(theta1+theta2))];
ys = [0 a1*sin(theta1) (a1*sin(theta1)+ a2*sin(theta1+theta2))];
subplot(1,2,1)
plot(xs,ys,'Linewidth',8)
hold on
scatter(xs,ys,'or','Linewidth',8)
% Plot obstacle
X = [0.25 0 -0.25];
Y = [0.25 0.75 0.25];
fill(X,Y,1)
grid on
grid minor
x0=10;
y0=10;
width=1000;
height=400;
set(gcf,'position',[x0,y0,width,height])

title('Workspace')
xlabel('x-position')
ylabel('y-position')

xlim([-0.5 2])
ylim([-0.5 2])

% Plot cspace
mat = csvread('cspace_map1.csv');
subplot(1,2,2)
%figure
imax = imshow(double(~mat),'InitialMagnification','fit');

xticklabels({'0','$\frac{\pi}{4}$','$\frac{\pi}{2}$','$\frac{3\pi}{4}$',...
       '$\pi$','$\frac{5\pi}{4}$','$\frac{3\pi}{2}$','$\frac{7\pi}{4}$','$2\pi$'})
yorder = {'0','$\frac{\pi}{4}$','$\frac{\pi}{2}$','$\frac{3\pi}{4}$',...
       '$\pi$','$\frac{5\pi}{4}$','$\frac{3\pi}{2}$','$\frac{7\pi}{4}$','$2\pi$'};
yorder = flip(yorder);
yticklabels(yorder)
set(imax.Parent,'TickLabelInterpreter','latex');
set(imax.Parent, 'XTick', linspace(0,100,9))
set(imax.Parent, 'YTick', linspace(0,100,9))
%set(imax.Parent, 'XTickLabel', linspace(0,2*pi,8))
title('C-space')
xlabel('\theta_1')
ylabel('\theta_2')
axis on;
set(groot,'defaultTextInterpreter','tex')


%% Case (b)
% Plot 2-link robot
a1 = 1;
a2 = 1;
figure
theta1 = 0;
theta2 = 0;
xs = [0 a1*cos(theta1) (a1*cos(theta1)+ a2*cos(theta1+theta2))];
ys = [0 a1*sin(theta1) (a1*sin(theta1)+ a2*sin(theta1+theta2))];
subplot(1,2,1)
plot(xs,ys,'Linewidth',8)
hold on
scatter(xs,ys,'or','Linewidth',8)

% Plot obstacle
X = [-0.25 -2; -0.25 -2; 0.25 2; 0.25 2];
Y = [1.1 -2; 2 -1.8; 2 -1.8; 1.1 -2];
fill(X,Y,1)
grid on
grid minor
set(gcf,'position',[x0,y0,width,height])
title('Workspace')
xlabel('x-position')
ylabel('y-position')

% Plot cspace
mat = csvread('cspace_map2.csv');
subplot(1,2,2)
%figure
imax = imshow(double(~mat),'InitialMagnification','fit');
xticklabels({'0','$\frac{\pi}{4}$','$\frac{\pi}{2}$','$\frac{3\pi}{4}$',...
       '$\pi$','$\frac{5\pi}{4}$','$\frac{3\pi}{2}$','$\frac{7\pi}{4}$','$2\pi$'})
yorder = {'0','$\frac{\pi}{4}$','$\frac{\pi}{2}$','$\frac{3\pi}{4}$',...
       '$\pi$','$\frac{5\pi}{4}$','$\frac{3\pi}{2}$','$\frac{7\pi}{4}$','$2\pi$'};
yorder = flip(yorder);
yticklabels(yorder)
set(imax.Parent,'TickLabelInterpreter','latex');
set(imax.Parent, 'XTick', linspace(0,100,9))
set(imax.Parent, 'YTick', linspace(0,100,9))
axis on;

title('C-space')
xlabel('\theta_1')
ylabel('\theta_2')

set(groot,'defaultTextInterpreter','tex')

%% Case (c)
% Plot 2-link robot
a1 = 1;
a2 = 1;
figure
theta1 = 0;
theta2 = 0;
xs = [0 a1*cos(theta1) (a1*cos(theta1)+ a2*cos(theta1+theta2))];
ys = [0 a1*sin(theta1) (a1*sin(theta1)+ a2*sin(theta1+theta2))];
subplot(1,2,1)
plot(xs,ys,'Linewidth',8)
hold on
scatter(xs,ys,'or','Linewidth',8)

% Plot obstacle
X = [-0.25 -2; -0.25 -2; 0.25 2; 0.25 2];
Y = [1.1 -0.5; 2 -0.3; 2 -0.3; 1.1 -0.5];
fill(X,Y,1)
grid on
grid minor
set(gcf,'position',[x0,y0,width,height])
title('Workspace')
xlabel('x-position')
ylabel('y-position')

%xlim([-0.5 2])
%ylim([-0.5 2])

% Plot cspace
mat = csvread('cspace_map3.csv');
subplot(1,2,2)
%figure
imax = imshow(double(~mat),'InitialMagnification','fit');
xticklabels({'0','$\frac{\pi}{4}$','$\frac{\pi}{2}$','$\frac{3\pi}{4}$',...
       '$\pi$','$\frac{5\pi}{4}$','$\frac{3\pi}{2}$','$\frac{7\pi}{4}$','$2\pi$'})
yorder = {'0','$\frac{\pi}{4}$','$\frac{\pi}{2}$','$\frac{3\pi}{4}$',...
       '$\pi$','$\frac{5\pi}{4}$','$\frac{3\pi}{2}$','$\frac{7\pi}{4}$','$2\pi$'};
yorder = flip(yorder);
yticklabels(yorder)
set(imax.Parent,'TickLabelInterpreter','latex');
set(imax.Parent, 'XTick', linspace(0,100,9))
set(imax.Parent, 'YTick', linspace(0,100,9))
axis on;

title('C-space')
xlabel('\theta_1')
ylabel('\theta_2')