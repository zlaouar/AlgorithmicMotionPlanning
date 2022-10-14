%% Housekeeping
clear all
close all
clc

%% Plot algebraic sets of birthday pacman
%{
syms t
r=3;
x = r*cos(t);
y = r*sin(t);
fplot(x,y)
grid on 
%}
syms x y
r=3;
fimplicit(x^2 + y^2 -r^2== 0)
hold on

%% eye
r1 = 0.5;
x2 = 1;
y2 = 1.5;
fimplicit((x-x2)^2 + (y-y2)^2 -r1^2== 0)
grid on
grid minor

%% mouth
fimplicit(x/3 - y) % >=0
fimplicit(-x/3 - y) % <=0

%% hat
% lower part of hat
x3=r*cos(3*pi/4);
y3=r*sin(3*pi/4);
m_rad = (y3-0)/(x3-0);
m_tan = -1/m_rad;

fimplicit(m_tan*(x-x3)+y3-y) % >=0

% upper part of hat
x4=(r+1)*cos(3*pi/4);
y4=(r+1)*sin(3*pi/4);
m_rad1 = (y4-0)/(x4-0);
m_tan1 = -1/m_rad1;

fimplicit(m_tan1*(x-x4)+y4-y) % >=0

% right part of hat
x5=(r+0.5)*cos(pi/2);
y5=(r+0.5)*sin(pi/2);
m_rad2 = (y5-0)/(x5-0);
m_tan2 = -1/m_rad2;

fimplicit(m_tan2*(x-x5)+y5-y) % >=0

% right part of hat
x6=(r+0.5)*cos(pi);
y6=(r+0.5)*sin(pi);
m_rad3 = (y6-0)/(x6-0);
m_tan3 = -1/m_rad3;

fimplicit(m_tan3*(x-x6)+y6-y) % >=0

% right pointy part
fimplicit(-(x+3)/2 - (y-3.5)) % >=0
fimplicit(-2*(x+3.8) - (y-3.5),'Linewidth',2) % >=0

