%% Housekeeping
clear all
close all
clc

%% Part b)
path = csvread('data/rrt3b1_path.csv');
valid = csvread('data/rrt3b1_valid.csv');
comp = csvread('data/rrt3b1_comp.csv');
comp = 1E-3*comp;
figure
boxplot(path)
title('Path Lengths')
xlabel('Runs and Radius')
ylabel('Path Length')
figure

boxplot(valid)
title('Validity of Solutions')
xlabel('Runs and Radius')
ylabel('Validity of Solution (1-valid, 0-invalid)')
figure

boxplot(comp)
title('Computation Time')
xlabel('Runs and Radius')
ylabel('milliseconds')

%x0=100;
%y0=50;
%width=900;
%height=350;
%set(gcf,'position',[x0,y0,width,height])

