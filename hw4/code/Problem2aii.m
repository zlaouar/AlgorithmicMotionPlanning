%% Housekeeping
clear all
close all
clc

%% Part a)
path = csvread('data/prm2aii_path.csv');
valid = csvread('data/prm2aii_valid.csv');
comp = csvread('data/prm2aii_comp.csv');
path = path(:,1:end-1);
valid = valid(:,1:end-1);
comp = 1E-3*comp(:,1:end-1);
figure
group = ["(200, 0.5)","(200, 1)","(200, 1.5)","(200, 2)","(500, 0.5)","(500, 1)","(500, 1.5)","(500, 2)"];
boxplot(path',group)
title('Path Lengths')
xlabel('Runs and Radius')
ylabel('Path Length')
figure

boxplot(valid',group)
title('Validity of Solutions')
xlabel('Runs and Radius')
ylabel('Validity of Solution (1-valid, 0-invalid)')
figure

boxplot(comp',group)
title('Computation Time')
xlabel('Runs and Radius')
ylabel('milliseconds')

%x0=100;
%y0=50;
%width=900;
%height=350;
%set(gcf,'position',[x0,y0,width,height])

