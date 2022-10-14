%% house keeping
clear all
close all
clc

a1 = 8;
a2 = 8;
a3 = 8;
a4 = 8;

theta1 = pi/4;
theta2 = pi/2;
theta3 = -pi/6;

T1 = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1];

T2 = [cos(theta2) -sin(theta2) a1; sin(theta2) cos(theta2) 0; 0 0 1];

T3 = [cos(theta3) -sin(theta3) a2; sin(theta3) cos(theta3) 0; 0 0 1];

T4 = [1 0 a3; 0 1 0; 0 0 1];

pos = T1*T2*T3*T4*[0 0 1]'

T_last = [1 0 4; 0 1 0; 0 0 1];
apos = T1*T_last*[0 0 1]'

T_last = [1 0 9; 0 1 0; 0 0 1];
bpos = T1*T2*T_last*[0 0 1]'

T_last = [1 0 9; 0 1 1; 0 0 1];
cpos = T1*T2*T3*T_last*[0 0 1]'
