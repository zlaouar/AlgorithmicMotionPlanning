%% Housekeeping
clear all
close all
clc

%% Problem 5a)
% Load path from csv
mat = csvread('c_obstacle.csv');

% Plot cobstacle in cspace
plot([mat(:,1);mat(1,1)],[mat(:,2);mat(1,2)],'Linewidth',3)
grid on
grid minor
xlim([-5 5])
ylim([-5 5])
xlabel('x-translation')
ylabel('y-translation')
title('C-space obstacle generated by Minkowski Difference')

fprintf("The vertices of the cspace obstacle are: \n");
fprintf("(%f.ff,%f,ff),(%f.ff,%f,ff),(%f.ff,%f,ff)", mat(1,1),mat(1,2),mat(2,1),mat(2,2),mat(3,1),mat(3,2));

%% Problem 5b)
figure
mat1 = csvread('c_obstacle_rot.csv');
size_obs = (length(mat1)-1)/mat1(end,1);
num_angles = mat1(end,1);
mat1 = mat1(1:end-1,:);

% Plot cobstacle in cspace
X = reshape(mat1(:,1),size_obs,length(mat1)/size_obs);
Y = reshape(mat1(:,2),size_obs,length(mat1)/size_obs);
Z = repmat(linspace(1,360,length(mat1)/size_obs),[size_obs,1]);
fill3(X,Y,Z,1)

grid on
grid minor
xlim([-5 5])
ylim([-5 5])
xlabel('x-translation')
ylabel('y-translation')
zlabel('rotation-\theta (degrees)')
title('C-space obstacle generated by Minkowski Difference')