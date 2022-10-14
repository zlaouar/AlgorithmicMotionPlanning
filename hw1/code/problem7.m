%% Housekeeping
clear all
close all
clc

%% Variable Init
qstrt = [0,0];
qgoal1 = [10,10];
qgoal2 = [35,0];

% Define obstacle vertices
W1_WO1 = [1,1; 2,1; 2,5; 1,5];
W1_WO2 = [3,4; 4,4; 4,12; 3,12];
W1_WO3 = [3,12; 12,12; 12,13; 3,13];
W1_WO4 = [12,5; 13,5; 13,13; 12,13];
W1_WO5 = [6,5; 12,5; 12,6; 6,6];
W1 = [W1_WO1 W1_WO2 W1_WO3 W1_WO4 W1_WO5];

W2_WO1 = [-6,-6; 25,-6; 25,-5; -6,-5];
W2_WO2 = [-6,5; 30,5; 30,6; -6,6];
W2_WO3 = [-6,-5; -5,-5; -5,5; -6,5];
W2_WO4 = [4,-5; 5,-5; 5,1; 4,1];
W2_WO5 = [9,0; 10,0; 10,5; 9,5];
W2_WO6 = [14,-5; 15,-5; 15,1; 14,1];
W2_WO7 = [19,0; 20,0; 20,5; 19,5];
W2_WO8 = [24,-5; 25,-5; 25,1; 24,1];
W2_WO9 = [29,0; 30,0; 30,5; 29,5];
W2 = [W2_WO1 W2_WO2 W2_WO3 W2_WO4 W2_WO5 W2_WO6 W2_WO7 W2_WO8 W2_WO9];

%% Bug1 Map1
scale_bug1_map1 = 4;
h = zeros(4,1);
% Plot obstacles
for i=1:2:length(W1)
   h(1) = plot([W1(:,i);W1(1,i)],[W1(:,i+1);W1(1,i+1)],'b');
   hold on
end
% Plot start and goal points
h(2) = plot(qstrt(1),qstrt(2),'dk');
h(3) = plot(qgoal1(1),qgoal1(2),'og');
grid on
grid minor

% Load path from csv
mat = csvread('bug1_map1.csv');
mat = mat./scale_bug1_map1;
length_bug1_map1 = length(mat)/scale_bug1_map1;


h(4) = plot(mat(:,1),mat(:,2),'r');
% Plot repeated points with thicker marker
duplicate_value = mat;
repeated = 1.5;
while(1)
    % indices to unique values in column 3
    [~, ind] = unique(duplicate_value, 'rows');
    % duplicate indices
    duplicate_ind = setdiff(1:size(duplicate_value, 1), ind);
    if isempty(duplicate_ind)
       break; 
    end
    % duplicate values
    duplicate_value = duplicate_value(duplicate_ind, :);
    h(5) = plot(duplicate_value(:,1),duplicate_value(:,2),'r','Linewidth',repeated);
end

title('Bug 1 - Map 1');
xlabel('x-position');
ylabel('y-position');

legend(h,'obstacles','start','goal','robot path','Location','best')
%% Bug2 Map1
scale_bug2_map1 = 4;
figure
h1 = zeros(4,1);
% Plot obstacles
for i=1:2:length(W1)
   h1(1) = plot([W1(:,i);W1(1,i)],[W1(:,i+1);W1(1,i+1)],'b');
   hold on
end

% Plot start and goal points
h1(2) = plot(qstrt(1),qstrt(2),'dk');
h1(3) = plot(qgoal1(1),qgoal1(2),'og');
grid on
grid minor

% Load path from csv
mat1 = csvread('bug2_map1.csv');
mat1 = mat1./scale_bug2_map1;
length_bug2_map1 = length(mat1)/scale_bug2_map1;

h1(4) = plot(mat1(:,1),mat1(:,2),'r');
% Plot repeated points with thicker marker
duplicate_value = mat1;
repeated = 1.5;
while(1)
    % indices to unique values in column 3
    [~, ind] = unique(duplicate_value, 'rows');
    % duplicate indices
    duplicate_ind = setdiff(1:size(duplicate_value, 1), ind);
    if isempty(duplicate_ind)
       break; 
    end
    % duplicate values
    duplicate_value = duplicate_value(duplicate_ind, :);
    h1(5) = plot(duplicate_value(:,1),duplicate_value(:,2),'r','Linewidth',repeated);
end
title('Bug 2 - Map 1');
xlabel('x-position');
ylabel('y-position');

legend(h1,'obstacles','start','goal','robot path','Location','best')


%% Bug1 Map2
scale_bug1_map2 = 4;
figure
h2 = zeros(4,1);
% Plot obstacles
for i=1:2:length(W2)
   h2(1) = plot([W2(:,i);W2(1,i)],[W2(:,i+1);W2(1,i+1)],'b');
   hold on
end

% Plot start and goal points
h2(2) = plot(qstrt(1),qstrt(2),'dk');
h2(3) = plot(qgoal2(1),qgoal2(2),'og');

% Load in path from csv
mat2 = csvread('bug1_map2.csv');
mat2 = mat2./scale_bug1_map2;
length_bug1_map2 = length(mat2)/scale_bug1_map2;

h2(4) = plot(mat2(:,1),mat2(:,2),'r');
% Plot repeated points with thicker marker
duplicate_value = mat2;
repeated = 1.5;
while(1)
    % indices to unique values in column 3
    [~, ind] = unique(duplicate_value, 'rows');
    % duplicate indices
    duplicate_ind = setdiff(1:size(duplicate_value, 1), ind);
    if isempty(duplicate_ind)
       break; 
    end
    % duplicate values
    duplicate_value = duplicate_value(duplicate_ind, :);
    h2(5) = plot(duplicate_value(:,1),duplicate_value(:,2),'r','Linewidth',repeated);
end
%plot([qstrt(1) qgoal1(1)],[qstrt(2) qgoal1(2)])
grid on
grid minor

title('Bug 1 - Map 2');
xlabel('x-position');
ylabel('y-position');
legend(h2,'obstacles','start','goal','robot path','Location','best')
ylim([-12, 8])

%% Bug2 Map2
scale_bug2_map2 = 4;
figure
h3 = zeros(4,1);
% Plot obstacles
for i=1:2:length(W2)
   h3(1) = plot([W2(:,i);W2(1,i)],[W2(:,i+1);W2(1,i+1)],'b');
   hold on
end
% Plot start and goal points
h3(2) = plot(qstrt(1),qstrt(2),'dk');
h3(3) = plot(qgoal2(1),qgoal2(2),'og');

% Load in path from csv
mat3 = csvread('bug2_map2.csv');
mat3 = mat3./scale_bug2_map2;
length_bug2_map2 = length(mat3)/scale_bug2_map2;

h3(4) = plot(mat3(:,1),mat3(:,2),'r');

% Plot repeated points with thicker marker
duplicate_value = mat3;
repeated = 1.5;
while(1)
    % indices to unique values in column 3
    [~, ind] = unique(duplicate_value, 'rows');
    % duplicate indices
    duplicate_ind = setdiff(1:size(duplicate_value, 1), ind);
    if isempty(duplicate_ind)
       break; 
    end
    % duplicate values
    duplicate_value = duplicate_value(duplicate_ind, :);
    h3(5) = plot(duplicate_value(:,1),duplicate_value(:,2),'r','Linewidth',repeated);
    repeated = repeated + 0.5;
end
grid on
grid minor

title('Bug 2 - Map 2');
xlabel('x-position');
ylabel('y-position');
legend(h3,'obstacles','start','goal','robot path','Location','best')
ylim([-12, 8])

fprintf("The path length for bug1 map1 = %f\n",length_bug1_map1)
fprintf("The path length for bug2 map1 = %f\n",length_bug2_map1)
fprintf("The path length for bug1 map2 = %f\n",length_bug1_map2)
fprintf("The path length for bug2 map2 = %f\n",length_bug2_map2)

fprintf("For the Bug 1 algorithm, I would expect the path length for a \n")
fprintf("right turning robot to be the same as for a left turning robot\n")
fprintf("because the robot has to circumnavigate the entire obstacle\n")
fprintf("regardless and then motion to goal at the closest point. \n")
fprintf("The upper bound on the path length is D+(3/2)*sum(obstacle_perimeter).\n")
fprintf("However, I would expect different lengths for the Bug 2\n")
fprintf("algorithm since the upper bound of the path length depends on\n")
fprintf("the number of m-line intersections and many of those intersections\n")
fprintf("could be further from the goal as the previous intersection thus\n")
fprintf("increasing the path length more.\n")
