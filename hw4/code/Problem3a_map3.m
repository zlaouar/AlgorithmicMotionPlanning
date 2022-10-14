%% Housekeeping
clear all
close all
clc

obstacle1 = {[-6,-6],[25,-6],[25,-5],[-6,-5]};
obstacle2 = {[-6,5],[30,5],[30,6],[-6,6]};
obstacle3 = {[-6,-5],[-5,-5],[-5,5],[-6,5]};
obstacle4 = {[4,-5],[5,-5],[5,1],[4,1]};
obstacle5 = {[9,0],[10,0],[10,5],[9,5]};
obstacle6 = {[14,-5],[15,-5],[15,1],[14,1]};
obstacle7 = {[19,0],[20,0],[20,5],[19,5]};
obstacle8 = {[24,-5],[25,-5],[25,1],[24,1]};
obstacle9 = {[29,0],[30,0],[30,5],[29,5]};
obstacles = {obstacle1,obstacle2,obstacle3,obstacle4,obstacle5,obstacle6,obstacle7,obstacle8,obstacle9};

%% Part a)
start = [0,0];
goal = [35,0];
verts = csvread('data/rrt3a3_verts.csv');
edges = csvread('data/rrt3a3_edges.csv');
path = csvread('data/rrt3a3_path.csv');
path_length = path(1);
path = path(2:end);
figure
x0=100;
y0=50;
width=900;
height=350;
set(gcf,'position',[x0,y0,width,height])
for obstacle = obstacles
   rectangle('Position',[obstacle{1}{1} abs(obstacle{1}{1}(1)-obstacle{1}{2}(1)) abs(obstacle{1}{2}(2)-obstacle{1}{3}(2))],'FaceColor',[0 .5 .5]);
end
hold on
% Plot PRM verts
x = verts(2:end-1,1);
y = verts(2:end-1,2);
scatter(x,y,'or')
scatter([start(1),goal(1)],[start(2),goal(2)],'ob','LineWidth',2) % Start and Goal
a = [1:length(verts)-2]'; b = num2str(a); c = cellstr(b);
dx = 0.1; dy = 0.1; % displacement so the text does not overlay the data points
%text(x+dx, y+dy, c);
% Plot PRM edges
for i=1:length(edges)
   plot([verts(edges(i,1)+1,1),verts(edges(i,2)+1,1)],[verts(edges(i,1)+1,2),verts(edges(i,2)+1,2)],'b') 
end

% Plot path
plot(verts(path+1,1),verts(path+1,2),'m','LineWidth',2);
grid on
grid minor
xlabel('q_1')
ylabel('q_2')
title("RRT - Path Length: " + path_length)