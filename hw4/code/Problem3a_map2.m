%% Housekeeping
clear all
close all
clc

obstacle1 = {[1,1],[2,1],[2,5],[1,5]};
obstacle2 = {[3,4],[4,4],[4,12],[3,12]};
obstacle3 = {[3,12],[12,12],[12,13],[3,13]};
obstacle4 = {[12,5],[13,5],[13,13],[12,13]};
obstacle5 = {[6,5],[12,5],[12,6],[6,6]};
obstacles = {obstacle1,obstacle2,obstacle3,obstacle4,obstacle5};

%% Part a)
start = [0,0];
goal = [10,10];
verts = csvread('data/rrt3a2_verts.csv');
edges = csvread('data/rrt3a2_edges.csv');
path = csvread('data/rrt3a2_path.csv');
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