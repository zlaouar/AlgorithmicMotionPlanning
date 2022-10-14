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

obstacle1 = {[-6,-6],[25,-6],[25,-5],[-6,-5]};
obstacle2 = {[-6,5],[30,5],[30,6],[-6,6]};
obstacle3 = {[-6,-5],[-5,-5],[-5,5],[-6,5]};
obstacle4 = {[4,-5],[5,-5],[5,1],[4,1]};
obstacle5 = {[9,0],[10,0],[10,5],[9,5]};
obstacle6 = {[14,-5],[15,-5],[15,1],[14,1]};
obstacle7 = {[19,0],[20,0],[20,5],[19,5]};
obstacle8 = {[24,-5],[25,-5],[25,1],[24,1]};
obstacle9 = {[29,0],[30,0],[30,5],[29,5]};
obstacles1 = {obstacle1,obstacle2,obstacle3,obstacle4,obstacle5,obstacle6,obstacle7,obstacle8,obstacle9};

%% Part b - W1)
verts = csvread('data/prm2b_verts.csv');
edges = csvread('data/prm2b_edges.csv');
path = csvread('data/prm2b_path.csv');
path_length = path(1);
path = path(2:end);
figure
x0=100;
y0=50;
width=900;
height=600;
set(gcf,'position',[x0,y0,width,height])
for obstacle = obstacles
   rectangle('Position',[obstacle{1}{1} abs(obstacle{1}{1}(1)-obstacle{1}{2}(1)) abs(obstacle{1}{2}(2)-obstacle{1}{3}(2))],'FaceColor',[0 .5 .5]);
end
hold on
% Plot PRM verts
x = verts(2:end-1,1);
y = verts(2:end-1,2);
scatter(x,y,'or')
a = [1:length(verts)-2]'; b = num2str(a); c = cellstr(b);
dx = 0.1; dy = 0.1; % displacement so the text does not overlay the data points
%text(x+dx, y+dy, c);
% Plot PRM edges
for i=1:length(edges)
   plot([verts(edges(i,1)+1,1),verts(edges(i,2)+1,1)],[verts(edges(i,1)+1,2),verts(edges(i,2)+1,2)],'b') 
end
scatter([verts(1,1),verts(end,1)],[verts(1,2),verts(end,2)],'og','LineWidth',4) % Start and Goal
% Plot path
plot(verts(path+1,1),verts(path+1,2),'m','LineWidth',2);
grid on
grid minor
xlabel('q_1')
ylabel('q_2')
title("PRM - Path Length: " + path_length)

%% Part b - W2)
verts1 = csvread('data/prm2b1_verts.csv');
edges1 = csvread('data/prm2b1_edges.csv');
path1 = csvread('data/prm2b1_path.csv');
path_length1 = path1(1);
path1 = path1(2:end);
figure
x0=100;
y0=50;
width=900;
height=350;
set(gcf,'position',[x0,y0,width,height])
for obstacle = obstacles1
   rectangle('Position',[obstacle{1}{1} abs(obstacle{1}{1}(1)-obstacle{1}{2}(1)) abs(obstacle{1}{2}(2)-obstacle{1}{3}(2))],'FaceColor',[0 .5 .5]);
end
hold on
% Plot PRM verts
x = verts1(2:end-1,1);
y = verts1(2:end-1,2);
scatter(x,y,'or')
a = [1:length(verts1)-2]'; b = num2str(a); c = cellstr(b);
dx = 0.1; dy = 0.1; % displacement so the text does not overlay the data points
%text(x+dx, y+dy, c);
% Plot PRM edges
for i=1:length(edges1)
   plot([verts1(edges1(i,1)+1,1),verts1(edges1(i,2)+1,1)],[verts1(edges1(i,1)+1,2),verts1(edges1(i,2)+1,2)],'b') 
end
scatter([verts1(1,1),verts1(end,1)],[verts1(1,2),verts1(end,2)],'og','LineWidth',4) % Start and Goal
% Plot path
plot(verts1(path1+1,1),verts1(path1+1,2),'m','LineWidth',2);
grid on
grid minor
xlabel('q_1')
ylabel('q_2')
title("PRM - Path Length: " + path_length1)