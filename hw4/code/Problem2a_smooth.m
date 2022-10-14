%% Housekeeping
clear all
close all
clc

obstacle1 = {[3,0],[5,0],[3,2],[5,2]};
obstacle2 = {[6,-2],[8,-2],[6,0],[8,0]};
obstacles = {obstacle1,obstacle2};

%% Part a)
verts = csvread('data/prm2a_verts.csv');
edges = csvread('data/prm2a_edges.csv');
path = csvread('data/prm2a_path.csv');
smooth_path = csvread('data/prm2a_smooth_path.csv');
path_length = path(1);
path = path(2:end);
smooth_path_length = smooth_path(1);
smooth_path = smooth_path(2:end);
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
scatter([verts(1,1),verts(end,1)],[verts(1,2),verts(end,2)],'ob','LineWidth',2) % Start and Goal
a = [1:length(verts)-2]'; b = num2str(a); c = cellstr(b);
dx = 0.1; dy = 0.1; % displacement so the text does not overlay the data points
text(x+dx, y+dy, c);
% Plot PRM edges
for i=1:length(edges)
   plot([verts(edges(i,1)+1,1),verts(edges(i,2)+1,1)],[verts(edges(i,1)+1,2),verts(edges(i,2)+1,2)],'b') 
end

% Plot path
plot(verts(path+1,1),verts(path+1,2),'m','LineWidth',2);

% Plot smooth Path
plot(verts(smooth_path+1,1),verts(smooth_path+1,2),'c','LineWidth',2);

grid on
grid minor
xlabel('q_1')
ylabel('q_2')
title("PRM - Path Length: " + path_length + " | Smooth Path Length: " + smooth_path_length)