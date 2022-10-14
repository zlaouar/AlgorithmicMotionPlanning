%% Housekeeping
clear all
close all
clc

obstacle1 = {[3,0],[5,0],[3,2],[5,2]};
obstacle2 = {[6,-2],[8,-2],[6,0],[8,0]};
obstacles = {obstacle1,obstacle2};
%% i) Plot Vector Field _________________________________________________
figure
subplot(2,1,1)
x0=100;
y0=50;
width=800;
height=700;
set(gcf,'position',[x0,y0,width,height])
field_info = csvread('field_2a.csv');
meta_data = num2cell(field_info(1,1:5));
[top,bottom,left,buffer,scale_factor] = deal(meta_data{:});
mat_field = field_info(2:end,1:end-1);
[h,w] = size(mat_field);
for i=1:h
   for j=1:w/2
       quiver((left-buffer+j-1)/scale_factor,(top+buffer-i)/scale_factor,mat_field(i,2*(j-1)+1),mat_field(i,2*(j-1)+2),'b')
       hold on
   end
end
for obstacle = obstacles
   rectangle('Position',[obstacle{1}{1} abs(obstacle{1}{1}(1)-obstacle{1}{2}(1)) abs(obstacle{1}{2}(2)-obstacle{1}{3}(2))],'FaceColor',[0 .5 .5]);
end
grid on
grid minor
xlabel('q_1')
ylabel('q_2')
title('Vector Field')
%% ii)

fprintf("I started with a large dstar_goal and slowly decreased it \n")
fprintf("until I saw that the attractive forces were decreasing smoothly\n")
fprintf("as the robot reached the gap between the obstacles. The Qstar\n")
fprintf("for both obstacles were the same and the robot was able to move\n")
fprintf("straight towards the goal without deviating from the obstacles\n")
%% iii) Plot Path 

mat = csvread('path_2a.csv');

start = [0,0];
goal = [10,0];
subplot(2,1,2)
h = zeros(3,1);
h(1) = plot(start(1),start(2),'om','Linewidth',2);
hold on
h(3) = plot(goal(1),goal(2),'xk','Linewidth',2);
h(2) = plot((mat(:,1)-abs(left)-buffer)/scale_factor,(mat(:,2)-abs(bottom)-buffer)/scale_factor,'b','Linewidth',2);
for obstacle = obstacles
   rectangle('Position',[obstacle{1}{1} abs(obstacle{1}{1}(1)-obstacle{1}{2}(1)) abs(obstacle{1}{2}(2)-obstacle{1}{3}(2))]);
end
legend(h,'start','path','goal')
grid on
grid minor
%xlim([-5 5])
%ylim([-5 5])
xlabel('q_1')
ylabel('q_2')
title('Attractive-Repulsive Gradient Descent Path')

set(gcf,'position',[x0,y0,width,height])

%% iv)
len_path=0;
for i=1:length(mat)-1
    len_path = len_path + sqrt((mat(i+1,1)-mat(i,1))^2 + (mat(i+1,2)-mat(i,2))^2);
end
fprintf("The length of the path is: %f\n",len_path/scale_factor)

%% v)
fprintf("Yes, I would expect different path lengths since the Qstar \n")
fprintf("influences how close a robot can come to a certain obstacle\n")
fprintf("and since the obstacles are near the robots path to goal\n")
fprintf("the robot would surely be swayed if Qstar is decreased\n")
