% Housekeeping
clear all
close all
clc

% Define obstacle vertices
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
% Plot Vector Field _________________________________________________
figure
%%subplot(2,1,1)
field_info = csvread('field_2b2.csv');
meta_data = num2cell(field_info(1,1:5));
[top,bottom,left,buffer,scale_factor] = deal(meta_data{:}); 
mat_field = field_info(2:end,1:end-1);
[h,w] = size(mat_field);
% for i=1:h
%    for j=1:w/2
%        quiver((left-buffer+j-1)/scale_factor,(top+buffer-i)/scale_factor,mat_field(i,2*(j-1)+1),mat_field(i,2*(j-1)+2),'b')
%        hold on
%    end
% end
% Plot obstacles
% for i=1:2:length(W2)
%    plot([W2(:,i);W2(1,i)],[W2(:,i+1);W2(1,i+1)],'r');
%    hold on
% end
% %xlim([0 15]);
% %ylim([0 15]);
% xlabel('q_1')
% ylabel('q_2')
% title('Vector Field')

%% i)
fprintf("I was unable to get the robot to move up from the start position\n")
fprintf("Since the robot was surrounded evenly by obstacles from the top and\n")
fprintf("bottom there was no force component in the up direction thus a local\n")
fprintf("minimum was encountered\n")
%% ii) Path
mat = csvread('path_2b2.csv');
%subplot(2,1,2)
start = [0,0];
goal = [35,0];
h = zeros(3,1);
h(1) = plot(start(1),start(2),'om','Linewidth',2);
hold on
h(3) = plot(goal(1),goal(2),'xk','Linewidth',2);
h(2) = plot((mat(:,1)-abs(left)-buffer)/scale_factor,(mat(:,2)-abs(bottom)-buffer)/scale_factor,'b','Linewidth',2);
% Plot obstacles
for i=1:2:length(W2)
   plot([W2(:,i);W2(1,i)],[W2(:,i+1);W2(1,i+1)],'r');
   hold on
end
legend(h,'start','path','goal')
grid on
grid minor
xlabel('q_1')
ylabel('q_2')
title('Attractive-Repulsive Gradient Descent Path')
x0=100;
y0=50;
width=900;
height=400;
set(gcf,'position',[x0,y0,width,height])

%% iii)
len_path=0;
for i=1:length(mat)-1
    len_path = len_path + sqrt((mat(i+1,1)-mat(i,1))^2 + (mat(i+1,2)-mat(i,2))^2);
end
fprintf("The lenght of path 2 is: %f\n",len_path/scale_factor)

%% iv) 
fprintf("No, I would expect different path lengths since the Qstar \n")
fprintf("influences how close a robot can come to a certain obstacle\n")
fprintf("and since the obstacles are near the robots path to goal\n")
fprintf("the robot would surely be swayed if Qstar is decreased\n")