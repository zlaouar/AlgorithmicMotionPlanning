%% Housekeeping 
close all
clear all
clc

set(groot,'defaulttextInterpreter','latex');

g = 9.81;
m = 0.65;
Ix = 7.5E-3;
Iy = 7.5E-3;
Iz = 1.3E-2;

% Plant
%{
A =[0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 -g 0 0 0 0;
    0 0 0 0 0 0 g 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;];

B = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     1/m 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 1/Ix 0 0;
     0 0 1/Iy 0;
     0 0 0 1/Iz;];

C = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0];
 
D = [0 0 0 0; 
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];
%}
A =[0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    1 0 0 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0];

B = [0 0 0 0;
     0 0 0 0;
     -1/m 0 0 0;
     0 1/Ix 0 0;
     0 0 1/Iy 0;
     0 0 0 1/Iz;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];

C = [0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1];
 
D = [0 0 0 0; 
     0 0 0 0;
     0 0 0 0];
%% Problem 1

% Plant Modes
[eigvecs,eigvals] = eig(A);
evals = diag(eigvals);
[distinct_evals,ia,~] = unique(evals,'stable');
rank(eigvecs);

sys = ss(A,B,C,D);
t = 0:0.1:70;  % 201 points
u = zeros(length(t),4);%max(0,min(t-1,1));
Ylabels = ["X-Position","Y-Position","Z-Position"];

% Eigenspaces and Modal Spaces (No complex evals -> modal same as eigenspace)
N1 = null(A-evals(1)*eye(length(A)));
N2 = null(A-evals(2)*eye(length(A)));
N3 = null(A-evals(6)*eye(length(A)));


% Simulate system response to nonzero initial condition in each modal space
eigvecs_practical = eigvecs;
eigvecs_practical(:,end) = eigvecs_practical(:,end)*-1;
for i=1:length(distinct_evals)
    figure
    x0 = eigvecs_practical(:,ia(i))';
    [y,t] = lsim(sys,u,t,x0);
    for j = 1:length(Ylabels)
        subplot(length(Ylabels),1,j)
        plot(t,y(:,j),'LineWidth',1.5);
        if j==1 
            disp("hello")
            title("Plant Mode for $\lambda_"+i+ "$ = " + distinct_evals(i),'FontSize',16)
        end
        ylabel(Ylabels(j),'FontSize',12)
        grid minor
    end
    xlabel("Time(s)",'FontSize',12)
    
    %saveas(gcf,"plots/prob1_modes_" + i + ".png")
end
%}
% The plant is stable i.s.l. but not a.s. since some evals = 0


%% Problem 2

% Reachable Subspace
[num_states,num_inputs] = size(B);
%P = zeros(num_states,num_states*num_inputs);
P = [];
for i=0:num_states-1
   P = [P (A^i)*B];
end

rankP = rank(P); % rank = n -> completely reachable
[~,colP] = size(P);

% Orthonormal Basis
%[orthBasis,~] = qr(P);
orthBasis = eye(12);

% Minimum Energy

t0 = 0;
t1 = 30;
syms tau
fun = ((expm(A*(t0-tau))*B)*(B'*expm(A'*(t0-tau))));
G = double(int(fun,tau,t0,t1));

u1 = -orthBasis(:,1)'*inv(G)*-orthBasis(:,1);
u2 = -orthBasis(:,2)'*inv(G)*-orthBasis(:,2);
u3 = -orthBasis(:,3)'*inv(G)*-orthBasis(:,3);
u4 = -orthBasis(:,4)'*inv(G)*-orthBasis(:,4);
u5 = -orthBasis(:,5)'*inv(G)*-orthBasis(:,5);
u6 = -orthBasis(:,6)'*inv(G)*-orthBasis(:,6);

uvec = [u1 u2 u3 u4 u5 u6];

%{
% Control Signal
ind_t1 = find(t==t1);
uvec_signal_ol = zeros(ind_t1,num_states);
ut = zeros(ind_t1,1);
u_temp3d = zeros(4,ind_t1,num_states);
for j = 1:num_states
    for i=1:ind_t1
       u = B'*expm(A'*(t0-t(i)))*(inv(G)*-orthBasis(:,j));
       u_temp3d(:,i,j) = u;
       ut(i) = u'*u;
    end
    uvec_signal_ol(:,j)= ut;
end
t = 0:0.1:t1;  % 201 points
u = zeros(length(t),4);
for i=1:num_states
    figure
    x0 = orthBasis(:,i)';
    [y,t] = lsim(sys,u_temp3d(:,:,i),t,x0);
    for j = 1:length(Ylabels)
        subplot(length(Ylabels),1,j)
        plot(t,y(:,j),'LineWidth',1.5);
        ylabel(Ylabels(j),'FontSize',12)
        grid minor
    end
    xlabel("Time(s)",'FontSize',12)
    
    %saveas(gcf,"plots/prob1_modes_" + i + ".png")
end
%}
%% Problem 3
% new eigvals
p = [-1.21 -1.21 -1.21 -1.51 -1.51 -1.51 -2 -2 -2 -4 -4 -4];
%% Problem 4

%
K = place(A,B,p);

% Plant Modes/Eigenvectors
[eigvecs_closed,eigvals_closed] = eig(A-B*K);
evals_closed = diag(eigvals_closed);
[distinct_evals_closed,ia_closed,~] = unique(evals_closed,'stable');
rankT_closed = rank(eigvecs_closed);

sys_closed = ss(A-B*K,B,C,D);
t = 0:0.1:70;  % 201 points
u = zeros(length(t),4);%max(0,min(t-1,1));
Ylabels = ["X-Position","Y-Position","Z-Position"];

% Closed loop modal spaces (No complex evals -> modal same as eigenspace)
N1_closed = null((A-B*K)-evals_closed(1)*eye(length(A)));
N2_closed = null((A-B*K)-evals_closed(2)*eye(length(A)));
N3_closed = null((A-B*K)-evals_closed(3)*eye(length(A)));
N4_closed = null((A-B*K)-evals_closed(4)*eye(length(A)));
N5_closed = null((A-B*K)-evals_closed(5)*eye(length(A)));
N6_closed = null((A-B*K)-evals_closed(6)*eye(length(A)));

% Reachable Subspace
%P = zeros(num_states,num_states*num_inputs);
P_closed = zeros(num_states,num_states*num_inputs);
for i=0:num_states-1
   P_closed(:,(i*num_inputs)+1:(i+1)*num_inputs) = ((A-B*K)^i)*B;
end

rankP_closed = rank(P_closed); % rank = n -> completely reachable
[~,colP_closed] = size(P_closed);

% Orthonormal Basis
%[orthBasis_closed,~] = qr(P_closed);
orthBasis_closed = eye(6);

dirs = ["X-position","X-velocity","Y-position","Y-velocity","Z-position","Z-velocity"];
uvec_closed = zeros(1,num_states);
uvec_signal = zeros(ind_t1,num_states);
% Simulate system response to nonzero initial condition in each modal space
for i=1:length(distinct_evals_closed)
    figure
    x0 = orthBasis_closed(:,ia_closed(i))';
    [y,t,state_vec] = lsim(sys_closed,u,t,x0);
    u_signal = -K*state_vec(1:ind_t1,:)';
    u_signal = u_signal'*u_signal;
    u_signal = u_signal(:,1);
    uvec_signal(:,i) = u_signal;
    uvec_norm = trapz(t(1:ind_t1),u_signal);
    uvec_closed(i) = uvec_norm;
    for j = 1:length(Ylabels)
        subplot(length(Ylabels),1,j)
        plot(t,y(:,j),'LineWidth',1.5);        
        ylabel(Ylabels(j),'FontSize',12)
        ylim([-0.1 1])
        grid minor
    end
    sgtitle("System Response for Unit "+dirs(i),'FontSize',16)
    xlabel("Time(s)",'FontSize',12)
    %saveas(gcf,"plots/prob4_closed_response_" + i + ".png")
end
%}

% Control Signals: open-loop vs closed-loop
figure
x0p=200;
y0p=0;
widthp=1100;
heightp=1100;
set(gcf,'position',[x0p,y0p,widthp,heightp])
for i = 1:num_states
    subplot(num_states,2,2*i-1)
    plot(t(1:ind_t1),uvec_signal_ol(:,i),'LineWidth',1.5);
    ylabel("$u^{T}(t)u(t)$",'FontSize',12)
    xlabel("Time(s)",'FontSize',12)
    grid minor
    subplot(num_states,2,2*i)
    plot(t(1:ind_t1),uvec_signal(:,i),'LineWidth',1.5);
    ylabel("$u^{T}(t)u(t)$",'FontSize',12)
    xlabel("Time(s)",'FontSize',12)
    grid minor
end
sgtitle("Control Signals: Open-loop vs Closed-loop",'FontSize',16)
%saveas(gcf,"plots/prob4_control_signals.png")


%% Problem 5
F = pinv(C*inv(B*K-A)*B);
save('F_and_K.mat','F','K');

sys_closed = ss(A-B*K,B*F,C,D*F);
t = 0:0.1:70;  % 201 points
tau_min = 1/min(abs(evals_closed));
tau_min = t(find(t>tau_min,1));

% Simulate system response to nonzero initial condition in each modal space
x0 = [0 0 0 0 0 0]';
dirs = ["X","Y","Z"];
for i = 1:(length(dirs))
    figure
    x0p=100;
    y0p=50;
    widthp=600;
    heightp=600;
    set(gcf,'position',[x0p,y0p,widthp,heightp])
    u = zeros(length(t),3);
    u(:,i) = ones(length(t),1);
    [y,t,state_vec] = lsim(sys_closed,u,t,x0);
    u_signal = F*u'-K*state_vec';
    u_signal = u_signal'*u_signal;
    u_signal = u_signal(:,1);
    uvec_signal1(:,i) = u_signal;
    uvec_norm = trapz(t(1:find(t>tau_min,1)),u_signal(1:find(t>tau_min,1)));
    uvec_closed1(i) = uvec_norm;
    for j = 1:length(Ylabels)
        subplot(length(Ylabels),1,j)
        plot(t',ones(length(t),1)','--r','LineWidth',1.5);
        hold on
        plot(t,y(:,j),'b');
        if j==1
            title("System Response for a "+dirs(j)+ "-Direction unit step input",'FontSize',16)
        end
        ylabel(Ylabels(j),'FontSize',12)
        ylim([0 1.2])
        grid minor
    end
    xlabel("Time(s)",'FontSize',12)
    %saveas(gcf,"plots/prob5_unit_response_" + i + ".png")

end

save('usig.mat','uvec_signal1');
%}

