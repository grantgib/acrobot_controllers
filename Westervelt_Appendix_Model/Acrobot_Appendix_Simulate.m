%% Simulate Acrobot from Appendix (pg. 436)
clear; clc; close all;

params = load('loadDynamics_Appendix');
params.PD = 1;
params.Kp = 12;
params.Kd = 2.5;
params.q_des = [pi/4; pi/2];
params.u_sat = 5;

x_init = [pi/2; -pi/2; 0; 0];  % [q; q_dot]
tspan = 0:0.05:100;

[t,x] = ode45(@(t,x) acrodyn(t,x,params),tspan,x_init);
x = x';
u = calcInput(x,params);  % Calculate Control Inputs that were used

%% Plots
set(0,'DefaultFigureWindowStyle','docked')
x_header = {'$q_1$','$q_2$','$\dot{q}_1$','$\dot{q}_1$'};
figure
for i = 1:size(x,1)
    subplot(2,2,i)
    plot(t,x(i,:));
    if i <= 2
        hold on; yline(params.q_des(i),":r");
        legend('actual','desired','location','best');
    end
    grid on;
    title(x_header{i},'interpreter','latex')
    xlabel('time');
    
end
sgtitle('States')

u_header = {'$u_1$','$u_2$'};
figure
for i = 1:size(u,1)
    subplot(1,2,i)
    stairs(t,u(i,:));
    grid on;
    title(u_header{i},'interpreter','latex');
    xlabel('time');
end
sgtitle('Motor Torques');

%% Animations

modelParams();
blue = [0 0.4470 0.7410];
orange = [0.8500 0.3250 0.0980];
green = [0.4660 0.6740 0.1880];
wd = 10;
sz = 40;
p_pivot = [0; 0];       % origin of pinned acrobot link

%
x_i = x_init;
q1_i = x_i(1);
q2_i = x_i(2);

p2_start = p_pivot;
p2_end = [L2*cos(q2_i); L2*sin(q2_i)];
p1_start = p2_end;
p1_end = p1_start + [L1*cos(q1_i+q2_i); L1*sin(q1_i+q2_i)];

pcm2 = p_pivot + [Lcm2*cos(q2_i); Lcm2*sin(q2_i)];
pcm1 = p_pivot + [L2*cos(q2_i) + Lcm1*cos(q1_i+q2_i); L2*sin(q2_i) + Lcm1*sin(q1_i+q2_i)];

figure
grid on;
axis equal
xlim([-2 2]);
ylim([-2 2]);
link1 = line([p1_start(1) p1_end(1)],[p1_start(2) p1_end(2)],'color',blue,'LineWidth',wd);
hold on;
link2 = line([p2_start(1) p2_end(1)],[p2_start(2) p2_end(2)],'color',blue,'LineWidth',wd);
hold on;
com1 = scatter(pcm1(1),pcm1(2),sz,green,'filled');
hold on;
com2 = scatter(pcm2(1),pcm2(2),sz,green,'filled');
hold on;
joint1 = scatter(p1_start(1),p1_start(2),5*sz,orange,'filled');
hold on;
joint2 = scatter(p2_start(1),p2_start(2),5*sz,orange,'filled');
hold on;

disp("Press Key to Begin Animation...");
pause
for i = 1:size(x,2)
    x_i = x(:,i);
    q1_i = x_i(1);
    q2_i = x_i(2);
    
    p2_start = p_pivot;
    p2_end = [L2*cos(q2_i); L2*sin(q2_i)];
    p1_start = p2_end;
    p1_end = p1_start + [L1*cos(q1_i+q2_i); L1*sin(q1_i+q2_i)];
    
    pcm2 = p_pivot + [Lcm2*cos(q2_i); Lcm2*sin(q2_i)];
    pcm1 = p_pivot + [L2*cos(q2_i) + Lcm1*cos(q1_i+q2_i); L2*sin(q2_i) + Lcm1*sin(q1_i+q2_i)];
    
    set(link1,'XData',[p1_start(1) p1_end(1)],'YData',[p1_start(2) p1_end(2)]);
    set(link2,'XData',[p2_start(1) p2_end(1)],'YData',[p2_start(2) p2_end(2)]);
    set(com1,'XData',pcm1(1),'YData',pcm1(2));
    set(com2,'XData',pcm2(1),'YData',pcm2(2));
    set(joint1,'XData',p1_start(1),'YData',p1_start(2));  % not changing
    set(joint2,'XData',p2_start(1),'YData',p2_start(2));
    drawnow;
    pause(0.01);
end
disp("Animation Complete");

%% ODE Function
function dxdt = acrodyn(t,x,p)
% Extract Inputs
D_func = p.D_func;
C_func = p.C_func;
G_func = p.G_func;
q = x(1:2);
q_dot = x(3:end);

% Calculate Matrices
D = D_func(q(1),q(2));
C = C_func(q(1),q(2),q_dot(1),q_dot(2));
G = G_func(q(1),q(2))';
B = eye(2);             % The motors act directly at the joints independently

if p.PD
    u = calcInput(x,p);
else
    u = [0; 0];
end

dxdt = [q_dot;
    D \ (-C*q_dot - G + B*u)];

end

function u = calcInput(x,p)
u = zeros(2,size(x,2));
Kp = p.Kp;
Kd = p.Kd;
q_desired = p.q_des;
u_max = p.u_sat*ones(2,1);
u_min = -u_max;
for i = 1:size(x,2)
    q = x(1:2,i);
    q_dot = x(3:end,i);
    q_error = q - q_desired;
    u_temp = -Kp*q_error - Kd*q_dot;
    u(:,i) = max(min(u_temp,u_max),u_min);
end
end








