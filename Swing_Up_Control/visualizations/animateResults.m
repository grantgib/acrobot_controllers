function [] = animateResults(x,p)
%ANIMATERESULTS Summary of this function goes here
%   Detailed explanation goes here
L1 = p.L1;
L2 = p.L2;
Lcm1 = p.Lcm1;
Lcm2 = p.Lcm2;
x_init = x(:,1);

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

p1_start = p_pivot;
p1_end = [L1*cos(q1_i); L1*sin(q1_i)];
p2_start = p1_end;
p2_end = p2_start + [L2*cos(q1_i+q2_i); L2*sin(q1_i+q2_i)];

pcm1 = p_pivot + [Lcm1*cos(q1_i); Lcm1*sin(q1_i)];
pcm2 = p_pivot + [L1*cos(q1_i) + Lcm2*cos(q1_i+q2_i); L1*sin(q1_i) + Lcm2*sin(q1_i+q2_i)];

figure
grid on;
axis equal
xlim([-4 4]);
ylim([-4 4]);
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

% disp("Press Key to Begin Animation...");
% pause
for i = 1:size(x,2)
    x_i = x(:,i);
    q1_i = x_i(1);
    q2_i = x_i(2);
    
    p1_start = p_pivot;
    p1_end = p1_start + [L1*cos(q1_i); L1*sin(q1_i)];
    p2_start = p1_end;
    p2_end = p2_start + [L2*cos(q1_i+q2_i); L2*sin(q1_i+q2_i)];
    
    pcm1 = p_pivot + [Lcm1*cos(q1_i); Lcm1*sin(q1_i)];
    pcm2 = p_pivot + [L1*cos(q1_i) + Lcm2*cos(q1_i+q2_i); L1*sin(q1_i) + Lcm2*sin(q1_i+q2_i)];
    
    set(link1,'XData',[p1_start(1) p1_end(1)],'YData',[p1_start(2) p1_end(2)]);
    set(link2,'XData',[p2_start(1) p2_end(1)],'YData',[p2_start(2) p2_end(2)]);
    set(com1,'XData',pcm1(1),'YData',pcm1(2));
    set(com2,'XData',pcm2(1),'YData',pcm2(2));
    set(joint1,'XData',p1_start(1),'YData',p1_start(2));  % not changing
    set(joint2,'XData',p2_start(1),'YData',p2_start(2));
    drawnow;
    pause(0.02);
end
end

