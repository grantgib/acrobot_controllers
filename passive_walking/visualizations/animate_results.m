function [] = animate_results(x,w)
%ANIMATERESULTS Summary of this function goes here
%   Detailed explanation goes here
n_q = 4;
x_init = x(:,1);

blue = [0 0.4470 0.7410];
orange = [0.8500 0.3250 0.0980];
green = [0.4660 0.6740 0.1880];

xs = 100;

%% Initialize figure
x_i = x_init;
q_i = x_init(1:4);

% Points of interest
p_stance = w.pst_func(q_i);       % origin of pinned acrobot link
p1 = w.p1_func(q_i);
p2 = w.p2_func(q_i);
p3 = w.p3_func(q_i);
p_swing = w.psw_func(q_i);
ground_top = p_stance + [-xs; xs*tan(w.alpha)];
ground_bottom = p_stance + [xs; -xs*tan(w.alpha)];

set(0,'DefaultFigureWindowStyle','docked');
figure(1)
wd = 3;
sz = 200;
grid on;
axis equal

% Draw links
hold on; link1 = line([p_stance(1) p2(1)],[p_stance(2) p2(2)],'color',blue,'LineWidth',wd);
hold on; link2 = line([p2(1) p_swing(1)],[p2(2) p_swing(2)],'color',blue,'LineWidth',wd);

% Draw Masses
hold on; M1 = scatter(p1(1),p1(2),sz,green,'filled');
hold on; Mp = scatter(p2(1),p2(2),2*sz,green,'filled');
hold on; M2 = scatter(p3(1),p3(2),sz,green,'filled');

% Draw Joints (non for passive)
% hold on; joint1 = scatter(p1_start(1),p1_start(2),5*sz,orange,'filled');

% Draw Ground
hold on; ground = line([ground_top(1) ground_bottom(1)],[ground_top(2) ground_bottom(2)],'color','k','LineWidth',5);

%% Run Animation
% disp("Press Key to Begin Animation...");
% pause
for i = 1:size(x,2)
    x_i = x(:,i);
    q_i = x_i(1:n_q,1);
    
    p_stance = w.pst_func(q_i);       % origin of pinned acrobot link
    p1 = w.p1_func(q_i);
    p2 = w.p2_func(q_i);
    p3 = w.p3_func(q_i);
    p_swing = w.psw_func(q_i);

    set(link1,'XData',[p_stance(1) p2(1)],'YData',[p_stance(2) p2(2)]);
    set(link2,'XData',[p2(1) p_swing(1)],'YData',[p2(2) p_swing(2)]);
    set(M1,'XData',p1(1),'YData',p1(2));
    set(Mp,'XData',p2(1),'YData',p2(2));
    set(M2,'XData',p3(1),'YData',p3(2));

    axis([-2+p2(1) 2+p2(1) -2+p2(2) 2+p2(2)])
    drawnow;
    pause(0.05);
    
end
end

