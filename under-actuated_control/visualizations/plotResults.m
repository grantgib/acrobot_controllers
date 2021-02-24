function [] = plotResults(t,x,u,params)
set(0,'DefaultFigureWindowStyle','docked')
x_header = {'$q_1$','$q_2$','$\dot{q}_1$','$\dot{q}_1$'};
figure
for i = 1:size(x,1)
    subplot(2,2,i)
    plot(t',x(i,:));
    if i <= 2
        hold on; yline(params.x_des(i),":r");
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
    subplot(1,1,i)
    stairs(t,u(i,:));
    grid on;
    title(u_header{i},'interpreter','latex');
    xlabel('time');
end
sgtitle('Motor Torques');

disp('Plots Complete');
end


