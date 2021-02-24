function [] = plot_results(t,x,params)
set(0,'DefaultFigureWindowStyle','docked')

%% State plots
x_header = {'$p_{st,x}$','$p_{st,z}$','$q_{st}$','$q_{sw}$',...
    '$\dot{p}_{st,x}$','$\dot{p}_{st,z}$','$\dot{q}_{st}$','$\dot{q}_{sw}$'};
figure
for i = 1:size(x,1)
    subplot(2,4,i)
    plot(t',x(i,:));
    grid on;
    title(x_header{i},'interpreter','latex')
    xlabel('time');
end
sgtitle('States')

%% Phase Plots
figure
iter = 1;
subplot(1,2,1)
plot(x(3,iter:end),x(7,iter:end));
grid on;
xlabel('$q_{st}$','interpreter','latex');
ylabel('$\dot{q}_{st}$','interpreter','latex');

subplot(1,2,2)
plot(x(4,iter:end),x(8,iter:end));
grid on;
xlabel('$q_{sw}$','interpreter','latex');
ylabel('$\dot{q}_{sw}$','interpreter','latex');
sgtitle('Phase Plots')

disp('Plots Complete');
end


