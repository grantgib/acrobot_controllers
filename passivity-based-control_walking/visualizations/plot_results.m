function [] = plot_results(t,x,E,params)
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
plot(x(3,iter:end),x(7,iter:end));
hold on; plot(x(4,iter:end),x(8,iter:end));
grid on;
xlabel('$q_{i}$','interpreter','latex');
ylabel('$\dot{q}_{i}$','interpreter','latex');
sgtitle('Phase Plots')

%% Energy Plots
figure
scatter(t,E,20,'filled');
grid on;
xlabel('time');
ylabel('Total Energy');

disp('Plots Complete');
end


