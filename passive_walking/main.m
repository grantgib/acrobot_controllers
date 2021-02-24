%% Passive Acrobot Walker
clear; clc; close all;

disp('Acrobot Walker with Passive Dynamics');

% Setup path
restoredefaultpath;
addpath(genpath('./'));

% Load dynamics
params = load('load_dynamics.mat');

% Choose dynamics
params.num_steps = input("How many steps should be simulated: ");
params.anim = input("Show Animation? \nEnter (1) for yes and (0) for no: ");

input("Slope set at pi/50. Press Enter to simulate\n");

params.alpha = pi/50;
t_traj = [];
x_traj = [];

%% Simulate (Animate after each impact)
q_init = [0; 0; 0; 0];          % [px,pz,qst,qsw]'
qdot_init = [0; 0; -0.4; 2.0];
x_init = [q_init; qdot_init];  
for i = 1:params.num_steps
    if i == 1
        x_init = x_init; % initial state remains unchanged
    else
        x_impact = impact_map(x_end,params);
        x_init = reset_map(x_impact,params);
    end
    tspan = 0:0.05:10;
    options = odeset('Event',@(t,x) impact_event(t,x,params));
    [t,x] = ode45(@(t,x) acrobot_passive_ode(t,x,params),tspan,x_init,options);
    t = t';
    x = x';
    
    % store traj
    t_traj = [t_traj, t];
    x_traj = [x_traj, x];
    
    % animate step
    if params.anim
        animate_results(x,params);
    end
    x_end = x(:,end);
end
%% Plot
plot_results(t_traj,x_traj,params);

%% Functions
function [xdot] = acrobot_passive_ode(t,x,p)
    xdot = p.x_dot_func(x);
end

function [x_impact] = impact_map(x,p)
    x_impact = p.x_impact_func(x);
end

function [x_switch] = reset_map(x,p)
    x_switch = p.x_switch_func(x);
end