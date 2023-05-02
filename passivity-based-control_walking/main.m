%% Passive Acrobot Walker
clear; clc; close all;

disp('Acrobot Walker with Passive Dynamics');

% Setup path
restoredefaultpath;
addpath(genpath('./'));

% Load dynamics
params = load('load_dynamics.mat');

% Choose dynamics
% params.num_steps = 1000;
% params.slope_actual = deg2rad(-1); % slopes that work: 6, 0, -1 
% params.kgain = 1.5;
% params.anim = 0;

params.num_steps = input("How many steps should be simulated: ");
params.slope_actual = deg2rad(input("Choose slope to walk on (deg).\n  Downward slope is positive and the reference is 3 deg.\n  Try 6,3,0,-1: "));
params.kgain = input("Choose passivity gain (default = 1.5): ");
params.anim = input("Show Animation? \nEnter (1) for yes and (0) for no: ");
input("Slope set at 3 deg. Press Enter to simulate\n");

params.slope_ref = deg2rad(3);
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
    [t,x] = ode45(@(t,x) acrobot_PBC_ode(t,x,params),tspan,x_init,options);
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

%% Compute energy trajectory
for i = 1:size(x_traj,2)
    E_traj(1,i) = params.E_func(x_traj(:,i));
end

% 10,000 steps -> E_limit = 153.2380
% E_limit = mean(E_traj(end-100:end));

%% Plot
plot_results(t_traj,x_traj,E_traj,params);

%% Functions
function [xdot] = acrobot_PBC_ode(t,x,p)
    % compute passivity based control
    k = p.kgain;
    u = p.u_func(x,p.slope_ref,p.slope_actual,k);
    
    % xdot
    xdot = p.x_dot_func(x,u);
end

function [x_impact] = impact_map(x,p)
    x_impact = p.x_impact_func(x);
end

function [x_switch] = reset_map(x,p)
    x_switch = p.x_switch_func(x);
end